use crate::routing::{
    models::{
        CarState, CarState2D, PlanningContext, PlanningDump, RoutePlan, RoutePlanError,
        RoutePlanner, SegmentPlan,
    },
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{Brake, Chain, ForwardDodge, Straight, StraightMode},
};
use arrayvec::ArrayVec;
use common::prelude::*;
use derive_new::new;
use nalgebra::Point2;
use nameof::name_of_type;
use ordered_float::NotNan;
use simulate::{Car1D, CarForwardDodge, CarForwardDodge1D};

/// Drive straight. Requires the car to already be facing the target (i.e., it
/// won't steer left or right).
#[derive(Clone)]
pub struct GroundStraightPlanner {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
    allow_dodging: bool,
    allow_boost: bool,
    always_prefer_dodge: bool,
}

impl GroundStraightPlanner {
    pub fn new(target_loc: Point2<f32>, mode: StraightMode) -> Self {
        Self {
            target_loc,
            target_time: None,
            end_chop: 0.0,
            mode,
            allow_dodging: true,
            allow_boost: true,
            always_prefer_dodge: true,
        }
    }

    pub fn target_time(mut self, target_time: f32) -> Self {
        assert!(target_time >= 0.0);
        self.target_time = Some(target_time);
        self
    }

    pub fn end_chop(mut self, end_chop: f32) -> Self {
        self.end_chop = end_chop;
        self
    }

    pub fn allow_dodging(mut self, allow_dodging: bool) -> Self {
        self.allow_dodging = allow_dodging;
        self
    }

    pub fn allow_boost(mut self, allow_boost: bool) -> Self {
        self.allow_boost = allow_boost;
        self
    }

    pub fn always_prefer_dodge(mut self, always_prefer_dodge: bool) -> Self {
        self.always_prefer_dodge = always_prefer_dodge;
        self
    }
}

impl RoutePlanner for GroundStraightPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(GroundStraightPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log_pretty(self, "target_loc", self.target_loc);

        assert!(!self.target_loc.x.is_nan());
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });

        let straight = StraightSimple::new(
            self.target_loc,
            self.target_time,
            self.end_chop,
            self.mode,
            self.allow_boost,
        );
        let straight = straight.plan(ctx, dump);

        let dodge = if self.allow_dodging {
            let planner =
                StraightWithDodge::new(self.target_loc, self.target_time, self.end_chop, self.mode);
            Some(planner.plan(ctx, dump))
        } else {
            None
        };

        // If we're prioritizing dodges and we have a dodge, return early.
        if self.always_prefer_dodge {
            if let Some(Ok(plan)) = dodge {
                return Ok(plan);
            }
        }

        // Return the fastest plan.
        let mut plans = ArrayVec::<[_; 4]>::new();
        plans.push(straight);
        if let Some(dodge) = dodge {
            plans.push(dodge);
        }
        Ok(fastest(at_least_one_ok(plans)?))
    }
}

fn at_least_one_ok<T, E>(results: impl IntoIterator<Item = Result<T, E>>) -> Result<Vec<T>, E> {
    let mut oks: Vec<T> = Vec::new();
    let mut error = None;
    for result in results {
        match result {
            Ok(x) => oks.push(x),
            Err(e) => error = Some(e),
        }
    }
    if oks.is_empty() {
        Err(error.unwrap())
    } else {
        Ok(oks)
    }
}

fn fastest(steps: impl IntoIterator<Item = RoutePlan>) -> RoutePlan {
    steps
        .into_iter()
        .min_by_key(|s| NotNan::new(s.segment.duration()).unwrap())
        .unwrap()
}

#[derive(Clone, new)]
struct StraightSimple {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
    allow_boost: bool,
}

impl RoutePlanner for StraightSimple {
    fn name(&self) -> &'static str {
        name_of_type!(StraightSimple)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        _dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });
        guard!(
            ctx.start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        if self.would_coasting_still_be_too_fast(ctx) {
            let target_dist = (self.target_loc - ctx.start.loc.to_2d()).norm();
            let target_speed = target_dist / self.target_time.unwrap();
            let brake_start = CarState2D {
                loc: ctx.start.loc.to_2d(),
                rot: ctx.start.rot.to_2d(),
                vel: ctx.start.vel.to_2d(),
                boost: ctx.start.boost,
            };
            return Ok(RoutePlan {
                segment: Box::new(Brake::new(brake_start, target_speed)),
                next: Some(Box::new(self.clone())),
            });
        }

        let segment = Straight::new(
            CarState2D {
                loc: ctx.start.loc.to_2d(),
                rot: ctx.start.rot.to_2d(),
                vel: ctx.start.vel.to_2d(),
                boost: ctx.start.boost,
            },
            self.target_loc,
            self.end_chop,
            self.mode,
            self.allow_boost,
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}

impl StraightSimple {
    fn would_coasting_still_be_too_fast(&self, ctx: &PlanningContext<'_, '_>) -> bool {
        let target_time = some_or_else!(self.target_time, {
            return false;
        });

        let target_dist = (self.target_loc - ctx.start.loc.to_2d()).norm();
        let mut sim = Car1D::new()
            .with_speed(ctx.start.vel.to_2d().norm())
            .with_boost(ctx.start.boost);
        sim.advance(target_time, 0.0, false);
        sim.distance() > target_dist
    }
}

/// Calculate a ground interception of the ball with a single dodge.
#[derive(Clone, new)]
struct StraightWithDodge {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
}

impl RoutePlanner for StraightWithDodge {
    fn name(&self) -> &'static str {
        name_of_type!(StraightWithDodge)
    }

    fn plan(
        &self,
        ctx: &PlanningContext<'_, '_>,
        _dump: &mut PlanningDump<'_>,
    ) -> Result<RoutePlan, RoutePlanError> {
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });
        guard!(
            ctx.start,
            NotFacingTarget2D::new(self.target_loc),
            RoutePlanError::MustBeFacingTarget,
        );

        let dodges = StraightDodgeCalculator::new(
            ctx.start.clone(),
            self.target_loc,
            self.target_time,
            self.end_chop,
        )
        .collect();
        let dodge = dodges
            .into_iter()
            .min_by_key(|d| NotNan::new(d.score).unwrap())
            .ok_or(RoutePlanError::MovingTooFast)?;

        let before = Straight::new(
            CarState2D {
                loc: ctx.start.loc.to_2d(),
                rot: ctx.start.rot.to_2d(),
                vel: ctx.start.vel.to_2d(),
                boost: ctx.start.boost,
            },
            ctx.start.loc.to_2d()
                + (self.target_loc - ctx.start.loc.to_2d()).normalize() * dodge.approach_distance,
            0.0,
            StraightMode::Asap,
            true,
        );

        let dodge = ForwardDodge::new(before.end(), dodge.dodge);

        let mut after = GroundStraightPlanner::new(self.target_loc, self.mode);
        after.target_time = self.target_time;
        after.end_chop = self.end_chop;

        let segment = Chain::new(vec![Box::new(before), Box::new(dodge)]);
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: Some(Box::new(after)),
        })
    }
}

/// Calculate motions consisting of straight, then dodge, then straight again.
#[derive(new)]
struct StraightDodgeCalculator {
    start: CarState,
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    end_chop: f32,
}

impl StraightDodgeCalculator {
    pub fn collect(&self) -> Vec<StraightDodge> {
        // Performance knob
        const GRANULARITY: f32 = 0.125;

        let mut result = Vec::new();

        let mut t = 0.0;
        loop {
            if let Some(target_time) = self.target_time {
                if t >= target_time {
                    break;
                }
            }

            match self.evaluate(t) {
                Some(dodge) => result.push(dodge),
                None => break,
            }

            t += GRANULARITY;
        }

        result
    }

    fn evaluate(&self, approach_time: f32) -> Option<StraightDodge> {
        let mut approach = Car1D::new()
            .with_speed(self.start.vel.to_2d().norm())
            .with_boost(self.start.boost);
        approach.advance(approach_time, 1.0, true);

        let dodge = CarForwardDodge::calc_1d(approach.speed());

        // `end_chop` is "dead time" that the caller requested we leave available for
        // the subsequent maneuver. Coasting on landing is the most conservative case.
        let mut landing = Car1D::new()
            .with_speed(dodge.end_speed)
            .with_boost(approach.boost());
        landing.advance(self.end_chop, 0.0, false);

        // Now we know where this dodge would take us. Let's check if it meets the
        // requirements:

        // Check if we can even complete the dodge by the target time.
        let total_time = approach.time() + dodge.duration() + landing.time();
        if let Some(target_time) = self.target_time {
            if total_time > target_time {
                return None;
            }
        }

        // Check that we don't land past the target.
        let target_traveled = (self.target_loc - self.start.loc.to_2d()).norm();
        let total_dist = approach.distance() + dodge.end_dist + landing.distance();
        if total_dist >= target_traveled {
            return None;
        }

        // Now simulate coasting after the landing. If we pass the target before the
        // target time, dodging made us go too fast.
        if let Some(target_time) = self.target_time {
            let mut coast = Car1D::new()
                .with_speed(dodge.end_speed)
                .with_boost(approach.boost());
            coast.advance(target_time - total_time, 0.0, false);
            if total_dist + coast.distance() > target_traveled {
                return None;
            }
        }

        // To calculate the "best" dodge, they all need to be compared on equal terms.
        // The equal term I choose is the minimum time needed to reach the target.
        // Do not boost here so that we don't outperform multiple dodges.
        let mut blitz = Car1D::new()
            .with_speed(dodge.end_speed)
            .with_boost(approach.boost());
        blitz.advance_by_distance(target_traveled - total_dist, 1.0, false);
        let score = total_time + blitz.time();

        Some(StraightDodge {
            approach_distance: approach.distance(),
            dodge,
            score,
        })
    }
}

struct StraightDodge {
    approach_distance: f32,
    dodge: CarForwardDodge1D,
    score: f32,
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        integration_tests::{TestRunner, TestScenario},
        routing::{
            behavior::FollowRoute, plan::ground_straight::GroundStraightPlanner,
            segments::StraightMode,
        },
    };
    use common::prelude::*;
    use nalgebra::{Point2, Point3, Vector3};

    #[test]
    fn brake_when_going_too_fast() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(1000.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, 2000.0, 0.0),
                ..Default::default()
            })
            .behavior(FollowRoute::new(
                GroundStraightPlanner::new(Point2::new(0.0, 1000.0), StraightMode::Fake)
                    .target_time(1.0),
            ))
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        let loc = packet.GameCars[0].Physics.loc();
        assert!(loc.y < 1000.0);
    }
}
