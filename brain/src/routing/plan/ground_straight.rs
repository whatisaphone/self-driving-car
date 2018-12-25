use crate::routing::{
    models::{
        CarState, CarState2D, PlanningContext, PlanningDump, RoutePlan, RoutePlanError,
        RoutePlanner, SegmentPlan,
    },
    recover::{IsSkidding, NotFacingTarget2D, NotOnFlatGround},
    segments::{Brake, Chain, ForwardDodge, Straight, StraightMode},
};
use arrayvec::ArrayVec;
use common::{prelude::*, PrettyPrint};
use derive_new::new;
use nalgebra::Point2;
use nameof::name_of_type;
use ordered_float::NotNan;
use simulate::{Car1D, CarForwardDodge, CarForwardDodge1D};

/// Drive straight. Requires the car to already be facing the target (i.e., it
/// won't steer left or right).
#[derive(Clone, new)]
pub struct GroundStraightPlanner {
    target_loc: Point2<f32>,
    target_time: Option<f32>,
    /// How early to return from the SegmentRunner. This can be used to give
    /// control to a subsequent behavior and leave it enough time to jump,
    /// shoot, position itself, etc.
    end_chop: f32,
    mode: StraightMode,
    #[new(value = "true")]
    allow_dodging: bool,
}

impl GroundStraightPlanner {
    pub fn allow_dodging(mut self, allow_dodging: bool) -> Self {
        self.allow_dodging = allow_dodging;
        self
    }
}

impl RoutePlanner for GroundStraightPlanner {
    fn name(&self) -> &'static str {
        name_of_type!(GroundStraightPlanner)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        dump: &mut PlanningDump,
    ) -> Result<RoutePlan, RoutePlanError> {
        dump.log_start(self, &ctx.start);
        dump.log(self, format!("target_loc = {}", self.target_loc.pretty()));

        assert!(!self.target_loc.x.is_nan());
        guard!(
            ctx.start,
            NotOnFlatGround,
            RoutePlanError::MustBeOnFlatGround,
        );
        guard!(ctx.start, IsSkidding, RoutePlanError::MustNotBeSkidding {
            recover_target_loc: self.target_loc,
        });

        let mut planners = ArrayVec::<[&RoutePlanner; 4]>::new();
        let straight =
            StraightSimple::new(self.target_loc, self.target_time, self.end_chop, self.mode);
        planners.push(&straight);

        let with_dodge;
        if self.allow_dodging {
            with_dodge =
                StraightWithDodge::new(self.target_loc, self.target_time, self.end_chop, self.mode);
            planners.push(&with_dodge);
        }

        let plans = planners.into_iter().map(|p| p.plan(ctx, dump));
        let plans = at_least_one_ok(plans)?;
        Ok(fastest(plans.into_iter()))
    }
}

fn at_least_one_ok<T, E>(results: impl Iterator<Item = Result<T, E>>) -> Result<Vec<T>, E> {
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

fn fastest(steps: impl Iterator<Item = RoutePlan>) -> RoutePlan {
    steps
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
}

impl RoutePlanner for StraightSimple {
    fn name(&self) -> &'static str {
        name_of_type!(StraightSimple)
    }

    fn plan(
        &self,
        ctx: &PlanningContext,
        _dump: &mut PlanningDump,
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
        );
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
        })
    }
}

impl StraightSimple {
    fn would_coasting_still_be_too_fast(&self, ctx: &PlanningContext) -> bool {
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
        ctx: &PlanningContext,
        _dump: &mut PlanningDump,
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
        );
        let dodge = ForwardDodge::new(before.end(), dodge.dodge);
        let dodge_end = dodge.end();
        let after = Straight::new(
            CarState2D {
                loc: dodge_end.loc.to_2d(),
                rot: dodge_end.rot.to_2d(),
                vel: dodge_end.vel.to_2d(),
                boost: dodge_end.boost,
            },
            self.target_loc,
            self.end_chop,
            self.mode,
        );
        let segment = Chain::new(vec![Box::new(before), Box::new(dodge), Box::new(after)]);
        Ok(RoutePlan {
            segment: Box::new(segment),
            next: None,
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
        integration_tests::helpers::{TestRunner, TestScenario},
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
            .behavior(FollowRoute::new(GroundStraightPlanner::new(
                Point2::new(0.0, 1000.0),
                Some(1.0),
                0.0,
                StraightMode::Fake,
            )))
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        let loc = packet.GameCars[0].Physics.loc();
        assert!(loc.y < 1000.0);
    }
}
