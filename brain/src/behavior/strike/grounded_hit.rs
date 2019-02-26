use crate::{
    behavior::{
        higher_order::Chain,
        movement::{simple_steer_towards, Dodge, JumpAndTurn},
        strike::BounceShot,
    },
    eeg::{Drawable, EEG},
    helpers::intercept::{naive_ground_intercept, NaiveIntercept},
    routing::recover::{IsSkidding, NotOnFlatGround},
    rules::SameBallTrajectory,
    strategy::{Action, Behavior, Context, Game, Priority, Scenario},
};
use common::{physics, prelude::*, rl, Coordinate, Distance};
use derive_new::new;
use nalgebra::{Point2, Point3, UnitQuaternion};
use nameof::name_of_type;
use simulate::{
    car_single_jump::{time_to_z, JUMP_MAX_Z},
    linear_interpolate, Car1D,
};
use std::f32::consts::PI;

pub struct GroundedHit<Aim>
where
    Aim: Fn(&mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> + Send,
{
    aim: Aim,
    same_ball_trajectory: SameBallTrajectory,
}

impl<Aim> GroundedHit<Aim>
where
    Aim: Fn(&mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> + Send,
{
    pub fn hit_towards(aim: Aim) -> Self {
        Self {
            aim,
            same_ball_trajectory: SameBallTrajectory::new(),
        }
    }
}

impl GroundedHit<fn(&mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()>> {
    const CONTACT_Z_OFFSET: f32 = -70.0; // This is misguided and should probably go away.
    pub const MAX_BALL_Z: f32 = 220.0 - Self::CONTACT_Z_OFFSET; // TODO: how high can I jump

    /// A preset for `Aim` that hits the ball straight ahead.
    pub fn opposite_of_self(
        car: &common::halfway_house::PlayerInfo,
        ball: Point3<f32>,
    ) -> Point2<f32> {
        ball.to_2d() + (ball.to_2d() - car.Physics.loc_2d())
    }
}

impl<Aim> Behavior for GroundedHit<Aim>
where
    Aim: Fn(&mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> + Send,
{
    fn name(&self) -> &str {
        stringify!(GroundedHit)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let me = ctx.me();

        if IsSkidding.evaluate(&me.into()) {
            ctx.eeg.log(self.name(), name_of_type!(IsSkidding));
            return Action::Abort;
        }
        if NotOnFlatGround.evaluate(&me.into()) {
            ctx.eeg.log(self.name(), name_of_type!(NotOnFlatGround));
            return Action::Abort;
        }
        return_some!(self.same_ball_trajectory.execute_old(ctx));

        let intercept = match self.intercept_loc(ctx) {
            Ok(i) => i,
            Err(()) => return Action::Abort,
        };

        let plan = match self.plan(ctx, &intercept) {
            Ok(x) => x,
            Err(()) => {
                ctx.eeg.log(self.name(), "error finding target_loc");
                return Action::Abort;
            }
        };

        let me_forward = me.Physics.forward_axis_2d();
        let steer = me_forward.angle_to(&(plan.target_loc - me.Physics.loc()).to_2d().to_axis());
        if steer.abs() >= PI / 3.0 {
            ctx.eeg.log(self.name(), "not facing the target");
            return Action::Abort;
        }

        match self.estimate_approach(ctx, &plan) {
            Do::Drive(throttle, boost) => self.drive(ctx, &plan, throttle, boost),
            Do::Jump => self.jump(&plan),
        }
    }
}

impl<Aim> GroundedHit<Aim>
where
    Aim: Fn(&mut GroundedHitAimContext<'_, '_>) -> Result<GroundedHitTarget, ()> + Send,
{
    fn intercept_loc(&mut self, ctx: &mut Context<'_>) -> Result<NaiveIntercept, ()> {
        let me = ctx.me();

        // First pass: get approximate jump height
        let intercept = naive_ground_intercept(
            ctx.scenario.ball_prediction().iter(),
            me.Physics.loc(),
            me.Physics.vel(),
            me.Boost as f32,
            |ball| ball.loc.z < GroundedHit::MAX_BALL_Z,
        );
        let intercept = some_or_else!(intercept, {
            ctx.eeg.log(self.name(), "can't find intercept");
            return Err(());
        });

        let mut aim_context = GroundedHitAimContext {
            game: ctx.game,
            scenario: &ctx.scenario,
            car: me,
            intercept_time: intercept.time,
            intercept_ball_loc: intercept.ball_loc,
            eeg: ctx.eeg,
        };
        let target = (self.aim)(&mut aim_context)
            .map_err(|_| ctx.eeg.log(self.name(), "error getting aim location"))?;
        let (target_loc, _target_rot) = Self::preliminary_target(ctx, &intercept, &target);
        let ball_max_z = JUMP_MAX_Z + (intercept.ball_loc.z - target_loc.z);

        // Second pass: Get a more accurate intercept based on how high we need to jump.
        let intercept = naive_ground_intercept(
            ctx.scenario.ball_prediction().iter(),
            me.Physics.loc(),
            me.Physics.vel(),
            me.Boost as f32,
            |ball| ball.loc.z < ball_max_z,
        );
        let intercept = some_or_else!(intercept, {
            ctx.eeg.log(self.name(), "can't find intercept");
            return Err(());
        });

        Ok(intercept)
    }

    fn plan(&mut self, ctx: &mut Context<'_>, intercept: &NaiveIntercept) -> Result<Plan, ()> {
        let me = ctx.me();
        let mut aim_context = GroundedHitAimContext {
            game: &ctx.game,
            scenario: &ctx.scenario,
            car: me,
            intercept_time: intercept.time,
            intercept_ball_loc: intercept.ball_loc,
            eeg: ctx.eeg,
        };
        let target = (self.aim)(&mut aim_context)?;

        let (target_loc, target_rot) = Self::preliminary_target(ctx, intercept, &target);

        // TODO: iteratively find contact point which hits the ball towards aim_loc

        ctx.eeg.print_time("intercept_time", intercept.time);
        ctx.eeg
            .print_value("intercept_loc_z", Coordinate(intercept.ball_loc.z));
        ctx.eeg.draw(Drawable::Crosshair(target.aim_loc));
        ctx.eeg.draw(Drawable::ghost_ball(intercept.ball_loc));
        ctx.eeg
            .draw(Drawable::GhostCar(target_loc, me.Physics.rot()));

        Ok(Plan {
            intercept_time: target.intercept_time,
            target_loc,
            target_rot,
            jump: target.jump,
            dodge: target.dodge,
        })
    }

    fn preliminary_target(
        ctx: &mut Context<'_>,
        intercept: &NaiveIntercept,
        target: &GroundedHitTarget,
    ) -> (Point3<f32>, UnitQuaternion<f32>) {
        // Pitch the nose higher if the target is further away.
        let pitch_from_distance = linear_interpolate(
            &[1000.0, 5000.0],
            &[PI / 15.0, PI / 4.0],
            (target.aim_loc - intercept.ball_loc.to_2d()).norm(),
        );
        // Also pitch the nose higher if the ball wlil be falling quickly when we make
        // contact.
        let pitch_from_ball_vel =
            linear_interpolate(&[-1200.0, 0.0], &[PI / 4.0, 0.0], intercept.ball_vel.z);
        // Take the more extreme of the two.
        let pitch = pitch_from_distance.max(pitch_from_ball_vel);

        // Just do something hacky for now
        let (naive_target_loc, target_rot) = car_ball_contact_with_pitch(
            ctx.game,
            intercept.ball_loc,
            ctx.me().Physics.loc(),
            pitch,
        );
        let target_loc = match target.adjust {
            GroundedHitTargetAdjust::RoughAim => {
                let rough_loc = BounceShot::rough_shooting_spot(intercept, target.aim_loc)
                    .to_3d(naive_target_loc.z);
                // Pre-tournament hack – hug the ball closer if it's falling faster, so we're
                // less likely to whiff.
                let (hug_loc, _hug_rot) =
                    car_ball_contact_with_pitch(ctx.game, intercept.ball_loc, rough_loc, pitch);
                let hug_factor =
                    linear_interpolate(&[-1500.0, -500.0], &[1.0, 0.0], intercept.ball_vel.z);
                ctx.eeg
                    .print_value("hug_factor", format!("{:.2}", hug_factor));
                rough_loc + (hug_loc - rough_loc) * hug_factor
            }
            GroundedHitTargetAdjust::StraightOn => naive_target_loc,
        };
        (target_loc, target_rot)
    }

    #[allow(clippy::if_same_then_else)]
    fn estimate_approach(&mut self, ctx: &mut Context<'_>, plan: &Plan) -> Do {
        let total_time = plan.intercept_time;
        let jump_duration = Self::jump_duration(plan.target_loc.z);
        let drive_time = total_time - jump_duration;

        if drive_time < 0.0 {
            return Do::Jump;
        }

        let would_reach = |throttle, boost| {
            // Phase 1: driving forward
            let mut drive = Car1D::new()
                .with_speed(ctx.me().Physics.vel().norm())
                .with_boost(ctx.me().Boost as f32);
            drive.advance(drive_time, throttle, boost);
            let drive_start_loc = ctx.me().Physics.loc_2d();
            let drive_forward = (plan.target_loc.to_2d() - drive_start_loc).to_axis();
            let drive_end_loc = drive_start_loc + drive_forward.as_ref() * drive.distance();
            let drive_end_vel = drive_forward.as_ref() * drive.speed();

            // Phase 2: a jump in which the xy-velocity stays constant
            let jump_end_loc = drive_end_loc + drive_end_vel * jump_duration;

            // Calculate how far ahead/behind the target location
            (jump_end_loc - plan.target_loc.to_2d()).dot(&drive_forward)
        };

        // Aim for a few uu behind the ball so we don't make contact before we dodge.
        let target_offset = -10.0;

        let coast_offset = would_reach(0.0, false);
        let throttle_offset = would_reach(1.0, false);
        let blitz_offset = would_reach(1.0, true);

        let (throttle, boost) = if coast_offset > target_offset {
            (0.0, false) // We're overshooting…
        } else if throttle_offset > target_offset {
            (0.0, false)
        } else if blitz_offset > target_offset {
            (1.0, false)
        } else {
            (1.0, true)
        };

        ctx.eeg.print_value("target", plan.target_loc);
        ctx.eeg.print_time("drive_time", drive_time);
        ctx.eeg.print_time("total_time", total_time);
        ctx.eeg.print_value("coast_offset", Distance(coast_offset));
        ctx.eeg
            .print_value("throttle_offset", Distance(throttle_offset));
        ctx.eeg.print_value("blitz_offset", Distance(blitz_offset));

        Do::Drive(throttle, boost)
    }

    fn drive(&self, ctx: &mut Context<'_>, plan: &Plan, throttle: f32, boost: bool) -> Action {
        let me = ctx.me();
        let steer = simple_steer_towards(&me.Physics, plan.target_loc.to_2d());
        Action::Yield(common::halfway_house::PlayerInput {
            Throttle: throttle,
            Steer: steer,
            Boost: boost && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED,
            ..Default::default()
        })
    }

    fn jump(&self, plan: &Plan) -> Action {
        // Simulate the jump to predict our exact location at the peak.
        let jump_time = Self::jump_duration(plan.target_loc.z);

        if !plan.jump {
            return Action::Return;
        }

        let mut steps = Vec::<Box<dyn Behavior>>::new();
        steps.push(Box::new(JumpAndTurn::new(
            jump_time - 0.05,
            jump_time,
            plan.target_rot,
        )));
        if plan.dodge {
            steps.push(Box::new(Dodge::new().towards_ball()));
        }

        Action::tail_call(Chain::new(Priority::Strike, steps))
    }

    fn jump_duration(z: f32) -> f32 {
        // Avoid a panic in `time_to_z()` from trying to jump too high. Assert that the
        // error is small before we clamp the value.
        let leeway = 20.0;
        assert!(z < JUMP_MAX_Z + leeway, "{} {} {}", z, JUMP_MAX_Z, leeway);
        let clamped = z.min(JUMP_MAX_Z);

        // Always leave at least enough time for the jump before the dodge.
        time_to_z(clamped).unwrap().max(JumpAndTurn::MIN_DURATION)
    }
}

pub fn car_ball_contact_with_pitch(
    game: &Game<'_>,
    ball_loc: Point3<f32>,
    car_reference_loc: Point3<f32>,
    pitch: f32,
) -> (Point3<f32>, UnitQuaternion<f32>) {
    let car_rot = physics::CAR_LOCAL_FORWARD_AXIS_2D
        .rotation_to(&(ball_loc - car_reference_loc).to_2d().to_axis())
        .around_z_axis();
    let up = UnitQuaternion::from_axis_angle(&physics::car_right_axis(car_rot), -pitch);
    let car_rot = up * car_rot;

    let contact_dist = game.ball_radius() + game.me_vehicle().pivot_to_front_dist();
    let flat_ball_to_car = (car_reference_loc - ball_loc).to_2d().normalize() * contact_dist;
    let car_loc = ball_loc + up * flat_ball_to_car.to_3d(0.0);

    (car_loc, car_rot)
}

pub struct GroundedHitAimContext<'a, 'b> {
    pub game: &'a Game<'b>,
    pub scenario: &'a Scenario<'b>,
    pub car: &'a common::halfway_house::PlayerInfo,
    pub intercept_time: f32,
    pub intercept_ball_loc: Point3<f32>,
    pub eeg: &'a mut EEG,
}

#[derive(new)]
pub struct GroundedHitTarget {
    intercept_time: f32,
    adjust: GroundedHitTargetAdjust,
    aim_loc: Point2<f32>,
    #[new(value = "true")]
    jump: bool,
    #[new(value = "true")]
    dodge: bool,
}

impl GroundedHitTarget {
    pub const MAX_BALL_Z: f32 = GroundedHit::MAX_BALL_Z;

    pub fn jump(mut self, jump: bool) -> Self {
        self.jump = jump;
        self
    }

    pub fn dodge(mut self, dodge: bool) -> Self {
        self.dodge = dodge;
        self
    }
}

pub enum GroundedHitTargetAdjust {
    StraightOn,
    RoughAim,
}

struct Plan {
    intercept_time: f32,
    target_loc: Point3<f32>,
    target_rot: UnitQuaternion<f32>,
    jump: bool,
    dodge: bool,
}

enum Do {
    /// `(throttle, boost)`
    Drive(f32, bool),
    Jump,
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::strike::{GroundedHit, GroundedHitTarget, GroundedHitTargetAdjust},
        integration_tests::{TestRunner, TestScenario},
    };
    use common::{prelude::*, rl};
    use nalgebra::{Point2, Point3, Rotation3, Vector3};

    #[test]
    #[ignore(note = "The great bankruptcy of 2018")]
    fn normal_shoot() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(-2000.0, 2000.0, 500.0),
                ball_vel: Vector3::new(1000.0, 0.0, 0.0),
                car_loc: Point3::new(0.0, 0.0, 17.01),
                ..Default::default()
            })
            .behavior(GroundedHit::hit_towards(|ctx| {
                Ok(GroundedHitTarget::new(
                    ctx.intercept_time,
                    GroundedHitTargetAdjust::RoughAim,
                    Point2::new(0.0, rl::FIELD_MAX_Y),
                ))
            }))
            .run_for_millis(3500);
        assert!(test.has_scored());
    }

    #[test]
    fn rolling_hit() {
        let test = TestRunner::new()
            .scenario(TestScenario {
                ball_loc: Point3::new(3962.02, -1981.12, 152.39),
                ball_vel: Vector3::new(-291.741, 890.49097, -303.581),
                car_loc: Point3::new(3821.52, -3021.23, 16.18),
                car_rot: Rotation3::from_unreal_angles(-0.018183012, 2.1181667, 0.012321899),
                car_vel: Vector3::new(-644.811, 1099.141, 4.311),
                ..Default::default()
            })
            .behavior(GroundedHit::hit_towards(|ctx| {
                Ok(GroundedHitTarget::new(
                    ctx.intercept_time,
                    GroundedHitTargetAdjust::RoughAim,
                    Point2::new(0.0, rl::FIELD_MAX_Y),
                ))
            }))
            .run_for_millis(2000);

        let packet = test.sniff_packet();
        assert!(packet.GameBall.Physics.vel().y > 1500.0);
        // We don't score it yet. This test just makes sure we actually hit the ball lol
        // assert!(test.has_scored());
    }
}
