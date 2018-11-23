use behavior::{Action, Behavior, Chain};
use common::{physics, prelude::*, rl};
use eeg::Drawable;
use mechanics::{simple_steer_towards, Dodge, JumpAndTurn};
use nalgebra::{Point2, Point3, UnitQuaternion};
use predict::{intercept::NaiveIntercept, naive_ground_intercept};
use rlbot;
use routing::recover::{IsSkidding, NotOnFlatGround};
use rules::SameBallTrajectory;
use simulate::{
    car_single_jump::{time_to_z, JUMP_MAX_Z},
    linear_interpolate, Car, CarSimulateError,
};
use std::f32::consts::PI;
use strategy::{Context, Game};

pub struct GroundedHit<Aim>
where
    Aim: Fn(&mut Context, Point3<f32>) -> Result<Point2<f32>, ()> + Send,
{
    aim: Aim,
    same_ball_trajectory: SameBallTrajectory,
    aim_loc: Option<Point2<f32>>,
    intercept: Option<NaiveIntercept>,
    intercept_time: Option<f32>,
}

impl<Aim> GroundedHit<Aim>
where
    Aim: Fn(&mut Context, Point3<f32>) -> Result<Point2<f32>, ()> + Send,
{
    const CONTACT_Z_OFFSET: f32 = -70.0; // This is misguided and should probably go away.
    pub const MAX_BALL_Z: f32 = 240.0 - Self::CONTACT_Z_OFFSET; // TODO: how high can I jump

    pub fn hit_towards(aim: Aim) -> Self {
        Self {
            aim,
            same_ball_trajectory: SameBallTrajectory::new(),
            aim_loc: None,
            intercept: None,
            intercept_time: None,
        }
    }
}

impl GroundedHit<fn(&mut Context, Point3<f32>) -> Result<Point2<f32>, ()>> {
    pub fn max_ball_z() -> f32 {
        Self::MAX_BALL_Z
    }

    /// A preset for `Aim` that hits the ball straight ahead.
    #[allow(dead_code)]
    pub fn opposite_of_self(car: &rlbot::ffi::PlayerInfo, ball: Point3<f32>) -> Point2<f32> {
        ball.to_2d() + (ball.to_2d() - car.Physics.locp().to_2d())
    }
}

impl<Aim> Behavior for GroundedHit<Aim>
where
    Aim: Fn(&mut Context, Point3<f32>) -> Result<Point2<f32>, ()> + Send,
{
    fn name(&self) -> &str {
        stringify!(GroundedHit)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();

        if IsSkidding.evaluate(&me.into()) {
            ctx.eeg.log("[GroundedHit] IsSkidding");
            return Action::Abort;
        }
        if NotOnFlatGround.evaluate(&me.into()) {
            ctx.eeg.log("[GroundedHit] NotOnFlatGround");
            return Action::Abort;
        }
        return_some!(self.same_ball_trajectory.execute(ctx));

        if let Err(()) = self.intercept_loc(ctx) {
            return Action::Abort;
        }

        let (target_loc, target_rot) = match self.target_loc(ctx) {
            Ok(x) => x,
            Err(()) => {
                ctx.eeg.log("[GroundedHit] error finding target_loc");
                return Action::Abort;
            }
        };

        let steer = me
            .Physics
            .forward_axis_2d()
            .rotation_to(&(target_loc - me.Physics.locp()).to_2d().to_axis());
        if steer.angle().abs() >= PI / 6.0 {
            ctx.eeg.log("[GroundedHit] not facing the target");
            return Action::Abort;
        }

        match self.estimate_approach(ctx, target_loc) {
            Ok(Do::Coast) => self.drive(ctx, target_loc, false),
            Ok(Do::Boost) => self.drive(ctx, target_loc, true),
            Ok(Do::Jump) => self.jump(target_loc, target_rot),
            Err(error) => {
                ctx.eeg.log(format!(
                    "[GroundedHit] can't estimate approach: {:?}",
                    error,
                ));
                return Action::Abort;
            }
        }
    }
}

impl<Aim> GroundedHit<Aim>
where
    Aim: Fn(&mut Context, Point3<f32>) -> Result<Point2<f32>, ()> + Send,
{
    fn intercept_loc(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let me = ctx.me();

        // First pass: get approximate jump height
        let intercept = naive_ground_intercept(
            ctx.scenario.ball_prediction().iter(),
            me.Physics.locp(),
            me.Physics.vel(),
            me.Boost as f32,
            |ball| ball.loc.z < Self::MAX_BALL_Z,
        );
        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[GroundedHit] can't find intercept");
            return Err(());
        });

        let aim_loc = (self.aim)(ctx, intercept.ball_loc)?;
        let (target_loc, _) = Self::preliminary_target(ctx, &intercept, aim_loc);
        let ball_max_z = JUMP_MAX_Z + (intercept.ball_loc.z - target_loc.z);

        // Second pass: Get a more accurate intercept based on how high we need to jump.
        let intercept = naive_ground_intercept(
            ctx.scenario.ball_prediction().iter(),
            me.Physics.locp(),
            me.Physics.vel(),
            me.Boost as f32,
            |ball| ball.loc.z < ball_max_z,
        );
        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[GroundedHit] can't find intercept");
            return Err(());
        });

        self.intercept_time = Some(ctx.packet.GameInfo.TimeSeconds + intercept.time);
        self.intercept = Some(intercept);
        Ok(())
    }

    fn target_loc(&mut self, ctx: &mut Context) -> Result<(Point3<f32>, UnitQuaternion<f32>), ()> {
        let intercept = self.intercept.as_ref().unwrap();
        let intercept_time = self.intercept_time.unwrap();

        let me = ctx.me();
        let aim_loc = (self.aim)(ctx, intercept.ball_loc)?;
        self.aim_loc = Some(aim_loc);

        let (target_loc, target_rot) = Self::preliminary_target(ctx, intercept, aim_loc);

        // TODO: iteratively find contact point which hits the ball towards aim_loc

        ctx.eeg.print_time("intercept_time", intercept_time);
        ctx.eeg
            .print_value("intercept_loc_z", format!("{:.0}", intercept.ball_loc.z));
        ctx.eeg.draw(Drawable::Crosshair(aim_loc.coords));
        ctx.eeg.draw(Drawable::GhostBall(intercept.ball_loc.coords));
        ctx.eeg
            .draw(Drawable::GhostCar(target_loc.coords, me.Physics.rot()));

        Ok((target_loc, target_rot))
    }

    fn preliminary_target(
        ctx: &mut Context,
        intercept: &NaiveIntercept<()>,
        aim_loc: Point2<f32>,
    ) -> (Point3<f32>, UnitQuaternion<f32>) {
        let pitch = linear_interpolate(
            &[1000.0, 5000.0],
            &[PI / 15.0, PI / 4.0],
            (aim_loc - intercept.ball_loc.to_2d()).norm(),
        );
        car_ball_contact_with_pitch(ctx.game, intercept.ball_loc, ctx.me().Physics.locp(), pitch)
    }

    fn estimate_approach(
        &mut self,
        ctx: &mut Context,
        target_loc: Point3<f32>,
    ) -> Result<Do, CarSimulateError> {
        let intercept_time = self.intercept_time.unwrap();
        let jump_time = intercept_time - ctx.packet.GameInfo.TimeSeconds;
        let jump_duration = time_to_z(target_loc.z).unwrap();
        let drive_time = jump_time - jump_duration;

        if drive_time < 0.0 {
            return Ok(Do::Jump);
        }

        // Phase 1: driving forward
        let mut drive = Car::from_player_info(ctx.me());
        let mut t = 0.0;
        while t < drive_time {
            drive.step_throttle_boost(rl::PHYSICS_DT, 1.0, true)?;
            t += rl::PHYSICS_DT;
        }

        // Phase 2: a jump in which the xy-velocity stays constant
        let car_loc_xy = drive.loc().to_2d() + drive.vel().to_2d() * jump_duration;
        let car_loc = car_loc_xy.to_3d(target_loc.z);

        // Calculate how far ahead/behind the target location
        let car_offset = (car_loc - target_loc).dot(&drive.forward());

        ctx.eeg
            .print_value("i_ball", self.intercept.as_ref().unwrap().ball_loc);
        ctx.eeg.print_value("target", target_loc);
        ctx.eeg.print_time("drive_time", drive_time);
        ctx.eeg.print_time("jump_time", jump_time);
        ctx.eeg
            .print_value("car_offset", format!("{:.0}", car_offset));

        if car_offset > 0.0 {
            Ok(Do::Coast)
        } else {
            Ok(Do::Boost)
        }
    }

    fn drive(&self, ctx: &mut Context, target_loc: Point3<f32>, throttle: bool) -> Action {
        let me = ctx.me();
        let steer = simple_steer_towards(&me.Physics, target_loc.to_2d().coords);
        Action::Yield(rlbot::ffi::PlayerInput {
            Throttle: throttle as i32 as f32,
            Steer: steer,
            Boost: throttle && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED,
            ..Default::default()
        })
    }

    fn jump(&self, target_loc: Point3<f32>, target_rot: UnitQuaternion<f32>) -> Action {
        let jump_time = time_to_z(target_loc.z).unwrap();
        Action::call(Chain::new(
            self.priority(),
            vec![
                Box::new(JumpAndTurn::new(jump_time - 0.05, jump_time, target_rot)),
                Box::new(Dodge::new()),
            ],
        ))
    }
}

pub fn car_ball_contact_with_pitch(
    game: &Game,
    ball_loc: Point3<f32>,
    car_reference_loc: Point3<f32>,
    pitch: f32,
) -> (Point3<f32>, UnitQuaternion<f32>) {
    let car_rot = physics::CAR_LOCAL_FORWARD_AXIS_2D
        .rotation_to(&(ball_loc - car_reference_loc).to_2d().to_axis())
        .around_z_axis();
    let up = UnitQuaternion::from_axis_angle(&physics::car_right_axis(car_rot), -pitch);
    let car_rot = up * car_rot;

    let contact_dist = game.ball_radius() + game.me_vehicle().front_half_extent();
    let flat_ball_to_car = (car_reference_loc - ball_loc).to_2d().normalize() * contact_dist;
    let car_loc = ball_loc + up * flat_ball_to_car.to_3d(0.0);

    (car_loc, car_rot)
}

enum Do {
    Coast,
    Boost,
    Jump,
}

#[cfg(test)]
mod integration_tests {
    use common::rl;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::grounded_hit::GroundedHit;
    use nalgebra::{Point2, Vector3};

    #[test]
    fn normal_shoot() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-2000.0, 2000.0, 500.0),
            ball_vel: Vector3::new(1000.0, 0.0, 0.0),
            car_loc: Vector3::new(0.0, 0.0, 17.01),
            car_vel: Vector3::new(0.0, 0.0, 0.0),
            ..Default::default()
        });
        test.set_behavior(GroundedHit::hit_towards(|_, _| {
            Ok(Point2::new(0.0, rl::FIELD_MAX_Y))
        }));

        test.sleep_millis(3000);
        assert!(test.has_scored());
    }
}
