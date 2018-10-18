use behavior::{Action, Behavior};
use common::ext::ExtendPhysics;
use eeg::{color, Drawable};
use maneuvers::BounceShot;
use mechanics::{simple_steer_towards, QuickJumpAndDodge};
use nalgebra::{Point2, Point3};
use predict::{estimate_intercept_car_ball, Intercept};
use rlbot;
use simulate::{ball_car_distance, car_single_jump::time_to_z, rl, Car};
use strategy::Context;
use utils::{ExtendPoint2, ExtendPoint3, ExtendVector2, ExtendVector3};

pub struct GroundedHit<Aim>
where
    Aim: Fn(&rlbot::ffi::PlayerInfo, Point3<f32>) -> Point2<f32> + Send,
{
    aim: Aim,
    aim_loc: Option<Point2<f32>>,
    intercept: Option<Intercept>,
    intercept_time: Option<f32>,
}

impl<Aim> GroundedHit<Aim>
where
    Aim: Fn(&rlbot::ffi::PlayerInfo, Point3<f32>) -> Point2<f32> + Send,
{
    pub const MAX_BALL_Z: f32 = 240.0; // TODO: how high can I jump

    #[allow(dead_code)] // This is a good behavior, just gotta slip it in somewhere.
    pub fn hit_towards(aim: Aim) -> Self {
        Self {
            aim,
            aim_loc: None,
            intercept: None,
            intercept_time: None,
        }
    }
}

impl GroundedHit<fn(&rlbot::ffi::PlayerInfo, Point3<f32>) -> Point2<f32>> {
    /// A preset for `Aim` that hits the ball straight ahead.
    pub fn opposite_of_self(car: &rlbot::ffi::PlayerInfo, ball: Point3<f32>) -> Point2<f32> {
        ball.to_2d() + (ball.to_2d() - car.Physics.locp().to_2d())
    }
}

impl<Aim> Behavior for GroundedHit<Aim>
where
    Aim: Fn(&rlbot::ffi::PlayerInfo, Point3<f32>) -> Point2<f32> + Send,
{
    fn name(&self) -> &str {
        stringify!(GroundedHit)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let me = ctx.me();

        if self.intercept.is_none() {
            // TODO: replace intercept finding with route planning
            let intercept =
                estimate_intercept_car_ball(ctx, me, |_t, loc, _vel| loc.z < Self::MAX_BALL_Z);

            let intercept = some_or_else!(intercept, {
                ctx.eeg.log("[GroundedHit] can't find intercept");
                return Action::Abort;
            });
            self.intercept_time = Some(ctx.packet.GameInfo.TimeSeconds + intercept.time);
            self.intercept = Some(intercept);
        }

        let target_loc = self.target_loc(ctx);

        match self.estimate_approach(ctx, target_loc) {
            Do::Coast => self.drive(ctx, target_loc, false),
            Do::Boost => self.drive(ctx, target_loc, true),
            Do::Jump => self.jump(ctx, target_loc),
        }
    }
}

impl<Aim> GroundedHit<Aim>
where
    Aim: Fn(&rlbot::ffi::PlayerInfo, Point3<f32>) -> Point2<f32> + Send,
{
    fn target_loc(&mut self, ctx: &mut Context) -> Point3<f32> {
        let intercept = self.intercept.as_ref().unwrap();
        let intercept_time = self.intercept_time.unwrap();
        let intercept_ball_loc = Point3::from_coordinates(intercept.ball_loc);

        let me = ctx.me();
        let aim_loc = (self.aim)(me, intercept_ball_loc);
        self.aim_loc = Some(aim_loc);
        let target_loc_xy = BounceShot::rough_shooting_spot(intercept, aim_loc.coords);
        let target_loc_xy = Point2::from_coordinates(target_loc_xy);
        let target_loc = target_loc_xy.to_3d(intercept.ball_loc.z);

        // Calculate the precise location where we get as close to the ball as possible.
        let intercept_ball_loc = Point3::from_coordinates(intercept.ball_loc);
        let distance = ball_car_distance(intercept_ball_loc, target_loc, me.Physics.quat());
        let target_loc = target_loc + (intercept_ball_loc - target_loc).normalize() * distance;

        ctx.eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept_time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("intercept_loc_z: {:.0}", intercept.ball_loc.z),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::Crosshair(aim_loc.coords));
        ctx.eeg.draw(Drawable::GhostBall(intercept_ball_loc.coords));
        ctx.eeg
            .draw(Drawable::GhostCar(target_loc.coords, me.Physics.rot()));

        target_loc
    }

    fn estimate_approach(&mut self, ctx: &mut Context, target_loc: Point3<f32>) -> Do {
        let intercept_time = self.intercept_time.unwrap();
        let jump_time = intercept_time - ctx.packet.GameInfo.TimeSeconds;
        let jump_duration = time_to_z(target_loc.z).unwrap();
        let drive_time = jump_time - jump_duration;

        if drive_time < 0.0 {
            return Do::Jump;
        }

        // Phase 1: driving forward
        let mut drive = Car::from_player_info(ctx.me());
        let mut t = 0.0;
        while t < drive_time {
            drive.step_throttle_boost(rl::PHYSICS_DT, 1.0, true);
            t += rl::PHYSICS_DT;
        }

        // Phase 2: a jump in which the xy-velocity stays constant
        let car_loc_xy = drive.loc().to_2d() + drive.vel().to_2d() * jump_duration;
        let car_loc = car_loc_xy.to_3d(target_loc.z);

        // Calculate how far ahead/behind the target location
        let car_offset = (car_loc - target_loc).dot(&drive.forward());

        ctx.eeg.draw(Drawable::print(
            format!(
                "i_ball: [{:.0}, {:.0}, {:.0}]",
                self.intercept.as_ref().unwrap().ball_loc.x,
                self.intercept.as_ref().unwrap().ball_loc.y,
                self.intercept.as_ref().unwrap().ball_loc.z
            ),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!(
                "target: [{:.0}, {:.0}, {:.0}]",
                target_loc.coords.x, target_loc.coords.y, target_loc.coords.z
            ),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("drive_time: {:.2}", drive_time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("jump_time: {:.2}", jump_time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("car_offset: {:.0}", car_offset),
            color::GREEN,
        ));

        if car_offset > 0.0 {
            Do::Coast
        } else {
            Do::Boost
        }
    }

    fn drive(&self, ctx: &mut Context, target_loc: Point3<f32>, boost: bool) -> Action {
        let me = ctx.me();
        let steer = simple_steer_towards(&me.Physics, target_loc.to_2d().coords);
        Action::Yield(rlbot::ffi::PlayerInput {
            Throttle: boost as i32 as f32,
            Steer: steer,
            Boost: boost,
            ..Default::default()
        })
    }

    fn jump(&self, ctx: &mut Context, target_loc: Point3<f32>) -> Action {
        let aim_loc = self.aim_loc.as_ref().unwrap();
        let intercept = self.intercept.as_ref().unwrap();
        let intercept_ball_loc = Point3::from_coordinates(intercept.ball_loc);

        let me = ctx.me();
        let forward = me.Physics.forward_axis().to_2d();
        let flip_dir = forward.rotation_to(aim_loc - intercept_ball_loc.to_2d());
        let dodge_time = time_to_z(target_loc.z).unwrap();
        Action::call(
            QuickJumpAndDodge::new()
                .dodge_time(dodge_time)
                .angle(flip_dir.angle()),
        )
    }
}

enum Do {
    Coast,
    Boost,
    Jump,
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::grounded_hit::GroundedHit;
    use nalgebra::Vector3;
    use utils::enemy_goal_center_point;

    // `Repeat` is used in these tests so the shot is not aborted by
    // `SameBallTrajectory` when the ball bounces.

    #[test]
    fn normal_shoot() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-2000.0, 2000.0, 500.0),
            ball_vel: Vector3::new(1000.0, 0.0, 0.0),
            car_loc: Vector3::new(0.0, 0.0, 17.01),
            car_vel: Vector3::new(0.0, 0.0, 0.0),
            ..Default::default()
        });
        test.set_behavior(GroundedHit::hit_towards(|_, _| enemy_goal_center_point()));

        test.sleep_millis(3000);
        assert!(test.has_scored());
    }
}
