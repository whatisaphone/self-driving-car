use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use mechanics::{simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use nalgebra::Vector2;
use predict::estimate_intercept_car_ball_2;
use rlbot;
use std::f32::consts::PI;
use utils::{enemy_goal_center, my_car, one_v_one, ExtendPhysics, ExtendVector3};

pub struct BounceShot {
    aim_loc: Vector2<f32>,
    finished: bool,
}

impl BounceShot {
    pub fn new() -> Self {
        Self {
            aim_loc: enemy_goal_center(),
            finished: false,
        }
    }

    pub fn with_target_loc(self, aim_loc: Vector2<f32>) -> Self {
        Self { aim_loc, ..self }
    }
}

impl Behavior for BounceShot {
    fn name(&self) -> &'static str {
        stringify!(BounceShot)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        if self.finished {
            return Action::Return;
        }

        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball_2(&me, &packet.GameBall, |_t, loc, vel| {
            // What we actually want is vel.z >= 0, e.g. the upward half of a bounce. But
            // velocity will be approx. -6.8 when the ball is stationary, due to gravity
            // being applied after collision handling.
            loc.z < 110.0 && vel.z >= -10.0
        });
        let desired_vel = (self.aim_loc - intercept.ball_loc.to_2d()).normalize() * 2000.0;
        let intercept_vel = intercept.ball_vel.to_2d();
        let impulse = desired_vel - intercept_vel;
        let intercept_car_loc = intercept.ball_loc.to_2d() - impulse.normalize() * 220.0;
        let distance = (me.Physics.loc().to_2d() - intercept_car_loc).norm();

        eeg.draw(Drawable::Crosshair(self.aim_loc));
        eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("distance: {:.0}", distance),
            color::GREEN,
        ));

        // TODO the threshold
        if distance < 100.0 {
            self.finished = true;
            return self.flip(packet, eeg);
        }

        // TODO: this is not how this works…
        let mut child = GroundAccelToLoc::new(
            intercept_car_loc,
            packet.GameInfo.TimeSeconds + intercept.time,
        );
        child.execute(packet, eeg)
    }
}

impl BounceShot {
    fn flip(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);
        let angle = simple_yaw_diff(&me.Physics, packet.GameBall.Physics.loc().to_2d());
        if angle.abs() >= PI / 2.0 {
            eeg.log("Incorrect approach angle");
            return Action::Return;
        }

        Action::call(QuickJumpAndDodge::begin(packet).yaw(angle))
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::RootBehavior;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::bounce_shot::BounceShot;
    use nalgebra::Vector3;

    #[test]
    fn normal() {
        let test = TestRunner::start(
            BounceShot::new(),
            TestScenario {
                ball_loc: Vector3::new(-2000.0, 2000.0, 500.0),
                ball_vel: Vector3::new(1000.0, 0.0, 0.0),
                car_loc: Vector3::new(0.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, -200.0, 0.0),
                ..Default::default()
            },
        );

        test.sleep_millis(5000);

        assert!(test.has_scored());
    }

    #[test]
    fn slow_no_boost() {
        let test = TestRunner::start(
            BounceShot::new(),
            TestScenario {
                ball_loc: Vector3::new(-2000.0, 2000.0, 1000.0),
                ball_vel: Vector3::new(500.0, 0.0, 0.0),
                car_loc: Vector3::new(0.0, 0.0, 0.0),
                car_vel: Vector3::new(0.0, -200.0, 0.0),
                boost: 0,
                ..Default::default()
            },
        );

        test.sleep_millis(6000);

        assert!(test.has_scored());
    }
}
