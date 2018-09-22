use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::BlitzToLocation;
use mechanics::{simple_yaw_diff, QuickJumpAndDodge};
use nalgebra::Vector2;
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use std::f32::consts::PI;
use utils::{my_goal_center, one_v_one, ExtendF32, ExtendPhysics, ExtendVector2, ExtendVector3};

pub struct FiftyFifty;

impl FiftyFifty {
    pub fn new() -> FiftyFifty {
        FiftyFifty
    }
}

impl Behavior for FiftyFifty {
    fn name(&self) -> &'static str {
        stringify!(FiftyFifty)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        let target_angle = blocking_angle(
            intercept.ball_loc.to_2d(),
            me.Physics.loc().to_2d(),
            my_goal_center(),
            PI / 6.0,
        );
        let target_loc = intercept.ball_loc.to_2d() + Vector2::unit(target_angle) * 200.0;
        let target_dist = (target_loc - me.Physics.loc().to_2d()).norm();

        eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));

        if intercept.time >= 0.2 {
            // TODO: this is not how this works…
            let mut child = BlitzToLocation::new(target_loc);
            child.execute(packet, eeg)
        } else {
            let angle = simple_yaw_diff(&me.Physics, intercept.ball_loc.to_2d());
            Action::call(QuickJumpAndDodge::begin(packet).angle(angle))
        }
    }
}

/// Calculate an angle from `car_loc` to `ball_loc`, trying to get between
/// `ball_loc` and `block_loc`, but not adjusting the approach angle by more
/// than `max_angle_diff`.
fn blocking_angle(
    ball_loc: Vector2<f32>,
    car_loc: Vector2<f32>,
    block_loc: Vector2<f32>,
    max_angle_diff: f32,
) -> f32 {
    let naive_angle = ball_loc.angle_to(car_loc);
    let block_angle = ball_loc.angle_to(block_loc);
    let adjust = (block_angle - naive_angle)
        .normalize_angle()
        .max(-max_angle_diff)
        .min(max_angle_diff);
    (naive_angle + adjust).normalize_angle()
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::FiftyFifty;
    use nalgebra::Vector3;

    #[test]
    fn kickoff_off_center() {
        let test = TestRunner::start(
            FiftyFifty,
            TestScenario {
                car_loc: Vector3::new(256.0, -3839.98, 17.01),
                ..Default::default()
            },
        );

        test.sleep_millis(5500);

        assert!(test.has_scored());
    }
}
