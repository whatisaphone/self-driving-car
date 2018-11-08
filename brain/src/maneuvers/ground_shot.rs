use behavior::{Action, Behavior};
use common::prelude::*;
use eeg::{color, Drawable};
use maneuvers::{BounceShot, GetToFlatGround};
use mechanics::{simple_yaw_diff, GroundAccelToLoc, QuickJumpAndDodge};
use nalgebra::{Vector2, Vector3};
use predict::naive_ground_intercept;
use std::f32::consts::PI;
use strategy::Context;
use utils::{enemy_goal_center, geometry::ExtendF32};

pub struct GroundShot {
    min_distance: Option<f32>,
}

impl GroundShot {
    pub const MAX_BALL_Z: f32 = 120.0; // sloppy number

    pub fn new() -> GroundShot {
        GroundShot { min_distance: None }
    }

    pub fn shot_angle(ball_loc: Vector3<f32>, car_loc: Vector3<f32>, aim_loc: Vector2<f32>) -> f32 {
        let angle_me_ball = car_loc.to_2d().angle_to(ball_loc.to_2d());
        let angle_ball_goal = ball_loc.to_2d().angle_to(aim_loc);
        (angle_me_ball - angle_ball_goal).normalize_angle().abs()
    }

    pub fn good_angle(
        ball_loc: Vector3<f32>,
        car_loc: Vector3<f32>,
        aim_loc: Vector2<f32>,
    ) -> bool {
        Self::shot_angle(ball_loc, car_loc, aim_loc) < PI / 6.0
    }
}

impl Behavior for GroundShot {
    fn name(&self) -> &str {
        stringify!(GroundShot)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        // This behavior currently just operates in 2D
        if !GetToFlatGround::on_flat_ground(ctx.packet) {
            return Action::Abort;
        }

        let me = ctx.me();
        let intercept = naive_ground_intercept(
            ctx.scenario.ball_prediction().iter(),
            me.Physics.locp(),
            me.Physics.vel(),
            me.Boost as f32,
            |ball| {
                ball.loc.z < Self::MAX_BALL_Z
                    && Self::good_angle(
                        ball.loc.coords,
                        me.Physics.locp().coords,
                        enemy_goal_center(),
                    )
            },
        );

        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[GroundShot] no good intercept");
            return Action::Abort;
        });

        let aim_loc = BounceShot::aim_loc(me.Physics.locp().to_2d(), intercept.ball_loc.to_2d());
        let target_loc = BounceShot::rough_shooting_spot(&intercept, aim_loc);
        let target_dist = (target_loc - me.Physics.locp().to_2d()).norm();

        // If the ball has moved further away, assume we hit it and we're done.
        match self.min_distance {
            Some(md) if target_dist >= md * 2.0 => return Action::Return,
            _ => self.min_distance = Some(target_dist),
        }

        ctx.eeg.draw(Drawable::GhostBall(intercept.ball_loc.coords));
        ctx.eeg.draw(Drawable::Crosshair(aim_loc.coords));
        ctx.eeg.draw(Drawable::print(
            format!("intercept_time: {:.2}", intercept.time),
            color::GREEN,
        ));
        ctx.eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));

        if target_dist <= 250.0 {
            return shoot(ctx);
        }

        // TODO: this is not how this works…
        let mut child = GroundAccelToLoc::new(
            target_loc.coords,
            ctx.packet.GameInfo.TimeSeconds + intercept.time,
        );
        child.execute2(ctx)
    }
}

fn shoot(ctx: &mut Context) -> Action {
    let me = ctx.me();
    let angle = simple_yaw_diff(&me.Physics, ctx.packet.GameBall.Physics.loc().to_2d());
    if angle.abs() >= PI / 2.0 {
        ctx.eeg.log("Incorrect approach angle");
        return Action::Return;
    }

    return Action::call(QuickJumpAndDodge::begin(ctx.packet).yaw(angle));
}

#[cfg(test)]
mod integration_tests {
    use behavior::runner::PUSHED;
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::GroundShot;
    use nalgebra::{Rotation3, Vector3};
    use strategy::Runner2;

    #[test]
    #[ignore] // TODO
    fn crossing_the_midfield() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-1794.4557, -681.9332, 99.93823),
            ball_vel: Vector3::new(-619.51764, 1485.6294, -12.806913),
            car_loc: Vector3::new(-3472.8125, -1983.225, 16.937647),
            car_rot: Rotation3::from_unreal_angles(-0.009779127, 2.4388378, -0.0011504856),
            car_vel: Vector3::new(-1599.1952, 1223.4504, 9.51471),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(4000);
        test.examine_eeg(|eeg| {
            assert!(eeg
                .log
                .iter()
                .any(|x| *x == format!("{} GroundShot", PUSHED)));
        });
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn crossing_the_box() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-726.1142, -673.77716, 118.28892),
            ball_vel: Vector3::new(1032.4805, 1531.884, -72.43818),
            car_loc: Vector3::new(-45.566628, -1993.5394, 16.711021),
            car_rot: Rotation3::from_unreal_angles(-0.010258497, 0.60458016, 0.0013422332),
            car_vel: Vector3::new(1566.5747, 1017.1486, 13.497895),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(3000);
        test.examine_eeg(|eeg| {
            assert!(eeg
                .log
                .iter()
                .any(|x| *x == format!("{} GroundShot", PUSHED)));
        });
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn high_bouncer() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-1725.8822, 4719.4307, 93.15),
            ball_vel: Vector3::new(1031.4242, 2151.6794, 0.0),
            car_loc: Vector3::new(-2374.2222, 3805.5469, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.009970875, 1.0610354, -0.0002876214),
            car_vel: Vector3::new(521.8343, 928.79755, 8.326952),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(4000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn easy_open_net() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(999.651, 3636.9731, 93.14),
            ball_vel: Vector3::new(-271.7422, -1642.4099, 0.0),
            car_loc: Vector3::new(1981.3068, -3343.5154, 16.99),
            car_rot: Rotation3::from_unreal_angles(-0.00958738, 1.9184347, 0.0),
            car_vel: Vector3::new(-544.83453, 1537.2355, 8.53),
            boost: 0,
            ..Default::default()
        });
        test.set_behavior(GroundShot::new());
        test.sleep_millis(4000);
        assert!(test.has_scored());
    }

    #[test]
    #[ignore] // TODO
    fn tight_angle_needs_correction() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-2618.1267, 4567.453, 93.14),
            ball_vel: Vector3::new(204.82155, -438.9531, 0.0),
            car_loc: Vector3::new(-3850.746, 3749.8147, 16.319502),
            car_rot: Rotation3::from_unreal_angles(-0.15867114, -0.33191508, 0.005273059),
            car_vel: Vector3::new(1287.4675, -433.82834, -183.28568),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(2000);
        assert!(test.has_scored());
    }

    #[test]
    fn corner_shot() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(-2616.377, 4173.1816, 122.709236),
            ball_vel: Vector3::new(662.0207, -114.385414, 294.32352),
            car_loc: Vector3::new(-3791.579, 2773.0996, 14.34),
            car_rot: Rotation3::from_unreal_angles(0.013038836, 0.08504006, -0.0035473306),
            car_vel: Vector3::new(1109.654, 62.572224, 22.532219),
            boost: 0,
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(3000);

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "> GroundShot"));
        });
        assert!(test.has_scored());
    }

    #[test]
    fn rolling_to_corner_of_goal() {
        let test = TestRunner::start0(TestScenario {
            ball_loc: Vector3::new(151.63426, 4371.35, 93.15),
            ball_vel: Vector3::new(-435.87555, 272.40097, 0.0),
            car_loc: Vector3::new(-2071.033, 4050.8577, 17.01),
            car_rot: Rotation3::from_unreal_angles(-0.009491506, -0.61694795, 0.0),
            car_vel: Vector3::new(232.67545, -405.04382, 8.36),
            ..Default::default()
        });
        test.set_behavior(Runner2::new());
        test.sleep_millis(2000);

        test.examine_eeg(|eeg| {
            assert!(eeg.log.iter().any(|x| x == "> GroundShot"));
        });
        assert!(test.has_scored());
    }
}
