use behavior::{Action, Behavior};
use common::rl;
use eeg::Drawable;
use maneuvers::AerialLocTime;
use predict::estimate_intercept_car_ball;
use simulate::CarAerial60Deg;
use strategy::Context;

pub struct AerialShot {
    finished: bool,
}

impl AerialShot {
    pub fn new() -> AerialShot {
        AerialShot { finished: false }
    }
}

impl Behavior for AerialShot {
    fn name(&self) -> &str {
        stringify!(AerialShot)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        if self.finished {
            return Action::Return;
        }

        let me = ctx.me();

        let intercept = estimate_intercept_car_ball(ctx, &me, |t, loc, _vel| {
            let max_comfortable_z = rl::CROSSBAR_Z + ctx.game.enemy_goal().center_2d.y - loc.y;
            if loc.z >= max_comfortable_z {
                return false;
            }
            let cost = CarAerial60Deg::cost(loc.z);
            cost.time < t
        });

        let intercept = some_or_else!(intercept, {
            ctx.eeg.log("[AerialShot] no intercept");
            return Action::Abort;
        });

        let cost = CarAerial60Deg::cost(intercept.ball_loc.z);

        ctx.eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        ctx.eeg
            .log(format!("intercept_ball_z: {:.0}", intercept.ball_loc.z));
        ctx.eeg
            .log(format!("intercept_time: {:.2}", intercept.time));
        ctx.eeg.log(format!("aerial_time: {:.2}", cost.time));

        self.finished = true;
        Action::call(AerialLocTime::new(
            intercept.ball_loc,
            ctx.packet.GameInfo.TimeSeconds + intercept.time,
        ))
    }
}

#[cfg(test)]
mod integration_tests {
    use behavior::aerial_shot::AerialShot;
    use common::prelude::*;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    #[ignore] // TODO
    fn easy_in_front_of_goal() {
        let test = TestRunner::start(
            AerialShot::new(),
            TestScenario {
                ball_loc: Vector3::new(2947.987, 2573.8042, 954.9597),
                ball_vel: Vector3::new(-1540.0411, 924.74066, -1316.2262),
                car_loc: Vector3::new(-1604.453, 2690.225, 17.01),
                car_rot: Rotation3::from_unreal_angles(-0.009683254, 1.2704238, -0.0000958738),
                car_vel: Vector3::new(357.6586, 1213.9453, 8.309999),
                ..Default::default()
            },
        );

        test.sleep_millis(4000);

        assert!(test.has_scored());
    }
}
