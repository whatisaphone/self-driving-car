use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::BlitzToLocation;
use mechanics::QuickJumpAndDodge;
use predict::intercept::estimate_intercept_car_ball;
use rlbot;
use utils::geometry::ExtendVector3;
use utils::{
    enemy_goal_center, enemy_goal_left_post, enemy_goal_right_post, one_v_one, ExtendPhysics,
};

pub struct Shoot;

impl Shoot {
    pub fn new() -> Shoot {
        Shoot
    }
}

impl Behavior for Shoot {
    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let intercept = estimate_intercept_car_ball(&me, &packet.GameBall);

        let mut target_loc = intercept.ball_loc;
        let target_dist = (target_loc - me.Physics.loc()).norm();

        eeg.draw(Drawable::print("Shoot", color::YELLOW));
        eeg.draw(Drawable::print(
            format!("target_dist: {:.0}", target_dist),
            color::GREEN,
        ));
        eeg.draw(Drawable::GhostBall(intercept.ball_loc));
        eeg.draw(Drawable::GhostCar(target_loc, me.Physics.rot()));

        if target_dist <= 400.0 {
            return Action::call(QuickJumpAndDodge::begin(packet));
        }

        if target_dist > 800.0 && !good_approach_angle(&packet) {
            target_loc += (target_loc - enemy_goal_center()).normalize() * 2000.0;
            eeg.draw(Drawable::GhostCar(target_loc, me.Physics.rot()));
        }

        // TODO: this is not how this works…
        let mut child = BlitzToLocation::new(target_loc);
        child.execute(packet, eeg)
    }
}

fn good_approach_angle(packet: &rlbot::LiveDataPacket) -> bool {
    let (me, _enemy) = one_v_one(packet);
    let angle_to_ball = me.Physics.loc().angle_to(&packet.GameBall.Physics.loc());
    let angle_to_left_post = me.Physics.loc().angle_to(&enemy_goal_left_post());
    let angle_to_right_post = me.Physics.loc().angle_to(&enemy_goal_right_post());

    let lenient = 0.25 * (angle_to_right_post - angle_to_left_post);
    angle_to_left_post - lenient <= angle_to_ball && angle_to_ball <= angle_to_right_post + lenient
}

#[cfg(test)]
mod integration_tests {
    use behavior::shoot::Shoot;
    use collect::ExtendRotation3;
    use integration_tests::helpers::{TestRunner, TestScenario};
    use nalgebra::{Rotation3, Vector3};

    #[test]
    fn score_awkwardly_angled_breakaway() {
        let test = TestRunner::start(
            Shoot::new(),
            TestScenario::from_collect_row("570.0018	61.44401	-2595.4697	94.76237	0.4878059	0.18810439	-2.468271	-345.76355	848.0587	-6.6958303	-5.441	-2.2591	1.1357	1352.9867	-4963.935	18.86079	-0.02751578	0.14409833	0.07267234	590.5667	116.87245	5.3080044	0.17997983	0.10499983	2.2173882	1081.7532	-5051.4766	37.36723	0.628261	-1.6557405	-0.10153035	-12.958379	-17.438751	21.12848	-0.21157923	-0.12765418	-0.0040784255"),
//            TestScenario::from_collect_row("	-0.02751578	0.14409833	0.07267234	590.5667	116.87245	5.3080044	0.17997983	0.10499983	2.2173882	1081.7532	-5051.4766	37.36723	0.628261	-1.6557405	-0.10153035	-12.958379	-17.438751	21.12848	-0.21157923	-0.12765418	-0.0040784255"),
//            TestScenario {
//                ball_loc: Vector3::new(60.0, -2500.0, 90.0),
//                ball_vel: Vector3::new(-350.0, 900.0, 0.0),
//                car_loc: Vector3::new(1500.0, -5000.0, 0.0),
//                car_vel: Vector3::new(0.0, 0.0, 0.0),
//                ..Default::default()
//            }
        );

        test.sleep_millis(7000);

        assert!(test.has_scored());
    }
}
