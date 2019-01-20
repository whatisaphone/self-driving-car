use crate::{
    behavior::{higher_order::Chain, movement::yielder::Yielder},
    strategy::{Action, Behavior, Context},
};
use common::prelude::*;
use nalgebra::{Point2, UnitComplex};
use nameof::name_of_type;
use vec_box::vec_box;

pub struct Dodge {
    direction: Direction,
    follow_through_time: f32,
}

enum Direction {
    Angle(UnitComplex<f32>),
    Towards(Point2<f32>),
    TowardsBall,
}

impl Dodge {
    pub fn new() -> Self {
        Self {
            direction: Direction::Angle(UnitComplex::identity()),
            follow_through_time: 0.45,
        }
    }

    /// The angle of the dodge, where 0° means straight forward.
    #[allow(dead_code)]
    pub fn angle(mut self, angle: UnitComplex<f32>) -> Self {
        self.direction = Direction::Angle(angle);
        self
    }

    pub fn towards(mut self, target_loc: Point2<f32>) -> Self {
        self.direction = Direction::Towards(target_loc);
        self
    }

    pub fn towards_ball(mut self) -> Self {
        self.direction = Direction::TowardsBall;
        self
    }

    pub fn follow_through_time(mut self, follow_through_time: f32) -> Self {
        self.follow_through_time = follow_through_time;
        self
    }
}

impl Behavior for Dodge {
    fn name(&self) -> &str {
        name_of_type!(Dodge)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        if ctx.me().OnGround {
            ctx.eeg.log(self.name(), "can't dodge while on ground");
            return Action::Abort;
        }

        let (pitch, yaw) = match self.direction {
            Direction::Angle(angle) => (-angle.cos_angle(), angle.sin_angle()),
            Direction::Towards(target_loc) => towards(ctx.me(), target_loc),
            Direction::TowardsBall => towards(ctx.me(), ctx.packet.GameBall.Physics.loc_2d()),
        };

        Action::tail_call(Chain::new(self.priority(), vec_box![
            // Dodge
            Yielder::new(
                rlbot::ffi::PlayerInput {
                    Pitch: pitch,
                    Yaw: yaw,
                    Jump: true,
                    ..Default::default()
                },
                0.05,
            ),
            // Follow-through
            Yielder::new(rlbot::ffi::PlayerInput::default(), self.follow_through_time),
        ]))
    }
}

fn towards(car: &rlbot::ffi::PlayerInfo, target_loc: Point2<f32>) -> (f32, f32) {
    let car_loc = car.Physics.loc_2d();
    let car_forward_axis = car.Physics.forward_axis();

    let me_to_target = (target_loc - car_loc).to_axis();
    let dodge_dir = car_forward_axis.to_2d().rotation_to(&me_to_target);
    (-dodge_dir.cos_angle(), dodge_dir.sin_angle())
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::{higher_order::Chain, movement::Dodge},
        integration_tests::helpers::{TestRunner, TestScenario},
        strategy::Priority,
    };
    use common::prelude::*;
    use nalgebra::{Point2, Point3, Rotation3};
    use vec_box::vec_box;

    #[test]
    #[ignore(note = "this is a slow test, and unlikely to break")]
    fn all_directions() {
        let angles = [-3.0, -1.5, 0.0, 1.5, 3.0];
        let targets = [
            Point2::new(1000.0, 0.0),
            Point2::new(0.0, 1000.0),
            Point2::new(-1000.0, 0.0),
            Point2::new(0.0, -1000.0),
        ];
        for &pitch in &angles {
            for &roll in &angles {
                for &target in &targets {
                    let test = TestRunner::new()
                        .scenario(TestScenario {
                            car_loc: Point3::new(0.1, 0.0, 1000.0),
                            car_rot: Rotation3::from_unreal_angles(pitch, 0.0, roll),
                            ..Default::default()
                        })
                        .behavior(Chain::new(Priority::Idle, vec_box![
                            Dodge::new().towards(target)
                        ]))
                        .run_for_millis(250);
                    let packet = test.sniff_packet();
                    let loc = packet.GameCars[0].Physics.loc_2d();
                    println!("loc = {:?}", loc);
                    let origin = Point2::origin();
                    let error = (target - origin).angle_to(&(loc - origin));
                    assert!(error.abs() < 5.0_f32.to_radians())
                }
            }
        }
    }
}
