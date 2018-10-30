use behavior::{Action, Behavior};
use common::prelude::*;
use integration_tests::helpers::{TestRunner, TestScenario};
use nalgebra::Vector3;
use rlbot;
use simulate::CarAerial60Deg;
use strategy::Context;

struct CarAerial60DegBehavior {
    start_time: f32,
}

impl CarAerial60DegBehavior {
    pub fn new(start_time: f32) -> Self {
        Self { start_time }
    }
}

impl Behavior for CarAerial60DegBehavior {
    fn name(&self) -> &str {
        stringify!(CarAerial60DegBehavior)
    }

    fn execute2(&mut self, ctx: &mut Context) -> Action {
        let elapsed = ctx.packet.GameInfo.TimeSeconds - self.start_time;
        let target_pitch = 60.0_f32.to_radians();
        let pitch = (target_pitch - ctx.me().Physics.Rotation.Pitch) / 2.0;
        let input = rlbot::ffi::PlayerInput {
            Pitch: pitch.max(-1.0).min(1.0),
            Jump: true,
            Boost: elapsed >= 0.25,
            ..Default::default()
        };
        Action::Yield(input)
    }
}

#[test]
fn car_aerial_60deg() {
    let test = TestRunner::start0(TestScenario {
        ball_loc: Vector3::new(1000.0, 0.0, 0.0),
        ..Default::default()
    });

    test.sleep_millis(1000);
    let packet = test.sniff_packet();
    test.set_behavior(CarAerial60DegBehavior::new(packet.GameInfo.TimeSeconds));

    test.sleep_millis(2000);
    let packet = test.sniff_packet();
    let car_loc = packet.GameCars[0].Physics.loc();

    let mut sim = CarAerial60Deg::new(0.0);
    const DT: f32 = 1.0 / 60.0;
    let mut t = DT;
    while t < 2.0 {
        t += DT;
        sim.step(DT);
    }

    let error = (car_loc - sim.loc()).norm();
    println!("car_loc: {:?}", car_loc);
    println!("sim_loc: {:?}", sim.loc());
    println!("error: {:.0}", error);
    assert!(error < 50.0);
}

#[test]
fn car_aerial_60deg_with_starting_velocity() {
    let starting_speed = 500.0;
    let test = TestRunner::start0(TestScenario {
        ball_loc: Vector3::new(1000.0, 0.0, 0.0),
        // The `+ 50.0` makes up for the few frames without player input before the behavior takes
        // effect.
        car_vel: Vector3::new(0.0, starting_speed + 25.0, 0.0),
        ..Default::default()
    });

    let packet = test.sniff_packet();
    test.set_behavior(CarAerial60DegBehavior::new(packet.GameInfo.TimeSeconds));

    test.sleep_millis(2000);
    let packet = test.sniff_packet();
    let car_loc = packet.GameCars[0].Physics.loc();

    let mut sim = CarAerial60Deg::new(starting_speed);
    const DT: f32 = 1.0 / 60.0;
    let mut t = DT;
    while t < 2.0 {
        t += DT;
        sim.step(DT);
    }

    let error = (car_loc - sim.loc()).norm();
    println!("car_loc: {:?}", car_loc);
    println!("sim_loc: {:?}", sim.loc());
    println!("error: {:.0}", error);
    assert!(error < 50.0);
}
