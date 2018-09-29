use behavior::{Action, Behavior};
use eeg::{color, Drawable, EEG};
use maneuvers::GetToFlatGround;
use mechanics::{simple_steer_towards, simple_yaw_diff};
use nalgebra::Vector3;
use rlbot;
use simulate::{rl, Car1D, CarAerial60Deg};
use std::f32::consts::PI;
use utils::{my_car, one_v_one, ExtendPhysics, ExtendVector3};

pub struct AerialLocTime {
    target_loc: Vector3<f32>,
    target_time: f32,
    phase: Phase,
    min_distance: Option<f32>,
}

#[derive(Debug)]
enum Phase {
    Ground,
    Air { start_time: f32, duration: f32 },
    Shoot,
}

impl AerialLocTime {
    pub fn new(target_loc: Vector3<f32>, target_time: f32) -> AerialLocTime {
        AerialLocTime {
            target_loc,
            target_time,
            phase: Phase::Ground,
            min_distance: None,
        }
    }
}

impl Behavior for AerialLocTime {
    fn name(&self) -> &'static str {
        stringify!(AerialLocTime)
    }

    fn execute(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let (me, _enemy) = one_v_one(packet);
        let distance = (me.Physics.loc() - self.target_loc).norm();

        // Return if we reached our target and are now moving away from it.
        match self.min_distance {
            Some(md) if distance >= md + 250.0 => return Action::Return,
            _ => self.min_distance = Some(distance),
        }

        eeg.draw(Drawable::GhostCar(self.target_loc, me.Physics.rot()));
        eeg.draw(Drawable::print(format!("{:?}", self.phase), color::GREEN));

        match self.phase {
            Phase::Ground => self.ground(packet, eeg),
            Phase::Air {
                start_time,
                duration,
            } => self.air(packet, eeg, start_time, duration),
            Phase::Shoot => Action::Return,
        }
    }
}

impl AerialLocTime {
    fn ground(&mut self, packet: &rlbot::LiveDataPacket, eeg: &mut EEG) -> Action {
        let me = my_car(packet);
        let target_dist_2d = (me.Physics.loc() - self.target_loc).to_2d().norm();
        let time_remaining = self.target_time - packet.GameInfo.TimeSeconds;
        let yaw_diff = simple_yaw_diff(&me.Physics, self.target_loc.to_2d());
        let cost = CarAerial60Deg::cost(self.target_loc.z);

        eeg.draw(Drawable::print(
            format!("target_dist_2d: {:.0}", target_dist_2d),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("time_remaining: {:.2}", time_remaining),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("yaw_diff: {:.0}°", yaw_diff.to_degrees()),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("climb_time: {:.2}°", cost.time),
            color::GREEN,
        ));

        if !GetToFlatGround::on_flat_ground(packet) {
            warn!("Not on flat ground");
            return Action::Abort;
        }

        let fly = if yaw_diff.abs() >= 3.0_f32.to_radians() {
            false
        } else {
            cost.time >= time_remaining - 2.0 / 120.0
        };

        if !fly {
            Action::Yield(self.drive_ground(
                &me,
                target_dist_2d,
                time_remaining - cost.time,
                time_remaining,
            ))
        } else {
            self.phase = Phase::Air {
                start_time: packet.GameInfo.TimeSeconds,
                duration: cost.time,
            };
            self.execute(packet, eeg)
        }
    }

    fn drive_ground(
        &self,
        me: &rlbot::PlayerInfo,
        target_dist_2d: f32,
        ground_time: f32,
        total_time: f32,
    ) -> rlbot::PlayerInput {
        let mut result = rlbot::PlayerInput::default();
        result.Steer = simple_steer_towards(&me.Physics, self.target_loc.to_2d());
        if !Self::estimate_approach(
            me.Physics.vel().norm(),
            target_dist_2d,
            ground_time,
            total_time,
        ) {
            result.Throttle = 1.0;
            if me.OnGround
                && result.Steer.abs() < PI / 4.0
                && me.Physics.vel().norm() < rl::CAR_ALMOST_MAX_SPEED
                && me.Boost > 0
            {
                result.Boost = true;
            }
        }
        result
    }

    // Simulate in two phases: ground, and then air. The time to reach the target z
    // is known, thus the moment we need to jump is also known.
    fn estimate_approach(
        initial_speed: f32,
        target_dist_2d: f32,
        ground_time: f32,
        total_time: f32,
    ) -> bool {
        const DT: f32 = 1.0 / 60.0;
        let mut t = 2.0 / 120.0;

        let mut sim_g = Car1D::new(initial_speed);
        while t < ground_time {
            sim_g.step(DT, 1.0, true);
            t += DT;
        }

        let mut sim_a = CarAerial60Deg::new(sim_g.speed());
        while t < total_time {
            sim_a.step(DT);
            t += DT;
        }

        let sim_dist_2d = sim_g.distance_traveled() + sim_a.loc().to_2d().norm();
        sim_dist_2d >= target_dist_2d
    }

    // This is really sloppy and cannot correct correct once you leave the ground.
    // Needs improvement.
    fn air(
        &mut self,
        packet: &rlbot::LiveDataPacket,
        eeg: &mut EEG,
        start_time: f32,
        duration: f32,
    ) -> Action {
        let elapsed = packet.GameInfo.TimeSeconds - start_time;
        if elapsed >= duration {
            self.phase = Phase::Shoot;
            return self.execute(packet, eeg);
        }

        let me = my_car(packet);
        let target_pitch = 60.0_f32.to_radians();
        let pitch = (target_pitch - me.Physics.Rotation.Pitch) / 2.0;
        let input = rlbot::PlayerInput {
            Pitch: pitch.max(-1.0).min(1.0),
            Jump: true,
            Boost: elapsed >= 0.25,
            ..Default::default()
        };
        Action::Yield(input)
    }
}

#[cfg(test)]
mod integration_tests {
    use integration_tests::helpers::{TestRunner, TestScenario};
    use maneuvers::AerialLocTime;
    use nalgebra::Vector3;
    use utils::ExtendPhysics;

    #[test]
    #[ignore] // This basically works but is inaccurate. See the comment above `air()`.
    fn simple() {
        let expected_loc = Vector3::new(400.0, 1100.0, 600.0);
        let test = TestRunner::start2(
            TestScenario {
                ball_loc: Vector3::new(1000.0, 0.0, 0.0),
                ..Default::default()
            },
            move |p| AerialLocTime::new(expected_loc, p.GameInfo.TimeSeconds + 3.0),
        );

        test.sleep_millis(3000);
        let packet = test.sniff_packet();
        let car_loc = packet.GameCars[0].Physics.loc();
        let distance = (car_loc - expected_loc).norm();
        println!("expected_loc: {:?}", expected_loc);
        println!("car_loc: {:?}", car_loc);
        println!("distance: {:?}", distance);
        assert!(distance < 50.0);
    }
}
