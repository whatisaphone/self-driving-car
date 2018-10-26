use common::{ext::ExtendPhysics, rl};
use game_state::{
    DesiredBallState, DesiredCarState, DesiredGameState, DesiredPhysics, RotatorPartial,
    Vector3Partial,
};
use rlbot;
use std::{error::Error, f32::consts::PI};

pub trait Scenario {
    fn name(&self) -> String;

    fn initial_state(&self) -> DesiredGameState {
        game_state_default()
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>>;
}

pub enum ScenarioStepResult {
    Continue,
    Break,
}

fn game_state_default() -> DesiredGameState {
    DesiredGameState {
        ball_state: Some(DesiredBallState {
            physics: Some(DesiredPhysics {
                location: Some(Vector3Partial::new(2000.0, 0.0, 0.0)),
                rotation: Some(RotatorPartial::new(0.0, 0.0, 0.0)),
                velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
                angular_velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
            }),
        }),
        car_states: vec![DesiredCarState {
            physics: Some(DesiredPhysics {
                location: Some(Vector3Partial::new(0.0, 0.0, 17.01)),
                rotation: Some(RotatorPartial::new(0.0, PI / 2.0, 0.0)),
                velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
                angular_velocity: Some(Vector3Partial::new(0.0, 0.0, 0.0)),
            }),
            boost_amount: Some(100.0),
            jumped: Some(false),
            double_jumped: Some(false),
        }],
    }
}

pub struct PowerslideTurn {
    start_speed: f32,
    handbrake_throttle: f32,
    start_time: Option<f32>,
}

impl PowerslideTurn {
    pub fn new(start_speed: f32, handbrake_throttle: f32) -> Self {
        Self {
            start_speed,
            handbrake_throttle,
            start_time: None,
        }
    }
}

impl Scenario for PowerslideTurn {
    fn name(&self) -> String {
        format!(
            "powerslide_turn_speed_{}_throttle_{}",
            self.start_speed, self.handbrake_throttle,
        )
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>> {
        if self.start_time.is_none() {
            let speed = packet.GameCars[0].Physics.vel().norm();
            if speed >= self.start_speed {
                self.start_time = Some(time);
            }
        }

        match self.start_time {
            None => {
                let input = rlbot::ffi::PlayerInput {
                    Throttle: (self.start_speed / 1000.0).min(1.0),
                    Boost: self.start_speed >= rl::CAR_NORMAL_SPEED,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;
            }
            Some(start_time) => {
                let input = rlbot::ffi::PlayerInput {
                    Throttle: self.handbrake_throttle,
                    Steer: 1.0,
                    Handbrake: true,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;

                if time >= start_time + 3.0 {
                    return Ok(ScenarioStepResult::Break);
                }
            }
        }

        Ok(ScenarioStepResult::Continue)
    }
}
