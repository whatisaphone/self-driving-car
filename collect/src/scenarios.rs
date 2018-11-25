//! This module contains an archive of (some of) the code that has been used to
//! generate the data used for simulation.

#![allow(dead_code)]

use common::{prelude::*, rl};
use game_state::{
    DesiredBallState, DesiredCarState, DesiredGameState, DesiredPhysics, RotatorPartial,
    Vector3Partial,
};
use rlbot;
use std::{error::Error, f32::consts::PI, fmt};

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
    Ignore,
    Write,
    Finish,
}

pub trait SimpleScenario {
    fn name(&self) -> String;

    fn initial_state(&self) -> DesiredGameState {
        game_state_default()
    }

    fn step(&mut self, time: f32, packet: &rlbot::ffi::LiveDataPacket) -> SimpleScenarioStepResult;
}

pub enum SimpleScenarioStepResult {
    Ignore(rlbot::ffi::PlayerInput),
    Write(rlbot::ffi::PlayerInput),
    Finish,
}

impl<S: SimpleScenario> Scenario for S {
    fn name(&self) -> String {
        self.name()
    }

    fn initial_state(&self) -> DesiredGameState {
        self.initial_state()
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>> {
        match self.step(time, packet) {
            SimpleScenarioStepResult::Ignore(i) => {
                rlbot.update_player_input(i, 0)?;
                Ok(ScenarioStepResult::Ignore)
            }
            SimpleScenarioStepResult::Write(i) => {
                rlbot.update_player_input(i, 0)?;
                Ok(ScenarioStepResult::Write)
            }
            SimpleScenarioStepResult::Finish => Ok(ScenarioStepResult::Finish),
        }
    }
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

pub struct Throttle {
    boost: bool,
}

impl Throttle {
    pub fn new(boost: bool) -> Self {
        Throttle { boost }
    }
}

impl SimpleScenario for Throttle {
    fn name(&self) -> String {
        if self.boost {
            "boost".to_string()
        } else {
            "throttle".to_string()
        }
    }

    fn step(
        &mut self,
        time: f32,
        _packet: &rlbot::ffi::LiveDataPacket,
    ) -> SimpleScenarioStepResult {
        if time < 2.0 {
            SimpleScenarioStepResult::Ignore(Default::default())
        } else if time < 5.0 {
            SimpleScenarioStepResult::Write(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Boost: self.boost,
                ..Default::default()
            })
        } else {
            SimpleScenarioStepResult::Finish
        }
    }
}

pub struct Turn {
    start_speed: f32,
    start_time: Option<f32>,
}

impl Turn {
    pub fn new(start_speed: f32) -> Self {
        Self {
            start_speed,
            start_time: None,
        }
    }
}

impl Scenario for Turn {
    fn name(&self) -> String {
        format!("turn_{}", self.start_speed)
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

        let throttle = (self.start_speed / 1000.0).min(1.0);
        let boost = self.start_speed > rl::CAR_NORMAL_SPEED;

        match self.start_time {
            None => {
                let input = rlbot::ffi::PlayerInput {
                    Throttle: throttle,
                    Boost: boost,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Ignore)
            }
            Some(start_time) if time < start_time + 3.0 => {
                let input = rlbot::ffi::PlayerInput {
                    Throttle: throttle,
                    Steer: 1.0,
                    Boost: boost,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Write)
            }
            _ => Ok(ScenarioStepResult::Finish),
        }
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

    fn initial_state(&self) -> DesiredGameState {
        let mut state = game_state_default();
        state.car_states[0].physics.as_mut().unwrap().location =
            Some(Vector3Partial::new(0.0, -5000.0, 0.0));
        state
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
                Ok(ScenarioStepResult::Ignore)
            }
            Some(start_time) => {
                let input = rlbot::ffi::PlayerInput {
                    Throttle: self.handbrake_throttle,
                    Steer: 1.0,
                    Handbrake: true,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;

                if time < start_time + 3.0 {
                    Ok(ScenarioStepResult::Write)
                } else {
                    Ok(ScenarioStepResult::Finish)
                }
            }
        }
    }
}

/// I didn't bother saving a CSV of this because I don't need the detailed data.
/// Here are the high-level numbers:
///
/// * The forward dodge impulse is exactly 500 uu/s.
/// * The time from dodge to landing always ends up between 1.2 and 1.25
///   seconds. (In game I will round this up to 1.333333 to be safe.)
pub struct Dodge {
    start_speed: f32,
    phase: DodgePhase,
}

enum DodgePhase {
    Accelerate,
    Jump(f32),
    Wait(f32),
    Dodge(f32),
    Land(f32),
}

impl Dodge {
    pub fn new(start_speed: f32) -> Self {
        Self {
            start_speed,
            phase: DodgePhase::Accelerate,
        }
    }
}

impl Scenario for Dodge {
    fn name(&self) -> String {
        format!("dodge_speed_{}", self.start_speed)
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>> {
        match self.phase {
            DodgePhase::Accelerate => {
                if packet.GameCars[0].Physics.vel().norm() >= self.start_speed {
                    self.phase = DodgePhase::Jump(time);
                    return self.step(rlbot, time, packet);
                }

                let input = rlbot::ffi::PlayerInput {
                    Throttle: (self.start_speed / 1000.0).min(1.0),
                    Boost: self.start_speed > rl::CAR_MAX_SPEED,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;
                return Ok(ScenarioStepResult::Write);
            }
            DodgePhase::Jump(start) => {
                if time - start >= 0.05 {
                    self.phase = DodgePhase::Wait(time);
                    return self.step(rlbot, time, packet);
                }

                let input = rlbot::ffi::PlayerInput {
                    Jump: true,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;
                return Ok(ScenarioStepResult::Write);
            }
            DodgePhase::Wait(start) => {
                if time - start >= 0.05 {
                    self.phase = DodgePhase::Dodge(time);
                    return self.step(rlbot, time, packet);
                }

                let input = Default::default();
                rlbot.update_player_input(input, 0)?;
                return Ok(ScenarioStepResult::Write);
            }
            DodgePhase::Dodge(start) => {
                if time - start >= 0.05 {
                    self.phase = DodgePhase::Land(time);
                    return self.step(rlbot, time, packet);
                }

                let input = rlbot::ffi::PlayerInput {
                    Pitch: -1.0,
                    Jump: true,
                    ..Default::default()
                };
                rlbot.update_player_input(input, 0)?;
                return Ok(ScenarioStepResult::Write);
            }
            DodgePhase::Land(start) => {
                if time - start >= 2.0 {
                    return Ok(ScenarioStepResult::Finish);
                }

                let input = Default::default();
                rlbot.update_player_input(input, 0)?;
                return Ok(ScenarioStepResult::Write);
            }
        }
    }
}

#[derive(Copy, Clone)]
pub enum AirAxis {
    Pitch,
    Yaw,
    Roll,
}

impl AirAxis {
    pub fn all() -> impl Iterator<Item = Self> {
        vec![AirAxis::Pitch, AirAxis::Yaw, AirAxis::Roll].into_iter()
    }

    fn get_input_axis_mut<'a>(&self, input: &'a mut rlbot::ffi::PlayerInput) -> &'a mut f32 {
        match *self {
            AirAxis::Pitch => &mut input.Pitch,
            AirAxis::Yaw => &mut input.Yaw,
            AirAxis::Roll => &mut input.Roll,
        }
    }
}

impl fmt::Display for AirAxis {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(match self {
            AirAxis::Pitch => "pitch",
            AirAxis::Yaw => "yaw",
            AirAxis::Roll => "roll",
        })
    }
}

fn game_state_default_air() -> DesiredGameState {
    let mut state = game_state_default();
    state.car_states[0].physics.as_mut().unwrap().location =
        Some(Vector3Partial::new(0.0, 0.0, 1000.0));
    state.car_states[0].physics.as_mut().unwrap().rotation =
        Some(RotatorPartial::new(0.0, 0.0, 0.0));
    state
}

pub struct AirRotateAccel {
    axis: AirAxis,
    start_time: Option<f32>,
}

impl AirRotateAccel {
    pub fn new(axis: AirAxis) -> Self {
        Self {
            axis,
            start_time: None,
        }
    }
}

impl Scenario for AirRotateAccel {
    fn name(&self) -> String {
        format!("air_rotate_{}_accel", self.axis)
    }

    fn initial_state(&self) -> DesiredGameState {
        game_state_default_air()
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        _packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>> {
        if self.start_time.is_none() {
            self.start_time = Some(time);
        }

        match self.start_time {
            Some(start_time) if time < start_time + 1.0 => {
                let mut input = rlbot::ffi::PlayerInput::default();
                *self.axis.get_input_axis_mut(&mut input) = 1.0;
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Write)
            }
            _ => Ok(ScenarioStepResult::Finish),
        }
    }
}

pub struct AirRotateCoast {
    axis: AirAxis,
    start_time: Option<f32>,
}

impl AirRotateCoast {
    pub fn new(axis: AirAxis) -> Self {
        Self {
            axis,
            start_time: None,
        }
    }
}

impl Scenario for AirRotateCoast {
    fn name(&self) -> String {
        format!("air_rotate_{}_coast", self.axis)
    }

    fn initial_state(&self) -> DesiredGameState {
        game_state_default_air()
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        _packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>> {
        if self.start_time.is_none() {
            self.start_time = Some(time);
        }

        match self.start_time {
            Some(start_time) if time < start_time + 1.0 => {
                let mut input = rlbot::ffi::PlayerInput::default();
                *self.axis.get_input_axis_mut(&mut input) = 1.0;
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Ignore)
            }
            Some(start_time) if time < start_time + 3.0 => {
                let input = rlbot::ffi::PlayerInput::default();
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Write)
            }
            _ => Ok(ScenarioStepResult::Finish),
        }
    }
}

pub struct AirRotateCounter {
    axis: AirAxis,
    start_time: Option<f32>,
}

impl AirRotateCounter {
    pub fn new(axis: AirAxis) -> Self {
        Self {
            axis,
            start_time: None,
        }
    }
}

impl Scenario for AirRotateCounter {
    fn name(&self) -> String {
        format!("air_rotate_{}_counter", self.axis)
    }

    fn initial_state(&self) -> DesiredGameState {
        game_state_default_air()
    }

    fn step(
        &mut self,
        rlbot: &rlbot::RLBot,
        time: f32,
        _packet: &rlbot::ffi::LiveDataPacket,
    ) -> Result<ScenarioStepResult, Box<Error>> {
        if self.start_time.is_none() {
            self.start_time = Some(time);
        }

        match self.start_time {
            Some(start_time) if time < start_time + 1.0 => {
                let mut input = rlbot::ffi::PlayerInput::default();
                *self.axis.get_input_axis_mut(&mut input) = 1.0;
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Ignore)
            }
            Some(start_time) if time < start_time + 2.0 => {
                let mut input = rlbot::ffi::PlayerInput::default();
                *self.axis.get_input_axis_mut(&mut input) = -1.0;
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Write)
            }
            _ => Ok(ScenarioStepResult::Finish),
        }
    }
}
