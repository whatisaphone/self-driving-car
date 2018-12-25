//! This module contains an archive of (some of) the code that has been used to
//! generate the data used for simulation.

#![allow(dead_code)]

use common::{prelude::*, rl};
use nalgebra::{Point3, Vector3};
use rlbot;
use std::{error::Error, f32::consts::PI, fmt};

pub trait Scenario {
    fn name(&self) -> String;

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
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

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
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

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
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

fn game_state_default() -> rlbot::state::DesiredGameState {
    rlbot::state::DesiredGameState::new()
        .ball_state(
            rlbot::state::DesiredBallState::new().physics(
                rlbot::state::DesiredPhysics::new()
                    .location(Point3::new(2000.0, 0.0, 0.0))
                    .rotation(
                        rlbot::state::RotatorPartial::new()
                            .pitch(0.0)
                            .yaw(0.0)
                            .roll(0.0),
                    )
                    .velocity(Vector3::new(0.0, 0.0, 0.0))
                    .angular_velocity(Vector3::new(0.0, 0.0, 0.0)),
            ),
        )
        .car_state(
            0,
            rlbot::state::DesiredCarState::new()
                .physics(
                    rlbot::state::DesiredPhysics::new()
                        .location(Point3::new(2000.0, 0.0, 17.01))
                        .rotation(
                            rlbot::state::RotatorPartial::new()
                                .pitch(0.0)
                                .yaw(PI / 2.0)
                                .roll(0.0),
                        )
                        .velocity(Vector3::new(0.0, 0.0, 0.0))
                        .angular_velocity(Vector3::new(0.0, 0.0, 0.0)),
                )
                .boost_amount(100.0),
        )
}

pub struct Throttle {
    boost: bool,
}

impl Throttle {
    pub fn new(boost: bool) -> Self {
        Self { boost }
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

pub struct Coast;

impl Coast {
    pub fn new() -> Self {
        Coast
    }
}

impl SimpleScenario for Coast {
    fn name(&self) -> String {
        "coast".to_string()
    }

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
        let mut state = game_state_default();
        state.car_states[0]
            .as_mut()
            .unwrap()
            .physics
            .as_mut()
            .unwrap()
            .location = Some(
            rlbot::state::Vector3Partial::new()
                .x(0.0)
                .y(-5000.0)
                .z(17.01),
        );
        state
    }

    fn step(
        &mut self,
        time: f32,
        _packet: &rlbot::ffi::LiveDataPacket,
    ) -> SimpleScenarioStepResult {
        if time < 1.5 {
            SimpleScenarioStepResult::Ignore(rlbot::ffi::PlayerInput {
                Throttle: 1.0,
                Boost: true,
                ..Default::default()
            })
        } else if time < 7.0 {
            SimpleScenarioStepResult::Write(Default::default())
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

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
        let mut state = game_state_default();
        state.car_states[0]
            .as_mut()
            .unwrap()
            .physics
            .as_mut()
            .unwrap()
            .location = Some(
            rlbot::state::Vector3Partial::new()
                .x(0.0)
                .y(-5000.0)
                .z(17.01),
        );
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
                Ok(ScenarioStepResult::Write)
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
                Ok(ScenarioStepResult::Write)
            }
            DodgePhase::Wait(start) => {
                if time - start >= 0.05 {
                    self.phase = DodgePhase::Dodge(time);
                    return self.step(rlbot, time, packet);
                }

                let input = Default::default();
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Write)
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
                Ok(ScenarioStepResult::Write)
            }
            DodgePhase::Land(start) => {
                if time - start >= 2.0 {
                    return Ok(ScenarioStepResult::Finish);
                }

                let input = Default::default();
                rlbot.update_player_input(input, 0)?;
                Ok(ScenarioStepResult::Write)
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

fn game_state_default_air() -> rlbot::state::DesiredGameState {
    let mut state = game_state_default();
    state.car_states[0]
        .as_mut()
        .unwrap()
        .physics
        .as_mut()
        .unwrap()
        .location = Some(rlbot::state::Vector3Partial::new().x(0.0).y(0.0).z(1000.0));
    state.car_states[0]
        .as_mut()
        .unwrap()
        .physics
        .as_mut()
        .unwrap()
        .rotation = Some(
        rlbot::state::RotatorPartial::new()
            .pitch(0.0)
            .yaw(0.0)
            .roll(0.0),
    );
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

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
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

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
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

    fn initial_state(&self) -> rlbot::state::DesiredGameState {
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
