use crate::integration_tests::utils::rotator;
use collect::{RecordingPlayerTick, RecordingRigidBodyState};
use common::{halfway_house::translate_player_input, prelude::*};
use ordered_float::NotNan;

const RECORDING_DISTANCE_THRESHOLD: f32 = 25.0;
const STATE_SET_DEBOUNCE: i32 = 10;
/// It takes a couple frames to set state, so set state to where the ball will
/// be in the future – not where it should have been on the current frame.
const LATENCY_COMPENSATION: f32 = 3.0 / 120.0;

pub struct BallRecording {
    times: Vec<NotNan<f32>>,
    states: Vec<RecordingRigidBodyState>,
}

impl BallRecording {
    pub fn new(times: Vec<impl Into<NotNan<f32>>>, states: Vec<RecordingRigidBodyState>) -> Self {
        Self {
            times: times.into_iter().map(Into::into).collect(),
            states,
        }
    }

    pub fn initial_state(&self) -> &RecordingRigidBodyState {
        &self.states[0]
    }
}

pub struct BallPlayback {
    scenario: BallRecording,
    start_time: f32,
    frames_since_state_set: i32,
}

impl BallPlayback {
    pub fn new(scenario: BallRecording, start_time: f32) -> Self {
        Self {
            scenario,
            start_time,
            frames_since_state_set: 0,
        }
    }

    pub fn tick(&mut self, rlbot: &rlbot::RLBot, packet: &common::halfway_house::LiveDataPacket) {
        self.frames_since_state_set += 1;
        if self.frames_since_state_set < STATE_SET_DEBOUNCE {
            return;
        }

        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;
        let data_time = self.scenario.times[0] + elapsed + LATENCY_COMPENSATION;
        let index = match self.scenario.times.binary_search(&data_time) {
            Ok(i) => i,
            Err(0) => 0,
            Err(i) if i == self.scenario.times.len() => return,
            Err(i) => i - 1,
        };

        let state = &self.scenario.states[index];
        let current_loc = packet.GameBall.Physics.loc();
        if (state.loc - current_loc).norm() >= RECORDING_DISTANCE_THRESHOLD {
            let ball_state = rlbot::DesiredBallState::new().physics(
                rlbot::DesiredPhysics::new()
                    .location(state.loc)
                    .rotation(rotator(state.rot))
                    .velocity(state.vel)
                    .angular_velocity(state.ang_vel),
            );
            let game_state = rlbot::DesiredGameState::new().ball_state(ball_state);

            rlbot.set_game_state(&game_state).unwrap();
        }
    }
}

pub struct CarRecording {
    times: Vec<NotNan<f32>>,
    ticks: Vec<RecordingPlayerTick>,
    pub starting_boost: f32,
}

impl CarRecording {
    pub fn new(
        times: Vec<impl Into<NotNan<f32>>>,
        ticks: Vec<RecordingPlayerTick>,
        starting_boost: f32,
    ) -> Self {
        Self {
            times: times.into_iter().map(Into::into).collect(),
            ticks,
            starting_boost,
        }
    }

    pub fn single_tick(state: RecordingRigidBodyState, starting_boost: f32) -> Self {
        Self {
            times: vec![NotNan::new(0.0).unwrap()],
            ticks: vec![RecordingPlayerTick {
                input: Default::default(),
                state,
            }],
            starting_boost,
        }
    }

    pub fn initial_state(&self) -> &RecordingRigidBodyState {
        &self.ticks[0].state
    }
}

pub struct CarPlayback {
    scenario: CarRecording,
    player_index: i32,
    start_time: f32,
    frames_since_state_set: i32,
}

impl CarPlayback {
    pub fn new(scenario: CarRecording, player_index: i32, start_time: f32) -> Self {
        Self {
            scenario,
            player_index,
            start_time,
            frames_since_state_set: 0,
        }
    }

    pub fn tick(&mut self, rlbot: &rlbot::RLBot, packet: &common::halfway_house::LiveDataPacket) {
        let elapsed = packet.GameInfo.TimeSeconds - self.start_time;
        let data_time = self.scenario.times[0] + elapsed + LATENCY_COMPENSATION;
        let index = match self.scenario.times.binary_search(&data_time) {
            Ok(i) => i,
            Err(0) => 0,
            Err(i) if i == self.scenario.times.len() => return,
            Err(i) => i - 1,
        };
        let tick = &self.scenario.ticks[index];

        rlbot
            .update_player_input(self.player_index, &translate_player_input(&tick.input))
            .unwrap();

        self.frames_since_state_set += 1;
        if self.frames_since_state_set < STATE_SET_DEBOUNCE {
            return;
        }

        let current_loc = packet.GameCars[self.player_index as usize].Physics.loc();
        if (tick.state.loc - current_loc).norm() >= RECORDING_DISTANCE_THRESHOLD {
            let car_state = rlbot::DesiredCarState::new().physics(
                rlbot::DesiredPhysics::new()
                    .location(tick.state.loc)
                    .rotation(rotator(tick.state.rot))
                    .velocity(tick.state.vel)
                    .angular_velocity(tick.state.ang_vel),
            );
            let game_state =
                rlbot::DesiredGameState::new().car_state(self.player_index as usize, car_state);

            rlbot.set_game_state(&game_state).unwrap();
        }
    }
}
