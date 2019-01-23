use crate::{
    behavior::higher_order::NullBehavior,
    integration_tests::{
        playback::{BallRecording, CarRecording},
        running_test::RunningTest,
        scenario::TestScenario,
    },
    strategy::{Behavior, Runner, Soccar},
};
use brain_test_data::OneVOneScenario;
use collect::{RecordingPlayerTick, RecordingRigidBodyState, RecordingTick};
use std::{
    fs::File,
    panic,
    path::Path,
    sync::{Arc, Barrier},
};

pub struct TestRunner {
    behavior: Option<Box<dyn FnMut(&rlbot::ffi::LiveDataPacket) -> Box<dyn Behavior> + Send>>,
    ball_recording: Option<(Vec<f32>, Vec<RecordingRigidBodyState>)>,
    car_inital_state: Option<(RecordingRigidBodyState, f32)>,
    enemy_recording: Option<(Vec<f32>, Vec<RecordingPlayerTick>)>,
    enemy_initial_boost: f32,
}

impl TestRunner {
    const DEFAULT_STARTING_BOOST: f32 = 100.0;

    pub fn new() -> Self {
        Self {
            behavior: None,
            ball_recording: None,
            car_inital_state: None,
            enemy_recording: None,
            enemy_initial_boost: Self::DEFAULT_STARTING_BOOST,
        }
    }

    pub fn scenario(mut self, scenario: TestScenario) -> Self {
        self = self.ball(vec![0.0], vec![scenario.ball()]);
        self = self.car(scenario.car());
        self = self.starting_boost(scenario.boost as f32);
        self = self.enemy(vec![0.0], vec![Default::default()], vec![scenario.enemy()]);
        self = self.enemy_starting_boost(scenario.boost as f32);
        self
    }

    fn ball(
        mut self,
        times: impl Into<Vec<f32>>,
        states: impl Into<Vec<RecordingRigidBodyState>>,
    ) -> Self {
        self.ball_recording = Some((times.into(), states.into()));
        self
    }

    fn car(mut self, state: RecordingRigidBodyState) -> Self {
        self.car_inital_state = Some((state, Self::DEFAULT_STARTING_BOOST));
        self
    }

    pub fn starting_boost(mut self, boost: f32) -> Self {
        self.car_inital_state.as_mut().unwrap().1 = boost;
        self
    }

    pub fn enemy_starting_boost(mut self, boost: f32) -> Self {
        self.enemy_initial_boost = boost;
        self
    }

    fn enemy(
        mut self,
        times: impl Into<Vec<f32>>,
        inputs: impl Into<Vec<rlbot::ffi::PlayerInput>>,
        states: impl Into<Vec<RecordingRigidBodyState>>,
    ) -> Self {
        let ticks = inputs
            .into()
            .iter()
            .zip(states.into())
            .map(|(input, state)| RecordingPlayerTick {
                input: input.clone(),
                state: state.clone(),
            })
            .collect();
        self.enemy_recording = Some((times.into(), ticks));
        self
    }

    /// Replay the ball and enemy from a 1v1 recording. Lock the ball to the
    /// recording until the timestamp given by `ball_release`, and afterwards
    /// let it behave naturally.
    pub fn one_v_one(mut self, scenario: &OneVOneScenario<'_>, ball_release: f32) -> Self {
        let ball_release_index = scenario
            .times
            .iter()
            .position(|&t| t >= ball_release)
            .unwrap_or(scenario.times.len());
        self = self.ball(
            &scenario.times[..ball_release_index],
            &scenario.ball_states[..ball_release_index],
        );
        self = self.car(scenario.car_initial_state.clone());
        self = self.enemy(scenario.times, scenario.enemy_inputs, scenario.enemy_states);
        self
    }

    pub fn behavior(mut self, behavior: impl Behavior + Send + 'static) -> Self {
        // Use an option as a workaround for FnOnce being uncallable
        // https://github.com/rust-lang/rust/issues/28796
        let mut behavior = Some(Box::new(behavior));
        self.behavior = Some(Box::new(move |_| behavior.take().unwrap()));
        self
    }

    pub fn soccar(mut self) -> Self {
        self.behavior = Some(Box::new(|_| Box::new(Runner::new(Soccar::new()))));
        self
    }

    #[deprecated(note = "Do not commit references to ephemeral files.")]
    pub fn preview_recording(
        mut self,
        path: impl AsRef<Path>,
        start_time: f32,
        ball_duration: f32,
        total_duration: f32,
    ) -> Self {
        let ball_release = start_time + ball_duration;
        let stop_time = start_time + total_duration;

        let ticks: Vec<_> = RecordingTick::parse(File::open(path).unwrap())
            // Sometimes there are bogus rows at the start of the file from
            // before the game was restarted and the time was reset. Skip them.
            .skip_while(|r| r.time > stop_time)
            .skip_while(|r| r.time < start_time)
            .take_while(|r| r.time < stop_time)
            .collect();
        let times: Vec<_> = ticks.iter().map(|t| t.time).collect();
        let ball: Vec<_> = ticks.iter().map(|t| t.ball.clone()).collect();
        let enemy_ticks: Vec<_> = ticks.iter().map(|t| t.players[1].clone()).collect();

        let ball_release_index = times
            .iter()
            .position(|&t| t >= ball_release)
            .unwrap_or(times.len());

        self = self.ball(&times[..ball_release_index], &ball[..ball_release_index]);
        self.car_inital_state = Some((
            ticks[0].players[0].state.clone(),
            Self::DEFAULT_STARTING_BOOST,
        ));
        self.enemy_recording = Some((times, enemy_ticks));
        self
    }

    pub fn run(self) -> RunningTest {
        let ball = match self.ball_recording {
            Some((times, states)) => BallRecording::new(times, states),
            None => panic!(),
        };

        let car = match self.car_inital_state {
            Some(state) => CarRecording::single_tick(state.0, state.1),
            None => panic!(),
        };

        let enemy = match self.enemy_recording {
            Some((times, ticks)) => CarRecording::new(times, ticks, self.enemy_initial_boost),
            None => panic!(),
        };

        let mut behavior = self
            .behavior
            .unwrap_or_else(|| Box::new(|_| Box::new(NullBehavior::new())));

        let ready_wait = Arc::new(Barrier::new(2));
        let ready_wait_send = ready_wait.clone();
        let (messages_tx, messages_rx) = crossbeam_channel::unbounded();
        let thread = RunningTest::spawn_thread(
            ball,
            car,
            enemy,
            move |p| behavior(p),
            ready_wait_send,
            messages_rx,
        );
        ready_wait.wait();

        RunningTest {
            messages: messages_tx,
            join_handle: Some(thread),
        }
    }

    pub fn run_for_millis(self, millis: u64) -> RunningTest {
        let test = self.run();
        test.sleep_millis(millis);
        test
    }
}
