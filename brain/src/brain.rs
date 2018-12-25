#[cfg(test)]
use crate::strategy::Behavior;
use crate::{
    eeg::{color, Drawable, EEG},
    plan::ball::{BallPredictor, ChipBallPrediction, FrameworkBallPrediction},
    strategy::{infer_game_mode, Context, Dropshot, Game, Runner, Soccar},
    utils::FPSCounter,
};
use common::ExtendDuration;
use nalgebra::clamp;
use rlbot;
use std::time::Instant;

pub struct Brain {
    runner: Runner,
    ball_predictor: Box<BallPredictor>,
    player_index: Option<i32>,
    fps_counter: FPSCounter,
}

impl Brain {
    fn new(runner: Runner, ball_predictor: impl BallPredictor + 'static) -> Self {
        Self {
            runner,
            ball_predictor: Box::new(ball_predictor),
            player_index: None,
            fps_counter: FPSCounter::new(),
        }
    }

    // This is just here so it's exported from the crate since I'm lazy
    pub fn infer_game_mode(field_info: &rlbot::ffi::FieldInfo) -> rlbot::ffi::GameMode {
        infer_game_mode(field_info)
    }

    pub fn soccar() -> Self {
        Self::new(Runner::new(Soccar::new()), ChipBallPrediction::new())
    }

    pub fn dropshot(rlbot: &'static rlbot::RLBot) -> Self {
        Self::new(
            Runner::new(Dropshot::new()),
            FrameworkBallPrediction::new(rlbot),
        )
    }

    pub fn hoops(rlbot: &'static rlbot::RLBot) -> Self {
        Self::new(
            Runner::new(Soccar::new()),
            FrameworkBallPrediction::new(rlbot),
        )
    }

    #[cfg(test)]
    pub fn with_behavior(behavior: impl Behavior + 'static) -> Self {
        Self::new(Runner::with_current(behavior), ChipBallPrediction::new())
    }

    #[cfg(test)]
    pub fn set_behavior(&mut self, behavior: impl Behavior + 'static, eeg: &mut EEG) {
        eeg.log(format!("! {}", behavior.name()));
        self.runner = Runner::with_current(behavior);
    }

    pub fn set_player_index(&mut self, player_index: i32) {
        self.player_index = Some(player_index);
    }

    pub fn tick(
        &mut self,
        field_info: &rlbot::ffi::FieldInfo,
        packet: &rlbot::ffi::LiveDataPacket,
        eeg: &mut EEG,
    ) -> rlbot::ffi::PlayerInput {
        self.fps_counter.tick(packet.GameInfo.TimeSeconds);

        eeg.draw(Drawable::print(
            format!("game_time: {:.2}", packet.GameInfo.TimeSeconds),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("fps: {}", format_fps(self.fps_counter.fps())),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!(
                "ball: ({:.0}, {:.0}, {:.0})",
                packet.GameBall.Physics.Location.X,
                packet.GameBall.Physics.Location.Y,
                packet.GameBall.Physics.Location.Z,
            ),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!(
                "p1 loc: ({:.0}, {:.0}, {:.0})",
                packet.GameCars[0].Physics.Location.X,
                packet.GameCars[0].Physics.Location.Y,
                packet.GameCars[0].Physics.Location.Z,
            ),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!(
                "p1 vel: ({:.0}, {:.0}, {:.0})",
                packet.GameCars[0].Physics.Velocity.X,
                packet.GameCars[0].Physics.Velocity.Y,
                packet.GameCars[0].Physics.Velocity.Z,
            ),
            color::GREEN,
        ));
        eeg.draw(Drawable::print("-----------------------", color::GREEN));

        let mut result = self.determine_controls(field_info, packet, eeg);

        result.Throttle = clamp(result.Throttle, -1.0, 1.0);
        result.Steer = clamp(result.Steer, -1.0, 1.0);
        result.Pitch = clamp(result.Pitch, -1.0, 1.0);
        result.Yaw = clamp(result.Yaw, -1.0, 1.0);
        result.Roll = clamp(result.Roll, -1.0, 1.0);

        eeg.draw(Drawable::print("-----------------------", color::GREEN));
        eeg.draw(Drawable::print(
            format!("throttle: {:.2}", result.Throttle),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("steer: {:.2}", result.Steer),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("pitch: {:.2}", result.Pitch),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("yaw: {:.2}", result.Yaw),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("roll: {:.2}", result.Roll),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("jump: {}", result.Jump),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("boost: {}", result.Boost),
            color::GREEN,
        ));
        eeg.draw(Drawable::print(
            format!("handbrake: {}", result.Handbrake),
            color::GREEN,
        ));

        result
    }

    fn determine_controls(
        &mut self,
        field_info: &rlbot::ffi::FieldInfo,
        packet: &rlbot::ffi::LiveDataPacket,
        eeg: &mut EEG,
    ) -> rlbot::ffi::PlayerInput {
        let start = Instant::now();

        let game = Game::new(field_info, packet, self.player_index.unwrap() as usize);
        let mut ctx = Context::new(&game, &*self.ball_predictor, packet, eeg);

        ctx.eeg.draw(Drawable::print(
            format!("possession: {:.2}", ctx.scenario.possession()),
            color::GREEN,
        ));

        let result = self.runner.execute(&mut ctx);

        let stop = Instant::now();
        let duration = stop - start;
        let calc_ms = duration.as_millis_polyfill();
        // RL's physics runs at 120Hz, which leaves us ~8ms to make a decision.
        if calc_ms >= 8 {
            ctx.eeg
                .log(format!("[Brain] Slow frame took {}ms.", calc_ms));
        }

        result
    }
}

fn format_fps(fps: Option<usize>) -> String {
    fps.map(|x| format!("{:.0}", x))
        .unwrap_or("...".to_string())
}
