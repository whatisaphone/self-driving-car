#[cfg(test)]
use behavior::Behavior;
use eeg::{color, Drawable, EEG};
use nalgebra::clamp;
use rlbot;
use strategy::{Context, Dropshot, Runner2, Soccar};
use utils::FPSCounter;

pub struct Brain {
    runner: Runner2,
    fps_counter: FPSCounter,
}

impl Brain {
    fn new(runner: Runner2) -> Self {
        Self {
            runner,
            fps_counter: FPSCounter::new(),
        }
    }

    pub fn soccar() -> Self {
        Self::new(Runner2::new(Soccar::new()))
    }

    pub fn dropshot() -> Self {
        Self::new(Runner2::new(Dropshot::new()))
    }

    #[cfg(test)]
    pub fn with_behavior(behavior: impl Behavior + 'static) -> Self {
        Self {
            runner: Runner2::with_current(behavior),
            fps_counter: FPSCounter::new(),
        }
    }

    #[cfg(test)]
    pub fn set_behavior(&mut self, behavior: impl Behavior + 'static, eeg: &mut EEG) {
        eeg.log(format!("! {}", behavior.name()));
        self.runner = Runner2::with_current(behavior);
    }

    pub fn tick(
        &mut self,
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

        let mut result = {
            let mut ctx = Context::new(packet, eeg);

            ctx.eeg.draw(Drawable::print(
                format!("possession: {:.2}", ctx.scenario.possession()),
                color::GREEN,
            ));
            ctx.eeg.draw(Drawable::print(
                format!(
                    "concede seconds: {:.2}",
                    ctx.scenario.enemy_shoot_score_seconds(),
                ),
                color::GREEN,
            ));

            self.runner.execute(&mut ctx)
        };

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
}

fn format_fps(fps: Option<usize>) -> String {
    fps.map(|x| format!("{:.0}", x))
        .unwrap_or("...".to_string())
}
