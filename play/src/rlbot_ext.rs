use std::{error::Error, time::Instant};

pub trait PacketeerExt {
    fn next_flatbuffer_without_timeout<'fb>(
        &mut self,
    ) -> Result<rlbot::flat::GameTickPacket<'fb>, Box<dyn Error>>;
}

impl PacketeerExt for rlbot::Packeteer<'_> {
    fn next_flatbuffer_without_timeout<'fb>(
        &mut self,
    ) -> Result<rlbot::flat::GameTickPacket<'fb>, Box<dyn Error>> {
        loop {
            let started = Instant::now();

            // The happy path is just returning immediately.
            let err = match self.next_flatbuffer() {
                Ok(packet) => break Ok(packet),
                Err(err) => err,
            };

            // The error path might mean the game is paused. In newer framework versions,
            // when you pause the game, packets are no longer sent. This will make
            // `rlbot-rust` think the game crashed. The framework also returns a `Box<dyn
            // Error>` instead of a specific error, so we need to make a guess of whether
            // that's the case. If so, keep trying forever. Otherwise, bubble up the error.
            let elapsed = Instant::now() - started;
            let likely_game_is_paused = elapsed.as_secs() > 3;
            if !likely_game_is_paused {
                break Err(err);
            }

            log::warn!(
                "No physics ticks received! I'm guessing the game is either paused or crashed."
            );
        }
    }
}
