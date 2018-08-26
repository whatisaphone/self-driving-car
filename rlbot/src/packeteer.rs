use ffi::LiveDataPacket;
use ratelimit;
use std::error::Error;
use std::mem;
use std::time::{Duration, Instant};
use RLBot;
use RLBotError;

/// An iterator-like object that yields `LiveDataPacket`s from the game as they
/// occur.
pub struct Packeteer<'a> {
    rlbot: &'a RLBot,
    ratelimiter: ratelimit::Limiter,
    prev_game_time: f32,
}

impl<'a> Packeteer<'a> {
    pub fn new(rlbot: &RLBot) -> Packeteer {
        // The goal is never to miss any packets. But if we poll too often, the
        // game crashes, so it's a fine line. With an interval of 3ms we can
        // catch 333 updates per second. That should be plenty.
        let ratelimiter = ratelimit::Builder::new()
            .interval(Duration::from_millis(3))
            .build();

        Packeteer {
            rlbot,
            ratelimiter,
            prev_game_time: 0.0,
        }
    }

    /// Block until we receive a unique `LiveDataPacket`, and then return it.
    pub fn next(&mut self) -> Result<LiveDataPacket, Box<Error>> {
        let started = Instant::now();
        let mut packet = unsafe { mem::uninitialized() };

        loop {
            self.ratelimiter.wait();

            self.rlbot
                .update_live_data_packet(&mut packet)
                .map_err(|_| RLBotError)?;

            // Wait until another "tick" has happened so we don't return duplicate data.
            let game_time = packet.GameInfo.TimeSeconds;
            if game_time != self.prev_game_time {
                self.prev_game_time = game_time;
                break;
            }

            if Instant::now() - started > Duration::from_secs(5) {
                return Err(From::from("rlbot seems to be frozen :("));
            }
        }

        Ok(packet)
    }
}
