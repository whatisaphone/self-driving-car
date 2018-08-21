use ffi::LiveDataPacket;
use ratelimit;
use std::mem;
use std::time::Duration;
use RLBot;

/// An iterator-like object that yields `LiveDataPacket`s from the game as they
/// occur.
pub struct Packeteer<'a> {
    rlbot: &'a RLBot,
    ratelimiter: ratelimit::Limiter,
    prev_elapsed: f32,
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
            prev_elapsed: 0.0,
        }
    }

    /// Block until we receive a unique `LiveDataPacket`, and then return it.
    pub fn next(&mut self) -> Result<LiveDataPacket, ()> {
        let mut packet = unsafe { mem::uninitialized() };

        loop {
            self.ratelimiter.wait();

            self.rlbot.update_live_data_packet(&mut packet)?;

            // Wait until another "tick" has happened so we don't return duplicate data.
            let elapsed = packet.GameInfo.TimeSeconds;
            if elapsed != self.prev_elapsed {
                self.prev_elapsed = elapsed;
                break;
            }
        }

        Ok(packet)
    }
}
