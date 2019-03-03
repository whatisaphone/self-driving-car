use collect::get_packet_and_inject_rigid_body_tick;
use std::{error::Error, time::Duration};

/// RL does not tick the ball physics during podium celebration, and the ball
/// physics are used to determine game time. So in order to celebrate we need to
/// fall back to the interpolated physics when the match ends.
pub struct Hacketeer<'rlbot> {
    rlbot: &'rlbot rlbot::RLBot,
    physicist: rlbot::Physicist<'rlbot>,
    packeteer: rlbot::Packeteer<'rlbot>,
    match_ended: bool,
}

type PacketPair<'fb> = (
    common::halfway_house::LiveDataPacket,
    Option<rlbot::flat::RigidBodyTick<'fb>>,
);

impl<'rlbot> Hacketeer<'rlbot> {
    pub fn new(rlbot: &'rlbot rlbot::RLBot) -> Self {
        Self {
            rlbot,
            physicist: rlbot.physicist(),
            packeteer: rlbot.packeteer(),
            match_ended: false,
        }
    }

    pub fn next<'fb>(&mut self) -> Result<PacketPair<'fb>, Box<dyn Error>> {
        loop {
            let (packet, tick) = self.fetch_next()?;
            // If there are no cars in the packet (which happens briefly during post-game),
            // the packet is of no use to us and we don't care.
            if packet.GameCars.is_empty() {
                log::warn!("skipping packet without any players");
                continue;
            }
            return Ok((packet, tick));
        }
    }

    fn fetch_next<'fb>(&mut self) -> Result<PacketPair<'fb>, Box<dyn Error>> {
        if self.match_ended {
            let (packet, none) = self.next_interpolated()?;
            // Make sure we don't silently start using the interpolated data during a game.
            // Only return inaccurate data if the match is ended. Inaccurate data is
            // suitable for celebration purposes only >:(
            if packet.GameInfo.MatchEnded {
                return Ok((packet, none));
            } else {
                log::warn!("switching to discrete physics");
                self.match_ended = false;
            }
        }

        match self.next_physics(Duration::from_millis(250)) {
            Ok(pair) => return Ok(pair),
            Err(err) => {
                // If the physicist times out, check whether the match is ended. If so,
                // transition to the match ended state, where we'll start returning interpolated
                // data at full speed.
                let (packet, none) = self.next_interpolated()?;
                if packet.GameInfo.MatchEnded {
                    log::warn!("switching to interpolated physics");
                    self.match_ended = true;
                    Ok((packet, none))
                } else {
                    Err(err)
                }
            }
        }
    }

    fn next_physics<'fb>(&mut self, timeout: Duration) -> Result<PacketPair<'fb>, Box<dyn Error>> {
        let tick = self.physicist.next_flat_with_timeout(timeout)?;
        let packet = get_packet_and_inject_rigid_body_tick(&self.rlbot, tick)?;
        return Ok((packet, Some(tick)));
    }

    fn next_interpolated<'fb>(&mut self) -> Result<PacketPair<'fb>, Box<dyn Error>> {
        let packet = self.packeteer.next_flatbuffer()?;
        let packet = common::halfway_house::deserialize_game_tick_packet(packet);
        Ok((packet, None))
    }
}
