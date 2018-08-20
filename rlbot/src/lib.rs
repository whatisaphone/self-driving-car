extern crate libloading;
extern crate ratelimit;

pub use ffi::{
    BallInfo, GameMap, GameMode, LiveDataPacket, MatchSettings, Physics, PlayerConfiguration,
    PlayerInfo, PlayerInput,
};
use std::mem;
use std::os::raw::c_int;
use std::ptr::null_mut;
use std::thread::sleep;
use std::time::Duration;

mod dll;
mod ffi;
mod ffi_impls;
mod inject;

/// Injects the RLBot core DLL into Rocket League, and initializes the interface
/// DLL.
///
/// This method can safely be called multiple times (as long as it's not called
/// concurrently).
pub fn init() -> Result<(), ()> {
    if unsafe { dll::is_loaded() } {
        return Ok(());
    }

    if inject::inject_dll()? == inject::InjectorCode::InjectionSuccessful {
        // If rlbot is freshly injected, give it some time to sink its hooks in.
        sleep(Duration::from_secs(10));
    }

    unsafe {
        dll::load().map_err(|_| ())?;
        wait_for_initialized()?;
    }

    Ok(())
}

unsafe fn wait_for_initialized() -> Result<(), ()> {
    for _ in 0..100 {
        if dll::is_initialized() {
            return Ok(());
        }
        sleep(Duration::from_millis(10));
    }

    Err(())
}

pub fn update_player_input(player_input: ffi::PlayerInput, player_index: c_int) -> Result<(), ()> {
    match unsafe { dll::update_player_input(player_input, player_index) } {
        ffi::RLBotCoreStatus::Success => Ok(()),
        _ => Err(()),
    }
}

pub fn update_live_data_packet(packet: &mut ffi::LiveDataPacket) -> Result<(), ()> {
    match unsafe { dll::update_live_data_packet(packet) } {
        ffi::RLBotCoreStatus::Success => Ok(()),
        _ => Err(()),
    }
}

pub fn start_match(match_settings: ffi::MatchSettings) -> Result<(), ()> {
    match unsafe { dll::start_match(match_settings, None, null_mut()) } {
        ffi::RLBotCoreStatus::Success => Ok(()),
        _ => Err(()),
    }
}

/// An iterator-like object that yields `LiveDataPacket`s from the game as they
/// occur.
pub struct LiveDataPackets {
    ratelimiter: ratelimit::Limiter,
    prev_elapsed: f32,
}

impl LiveDataPackets {
    pub fn new() -> LiveDataPackets {
        // The goal is never to miss any packets. With an interval of 3ms we can catch 333 updates
        // per second. That should be plenty.
        let ratelimiter = ratelimit::Builder::new()
            .interval(Duration::from_millis(3))
            .build();

        LiveDataPackets {
            ratelimiter,
            prev_elapsed: 0.0,
        }
    }

    /// Block until we receive a unique `LiveDataPacket`, and then return it.
    pub fn wait(&mut self) -> Result<ffi::LiveDataPacket, ()> {
        let mut packet = unsafe { mem::uninitialized() };

        loop {
            self.ratelimiter.wait();

            update_live_data_packet(&mut packet)?;

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

pub fn match_settings_1v1() -> MatchSettings {
    let mut match_settings = MatchSettings {
        NumPlayers: 2,
        SkipReplays: true,
        InstantStart: true,
        ..Default::default()
    };

    match_settings.PlayerConfiguration[0].Bot = true;
    match_settings.PlayerConfiguration[0].RLBotControlled = true;
    match_settings.PlayerConfiguration[0].set_name("Blue Car");

    match_settings.PlayerConfiguration[1].Bot = true;
    match_settings.PlayerConfiguration[1].BotSkill = 1.0;
    match_settings.PlayerConfiguration[1].set_name("Orange Car");
    match_settings.PlayerConfiguration[1].Team = 1;

    match_settings
}
