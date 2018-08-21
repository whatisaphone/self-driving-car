extern crate libloading;
extern crate ratelimit;

use dll::RLBotCoreInterface;
pub use ffi::{
    BallInfo, GameMap, GameMode, LiveDataPacket, MatchSettings, Physics, PlayerConfiguration,
    PlayerInfo, PlayerInput,
};
use packeteer::Packeteer;
pub use shortcuts::*;
use std::os::raw::c_int;
use std::ptr::null_mut;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::sleep;
use std::time::Duration;

mod dll;
mod ffi;
mod ffi_impls;
mod inject;
mod packeteer;
mod shortcuts;

/// Tracks whether a RLBot instance exists at any given time.
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Injects the RLBot core DLL into Rocket League, and initializes the interface
/// DLL.
///
/// # Panics
///
/// Only one RLBot instance may exist at any given time. If you call this
/// function while another RLBot exists, it will panic.
pub fn init() -> Result<RLBot, ()> {
    if INITIALIZED.swap(true, Ordering::SeqCst) {
        panic!("Only one RLBot may exist at a time.");
    }

    if inject::inject_dll()? == inject::InjectorCode::InjectionSuccessful {
        // If rlbot is freshly injected, give it ample time to sink its hooks in.
        sleep(Duration::from_secs(10));
    }

    let interface = RLBotCoreInterface::load().map_err(|_| ())?;
    interface.wait_for_initialized()?;

    Ok(RLBot { interface })
}

pub struct RLBot {
    interface: RLBotCoreInterface,
}

impl Drop for RLBot {
    fn drop(&mut self) {
        INITIALIZED.swap(false, Ordering::SeqCst);
    }
}

impl RLBot {
    pub fn packeteer(&self) -> Packeteer {
        Packeteer::new(self)
    }

    pub fn update_player_input(
        &self,
        player_input: ffi::PlayerInput,
        player_index: c_int,
    ) -> Result<(), ()> {
        core_result((self.interface.update_player_input)(
            player_input,
            player_index,
        ))
    }

    pub fn update_live_data_packet(&self, packet: &mut ffi::LiveDataPacket) -> Result<(), ()> {
        core_result((self.interface.update_live_data_packet)(packet))
    }

    pub fn start_match(&self, match_settings: ffi::MatchSettings) -> Result<(), ()> {
        core_result((self.interface.start_match)(
            match_settings,
            None,
            null_mut(),
        ))
    }
}

fn core_result(result: ffi::RLBotCoreStatus) -> Result<(), ()> {
    match result {
        ffi::RLBotCoreStatus::Success => Ok(()),
        _ => Err(()),
    }
}
