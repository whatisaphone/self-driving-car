use ffi::*;
use libloading::Library;
use std::io;
use std::os::raw::{c_int, c_uint};
use std::thread::sleep;
use std::time::Duration;

type UpdatePlayerInput = extern "C" fn(PlayerInput, c_int) -> RLBotCoreStatus;
type UpdateLiveDataPacket = extern "C" fn(*mut LiveDataPacket) -> RLBotCoreStatus;
type StartMatch = extern "C" fn(MatchSettings, CallbackFunction, *mut c_uint) -> RLBotCoreStatus;
type IsInitialized = extern "C" fn() -> bool;

pub struct RLBotCoreInterface {
    #[allow(dead_code)]
    library: Library, // This "unused" field prevents Library::drop from unloading the DLL.
    pub update_player_input: UpdatePlayerInput,
    pub update_live_data_packet: UpdateLiveDataPacket,
    pub start_match: StartMatch,
    pub is_initialized: IsInitialized,
}

impl RLBotCoreInterface {
    pub fn load() -> io::Result<RLBotCoreInterface> {
        let library = Library::new("RLBot_Core_Interface.dll")?;

        Ok(unsafe {
            RLBotCoreInterface {
                update_player_input: *library.get(b"UpdatePlayerInput")?,
                update_live_data_packet: *library.get(b"UpdateLiveDataPacket")?,
                start_match: *library.get(b"StartMatch")?,
                is_initialized: *library.get(b"IsInitialized")?,
                library,
            }
        })
    }

    pub fn wait_for_initialized(&self) -> Result<(), ()> {
        for _ in 0..100 {
            if (self.is_initialized)() {
                return Ok(());
            }
            sleep(Duration::from_millis(10));
        }

        Err(())
    }
}
