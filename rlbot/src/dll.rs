use ffi::*;
use libloading::Library;
use std::io;
use std::os::raw::{c_int, c_uint};

pub(crate) unsafe fn is_loaded() -> bool {
    RLBOT_INTERFACE.is_some()
}

pub(crate) unsafe fn load() -> io::Result<()> {
    RLBOT_INTERFACE = Some(Library::new("RLBot_Core_Interface.dll")?);
    let interface = RLBOT_INTERFACE.as_ref().unwrap();

    UPDATE_PLAYER_INPUT = Some(*interface.get(b"UpdatePlayerInput")?);
    UPDATE_LIVE_DATA_PACKET = Some(*interface.get(b"UpdateLiveDataPacket")?);
    START_MATCH = Some(*interface.get(b"StartMatch")?);
    IS_INITIALIZED = Some(*interface.get(b"IsInitialized")?);

    Ok(())
}

static mut RLBOT_INTERFACE: Option<Library> = None;

type UpdatePlayerInput = unsafe extern "C" fn(PlayerInput, c_int) -> RLBotCoreStatus;
static mut UPDATE_PLAYER_INPUT: Option<UpdatePlayerInput> = None;

pub unsafe fn update_player_input(
    player_input: PlayerInput,
    player_index: c_int,
) -> RLBotCoreStatus {
    UPDATE_PLAYER_INPUT.unwrap()(player_input, player_index as c_int)
}

type UpdateLiveDataPacket = unsafe extern "C" fn(*mut LiveDataPacket) -> RLBotCoreStatus;
static mut UPDATE_LIVE_DATA_PACKET: Option<UpdateLiveDataPacket> = None;

pub unsafe fn update_live_data_packet(live_data: *mut LiveDataPacket) -> RLBotCoreStatus {
    UPDATE_LIVE_DATA_PACKET.unwrap()(live_data)
}

type StartMatch =
    unsafe extern "C" fn(MatchSettings, CallbackFunction, *mut c_uint) -> RLBotCoreStatus;
static mut START_MATCH: Option<StartMatch> = None;

pub unsafe fn start_match(
    match_settings: MatchSettings,
    callback: CallbackFunction,
    id: *mut c_uint,
) -> RLBotCoreStatus {
    START_MATCH.unwrap()(match_settings, callback, id)
}

type IsInitialized = unsafe extern "C" fn() -> bool;
static mut IS_INITIALIZED: Option<IsInitialized> = None;

pub unsafe fn is_initialized() -> bool {
    IS_INITIALIZED.unwrap()()
}
