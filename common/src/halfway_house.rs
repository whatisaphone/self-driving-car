#![allow(non_snake_case)] // TODO: fix this

use crate::flatbuffers::vector_iter;
use smallvec::SmallVec;

#[derive(Debug, Default, Copy, Clone)]
pub struct Vector3 {
    pub X: f32,
    pub Y: f32,
    pub Z: f32,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Rotator {
    pub Pitch: f32,
    pub Yaw: f32,
    pub Roll: f32,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Physics {
    pub Location: Vector3,
    pub Rotation: Rotator,
    pub Velocity: Vector3,
    pub AngularVelocity: Vector3,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct PlayerInfo {
    pub Physics: Physics,
    pub Demolished: bool,
    pub OnGround: bool,
    pub DoubleJumped: bool,
    pub Team: ::std::os::raw::c_uchar,
    pub Boost: ::std::os::raw::c_int,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct TeamInfo {
    pub TeamIndex: ::std::os::raw::c_int,
    pub Score: ::std::os::raw::c_int,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct BallInfo {
    pub Physics: Physics,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct GameInfo {
    pub TimeSeconds: f32,
    pub GameTimeRemaining: f32,
    pub RoundActive: bool,
    pub MatchEnded: bool,
}

#[derive(Clone)]
pub struct LiveDataPacket {
    pub GameCars: SmallVec<[PlayerInfo; 4]>,
    pub NumCars: ::std::os::raw::c_int,
    pub GameBall: BallInfo,
    pub GameInfo: GameInfo,
    pub Teams: SmallVec<[TeamInfo; 2usize]>,
    pub NumTeams: ::std::os::raw::c_int,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct PlayerInput {
    pub Throttle: f32,
    pub Steer: f32,
    pub Pitch: f32,
    pub Yaw: f32,
    pub Roll: f32,
    pub Jump: bool,
    pub Boost: bool,
    pub Handbrake: bool,
}

impl LiveDataPacket {
    pub fn cars(&self) -> impl Iterator<Item = &PlayerInfo> {
        self.GameCars.iter().take(self.NumCars as usize)
    }
}

pub fn deserialize_game_tick_packet(packet: rlbot::flat::GameTickPacket<'_>) -> LiveDataPacket {
    LiveDataPacket {
        GameCars: packet
            .players()
            .map(|ps| vector_iter(ps).map(deserialize_player_info).collect())
            .unwrap_or_default(),
        NumCars: packet.players().map(|ps| ps.len() as i32).unwrap_or(0),
        GameBall: packet.ball().map(deserialize_ball_info).unwrap_or_default(),
        GameInfo: packet
            .gameInfo()
            .map(deserialize_game_info)
            .unwrap_or_default(),
        Teams: packet
            .teams()
            .map(|ts| vector_iter(ts).map(deserialize_team_info).collect())
            .unwrap_or_default(),
        NumTeams: packet.teams().map(|ts| ts.len() as i32).unwrap_or(0),
    }
}

fn deserialize_player_info(info: rlbot::flat::PlayerInfo<'_>) -> PlayerInfo {
    PlayerInfo {
        Physics: info.physics().map(deserialize_physics).unwrap_or_default(),
        Demolished: info.isDemolished(),
        OnGround: info.hasWheelContact(),
        DoubleJumped: info.doubleJumped(),
        Team: info.team() as u8,
        Boost: info.boost(),
    }
}

fn deserialize_ball_info(info: rlbot::flat::BallInfo<'_>) -> BallInfo {
    BallInfo {
        Physics: info.physics().map(deserialize_physics).unwrap_or_default(),
    }
}

fn deserialize_game_info(info: rlbot::flat::GameInfo<'_>) -> GameInfo {
    GameInfo {
        TimeSeconds: info.secondsElapsed(),
        GameTimeRemaining: info.gameTimeRemaining(),
        RoundActive: info.isRoundActive(),
        MatchEnded: info.isMatchEnded(),
    }
}

fn deserialize_team_info(info: rlbot::flat::TeamInfo<'_>) -> TeamInfo {
    TeamInfo {
        TeamIndex: info.teamIndex(),
        Score: info.score(),
    }
}

fn deserialize_physics(physics: rlbot::flat::Physics<'_>) -> Physics {
    Physics {
        Location: physics
            .location()
            .map(deserialize_vector3)
            .unwrap_or_default(),
        Rotation: physics
            .rotation()
            .map(deserialize_rotator)
            .unwrap_or_default(),
        Velocity: physics
            .velocity()
            .map(deserialize_vector3)
            .unwrap_or_default(),
        AngularVelocity: physics
            .angularVelocity()
            .map(deserialize_vector3)
            .unwrap_or_default(),
    }
}

fn deserialize_vector3(vector3: &rlbot::flat::Vector3) -> Vector3 {
    Vector3 {
        X: vector3.x(),
        Y: vector3.y(),
        Z: vector3.z(),
    }
}

fn deserialize_rotator(rotator: &rlbot::flat::Rotator) -> Rotator {
    Rotator {
        Pitch: rotator.pitch(),
        Yaw: rotator.yaw(),
        Roll: rotator.roll(),
    }
}

pub fn translate_player_input(input: &PlayerInput) -> rlbot::ControllerState {
    rlbot::ControllerState {
        throttle: input.Throttle,
        steer: input.Steer,
        pitch: input.Pitch,
        yaw: input.Yaw,
        roll: input.Roll,
        jump: input.Jump,
        boost: input.Boost,
        handbrake: input.Handbrake,
    }
}
