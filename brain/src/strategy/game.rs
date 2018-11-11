use common::{prelude::*, rl};
use nalgebra::{Point2, Point3};
use rlbot;
use std::ops::RangeTo;

pub struct Game<'a> {
    packet: &'a rlbot::ffi::LiveDataPacket,
    mode: rlbot::ffi::GameMode,
    player_index: usize,
    pub team: Team,
    pub enemy_team: Team,
    boost_dollars: Box<[BoostPickup]>,
}

impl<'a> Game<'a> {
    pub fn new(
        field_info: &'a rlbot::ffi::FieldInfo,
        packet: &'a rlbot::ffi::LiveDataPacket,
        player_index: usize,
    ) -> Self {
        let team = Team::from_ffi(packet.GameCars[player_index].Team);
        Self {
            packet,
            mode: infer_game_mode(field_info),
            player_index,
            team,
            enemy_team: team.opposing(),
            boost_dollars: field_info
                .BoostPads
                .iter()
                .take(field_info.NumBoosts as usize)
                .filter(|info| info.FullBoost)
                .map(|info| BoostPickup {
                    loc: point3(info.Location).to_2d(),
                })
                .collect::<Vec<_>>()
                .into_boxed_slice(),
        }
    }

    pub fn field_max_x(&self) -> f32 {
        match self.mode {
            rlbot::ffi::GameMode::Soccer => 4096.0,
            rlbot::ffi::GameMode::Dropshot => 5026.0,
            rlbot::ffi::GameMode::Hoops => 2966.67,
            mode => panic!("unexpected game mode {:?}", mode),
        }
    }

    pub fn field_max_y(&self) -> f32 {
        match self.mode {
            rlbot::ffi::GameMode::Soccer => 5120.0,
            rlbot::ffi::GameMode::Dropshot => 4555.0,
            rlbot::ffi::GameMode::Hoops => 3586.0,
            mode => panic!("unexpected game mode {:?}", mode),
        }
    }

    pub fn me(&self) -> &rlbot::ffi::PlayerInfo {
        &self.packet.GameCars[self.player_index]
    }

    pub fn enemy(&self) -> &rlbot::ffi::PlayerInfo {
        assert_eq!(self.packet.NumCars, 2);
        &self.packet.GameCars[1 - self.player_index]
    }

    pub fn one_v_one(&self) -> (&rlbot::ffi::PlayerInfo, &rlbot::ffi::PlayerInfo) {
        (self.me(), self.enemy())
    }

    pub fn own_goal(&self) -> &Goal {
        Goal::for_team(self.team)
    }

    pub fn enemy_goal(&self) -> &Goal {
        Goal::for_team(self.enemy_team)
    }

    pub fn boost_dollars(&self) -> &[BoostPickup] {
        &*self.boost_dollars
    }
}

pub fn infer_game_mode(field_info: &rlbot::ffi::FieldInfo) -> rlbot::ffi::GameMode {
    match field_info.NumBoosts {
        0 => rlbot::ffi::GameMode::Dropshot,
        20 => rlbot::ffi::GameMode::Hoops,
        34 => rlbot::ffi::GameMode::Soccer,
        _ => panic!("unknown game mode"),
    }
}

fn point3(v: rlbot::ffi::Vector3) -> Point3<f32> {
    Point3::new(v.X, v.Y, v.Z)
}

#[derive(Copy, Clone)]
pub enum Team {
    Blue,
    Orange,
}

impl Team {
    pub fn from_ffi(index: u8) -> Self {
        match index {
            0 => Team::Blue,
            1 => Team::Orange,
            _ => panic!("wonky team index {}", index),
        }
    }

    fn opposing(&self) -> Self {
        match self {
            Team::Blue => Team::Orange,
            Team::Orange => Team::Blue,
        }
    }
}

pub struct Goal {
    pub center_2d: Point2<f32>,
}

impl Goal {
    pub fn for_team(team: Team) -> &'static Self {
        match team {
            Team::Blue => &BLUE_GOAL,
            Team::Orange => &ORANGE_GOAL,
        }
    }

    /// Returns true if the given y value is less than `range` uu in front of
    /// the goal.
    pub fn is_y_within_range(&self, y: f32, range: RangeTo<f32>) -> bool {
        // range < 0 would be behind the goal, which we have no use for supporting.
        assert!(range.end >= 0.0);

        if self.center_2d.y < 0.0 {
            y < self.center_2d.y + range.end
        } else {
            y > self.center_2d.y - range.end
        }
    }

    pub fn ball_is_scored(&self, ball_loc: Point3<f32>) -> bool {
        // This is just an estimate, it doesn't take into account ball radius, etc.
        self.is_y_within_range(ball_loc.y, ..0.0)
    }
}

pub struct BoostPickup {
    pub loc: Point2<f32>,
}

lazy_static! {
    static ref BLUE_GOAL: Goal = Goal {
        center_2d: Point2::new(0.0, -rl::FIELD_MAX_Y)
    };
    static ref ORANGE_GOAL: Goal = Goal {
        center_2d: Point2::new(0.0, rl::FIELD_MAX_Y)
    };
    static ref BOOST_DOLLARS: Vec<BoostPickup> = vec![
        Point2::new(-3072.0, -4096.0),
        Point2::new(3072.0, -4096.0),
        Point2::new(-3584.0, 0.0),
        Point2::new(3584.0, 0.0),
        Point2::new(-3072.0, 4096.0),
        Point2::new(3072.0, 4096.0),
    ]
    .into_iter()
    .map(|loc| BoostPickup { loc })
    .collect();
}
