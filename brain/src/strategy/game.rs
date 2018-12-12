use common::{prelude::*, rl};
use nalgebra::{Point2, Point3, Vector3};
use rlbot;
use std::ops::RangeTo;

pub struct Game<'a> {
    packet: &'a rlbot::ffi::LiveDataPacket,
    mode: rlbot::ffi::GameMode,
    player_index: usize,
    pub team: Team,
    pub enemy_team: Team,
    boost_dollars: Box<[BoostPickup]>,
    me_vehicle: &'a Vehicle,
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
            me_vehicle: &OCTANE,
        }
    }

    pub fn field_max_x(&self) -> f32 {
        match self.mode {
            rlbot::ffi::GameMode::Soccer => rl::FIELD_MAX_X,
            rlbot::ffi::GameMode::Dropshot => 5026.0,
            rlbot::ffi::GameMode::Hoops => 2966.67,
            mode => panic!("unexpected game mode {:?}", mode),
        }
    }

    pub fn field_max_y(&self) -> f32 {
        match self.mode {
            rlbot::ffi::GameMode::Soccer => rl::FIELD_MAX_Y,
            rlbot::ffi::GameMode::Dropshot => 4555.0,
            rlbot::ffi::GameMode::Hoops => 3586.0,
            mode => panic!("unexpected game mode {:?}", mode),
        }
    }

    pub fn is_inside_field(&self, p: Point2<f32>) -> bool {
        p.x.abs() < self.field_max_x() && p.y.abs() < self.field_max_y()
    }

    pub fn me(&self) -> &rlbot::ffi::PlayerInfo {
        &self.packet.GameCars[self.player_index]
    }

    pub fn me_vehicle(&self) -> &Vehicle {
        &self.me_vehicle
    }

    pub fn cars(&self, team: Team) -> impl Iterator<Item = &rlbot::ffi::PlayerInfo> {
        self.packet
            .cars()
            .filter(move |p| Team::from_ffi(p.Team) == team)
    }

    pub fn own_goal(&self) -> &Goal {
        match self.mode {
            rlbot::ffi::GameMode::Soccer => Goal::soccar(self.team),
            rlbot::ffi::GameMode::Hoops => Goal::hoops(self.team),
            _ => panic!("unexpected game mode"),
        }
    }

    pub fn enemy_goal(&self) -> &Goal {
        match self.mode {
            rlbot::ffi::GameMode::Soccer => Goal::soccar(self.enemy_team),
            rlbot::ffi::GameMode::Hoops => Goal::hoops(self.enemy_team),
            _ => panic!("unexpected game mode"),
        }
    }

    pub fn own_back_wall_center(&self) -> Point2<f32> {
        let signum = match self.team {
            Team::Blue => -1.0,
            Team::Orange => 1.0,
        };
        Point2::new(0.0, self.field_max_y() * signum)
    }

    pub fn enemy_back_wall_center(&self) -> Point2<f32> {
        -self.own_back_wall_center()
    }

    pub fn boost_dollars(&self) -> &[BoostPickup] {
        &*self.boost_dollars
    }

    pub fn ball_radius(&self) -> f32 {
        rl::BALL_RADIUS
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

#[derive(Copy, Clone, Eq, PartialEq)]
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

    pub fn to_ffi(&self) -> u8 {
        match *self {
            Team::Blue => 0,
            Team::Orange => 1,
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
    fn soccar(team: Team) -> &'static Self {
        match team {
            Team::Blue => &SOCCAR_GOAL_BLUE,
            Team::Orange => &SOCCAR_GOAL_ORANGE,
        }
    }

    /// This is just a hack to get a hoops bot working in time for the tourney.
    /// Hoops doesn't really have "goals" the way they're modeled – it has
    /// baskets.
    fn hoops(team: Team) -> &'static Self {
        match team {
            Team::Blue => &HOOPS_GOAL_BLUE,
            Team::Orange => &HOOPS_GOAL_ORANGE,
        }
    }

    /// Returns true if the given y value is less than `range.end` uu in front
    /// of the goal.
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

pub struct Vehicle {
    half_size: Vector3<f32>,
    pivot_offset: Vector3<f32>,
}

impl Vehicle {
    /// Distance from the pivot point to the nose of the car.
    pub fn pivot_to_front_dist(&self) -> f32 {
        self.half_size.x + self.pivot_offset.x
    }
}

lazy_static! {
    static ref SOCCAR_GOAL_BLUE: Goal = Goal {
        center_2d: Point2::new(0.0, -rl::FIELD_MAX_Y)
    };
    static ref SOCCAR_GOAL_ORANGE: Goal = Goal {
        center_2d: Point2::new(0.0, rl::FIELD_MAX_Y)
    };
    static ref HOOPS_GOAL_BLUE: Goal = Goal {
        center_2d: Point2::new(0.0, -3586.0)
    };
    static ref HOOPS_GOAL_ORANGE: Goal = Goal {
        center_2d: Point2::new(0.0, 3586.0)
    };
    static ref OCTANE: Vehicle = Vehicle {
        // Source:
        // https://www.youtube.com/watch?v=4OBMq9faWzg
        // https://1drv.ms/x/s!Av9du64LKhjw8Xe7tHDJA2Q6FjsL
        half_size: Vector3::new(59.003689, 42.099705, 18.079536),
        pivot_offset: Vector3::new(13.87566, 0.0, 20.75499),
    };
}
