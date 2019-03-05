use crate::{
    strategy::pitch::{Pitch, DFH_STADIUM},
    utils::geometry::Line2,
};
use common::{prelude::*, rl, vector_iter};
use lazy_static::lazy_static;
use nalgebra::{Point2, Point3, Unit, Vector2, Vector3};
use std::ops::RangeTo;

pub struct Game<'a> {
    packet: &'a common::halfway_house::LiveDataPacket,
    mode: rlbot::GameMode,
    pitch: &'a Pitch,
    player_index: usize,
    pub team: Team,
    pub enemy_team: Team,
    boost_dollars: Box<[BoostPickup]>,
    me_vehicle: &'a Vehicle,
}

impl<'a> Game<'a> {
    pub fn new(
        field_info: rlbot::flat::FieldInfo<'_>,
        packet: &'a common::halfway_house::LiveDataPacket,
        player_index: usize,
    ) -> Self {
        let team = Team::from_ffi(packet.GameCars[player_index].Team);
        Self {
            packet,
            mode: infer_game_mode(field_info),
            pitch: &*DFH_STADIUM,
            player_index,
            team,
            enemy_team: team.opposing(),
            boost_dollars: vector_iter(field_info.boostPads().unwrap())
                .filter(|info| info.isFullBoost())
                .map(|info| BoostPickup {
                    loc: point3(info.location().unwrap()).to_2d(),
                })
                .collect::<Vec<_>>()
                .into_boxed_slice(),
            me_vehicle: &OCTANE,
        }
    }

    pub fn pitch(&self) -> &Pitch {
        self.pitch
    }

    pub fn field_max_x(&self) -> f32 {
        match self.mode {
            rlbot::GameMode::Soccer => rl::FIELD_MAX_X,
            rlbot::GameMode::Dropshot => 5026.0,
            rlbot::GameMode::Hoops => 2966.67,
            mode => panic!("unexpected game mode {:?}", mode),
        }
    }

    pub fn field_max_y(&self) -> f32 {
        match self.mode {
            rlbot::GameMode::Soccer => rl::FIELD_MAX_Y,
            rlbot::GameMode::Dropshot => 4555.0,
            rlbot::GameMode::Hoops => 3586.0,
            mode => panic!("unexpected game mode {:?}", mode),
        }
    }

    pub fn is_inside_field(&self, p: Point2<f32>) -> bool {
        p.x.abs() < self.field_max_x() && p.y.abs() < self.field_max_y()
    }

    pub fn me(&self) -> &common::halfway_house::PlayerInfo {
        &self.packet.GameCars[self.player_index]
    }

    pub fn me_vehicle(&self) -> &Vehicle {
        &self.me_vehicle
    }

    pub fn cars(&self, team: Team) -> impl Iterator<Item = &common::halfway_house::PlayerInfo> {
        self.packet
            .cars()
            .filter(move |p| Team::from_ffi(p.Team) == team)
    }

    pub fn own_goal(&self) -> &Goal {
        match self.mode {
            rlbot::GameMode::Soccer => Goal::soccar(self.team),
            rlbot::GameMode::Hoops => Goal::hoops(self.team),
            _ => panic!("unexpected game mode"),
        }
    }

    pub fn enemy_goal(&self) -> &Goal {
        match self.mode {
            rlbot::GameMode::Soccer => Goal::soccar(self.enemy_team),
            rlbot::GameMode::Hoops => Goal::hoops(self.enemy_team),
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

pub fn infer_game_mode(field_info: rlbot::flat::FieldInfo<'_>) -> rlbot::GameMode {
    match field_info.boostPads().unwrap().len() {
        0 => rlbot::GameMode::Dropshot,
        20 => rlbot::GameMode::Hoops,
        34 => rlbot::GameMode::Soccer,
        _ => panic!("unknown game mode"),
    }
}

fn point3(v: &rlbot::flat::Vector3) -> Point3<f32> {
    Point3::new(v.x(), v.y(), v.z())
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

    pub fn to_ffi(self) -> u8 {
        match self {
            Team::Blue => 0,
            Team::Orange => 1,
        }
    }

    fn opposing(self) -> Self {
        match self {
            Team::Blue => Team::Orange,
            Team::Orange => Team::Blue,
        }
    }
}

pub struct Goal {
    pub center_2d: Point2<f32>,
    pub normal_2d: Unit<Vector2<f32>>,
    pub max_x: f32,
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

    pub fn goalline(&self) -> Line2 {
        Line2::from_origin_dir(self.center_2d, self.normal_2d.ortho().to_axis())
    }

    /// Returns the point on the surface of the goal which is closest to the
    /// given point.
    pub fn closest_point(&self, target: Point2<f32>) -> Point2<f32> {
        Point2::new(target.x.max(-self.max_x).min(self.max_x), self.center_2d.y)
    }

    /// Returns true if the given y value is less than `range.end` uu in front
    /// of the goal.
    ///
    /// Possible values for `range`:
    ///
    /// - `..400` – 400 uu in front of the goal. Pretty close to being scored
    ///   on.
    /// - `..0` – On or behind the goal-line. At least half the ball is
    ///   shimmering.
    /// - `..-rl::BALL_RADIUS` – The ball has entered and will never return.
    pub fn is_y_within_range(&self, y: f32, range: RangeTo<f32>) -> bool {
        if self.center_2d.y < 0.0 {
            y < self.center_2d.y + range.end
        } else {
            y > self.center_2d.y - range.end
        }
    }

    pub fn ball_is_scored(&self, ball_loc: Point3<f32>) -> bool {
        // This is just an estimate, it doesn't take into account ball radius, etc.
        ball_loc.x.abs() < self.max_x && self.is_y_within_range(ball_loc.y, ..0.0)
    }

    pub fn ball_is_scored_conservative(&self, ball_loc: Point3<f32>) -> bool {
        ball_loc.x.abs() < self.max_x - rl::BALL_RADIUS
            && self.is_y_within_range(ball_loc.y, ..-rl::BALL_RADIUS)
    }

    pub fn shot_angle_2d(&self, ball_loc: Point2<f32>) -> f32 {
        let goal_to_ball_axis = (ball_loc - self.center_2d).to_axis();
        goal_to_ball_axis.angle_to(&self.normal_2d).abs()
    }
}

#[derive(Clone)]
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
    pub static ref SOCCAR_GOAL_BLUE: Goal = Goal {
        center_2d: Point2::new(0.0, -rl::FIELD_MAX_Y),
        normal_2d: Vector2::y_axis(),
        max_x: rl::GOALPOST_X,
    };
    pub static ref SOCCAR_GOAL_ORANGE: Goal = Goal {
        center_2d: Point2::new(0.0, rl::FIELD_MAX_Y),
        normal_2d: -Vector2::y_axis(),
        max_x: rl::GOALPOST_X,
    };
    static ref HOOPS_GOAL_BLUE: Goal = Goal {
        center_2d: Point2::new(0.0, -3586.0),
        normal_2d: Vector2::y_axis(),
        max_x: rl::GOALPOST_X,
    };
    static ref HOOPS_GOAL_ORANGE: Goal = Goal {
        center_2d: Point2::new(0.0, 3586.0),
        normal_2d: -Vector2::y_axis(),
        max_x: rl::GOALPOST_X,
    };
    static ref OCTANE: Vehicle = Vehicle {
        // Source:
        // https://www.youtube.com/watch?v=4OBMq9faWzg
        // https://1drv.ms/x/s!Av9du64LKhjw8Xe7tHDJA2Q6FjsL
        half_size: Vector3::new(59.00369, 42.099705, 18.079536),
        pivot_offset: Vector3::new(13.87566, 0.0, 20.75499),
    };
}
