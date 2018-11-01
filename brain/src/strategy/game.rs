use common::rl;
use nalgebra::{Point2, Point3};
use rlbot;

pub struct Game<'a> {
    pub packet: &'a rlbot::ffi::LiveDataPacket,
    pub team: Team,
    pub enemy_team: Team,
}

impl<'a> Game<'a> {
    pub(in strategy) fn new(packet: &'a rlbot::ffi::LiveDataPacket) -> Self {
        Self {
            packet,
            team: Team::Blue,
            enemy_team: Team::Orange,
        }
    }

    pub fn enemy(&self) -> &rlbot::ffi::PlayerInfo {
        self.packet
            .cars()
            .find(|p| p.Team == self.enemy_team.to_ffi())
            .expect("No car found on enemy team")
    }

    pub fn own_goal(&self) -> &Goal {
        Goal::for_team(self.team)
    }
}

#[derive(Copy, Clone)]
pub enum Team {
    Blue,
    Orange,
}

impl Team {
    fn to_ffi(&self) -> u8 {
        match self {
            Team::Blue => 0,
            Team::Orange => 1,
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

    pub fn ball_is_scored(&self, ball_loc: Point3<f32>) -> bool {
        // This is just an estimate, it doesn't take into account ball radius, etc.
        if self.center_2d.y < 0.0 {
            ball_loc.y < self.center_2d.y
        } else {
            ball_loc.y > self.center_2d.y
        }
    }
}

lazy_static! {
    static ref BLUE_GOAL: Goal = Goal {
        center_2d: Point2::new(0.0, -rl::FIELD_MAX_Y)
    };
    static ref ORANGE_GOAL: Goal = Goal {
        center_2d: Point2::new(0.0, rl::FIELD_MAX_Y)
    };
}
