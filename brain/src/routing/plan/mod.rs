pub use self::{
    boost::GetDollar, ground_drive::GroundDrive, ground_intercept::GroundIntercept,
    ground_straight::GroundStraightPlanner, ground_turn::TurnPlanner, higher_order::ChainedPlanner,
    pathing::avoid_goal_wall_waypoint, wall_intercept::WallIntercept,
};

macro_rules! guard {
    ($start:expr, $predicate:expr, $return:expr $(,)?) => {
        if $predicate.evaluate(&$start) {
            return Err($return);
        }
    };
}

mod boost;
mod ground_drive;
mod ground_intercept;
mod ground_jump_and_dodge;
mod ground_powerslide;
mod ground_straight;
mod ground_turn;
mod higher_order;
mod pathing;
mod wall_intercept;
mod wall_straight;
mod wall_turn;
mod wall_utils;
