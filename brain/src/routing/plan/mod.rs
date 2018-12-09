pub use self::{
    boost::GetDollar, ground_intercept::GroundIntercept, pathing::avoid_goal_wall_waypoint,
};

macro_rules! guard {
    ($start:expr, $predicate:expr, $return:expr $(,)*) => {
        if $predicate.evaluate(&$start) {
            return Err($return);
        }
    };
}

mod boost;
mod ground_intercept;
mod ground_powerslide;
mod ground_straight;
mod ground_turn;
mod higher_order;
mod pathing;
