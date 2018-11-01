pub use self::ground_intercept::GroundIntercept;

macro_rules! guard {
    ($start:expr, $predicate:expr, $return:expr $(,)*) => {
        if $predicate.evaluate($start) {
            return Err($return);
        }
    };
}

mod ground_intercept;
mod ground_straight;
mod ground_turn;
