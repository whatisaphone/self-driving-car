pub use self::{
    offense::Offense, reset_behind_ball::ResetBehindBall, shoot::Shoot, tepid_hit::TepidHit,
};

mod bounce_dribble;
#[allow(clippy::module_inception)]
mod offense;
mod regroup;
mod reset_behind_ball;
mod shoot;
mod side_wall_self_pass;
mod tepid_hit;
