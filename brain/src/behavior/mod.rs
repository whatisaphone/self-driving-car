pub use behavior::behavior::{Action, Behavior};
pub use behavior::null::NullBehavior;
pub use behavior::root::RootBehavior;
pub use behavior::runner::BehaviorRunner;
pub use behavior::shoot::Shoot;

mod behavior;
mod null;
mod root;
mod runner;
mod shoot;
