use crate::routing::models::CarState;
use common::{kinematics::kinematic, rl};
use nalgebra::{UnitQuaternion, Vector3};

pub struct SimJump;

impl SimJump {
    pub fn simulate(
        &self,
        start: &CarState,
        time: f32,
        target_rot: &UnitQuaternion<f32>,
    ) -> CarState {
        // Phase 1: Include the initial jump impulse, and the extra force from holding
        // the jump button down for the maximum 0.2 seconds.
        let force_time = time.min(rl::CAR_JUMP_FORCE_TIME);
        let v_0 = start.vel + start.roof_axis().into_inner() * rl::CAR_JUMP_IMPULSE_SPEED;
        let a = start.roof_axis().into_inner() + Vector3::z() * rl::GRAVITY;
        let (d, vel) = kinematic(v_0, a, force_time);
        let loc = start.loc + d;

        // Phase 2: simple freefall.
        let coast_time = (force_time - rl::CAR_JUMP_FORCE_TIME).max(0.0);
        let a = Vector3::z() * rl::GRAVITY;
        let (d, vel) = kinematic(vel, a, coast_time);
        let loc = loc + d;

        CarState {
            loc,
            rot: *target_rot,
            vel,
            boost: start.boost,
        }
    }
}
