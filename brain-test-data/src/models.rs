use collect::RecordingRigidBodyState;

pub struct OneVOneScenario<'a> {
    pub times: &'a [f32],
    pub ball_states: &'a [RecordingRigidBodyState],
    pub car_initial_state: RecordingRigidBodyState,
    pub enemy_inputs: &'a [rlbot::ffi::PlayerInput],
    pub enemy_states: &'a [RecordingRigidBodyState],
}
