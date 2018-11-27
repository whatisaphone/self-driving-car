use common::rl;

const JUMP_TIME: f32 = 2.0 / 120.0; // One frame, and another for input lag
const WAIT_TIME: f32 = 0.05;
const LANDING_TIME: f32 = 1.333333;
const DODGE_IMPULSE: f32 = 500.0;

pub struct CarForwardDodge;

impl CarForwardDodge {
    pub fn calc_1d(start_speed: f32) -> CarForwardDodge1D {
        let dodge_loc = start_speed * (JUMP_TIME + WAIT_TIME);
        let dodge_vel = (start_speed + DODGE_IMPULSE).min(rl::CAR_MAX_SPEED);
        let land_loc = dodge_loc + dodge_vel * LANDING_TIME;

        CarForwardDodge1D {
            start_speed,
            end_dist: land_loc,
            end_speed: dodge_vel,
            jump_duration: JUMP_TIME,
            wait_duration: WAIT_TIME,
            dodge_duration: LANDING_TIME,
        }
    }
}

#[derive(Clone)]
pub struct CarForwardDodge1D {
    pub start_speed: f32,
    pub end_dist: f32,
    pub end_speed: f32,
    pub jump_duration: f32,
    pub wait_duration: f32,
    pub dodge_duration: f32,
}

impl CarForwardDodge1D {
    pub fn duration(&self) -> f32 {
        self.jump_duration + self.wait_duration + self.dodge_duration
    }
}
