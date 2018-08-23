use std::f32::consts::PI;

/// Normalize an angle to between -pi and pi.
pub fn normalize_angle(theta: f32) -> f32 {
    let result = theta % (PI * 2.0);
    if result >= PI {
        result - (PI * 2.0)
    } else {
        result
    }
}
