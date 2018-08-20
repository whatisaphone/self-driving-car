use tables::{THROTTLE_TIME, THROTTLE_VEL_Y};

pub struct Car1D {
    loc: f32,
    vel: f32,
}

impl Car1D {
    pub fn new(speed: f32) -> Car1D {
        Car1D {
            loc: 0.0,
            vel: speed,
        }
    }

    pub fn distance_traveled(&self) -> f32 {
        self.loc
    }

    pub fn speed(&self) -> f32 {
        self.vel
    }

    pub fn step(&mut self, dt: f32, throttle: f32, boost: bool) {
        assert_eq!(throttle, 1.0, "unimplemented");
        assert_eq!(boost, false, "unimplemented");

        let old_time = linear_interpolate(THROTTLE_VEL_Y, THROTTLE_TIME, self.vel);
        let new_time = old_time + dt;
        let new_vel = linear_interpolate(THROTTLE_TIME, THROTTLE_VEL_Y, new_time);

        self.loc += self.vel * dt;
        self.vel = new_vel;
    }
}

fn linear_interpolate(xs: &[f32], ys: &[f32], x: f32) -> f32 {
    let index = match xs.binary_search_by(|n| n.partial_cmp(&x).unwrap()) {
        Ok(x) => x,
        Err(0) => 0,
        Err(x) => x - 1,
    };
    // TODO: This should do a linear interpolation instead of naively returning
    // the lower endpoint.
    return ys[index];
}

#[cfg(test)]
mod tests {
    use car1d::Car1D;

    const DT: f32 = 1.0 / 60.0;

    #[test]
    fn zero_throttle() {
        Car1D::new(0.0).step(DT, 1.0, false);
    }

    #[test]
    fn max_throttle() {
        Car1D::new(99999.0).step(DT, 1.0, false);
    }
}
