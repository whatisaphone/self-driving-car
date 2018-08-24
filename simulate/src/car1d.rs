use rl;
use tables;

pub struct Car1D {
    loc: f32,
    vel: f32,
    boost: f32,
}

impl Car1D {
    pub fn new(speed: f32) -> Car1D {
        Car1D {
            loc: 0.0,
            vel: speed,
            boost: 100.0,
        }
    }

    pub fn with_boost(mut self, boost: i32) -> Self {
        self.boost = boost as f32;
        self
    }

    pub fn distance_traveled(&self) -> f32 {
        self.loc
    }

    pub fn speed(&self) -> f32 {
        self.vel
    }

    pub fn step(&mut self, dt: f32, throttle: f32, mut boost: bool) {
        assert_eq!(throttle, 1.0, "Throttle must be 1.0");

        if boost {
            boost = self.boost > 0.0;
        }

        let (reference_time, reference_vel_y) = if boost {
            (tables::BOOST_TIME, tables::BOOST_CAR_VEL_Y)
        } else {
            (tables::THROTTLE_TIME, tables::THROTTLE_CAR_VEL_Y)
        };

        let old_time = linear_interpolate(reference_vel_y, reference_time, self.vel);
        let new_time = old_time + dt;
        let new_vel = linear_interpolate(reference_time, reference_vel_y, new_time);

        self.loc += self.vel * dt;
        self.vel = new_vel;
        if boost {
            self.boost -= rl::BOOST_DEPLETION * dt;
        }
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

    #[test]
    fn zero_boost() {
        Car1D::new(0.0).step(DT, 1.0, true);
    }

    #[test]
    fn max_boost() {
        Car1D::new(99999.0).step(DT, 1.0, true);
    }
}
