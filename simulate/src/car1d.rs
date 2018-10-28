use math::linear_interpolate;
use oven::data;
use rl;

pub struct Car1D {
    time: f32,
    loc: f32,
    vel: f32,
    boost: f32,
}

impl Car1D {
    pub fn new(speed: f32) -> Car1D {
        Car1D {
            time: 0.0,
            loc: 0.0,
            vel: speed,
            boost: 100.0,
        }
    }

    pub fn with_boost(mut self, boost: i32) -> Self {
        self.boost = boost as f32;
        self
    }

    pub fn with_boost_float(mut self, boost: f32) -> Self {
        self.boost = boost;
        self
    }

    pub fn time(&self) -> f32 {
        self.time
    }

    pub fn distance_traveled(&self) -> f32 {
        self.loc
    }

    pub fn speed(&self) -> f32 {
        self.vel
    }

    pub fn boost(&self) -> f32 {
        self.boost
    }

    pub fn step(&mut self, dt: f32, throttle: f32, mut boost: bool) {
        if boost {
            boost = self.boost > 0.0;
        }

        let new_vel = self.compute_new_vel(dt, throttle, boost);

        self.time += dt;
        self.loc += self.vel * dt;
        self.vel = new_vel;
        if boost {
            self.boost -= rl::BOOST_DEPLETION * dt;
        }
    }

    /// Simulate time running backwards, given the *previous* frame's inputs.
    ///
    /// A given [`Car1D`] instance must never change between running forwards
    /// and backwards, or the results will be nonsensical.
    ///
    /// This method does not increase boost (as you'd expect if time is running
    /// backwards). This results in a slightly counterintuitive state, but
    /// leaves the implementation simpler because boost will run out at the
    /// proper time just like in [`step`].
    pub fn step_rev(&mut self, dt: f32, throttle: f32, mut boost: bool) {
        if boost {
            boost = self.boost > 0.0;
        }

        let new_vel = self.compute_new_vel(-dt, throttle, boost);

        self.time += dt;
        self.vel = new_vel;
        self.loc += self.vel * dt;
        if boost {
            self.boost -= rl::BOOST_DEPLETION * dt;
        }
    }

    fn compute_new_vel(&self, dt: f32, throttle: f32, boost: bool) -> f32 {
        if !boost && self.vel >= rl::CAR_NORMAL_SPEED && throttle == 1.0 {
            return self.vel;
        }

        let (src_vel_table, src_time_table, time_table, vel_table) = match boost {
            false if throttle == 0.0 => (
                &data::coast::CAR_VEL_Y_REV,
                &data::coast::TIME_REV,
                &data::coast::TIME,
                &data::coast::CAR_VEL_Y,
            ),
            false if throttle == 1.0 => (
                &data::throttle::CAR_VEL_Y,
                &data::throttle::TIME,
                &data::throttle::TIME,
                &data::throttle::CAR_VEL_Y,
            ),
            true if throttle == 1.0 => (
                &data::boost::CAR_VEL_Y,
                &data::boost::TIME,
                &data::boost::TIME,
                &data::boost::CAR_VEL_Y,
            ),
            _ => panic!("Unsupported inputs"),
        };

        let old_time = linear_interpolate(src_vel_table, src_time_table, self.vel);
        let new_time = old_time + dt;
        linear_interpolate(time_table, vel_table, new_time)
    }
}

#[cfg(test)]
mod tests {
    use car1d::Car1D;

    const DT: f32 = 1.0 / 60.0;

    #[test]
    fn rest_throttle() {
        let mut car = Car1D::new(0.0);
        car.step(DT, 1.0, false);
        assert!(1.0 <= car.vel && car.vel < 50.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn supersonic_throttle() {
        let mut car = Car1D::new(2000.0);
        car.step(DT, 1.0, false);
        assert_eq!(car.vel, 2000.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn rest_boost() {
        let mut car = Car1D::new(0.0);
        car.step(DT, 1.0, true);
        assert!(1.0 <= car.vel && car.vel < 50.0);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn supersonic_boost() {
        let mut car = Car1D::new(2000.0);
        car.step(DT, 1.0, true);
        assert!(2010.0 <= car.vel && car.vel < 2020.0);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn max_speed_boost() {
        let mut car = Car1D::new(2299.98);
        car.step(DT, 1.0, true);
        assert_eq!(car.vel, 2299.98);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn rest_coast() {
        let mut car = Car1D::new(0.0);
        for _ in 0..100 {
            car.step(DT, 0.0, false);
        }
        assert!(car.vel < 1.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn slow_coast() {
        let mut car = Car1D::new(100.0);
        car.step(DT, 0.0, false);
        assert!(85.0 <= car.vel && car.vel < 95.0);
    }

    #[test]
    fn supersonic_coast() {
        let mut car = Car1D::new(2000.0);
        car.step(DT, 0.0, false);
        assert!(1980.0 <= car.vel && car.vel < 1995.0);
    }

    #[test]
    fn step_rev() {
        let mut car = Car1D::new(1000.0);
        car.step_rev(DT, 1.0, true);
        assert!(960.0 <= car.vel && car.vel < 980.0);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn coast_rev() {
        let mut car = Car1D::new(1000.0);
        car.step_rev(DT, 0.0, false);
        assert!(1005.0 <= car.vel && car.vel < 1015.0);
        assert_eq!(car.boost, 100.0);
    }
}
