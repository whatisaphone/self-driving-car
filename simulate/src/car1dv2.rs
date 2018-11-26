use common::rl;
use math::{linear_interpolate_find_index, linear_interpolate_use_index};
use oven::data;

const EPS: f32 = 1e-6;

pub struct Car1Dv2 {
    time: f32,
    distance: f32,
    speed: f32,
    boost: f32,
}

impl Car1Dv2 {
    pub fn new() -> Self {
        Self {
            time: 0.0,
            distance: 0.0,
            speed: 0.0,
            boost: 100.0,
        }
    }

    pub fn with_speed(mut self, speed: f32) -> Self {
        self.speed = speed;
        self
    }

    pub fn with_boost(mut self, boost: f32) -> Self {
        self.boost = boost;
        self
    }

    pub fn time(&self) -> f32 {
        self.time
    }

    pub fn distance(&self) -> f32 {
        self.distance
    }

    pub fn speed(&self) -> f32 {
        self.speed
    }

    pub fn boost(&self) -> f32 {
        self.boost
    }

    pub fn step(&mut self, dt: f32, throttle: f32, boost: bool) {
        let curve = match Self::next_curve(self.speed, throttle, boost) {
            Curve::Throttle => self.calc_throttle(dt),
            Curve::Boost => self.calc_boost(dt),
            Curve::Coast => self.calc_coast(dt),
            Curve::ConstantVelocity => self.calc_constant_velocity(dt, boost),
        };

        // Some sanity checks.
        assert!(curve.dt > 0.0 && curve.dt <= dt);
        assert!(curve.distance >= 0.0 && curve.distance <= dt * rl::CAR_MAX_SPEED);
        assert!(curve.new_speed >= 0.0 && curve.new_speed <= rl::CAR_MAX_SPEED);
        assert!(curve.boost_used >= 0.0 && curve.boost_used <= self.boost);

        self.time += curve.dt;
        self.distance += curve.distance;
        self.speed = curve.new_speed;
        self.boost -= curve.boost_used;

        if dt - curve.dt <= EPS {
            return; // We're done!
        }

        // Otherwise, proceed to the next curve.
        self.step(dt - curve.dt, throttle, boost);
    }

    fn next_curve(speed: f32, throttle: f32, boost: bool) -> Curve {
        if throttle == 0.0 && !boost && speed == 0.0 {
            Curve::ConstantVelocity
        } else if throttle == 0.0 && !boost {
            Curve::Coast
        } else if throttle == 1.0 && !boost && speed < rl::CAR_NORMAL_SPEED {
            Curve::Throttle
        } else if throttle == 1.0 && !boost {
            Curve::ConstantVelocity
        } else if throttle == 1.0 && boost && speed < rl::CAR_MAX_SPEED {
            Curve::Boost
        } else if throttle == 1.0 && boost {
            Curve::ConstantVelocity
        } else {
            panic!("unsupported inputs")
        }
    }

    fn calc_coast(&self, dt: f32) -> CurveResult {
        let (so_far_dt, distance, new_speed) = Self::lookup(dt, self.speed, 0.0, false);
        CurveResult {
            dt: so_far_dt,
            distance,
            new_speed,
            boost_used: 0.0,
        }
    }

    fn calc_throttle(&self, dt: f32) -> CurveResult {
        let (so_far_dt, distance, new_speed) = Self::lookup(dt, self.speed, 1.0, false);
        CurveResult {
            dt: so_far_dt,
            distance,
            new_speed,
            boost_used: 0.0,
        }
    }

    fn calc_boost(&self, dt: f32) -> CurveResult {
        let (so_far_dt, distance, new_speed) = Self::lookup(dt, self.speed, 1.0, true);
        CurveResult {
            dt: so_far_dt,
            distance,
            new_speed,
            boost_used: dt * rl::BOOST_DEPLETION,
        }
    }

    fn calc_constant_velocity(&self, dt: f32, boost: bool) -> CurveResult {
        let distance = dt * self.speed;
        let boost_used = if boost {
            // It's inefficient to boost at max speed, but we should still simulate it
            // accurately.
            assert!(self.speed == rl::CAR_MAX_SPEED);
            dt * rl::BOOST_DEPLETION
        } else {
            0.0
        };
        CurveResult {
            dt,
            distance,
            new_speed: self.speed,
            boost_used,
        }
    }

    fn lookup(dt: f32, old_speed: f32, throttle: f32, boost: bool) -> (f32, f32, f32) {
        let (src_time_table, src_dist_table, src_vel_table, time_table, dist_table, speed_table) =
            match boost {
                false if throttle == 0.0 => (
                    &data::coast::TIME_REV,
                    &data::coast::CAR_LOC_Y_REV,
                    &data::coast::CAR_VEL_Y_REV,
                    &data::coast::TIME,
                    &data::coast::CAR_LOC_Y,
                    &data::coast::CAR_VEL_Y,
                ),
                false if throttle == 1.0 => (
                    &data::throttle::TIME,
                    &data::throttle::CAR_LOC_Y,
                    &data::throttle::CAR_VEL_Y,
                    &data::throttle::TIME,
                    &data::throttle::CAR_LOC_Y,
                    &data::throttle::CAR_VEL_Y,
                ),
                true if throttle == 1.0 => (
                    &data::boost::TIME,
                    &data::boost::CAR_LOC_Y,
                    &data::boost::CAR_VEL_Y,
                    &data::boost::TIME,
                    &data::boost::CAR_LOC_Y,
                    &data::boost::CAR_VEL_Y,
                ),
                _ => panic!("Unsupported inputs"),
            };

        let old_index = linear_interpolate_find_index(src_vel_table, old_speed);
        let old_time = linear_interpolate_use_index(src_time_table, old_index);
        let old_dist = linear_interpolate_use_index(src_dist_table, old_index);

        let new_index = linear_interpolate_find_index(time_table, old_time + dt);
        let new_time = linear_interpolate_use_index(time_table, new_index);
        let new_dist = linear_interpolate_use_index(dist_table, new_index);
        let new_speed = linear_interpolate_use_index(speed_table, new_index);
        (new_time - old_time, new_dist - old_dist, new_speed)
    }
}

enum Curve {
    Throttle,
    Boost,
    Coast,
    ConstantVelocity,
}

struct CurveResult {
    dt: f32,
    distance: f32,
    new_speed: f32,
    boost_used: f32,
}

#[cfg(test)]
mod tests {
    use car1dv2::Car1Dv2;
    use common::rl;
    use oven::data;

    const DT: f32 = 1.0 / 60.0;

    #[test]
    fn throttle_table() {
        assert_eq!(
            rl::CAR_NORMAL_SPEED,
            *data::throttle::CAR_VEL_Y.last().unwrap(),
        );
    }

    #[test]
    fn boost_table() {
        assert_eq!(rl::CAR_MAX_SPEED, *data::boost::CAR_VEL_Y.last().unwrap());
    }

    #[test]
    fn rest_throttle() {
        let mut car = Car1Dv2::new().with_speed(0.0);
        car.step(DT, 1.0, false);
        assert!(1.0 <= car.speed && car.speed < 50.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn supersonic_throttle() {
        let mut car = Car1Dv2::new().with_speed(2000.0);
        car.step(DT, 1.0, false);
        assert_eq!(car.speed, 2000.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn rest_boost() {
        let mut car = Car1Dv2::new().with_speed(0.0);
        car.step(DT, 1.0, true);
        assert!(1.0 <= car.speed && car.speed < 50.0);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn supersonic_boost() {
        let mut car = Car1Dv2::new().with_speed(2000.0);
        car.step(DT, 1.0, true);
        assert!(2010.0 <= car.speed && car.speed < 2020.0);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn max_speed_boost() {
        let mut car = Car1Dv2::new().with_speed(rl::CAR_MAX_SPEED);
        car.step(DT, 1.0, true);
        assert_eq!(car.speed, rl::CAR_MAX_SPEED);
        assert!(99.4 <= car.boost && car.boost < 99.5);
    }

    #[test]
    fn rest_coast() {
        let mut car = Car1Dv2::new().with_speed(0.0);
        for _ in 0..100 {
            car.step(DT, 0.0, false);
        }
        assert!(car.speed < 1.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn slow_coast() {
        let mut car = Car1Dv2::new().with_speed(100.0);
        car.step(DT, 0.0, false);
        assert!(85.0 <= car.speed && car.speed < 95.0);
    }

    #[test]
    fn supersonic_coast() {
        let mut car = Car1Dv2::new().with_speed(2000.0);
        car.step(DT, 0.0, false);
        assert!(1980.0 <= car.speed && car.speed < 1995.0);
    }
}
