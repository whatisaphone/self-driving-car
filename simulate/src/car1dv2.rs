use crate::math::{linear_interpolate_find_index, linear_interpolate_use_index};
use common::rl;
use oven::data;

const EPS: f32 = 1e-3;

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
        assert!(speed >= 0.0);
        assert!(speed < rl::CAR_MAX_SPEED + 1.0); // Allow slight rounding errors
        let speed = speed.min(rl::CAR_MAX_SPEED);

        self.speed = speed;
        self
    }

    pub fn with_boost(mut self, boost: f32) -> Self {
        assert!(boost >= 0.0);
        assert!(boost <= 100.0);

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

    pub fn advance(&mut self, dt: f32, throttle: f32, mut boost: bool) {
        assert!(dt >= 0.0);
        if dt < EPS {
            return; // Rigorously simulate zero time elapsing.
        }

        if boost && self.boost <= EPS {
            boost = false;
        }

        let curve = match Self::next_curve(self.speed, throttle, boost) {
            Curve::Throttle => self.calc_throttle_by_time(dt),
            Curve::Boost => self.calc_boost_by_time(dt),
            Curve::Coast => self.calc_coast_by_time(dt),
            Curve::ConstantSpeed => self.calc_constant_speed_by_time(dt, boost),
        };

        // Some sanity checks.
        assert!(curve.dt >= 0.0 && curve.dt <= dt + EPS);
        assert!(curve.distance >= 0.0 && curve.distance <= dt * rl::CAR_MAX_SPEED * 1.01);
        assert!(curve.new_speed >= 0.0 && curve.new_speed <= rl::CAR_MAX_SPEED);
        assert!(curve.boost_used >= 0.0 && curve.boost_used <= self.boost + EPS);

        self.time += curve.dt;
        self.distance += curve.distance;
        self.speed = curve.new_speed;
        self.boost -= curve.boost_used;

        if dt - curve.dt <= EPS {
            return; // We're done!
        }

        // Otherwise, proceed to the next curve.
        self.advance(dt - curve.dt, throttle, boost);
    }

    pub fn advance_by_distance(&mut self, distance: f32, throttle: f32, mut boost: bool) {
        assert!(distance > 0.0);

        if boost && self.boost <= EPS {
            boost = false;
        }

        let curve = match Self::next_curve(self.speed, throttle, boost) {
            Curve::Throttle => self.calc_throttle_by_distance(distance),
            Curve::Boost => self.calc_boost_by_distance(distance),
            Curve::Coast => panic!("unsupported inputs"),
            Curve::ConstantSpeed => self.calc_constant_speed_by_distance(distance, boost),
        };

        // Some sanity checks.
        assert!(curve.dt >= curve.distance / rl::CAR_MAX_SPEED - EPS);
        assert!(curve.distance >= 0.0 && curve.distance <= distance + EPS);
        assert!(curve.new_speed >= 0.0 && curve.new_speed <= rl::CAR_MAX_SPEED);
        assert!(curve.boost_used >= 0.0 && curve.boost_used <= self.boost);

        self.time += curve.dt;
        self.distance += curve.distance;
        self.speed = curve.new_speed;
        self.boost -= curve.boost_used;

        if distance - curve.distance <= EPS {
            return; // We're done!
        }

        // Otherwise, proceed to the next curve.
        self.advance_by_distance(distance - curve.distance, throttle, boost);
    }

    fn next_curve(speed: f32, throttle: f32, boost: bool) -> Curve {
        if throttle == 0.0 && !boost && speed == 0.0 {
            Curve::ConstantSpeed
        } else if throttle == 0.0 && !boost {
            Curve::Coast
        } else if throttle == 1.0 && !boost && speed < rl::CAR_NORMAL_SPEED {
            Curve::Throttle
        } else if throttle == 1.0 && !boost {
            Curve::ConstantSpeed
        } else if throttle == 1.0 && boost && speed < rl::CAR_MAX_SPEED {
            Curve::Boost
        } else if throttle == 1.0 && boost {
            Curve::ConstantSpeed
        } else {
            panic!("unsupported inputs")
        }
    }

    fn calc_coast_by_time(&self, dt: f32) -> CurveResult {
        let (dt, distance, new_speed) = Self::lookup_advance_by_time(dt, self.speed, 0.0, false);
        CurveResult {
            dt,
            distance,
            new_speed,
            boost_used: 0.0,
        }
    }

    fn calc_throttle_by_time(&self, dt: f32) -> CurveResult {
        let (dt, distance, new_speed) = Self::lookup_advance_by_time(dt, self.speed, 1.0, false);
        CurveResult {
            dt,
            distance,
            new_speed,
            boost_used: 0.0,
        }
    }

    fn calc_boost_by_time(&self, dt: f32) -> CurveResult {
        let dt = dt.min(self.boost / rl::BOOST_DEPLETION);
        let boost_used = dt * rl::BOOST_DEPLETION;
        let (dt, distance, new_speed) = Self::lookup_advance_by_time(dt, self.speed, 1.0, true);
        CurveResult {
            dt,
            distance,
            new_speed,
            boost_used,
        }
    }

    fn calc_constant_speed_by_time(&self, dt: f32, boost: bool) -> CurveResult {
        let distance = dt * self.speed;
        let boost_used = if boost {
            // It's inefficient to boost at max speed, but that's not our decision to make
            // here. We should still simulate it accurately.
            assert!(self.speed == rl::CAR_MAX_SPEED);
            (dt * rl::BOOST_DEPLETION).min(self.boost)
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

    fn calc_throttle_by_distance(&self, distance: f32) -> CurveResult {
        let (dt, distance, new_speed) =
            Self::lookup_advance_by_distance(distance, self.speed, 1.0, false);

        CurveResult {
            dt,
            distance,
            new_speed,
            boost_used: 0.0,
        }
    }

    fn calc_boost_by_distance(&self, distance: f32) -> CurveResult {
        let (dt, distance, new_speed) =
            Self::lookup_advance_by_distance(distance, self.speed, 1.0, true);

        let boost_used = dt * rl::BOOST_DEPLETION;
        if boost_used > self.boost {
            return self.calc_boost_by_time(self.boost / rl::BOOST_DEPLETION);
        }

        CurveResult {
            dt,
            distance,
            new_speed,
            boost_used,
        }
    }

    fn calc_constant_speed_by_distance(&self, distance: f32, boost: bool) -> CurveResult {
        assert!(self.speed >= rl::CAR_NORMAL_SPEED);
        let dt = distance / self.speed;
        self.calc_constant_speed_by_time(dt, boost)
    }

    fn lookup_advance_by_time(
        dt: f32,
        old_speed: f32,
        throttle: f32,
        boost: bool,
    ) -> (f32, f32, f32) {
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

    fn lookup_advance_by_distance(
        distance: f32,
        old_speed: f32,
        throttle: f32,
        boost: bool,
    ) -> (f32, f32, f32) {
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

        let new_index = linear_interpolate_find_index(dist_table, old_dist + distance);
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
    ConstantSpeed,
}

struct CurveResult {
    dt: f32,
    distance: f32,
    new_speed: f32,
    boost_used: f32,
}

#[cfg(test)]
mod tests {
    use crate::car1dv2::Car1Dv2;
    use common::rl;
    use oven::data;

    const DT: f32 = 1.0 / 60.0;
    const EPS: f32 = 1e-4;

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
    fn clamp_speed() {
        let car = Car1Dv2::new().with_speed(rl::CAR_MAX_SPEED + 0.1);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
    }

    #[test]
    #[should_panic]
    fn dont_clamp_outrageous_speed() {
        Car1Dv2::new().with_speed(rl::CAR_MAX_SPEED + 100.0);
    }

    #[test]
    fn advance_0() {
        let mut car = Car1Dv2::new().with_speed(1000.0);
        car.advance(0.0, 1.0, false);
        assert_eq!(car.time(), 0.0);
        assert_eq!(car.boost(), 100.0);
    }

    #[test]
    fn advance_throttle_rest() {
        let mut car = Car1Dv2::new().with_speed(0.0);
        car.advance(DT, 1.0, false);
        assert!(20.0 <= car.speed() && car.speed() < 30.0);
        assert_eq!(car.boost(), 100.0);
    }

    #[test]
    fn advance_throttle_supersonic() {
        let mut car = Car1Dv2::new().with_speed(2000.0);
        car.advance(DT, 1.0, false);
        assert_eq!(car.speed(), 2000.0);
        assert_eq!(car.boost(), 100.0);
    }

    #[test]
    fn advance_throttle_max_speed() {
        let mut car = Car1Dv2::new().with_speed(rl::CAR_MAX_SPEED);
        car.advance(DT, 1.0, false);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
        assert_eq!(car.boost(), 100.0);
    }

    #[test]
    fn advance_boost_rest() {
        let mut car = Car1Dv2::new().with_speed(0.0);
        car.advance(DT, 1.0, true);
        assert!(40.0 <= car.speed() && car.speed() < 45.0);
        assert!(99.4 <= car.boost() && car.boost() < 99.5);
    }

    #[test]
    fn advance_boost_supersonic() {
        let mut car = Car1Dv2::new().with_speed(2000.0);
        car.advance(DT, 1.0, true);
        assert!(2010.0 <= car.speed() && car.speed() < 2020.0);
        assert!(99.4 <= car.boost() && car.boost() < 99.5);
    }

    #[test]
    fn advance_boost_supersonic_dont_trigger_assert() {
        let mut car = Car1Dv2::new().with_speed(2293.8083).with_boost(34.833214);
        car.advance(1.0 / 120.0, 1.0, true);
        assert!(2299.0 <= car.speed() && car.speed() < rl::CAR_MAX_SPEED);
    }

    #[test]
    fn advance_boost_supersonic_dont_trigger_assert_2() {
        let mut car = Car1Dv2::new().with_speed(1673.71265).with_boost(3.00000286);
        car.advance(0.125, 1.0, true);
        assert!(car.boost().abs() <= EPS);
    }

    #[test]
    fn advance_boost_max_speed() {
        let mut car = Car1Dv2::new().with_speed(rl::CAR_MAX_SPEED);
        car.advance(DT, 1.0, true);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
        assert!(99.4 <= car.boost() && car.boost() < 99.5);
    }

    #[test]
    fn advance_boost_with_no_boost() {
        let mut car = Car1Dv2::new().with_boost(0.0);
        car.advance(DT, 1.0, true);
        assert!(20.0 <= car.speed() && car.speed() < 30.0);
        assert_eq!(car.boost(), 0.0);
    }

    #[test]
    fn advance_boost_supersonic_boost_underflow() {
        let mut car = Car1Dv2::new().with_speed(2000.0).with_boost(0.01);
        car.advance(DT, 1.0, true);
        assert!(2000.001 <= car.speed() && car.speed() < 2001.0);
        assert_eq!(car.boost(), 0.0);
    }

    #[test]
    fn advance_boost_max_speed_boost_underflow() {
        let mut car = Car1Dv2::new()
            .with_speed(rl::CAR_MAX_SPEED)
            .with_boost(0.0001);
        car.advance(DT, 1.0, true);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
        assert!(car.boost().abs() <= EPS);
    }

    #[test]
    fn advance_boost_max_speed_deplete_boost() {
        let mut car = Car1Dv2::new()
            .with_speed(rl::CAR_MAX_SPEED)
            .with_boost(10.0);
        car.advance(1.0, 1.0, true);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
        assert_eq!(car.boost(), 0.0);
    }

    #[test]
    fn advance_coast_rest() {
        let mut car = Car1Dv2::new().with_speed(0.0);
        for _ in 0..100 {
            car.advance(DT, 0.0, false);
        }
        assert_eq!(car.speed(), 0.0);
        assert_eq!(car.boost(), 100.0);
    }

    #[test]
    fn advance_coast_slow() {
        let mut car = Car1Dv2::new().with_speed(100.0);
        car.advance(DT, 0.0, false);
        assert!(85.0 <= car.speed() && car.speed() < 95.0);
    }

    #[test]
    fn advance_coast_supersonic() {
        let mut car = Car1Dv2::new().with_speed(2000.0);
        car.advance(DT, 0.0, false);
        assert!(1980.0 <= car.speed() && car.speed() < 1995.0);
    }

    #[test]
    fn advance_by_distance_throttle_slow() {
        let mut car = Car1Dv2::new().with_speed(1000.0);
        car.advance_by_distance(1000.0, 1.0, false);
        assert!((car.distance() - 1000.0).abs() <= EPS);
        assert!(1300.0 <= car.speed() && car.speed() < 1400.0);
        assert_eq!(car.boost, 100.0);
    }

    #[test]
    fn advance_by_distance_throttle_supersonic_dont_trigger_assert() {
        let mut car = Car1Dv2::new().with_speed(1878.42249);
        car.advance_by_distance(1000.0, 1.0, false);
        assert!((car.distance() - 1000.0).abs() <= EPS);
    }

    #[test]
    fn advance_by_distance_boost_with_low_boost_rest() {
        let mut car = Car1Dv2::new().with_boost(10.0);
        car.advance_by_distance(1000.0, 1.0, true);
        assert!((car.distance() - 1000.0).abs() <= EPS);
        assert!(1200.0 <= car.speed() && car.speed() < 1300.0);
        assert_eq!(car.boost(), 0.0);
    }

    #[test]
    fn advance_by_distance_boost_with_low_boost_slow() {
        let mut car = Car1Dv2::new().with_speed(347.61176).with_boost(34.0);
        car.advance_by_distance(5000.0, 1.0, true);
        assert!((car.distance() - 5000.0).abs() <= EPS);
        assert!(1700.0 <= car.speed() && car.speed() < 1800.0);
        assert!(car.boost().abs() <= EPS);
    }

    #[test]
    fn advance_by_distance_boost_with_no_boost() {
        let mut car = Car1Dv2::new().with_speed(1000.0).with_boost(0.0);
        car.advance_by_distance(1000.0, 1.0, true);
        assert!((car.distance() - 1000.0).abs() <= EPS);
        assert!(1300.0 <= car.speed() && car.speed() < 1400.0);
        assert_eq!(car.boost(), 0.0);
    }

    #[test]
    fn advance_by_distance_boost_max_speed_huge_distance() {
        let mut car = Car1Dv2::new().with_speed(rl::CAR_MAX_SPEED);
        car.advance_by_distance(8000.0, 1.0, true);
        assert!((car.distance() - 8000.0).abs() <= EPS);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
        assert_eq!(car.boost(), 0.0);
    }

    #[test]
    fn advance_by_distance_boost_almost_supersonic_dont_trigger_assert() {
        let mut car = Car1Dv2::new().with_speed(2292.90942);
        car.advance_by_distance(1000.0, 1.0, true);
        assert_eq!(car.speed(), rl::CAR_MAX_SPEED);
    }
}
