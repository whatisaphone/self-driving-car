//! This module simulates the aerial trajectory defined in
//! collect:scenario::aerial_60deg.

use math::linear_interpolate;
use nalgebra::Vector3;
use rl;
use tables;

pub struct CarAerial60Deg {
    time: f32,
    loc: Vector3<f32>,
    vel_y: f32,
}

impl CarAerial60Deg {
    fn start_loc() -> Vector3<f32> {
        Vector3::new(0.0, 0.0, tables::AERIAL_60DEG_CAR_LOC_Z[0])
    }

    /// Using a jump plus 60° aerial, what will it take to climb to the given
    /// height?
    pub fn cost(z: f32) -> Cost {
        let i = tables::AERIAL_60DEG_CAR_LOC_Z
            .binary_search_by(|n| n.partial_cmp(&z).unwrap())
            .unwrap_or_else(|i| i);
        let time = tables::AERIAL_60DEG_TIME[i] - tables::AERIAL_60DEG_TIME[0];
        let boost = time * rl::BOOST_DEPLETION;
        return Cost { time, boost };
    }

    pub fn new(speed: f32) -> CarAerial60Deg {
        CarAerial60Deg {
            time: 0.0,
            loc: Self::start_loc(),
            vel_y: speed,
        }
    }

    pub fn time(&self) -> f32 {
        self.time
    }

    pub fn loc(&self) -> Vector3<f32> {
        self.loc
    }

    pub fn step(&mut self, dt: f32) {
        let old_loc_z = self.loc.z;
        let old_time = linear_interpolate(
            tables::AERIAL_60DEG_CAR_LOC_Z,
            tables::AERIAL_60DEG_TIME,
            old_loc_z,
        );
        let new_time = old_time + dt;
        let old_loc_y = linear_interpolate(
            tables::AERIAL_60DEG_TIME,
            tables::AERIAL_60DEG_CAR_LOC_Y,
            old_time,
        );
        let new_loc_y = linear_interpolate(
            tables::AERIAL_60DEG_TIME,
            tables::AERIAL_60DEG_CAR_LOC_Y,
            new_time,
        );
        let new_loc_z = linear_interpolate(
            tables::AERIAL_60DEG_TIME,
            tables::AERIAL_60DEG_CAR_LOC_Z,
            new_time,
        );

        self.time += dt;
        //        println!("step before {:?} {} {} {}", self.loc, old_loc_y, new_loc_y,
        // self.vel_y * dt);
        self.loc += Vector3::new(
            0.0,
            new_loc_y - old_loc_y + self.vel_y * dt,
            new_loc_z - old_loc_z,
        );
        //        println!("step {:.3} {:.0} {:?}", dt, self.vel_y, self.loc);
    }
}

pub struct Cost {
    pub time: f32,
    pub boost: f32,
}

#[cfg(test)]
mod tests {
    use car_aerial_60deg::CarAerial60Deg;

    #[test]
    fn time_to_z() {
        let cost = CarAerial60Deg::cost(1000.0);
        assert!(cost.time - (18.466011 - 16.360859) < 0.01);
    }
}
