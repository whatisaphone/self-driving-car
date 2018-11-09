use oven::data;

pub const JUMP_MAX_Z: f32 = 241.45967;
const JUMP_MAX_Z_TABLE_INDEX: usize = 220;

pub fn time_to_z(z: f32) -> Option<f32> {
    if z <= data::jump::CAR_LOC_Z[0] {
        Some(0.0)
    } else if z > JUMP_MAX_Z {
        None
    } else {
        let i = data::jump::CAR_LOC_Z[..JUMP_MAX_Z_TABLE_INDEX]
            .binary_search_by(|n| n.partial_cmp(&z).unwrap())
            .unwrap_or_else(|i| i);
        Some(data::jump::TIME[i] - data::jump::TIME[0])
    }
}

#[cfg(test)]
mod tests {
    use car_single_jump;
    use oven::data;

    #[test]
    fn max_z() {
        assert_eq!(
            car_single_jump::JUMP_MAX_Z,
            *data::jump::CAR_LOC_Z
                .iter()
                .max_by(|x, y| x.partial_cmp(y).unwrap())
                .unwrap(),
        );
        assert_eq!(
            car_single_jump::JUMP_MAX_Z_TABLE_INDEX,
            data::jump::CAR_LOC_Z
                .iter()
                .enumerate()
                .max_by(|(_, x), (_, y)| x.partial_cmp(y).unwrap())
                .unwrap()
                .0
        );
    }

    #[test]
    fn time_to_z() {
        assert_eq!(
            car_single_jump::time_to_z(106.15358),
            Some(10.889706 - 10.652489)
        );
        assert_eq!(
            car_single_jump::time_to_z(car_single_jump::JUMP_MAX_Z),
            Some(11.537092 - 10.652489)
        );
        assert_eq!(car_single_jump::time_to_z(250.0), None);
    }
}
