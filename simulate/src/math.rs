pub fn linear_interpolate(xs: &[f32], ys: &[f32], x: f32) -> f32 {
    match xs.binary_search_by(|n| n.partial_cmp(&x).unwrap()) {
        Ok(i) => ys[i],
        Err(0) => ys[0],
        Err(i) if i == xs.len() => *ys.last().unwrap(),
        Err(i) => {
            let left = xs[i - 1];
            let right = xs[i];
            let ratio = (x - left) / (right - left);
            ys[i - 1] + (ys[i] - ys[i - 1]) * ratio
        }
    }
}

/// This lets you split interpolation into two steps, to save on unnecessary
/// binary searches. Pair with [`linear_interpolate_use_index`].
pub fn linear_interpolate_find_index(xs: &[f32], x: f32) -> f64 {
    match xs.binary_search_by(|n| n.partial_cmp(&x).unwrap()) {
        Ok(i) => i as f64,
        Err(0) => 0.0,
        Err(i) if i == xs.len() => (i - 1) as f64,
        Err(i) => {
            let left = xs[i - 1];
            let right = xs[i];
            // Use `f64` to compensate for precision loss when adding a large integral part.
            (i - 1) as f64 + ((x - left) / (right - left)) as f64
        }
    }
}

pub fn linear_interpolate_use_index(ys: &[f32], x: f64) -> f32 {
    let int = x.floor() as usize;
    let frac = (x - int as f64) as f32;
    if frac == 0.0 {
        return ys[int];
    }
    ys[int] + (ys[int + 1] - ys[int]) * frac
}

#[cfg(test)]
mod tests {
    use math;

    #[test]
    fn linear_interpolate() {
        let xs = &[0.0, 1.0, 3.0];
        let ys = &[6.0, 8.0, 13.0];
        assert_eq!(math::linear_interpolate(xs, ys, -0.1), 6.0);
        assert_eq!(math::linear_interpolate(xs, ys, 0.0), 6.0);
        assert_eq!(math::linear_interpolate(xs, ys, 0.5), 7.0);
        assert_eq!(math::linear_interpolate(xs, ys, 1.0), 8.0);
        assert_eq!(math::linear_interpolate(xs, ys, 2.0), 10.5);
        assert_eq!(math::linear_interpolate(xs, ys, 3.0), 13.0);
        assert_eq!(math::linear_interpolate(xs, ys, 3.1), 13.0);
    }

    #[test]
    fn linear_interpolate_find_index() {
        let xs = &[0.0, 1.0, 3.0];
        assert_eq!(math::linear_interpolate_find_index(xs, -0.1), 0.0);
        assert_eq!(math::linear_interpolate_find_index(xs, 0.0), 0.0);
        assert_eq!(math::linear_interpolate_find_index(xs, 0.5), 0.5);
        assert_eq!(math::linear_interpolate_find_index(xs, 1.0), 1.0);
        assert_eq!(math::linear_interpolate_find_index(xs, 2.0), 1.5);
        assert_eq!(math::linear_interpolate_find_index(xs, 3.0), 2.0);
        assert_eq!(math::linear_interpolate_find_index(xs, 3.1), 2.0);
    }

    #[test]
    fn two_step_linear_interpolate() {
        let xs = &[0.0, 1.0, 3.0];
        let ys = &[6.0, 8.0, 13.0];
        for &x in &[-0.1, 0.0, 0.5, 1.0, 2.0, 3.0, 3.1] {
            eprintln!("x = {:?}", x);
            let index = math::linear_interpolate_find_index(xs, x);
            let two_step = math::linear_interpolate_use_index(ys, index);
            let one_step = math::linear_interpolate(xs, ys, x);
            // The two-step sequence and the one-step function should match.
            assert_eq!(two_step, one_step);
        }
    }
}
