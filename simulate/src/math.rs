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

#[cfg(test)]
mod tests {
    use math;

    #[test]
    fn linear_interpolate() {
        let xs = &[0.0, 1.0, 2.0];
        let ys = &[6.0, 8.0, 13.0];
        assert_eq!(math::linear_interpolate(xs, ys, -0.1), 6.0);
        assert_eq!(math::linear_interpolate(xs, ys, 0.0), 6.0);
        assert_eq!(math::linear_interpolate(xs, ys, 0.2), 6.4);
        assert_eq!(math::linear_interpolate(xs, ys, 1.0), 8.0);
        assert_eq!(math::linear_interpolate(xs, ys, 1.5), 10.5);
        assert_eq!(math::linear_interpolate(xs, ys, 2.0), 13.0);
        assert_eq!(math::linear_interpolate(xs, ys, 2.1), 13.0);
    }
}
