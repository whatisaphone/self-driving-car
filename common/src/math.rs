/// Returns the absolute difference between `n` and the closest integer.
///
/// The result will be within `0..=0.5`.
pub fn fractionality(n: f32) -> f32 {
    (n - n.round()).abs()
}

/// Solves the quadratic formula over the real numbers. Returns `None` if there
/// is no real solution.
///
/// >ax^2 + bx + c = 0
pub fn quadratic(a: f32, b: f32, c: f32) -> Option<[f32; 2]> {
    let square = b * b - 4.0 * a * c;
    if square < 0.0 {
        return None;
    }
    let root = square.sqrt();
    Some([(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)])
}

#[cfg(test)]
mod tests {
    use crate::math::fractionality;

    #[test]
    fn test_fractionality() {
        let cases = [
            (-8.0, 0.0),
            (-7.8, 0.2),
            (-7.5, 0.5),
            (-7.4, 0.4),
            (-1.0, 0.0),
            (-0.8, 0.2),
            (-0.2, 0.2),
            (0.0, 0.0),
            (0.1, 0.1),
            (0.5, 0.5),
            (0.7, 0.3),
            (1.0, 0.0),
            (3.2, 0.2),
            (3.5, 0.5),
            (3.7, 0.3),
            (4.0, 0.0),
        ];
        for &(n, expected) in &cases {
            let result = fractionality(n);
            assert!(
                (result - expected).abs() <= 1e-6,
                "{} {} {}",
                n,
                expected,
                result,
            );
        }
    }
}
