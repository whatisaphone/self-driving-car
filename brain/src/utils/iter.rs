use std::cmp::Ordering;

#[derive(PartialEq, PartialOrd)]
pub struct TotalF32(pub f32);

impl Eq for TotalF32 {}

impl Ord for TotalF32 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.partial_cmp(&other.0).unwrap()
    }
}
