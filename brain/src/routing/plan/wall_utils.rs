use crate::{strategy::Pitch, utils::geometry::Plane};
use nalgebra::Point3;

const SURFACE_DIST_THRESHOLD: f32 = 500.0;

pub fn which_surface<'p>(pitch: &'p Pitch, loc: &Point3<f32>) -> Result<&'p Plane, ()> {
    let wall = pitch.closest_plane(loc);
    if wall.distance_to_point(loc) >= SURFACE_DIST_THRESHOLD {
        return Err(());
    }
    Ok(wall)
}
