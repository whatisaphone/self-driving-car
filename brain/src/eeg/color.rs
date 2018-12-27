use crate::strategy::Team;
use graphics::types::Color;

pub const BLACK: Color = [0.0, 0.0, 0.0, 1.0];
pub const WHITE: Color = [1.0, 1.0, 1.0, 1.0];
pub const PITCH: Color = [0.0, 0.2, 0.0, 1.0];
pub const RED: Color = [1.0, 0.0, 0.0, 1.0];
pub const ORANGE: Color = [1.0, 0.5, 0.0, 1.0];
pub const ORANGE_DARK: Color = [0.5, 0.25, 0.0, 1.0];
pub const GREEN: Color = [0.0, 1.0, 0.0, 1.0];
pub const YELLOW: Color = [1.0, 1.0, 0.0, 1.0];
pub const BLUE: Color = [0.5, 0.5, 1.0, 1.0];
pub const BLUE_DARK: Color = [0.25, 0.25, 0.5, 1.0];

pub fn for_team(team: Team) -> Color {
    match team {
        Team::Blue => BLUE,
        Team::Orange => ORANGE,
    }
}
