use ffi;

impl ffi::PlayerConfiguration {
    pub fn set_name(&mut self, name: &str) {
        for (i, cp) in name.encode_utf16().enumerate() {
            self.Name[i] = cp;
        }
    }
}

impl ffi::LiveDataPacket {
    /// Yields the `PlayerInfo` for each car participating in the match.
    pub fn cars(&self) -> impl Iterator<Item = &ffi::PlayerInfo> {
        self.GameCars.iter().take(self.NumCars as usize)
    }

    /// Return the scores for the blue and orange teams.
    ///
    /// The API doesn't seem to return this info(?) so instead we compute it
    /// manually (and inaccurately) by adding up the goals for each individual
    /// player.
    pub fn match_score(&self) -> [i32; 2] {
        let mut result = [0, 0];
        for car in self.cars() {
            result[car.Team as usize] += car.Score.Goals;
        }
        result
    }
}
