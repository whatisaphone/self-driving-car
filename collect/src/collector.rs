use csv;
use rlbot;
use std::fs::File;

pub struct Collector {
    w: csv::Writer<File>,
}

impl Collector {
    pub fn new(file: File) -> Collector {
        let w = csv::Writer::from_writer(file);
        Collector { w }
    }

    pub fn write(&mut self, packet: &rlbot::LiveDataPacket) -> csv::Result<()> {
        self.w.write_record(
            vec![packet.GameInfo.TimeSeconds.to_string()]
                .iter()
                .chain(physics_items(&packet.GameBall.Physics).iter())
                .chain(physics_items(&packet.GameCars[0].Physics).iter())
                .collect::<Vec<_>>(),
        )
    }
}

fn physics_items(physics: &rlbot::Physics) -> [String; 12] {
    [
        physics.Location.X.to_string(),
        physics.Location.Y.to_string(),
        physics.Location.Z.to_string(),
        physics.Rotation.Pitch.to_string(),
        physics.Rotation.Yaw.to_string(),
        physics.Rotation.Roll.to_string(),
        physics.Velocity.X.to_string(),
        physics.Velocity.Y.to_string(),
        physics.Velocity.Z.to_string(),
        physics.AngularVelocity.X.to_string(),
        physics.AngularVelocity.Y.to_string(),
        physics.AngularVelocity.Z.to_string(),
    ]
}
