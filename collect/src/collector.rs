use std::{fs::File, iter::once};

pub struct Collector {
    w: csv::Writer<File>,
    wrote_header: bool,
}

impl Collector {
    pub fn new(file: File) -> Collector {
        let w = csv::Writer::from_writer(file);
        Collector {
            w,
            wrote_header: false,
        }
    }

    pub fn write(&mut self, tick: rlbot::flat::RigidBodyTick<'_>) -> csv::Result<()> {
        if !self.wrote_header {
            self.wrote_header = true;
            self.w.write_record(
                once(String::from("time"))
                    .chain(rigid_body_header("ball"))
                    .chain(
                        (0..tick.players().unwrap().len())
                            .map(|i| format!("player{}", i))
                            .flat_map(|s| controller_header(s.clone()).chain(rigid_body_header(s))),
                    ),
            )?;
        }

        self.w.write_record(
            once((tick.ball().unwrap().state().unwrap().frame() as f32 / 120.0).to_string())
                .chain(rigid_body(tick.ball().unwrap().state().unwrap()))
                .chain(flat_vector_iter(tick.players().unwrap()).flat_map(|c| {
                    controller(c.input().unwrap()).chain(rigid_body(c.state().unwrap()))
                })),
        )
    }
}

fn flat_vector_iter<'a>(
    xs: flatbuffers::Vector<
        'a,
        flatbuffers::ForwardsUOffset<rlbot::flat::PlayerRigidBodyState<'a>>,
    >,
) -> impl Iterator<Item = rlbot::flat::PlayerRigidBodyState<'a>> {
    (0..xs.len()).map(move |i| xs.get(i))
}

fn rigid_body_header(prefix: impl AsRef<str>) -> impl Iterator<Item = String> {
    [
        "_loc_x", "_loc_y", "_loc_z", "_rot_x", "_rot_y", "_rot_z", "_rot_w", "_vel_x", "_vel_y",
        "_vel_z", "_ang_x", "_ang_y", "_ang_z",
    ]
    .iter()
    .map(move |s| format!("{}{}", prefix.as_ref(), s))
}

fn rigid_body(state: rlbot::flat::RigidBodyState<'_>) -> impl Iterator<Item = String> {
    vec![
        state.location().unwrap().x().to_string(),
        state.location().unwrap().y().to_string(),
        state.location().unwrap().z().to_string(),
        state.rotation().unwrap().x().to_string(),
        state.rotation().unwrap().y().to_string(),
        state.rotation().unwrap().z().to_string(),
        state.rotation().unwrap().w().to_string(),
        state.velocity().unwrap().x().to_string(),
        state.velocity().unwrap().y().to_string(),
        state.velocity().unwrap().z().to_string(),
        state.angularVelocity().unwrap().x().to_string(),
        state.angularVelocity().unwrap().y().to_string(),
        state.angularVelocity().unwrap().z().to_string(),
    ]
    .into_iter()
}

fn controller_header(prefix: impl AsRef<str>) -> impl Iterator<Item = String> {
    [
        "_throttle",
        "_steer",
        "_pitch",
        "_yaw",
        "_roll",
        "_jump",
        "_boost",
        "_handbrake",
    ]
    .iter()
    .map(move |s| format!("{}{}", prefix.as_ref(), s))
}

fn controller(state: rlbot::flat::ControllerState<'_>) -> impl Iterator<Item = String> {
    vec![
        state.throttle().to_string(),
        state.steer().to_string(),
        state.pitch().to_string(),
        state.yaw().to_string(),
        state.roll().to_string(),
        state.jump().to_string(),
        state.boost().to_string(),
        state.handbrake().to_string(),
    ]
    .into_iter()
}
