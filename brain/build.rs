extern crate csv;

use std::{
    env,
    fs::File,
    io::{Read, Write},
    path::PathBuf,
};

fn main() {
    let crate_dir = env::current_dir().unwrap();
    let csv_dir = crate_dir.join("recordings");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let mut out = File::create(out_dir.join("recordings.rs")).unwrap();

    writeln!(out, "use collect::RecordingRigidBodyState;\n").unwrap();
    writeln!(out, "use integration_tests::helpers::OneVOneScenario;\n").unwrap();
    writeln!(
        out,
        "use nalgebra::{{Point3, Quaternion, UnitQuaternion, Vector3}};\n"
    )
    .unwrap();
    writeln!(out, "use rlbot::ffi::PlayerInput;\n").unwrap();
    writeln!(out).unwrap();
    writeln!(out, "lazy_static! {{").unwrap();

    for entry in csv_dir.read_dir().unwrap().map(Result::unwrap) {
        let filename = entry.file_name().into_string().unwrap();
        if !filename.ends_with(".csv") {
            continue;
        }

        let basename = filename.split_terminator(".").next().unwrap();
        let file = File::open(entry.path()).unwrap();
        let mut r = csv::ReaderBuilder::new().from_reader(file);
        translate_csv(basename, &mut r, &mut out);
    }

    writeln!(out, "}}").unwrap(); // lazy_static!
}

fn translate_csv(name: &str, csv: &mut csv::Reader<impl Read>, out: &mut impl Write) {
    let name = name.to_ascii_uppercase();

    let rows: Vec<_> = csv.records().collect();
    let headers = csv.headers().unwrap();

    macro_rules! column_index {
        ($header:expr) => {
            headers.iter().position(|h| h == $header).unwrap()
        };
    }

    macro_rules! write_float_array {
        ($column:expr, $suffix:expr) => {
            let i_col = column_index!($column);

            writeln!(
                out,
                "pub static ref {}{}: [f32; {}] = [",
                name,
                $suffix,
                rows.len()
            )
            .unwrap();
            for row in rows.iter() {
                let row = row.as_ref().unwrap();
                writeln!(out, "    {},", floatify(&row[i_col])).unwrap();
            }
            writeln!(out, "];\n").unwrap();
        };
    }

    macro_rules! write_player_input_array {
        ($column_prefix:expr, $suffix:expr) => {
            let i_throttle = column_index!(concat!($column_prefix, "_throttle"));
            let i_steer = column_index!(concat!($column_prefix, "_steer"));
            let i_pitch = column_index!(concat!($column_prefix, "_pitch"));
            let i_yaw = column_index!(concat!($column_prefix, "_yaw"));
            let i_roll = column_index!(concat!($column_prefix, "_roll"));
            let i_jump = column_index!(concat!($column_prefix, "_jump"));
            let i_boost = column_index!(concat!($column_prefix, "_boost"));
            let i_handbrake = column_index!(concat!($column_prefix, "_handbrake"));

            writeln!(
                out,
                "pub static ref {}{}: [PlayerInput; {}] = [",
                name,
                $suffix,
                rows.len()
            )
            .unwrap();
            for row in rows.iter() {
                let row = row.as_ref().unwrap();
                writeln!(out, "    PlayerInput {{").unwrap();
                writeln!(out, "        Throttle: {},", floatify(&row[i_throttle])).unwrap();
                writeln!(out, "        Steer: {},", floatify(&row[i_steer])).unwrap();
                writeln!(out, "        Pitch: {},", floatify(&row[i_pitch])).unwrap();
                writeln!(out, "        Yaw: {},", floatify(&row[i_yaw])).unwrap();
                writeln!(out, "        Roll: {},", floatify(&row[i_roll])).unwrap();
                writeln!(out, "        Jump: {},", &row[i_jump]);
                writeln!(out, "        Boost: {},", &row[i_boost]);
                writeln!(out, "        Handbrake: {},", &row[i_handbrake]);
                writeln!(out, "    }},").unwrap();
            }
            writeln!(out, "];\n").unwrap();
        };
    }

    macro_rules! write_rigid_body_state_array {
        ($column_prefix:expr, $suffix:expr) => {
            let i_loc_x = column_index!(concat!($column_prefix, "_loc_x"));
            let i_loc_y = column_index!(concat!($column_prefix, "_loc_y"));
            let i_loc_z = column_index!(concat!($column_prefix, "_loc_z"));
            let i_rot_x = column_index!(concat!($column_prefix, "_rot_x"));
            let i_rot_y = column_index!(concat!($column_prefix, "_rot_y"));
            let i_rot_z = column_index!(concat!($column_prefix, "_rot_z"));
            let i_rot_w = column_index!(concat!($column_prefix, "_rot_w"));
            let i_vel_x = column_index!(concat!($column_prefix, "_vel_x"));
            let i_vel_y = column_index!(concat!($column_prefix, "_vel_y"));
            let i_vel_z = column_index!(concat!($column_prefix, "_vel_z"));
            let i_ang_vel_x = column_index!(concat!($column_prefix, "_ang_x"));
            let i_ang_vel_y = column_index!(concat!($column_prefix, "_ang_y"));
            let i_ang_vel_z = column_index!(concat!($column_prefix, "_ang_z"));

            writeln!(out, "pub static ref {}{}: [RecordingRigidBodyState; {}] = [", name, $suffix, rows.len()).unwrap();
            for row in rows.iter() {
                let row = row.as_ref().unwrap();
                writeln!(out, "    RecordingRigidBodyState {{").unwrap();
                writeln!(
                    out,
                    "        loc: Point3::new({x}, {y}, {z}),",
                    x = floatify(&row[i_loc_x]),
                    y = floatify(&row[i_loc_y]),
                    z = floatify(&row[i_loc_z]),
                )
                .unwrap();
                writeln!(
                    out,
                    "        rot: UnitQuaternion::from_quaternion(Quaternion::new({w}, {x}, {y}, {z})),",
                    w = floatify(&row[i_rot_w]),
                    x = floatify(&row[i_rot_x]),
                    y = floatify(&row[i_rot_y]),
                    z = floatify(&row[i_rot_z]),
                )
                .unwrap();
                writeln!(
                    out,
                    "        vel: Vector3::new({x}, {y}, {z}),",
                    x = floatify(&row[i_vel_x]),
                    y = floatify(&row[i_vel_y]),
                    z = floatify(&row[i_vel_z]),
                )
                .unwrap();
                writeln!(
                    out,
                    "        ang_vel: Vector3::new({x}, {y}, {z}),",
                    x = floatify(&row[i_ang_vel_x]),
                    y = floatify(&row[i_ang_vel_y]),
                    z = floatify(&row[i_ang_vel_z]),
                )
                .unwrap();
                writeln!(out, "    }},").unwrap();
            }
            writeln!(out, "];\n").unwrap();
        };
    }

    macro_rules! write_one_v_one {
        () => {
            writeln!(
                out,
                "pub static ref {}: OneVOneScenario<'static> = OneVOneScenario {{",
                name,
            )
            .unwrap();
            writeln!(out, "    times: &{}_TIME[..],", name).unwrap();
            writeln!(out, "    ball_states: &{}_BALL[..],", name).unwrap();
            writeln!(
                out,
                "    car_initial_state: {}_PLAYER0_STATE[0].clone(),",
                name
            )
            .unwrap();
            writeln!(out, "    enemy_inputs: &{}_PLAYER1_INPUT[..],", name).unwrap();
            writeln!(out, "    enemy_states: &{}_PLAYER1_STATE[..],", name).unwrap();
            writeln!(out, "}};").unwrap();
        };
    }

    write_float_array!("time", "_TIME");
    write_rigid_body_state_array!("ball", "_BALL");
    write_player_input_array!("player0", "_PLAYER0_INPUT");
    write_rigid_body_state_array!("player0", "_PLAYER0_STATE");
    write_player_input_array!("player1", "_PLAYER1_INPUT");
    write_rigid_body_state_array!("player1", "_PLAYER1_STATE");
    write_one_v_one!();
}

fn floatify(s: impl Into<String>) -> String {
    let s = s.into();
    if s.contains(".") {
        s
    } else {
        format!("{}.0", s)
    }
}
