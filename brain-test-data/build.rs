#![warn(future_incompatible, rust_2018_compatibility, rust_2018_idioms, unused)]
#![cfg_attr(feature = "strict", deny(warnings))]
#![deny(clippy::all)]

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

    writeln!(out, "use crate::models::OneVOneScenario;").unwrap();
    writeln!(out, "use lazy_static::lazy_static;").unwrap();
    writeln!(out).unwrap();

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
}

fn translate_csv(name: &str, csv: &mut csv::Reader<impl Read>, out: &mut impl Write) {
    let rows: Vec<_> = csv.records().map(Result::unwrap).collect();
    let headers = csv.headers().unwrap();

    macro_rules! column_index {
        ($header:expr) => {
            headers.iter().position(|h| h == $header).unwrap()
        };
    }

    macro_rules! col {
        ($header:expr) => {{
            let index = column_index!($header);
            rows.iter().map(move |ref row| &row[index])
        }};
    }

    macro_rules! write_array {
        ($name:expr, $type:expr, $items:expr $(,)*) => {
            let items = $items;
            writeln!(out, "    pub const {}: &[{}] = &[", $name, $type).unwrap();
            for x in items {
                writeln!(out, "        {},", x).unwrap();
            }
            writeln!(out, "    ];\n").unwrap();
        };
    }

    macro_rules! write_player_input_array {
        ($name:expr, $column_prefix:expr) => {
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
                "    pub const {}: &[rlbot::ffi::PlayerInput] = &[",
                $name,
            )
            .unwrap();
            for row in rows.iter() {
                writeln!(out, "        rlbot::ffi::PlayerInput {{").unwrap();
                writeln!(out, "            Throttle: {},", floatify(&row[i_throttle])).unwrap();
                writeln!(out, "            Steer: {},", floatify(&row[i_steer])).unwrap();
                writeln!(out, "            Pitch: {},", floatify(&row[i_pitch])).unwrap();
                writeln!(out, "            Yaw: {},", floatify(&row[i_yaw])).unwrap();
                writeln!(out, "            Roll: {},", floatify(&row[i_roll])).unwrap();
                writeln!(out, "            Jump: {},", &row[i_jump]).unwrap();
                writeln!(out, "            Boost: {},", &row[i_boost]).unwrap();
                writeln!(out, "            Handbrake: {},", &row[i_handbrake]).unwrap();
                writeln!(out, "        }},").unwrap();
            }
            writeln!(out, "    ];\n").unwrap();
        };
    }

    macro_rules! write_rigid_body_state_array {
        ($name:expr, $column_prefix:expr) => {
            write_array!(format!("{}_LOC_X", $name), "f32", col!(format!("{}_loc_x", $column_prefix)).map(floatify));
            write_array!(format!("{}_LOC_Y", $name), "f32", col!(format!("{}_loc_y", $column_prefix)).map(floatify));
            write_array!(format!("{}_LOC_Z", $name), "f32", col!(format!("{}_loc_z", $column_prefix)).map(floatify));
            write_array!(format!("{}_ROT_X", $name), "f32", col!(format!("{}_rot_x", $column_prefix)).map(floatify));
            write_array!(format!("{}_ROT_Y", $name), "f32", col!(format!("{}_rot_y", $column_prefix)).map(floatify));
            write_array!(format!("{}_ROT_Z", $name), "f32", col!(format!("{}_rot_z", $column_prefix)).map(floatify));
            write_array!(format!("{}_ROT_W", $name), "f32", col!(format!("{}_rot_w", $column_prefix)).map(floatify));
            write_array!(format!("{}_VEL_X", $name), "f32", col!(format!("{}_vel_x", $column_prefix)).map(floatify));
            write_array!(format!("{}_VEL_Y", $name), "f32", col!(format!("{}_vel_y", $column_prefix)).map(floatify));
            write_array!(format!("{}_VEL_Z", $name), "f32", col!(format!("{}_vel_z", $column_prefix)).map(floatify));
            write_array!(format!("{}_ANG_VEL_X", $name), "f32", col!(format!("{}_ang_x", $column_prefix)).map(floatify));
            write_array!(format!("{}_ANG_VEL_Y", $name), "f32", col!(format!("{}_ang_y", $column_prefix)).map(floatify));
            write_array!(format!("{}_ANG_VEL_Z", $name), "f32", col!(format!("{}_ang_z", $column_prefix)).map(floatify));

            writeln!(out, "    lazy_static! {{").unwrap();
            writeln!(out, "        pub static ref {name}_LOC: Vec<Point3<f32>> = {name}_LOC_X.iter().zip({name}_LOC_Y.iter()).zip({name}_LOC_Z.iter())", name = $name).unwrap();
            writeln!(out, "            .map(|((&x, &y), &z)| Point3::new(x, y, z)).collect();").unwrap();
            writeln!(out).unwrap();
            writeln!(out, "        pub static ref {name}_ROT: Vec<UnitQuaternion<f32>> = {name}_ROT_X.iter().zip({name}_ROT_Y.iter()).zip({name}_ROT_Z.iter()).zip({name}_ROT_W.iter())", name = $name).unwrap();
            writeln!(out, "            .map(|(((&x, &y), &z), &w)| UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))).collect();").unwrap();
            writeln!(out).unwrap();
            writeln!(out, "        pub static ref {name}_VEL: Vec<Vector3<f32>> = {name}_VEL_X.iter().zip({name}_VEL_Y.iter()).zip({name}_VEL_Z.iter())", name = $name).unwrap();
            writeln!(out, "            .map(|((&x, &y), &z)| Vector3::new(x, y, z)).collect();").unwrap();
            writeln!(out).unwrap();
            writeln!(out, "        pub static ref {name}_ANG_VEL: Vec<Vector3<f32>> = {name}_ANG_VEL_X.iter().zip({name}_ANG_VEL_Y.iter()).zip({name}_ANG_VEL_Z.iter())", name = $name).unwrap();
            writeln!(out, "            .map(|((&x, &y), &z)| Vector3::new(x, y, z)).collect();").unwrap();
            writeln!(out).unwrap();
            writeln!(out, "        pub static ref {name}: Vec<RecordingRigidBodyState> = {name}_LOC.iter().zip({name}_ROT.iter()).zip({name}_VEL.iter()).zip({name}_ANG_VEL.iter())", name = $name).unwrap();
            writeln!(out, "            .map(|(((&loc, &rot), &vel), &ang_vel)| RecordingRigidBodyState {{").unwrap();
            writeln!(out, "                loc, rot, vel, ang_vel,").unwrap();
            writeln!(out, "            }})").unwrap();
            writeln!(out, "            .collect();").unwrap();
            writeln!(out, "    }}\n").unwrap(); // lazy_static!
        };
    }

    macro_rules! write_one_v_one {
        () => {
            writeln!(out, "lazy_static! {{").unwrap();
            writeln!(
                out,
                "    pub static ref {}: OneVOneScenario<'static> = OneVOneScenario {{",
                name.to_ascii_uppercase(),
            )
            .unwrap();
            writeln!(out, "        times: &{}::TIME,", name).unwrap();
            writeln!(out, "        ball_states: &{}::BALL,", name).unwrap();
            writeln!(
                out,
                "        car_initial_state: {}::PLAYER0_STATE[0].clone(),",
                name,
            )
            .unwrap();
            writeln!(out, "        enemy_inputs: &{}::PLAYER1_INPUT,", name).unwrap();
            writeln!(out, "        enemy_states: &{}::PLAYER1_STATE,", name).unwrap();
            writeln!(out, "    }};").unwrap(); // OneVOneScenario
            writeln!(out, "}}\n").unwrap(); // lazy_static!
        };
    }

    writeln!(out, "mod {} {{", name).unwrap();
    writeln!(out, "    use collect::RecordingRigidBodyState;").unwrap();
    writeln!(out, "    use lazy_static::lazy_static;").unwrap();
    writeln!(
        out,
        "    use nalgebra::{{Point3, Quaternion, UnitQuaternion, Vector3}};",
    )
    .unwrap();
    writeln!(out).unwrap();
    write_array!("TIME", "f32", col!("time").map(floatify));
    write_rigid_body_state_array!("BALL", "ball");
    write_player_input_array!("PLAYER0_INPUT", "player0");
    write_rigid_body_state_array!("PLAYER0_STATE", "player0");
    write_player_input_array!("PLAYER1_INPUT", "player1");
    write_rigid_body_state_array!("PLAYER1_STATE", "player1");
    writeln!(out, "}}\n").unwrap(); // mod
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
