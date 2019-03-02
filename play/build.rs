use std::{env, fmt::Write, fs, path::PathBuf};

fn main() {
    let build_date = chrono::Local::now().format("%Y-%m-%d").to_string();

    let mut data = String::new();
    writeln!(
        &mut data,
        "pub const BUILD_DATE: &str = \"{}\";",
        build_date,
    )
    .unwrap();

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out_dir.join("built.rs"), data).unwrap();
}
