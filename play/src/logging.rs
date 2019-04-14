use env_logger::fmt::{Color, Formatter};
use lazy_static::lazy_static;
use log::{Level, Record};
use std::{
    io::{self, Write},
    sync::Mutex,
};

pub struct State {
    pub game_time: Option<f32>,
}

lazy_static! {
    pub static ref STATE: Mutex<State> = Mutex::new(State { game_time: None });
}

// env_logger does not expose these bits of the default formatter for
// customization, but at least we have copy and paste :^)
pub fn format(buf: &mut Formatter, record: &Record<'_>) -> io::Result<()> {
    let level = record.level();
    let mut level_style = buf.style();
    match level {
        Level::Trace => level_style.set_color(Color::White),
        Level::Debug => level_style.set_color(Color::Blue),
        Level::Info => level_style.set_color(Color::Green),
        Level::Warn => level_style.set_color(Color::Yellow),
        Level::Error => level_style.set_color(Color::Red).set_bold(true),
    };
    write!(buf, "{:>5} ", level_style.value(level))?;

    let ts = match STATE.lock().unwrap().game_time {
        Some(game_time) => format!("{}", game_time),
        None => format!("{}", buf.timestamp()),
    };
    write!(buf, "{}: ", ts)?;

    let mut module_style = buf.style();
    module_style.set_color(Color::Yellow);
    if let Some(module_path) = record.module_path() {
        write!(buf, "{}: ", module_style.value(module_path))?;
    }

    writeln!(buf, "{}", record.args())
}
