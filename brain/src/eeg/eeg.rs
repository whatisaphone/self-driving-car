use crate::eeg::{color, window::Window};
use common::{prelude::*, rl, PrettyPrint};
use graphics::types::Color;
use nalgebra::{Point2, Point3, Rotation3};
use std::{collections::HashSet, mem};

pub struct EEG {
    window: Window,
    current_packet_time: f32,
    draw_list: DrawList,
    pub events: Option<HashSet<Event>>,
}

#[derive(Eq, PartialEq, Hash)]
pub enum Event {
    Defense,
    HitToOwnCorner,
    PushFromLeftToRight,
    PushFromRightToLeft,
    Offense,
    BounceShot,
    TepidHitTowardEnemyGoal,
    TepidHitAwayFromOwnGoal,
    PanicDefense,
}

impl EEG {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        EEG {
            window: Window::new(),
            current_packet_time: 0.0,
            draw_list: DrawList::new(),
            events: None,
        }
    }

    pub fn with_tracked_events(mut self) -> Self {
        self.events = Some(HashSet::new());
        self
    }
}

impl EEG {
    pub fn draw(&mut self, drawable: Drawable) {
        self.draw_list.draw(drawable);
    }

    pub fn print_value(&mut self, label: &str, value: impl PrettyPrint) {
        self.draw_list.print_value(label, value);
    }

    pub fn print_time(&mut self, label: &str, time: f32) {
        self.draw_list.print_time(label, time);
    }

    pub fn begin(&mut self, packet: &rlbot::ffi::LiveDataPacket) {
        self.current_packet_time = packet.GameInfo.TimeSeconds;
    }

    pub fn log(&mut self, tag: &str, message: impl Into<String>) {
        println!(
            "{:>8.3} [{}] {}",
            self.current_packet_time,
            tag,
            message.into()
        );
    }

    pub fn log_pretty(&mut self, tag: &str, name: &str, value: impl PrettyPrint) {
        self.log(tag, format!("{} = {}", name, value.pretty()))
    }

    pub fn track(&mut self, event: Event) {
        if let Some(ref mut events) = self.events {
            events.insert(event);
        }
    }

    pub fn show(&mut self, packet: &rlbot::ffi::LiveDataPacket) {
        let drawables = mem::replace(&mut self.draw_list.drawables, Vec::new());
        self.window.draw(packet.clone(), drawables);
    }
}

pub struct DrawList {
    pub drawables: Vec<Drawable>,
}

impl DrawList {
    pub fn new() -> Self {
        Self {
            drawables: Vec::new(),
        }
    }

    fn draw(&mut self, drawable: Drawable) {
        self.drawables.push(drawable);
    }

    fn print_value(&mut self, label: &str, value: impl PrettyPrint) {
        self.draw(Drawable::print(
            format!("{}: {}", label, value.pretty()),
            color::GREEN,
        ));
    }

    fn print_time(&mut self, label: &str, time: f32) {
        self.draw(Drawable::print(
            format!("{}: {:.2}", label, time),
            color::GREEN,
        ));
    }
}

#[derive(Clone)]
pub enum Drawable {
    GhostBall(Point3<f32>, Color),
    GhostCar(Point3<f32>, Rotation3<f32>),
    Crosshair(Point2<f32>),
    Line(Point2<f32>, Point2<f32>, Color),
    Arc(Point2<f32>, f32, f32, f32, Color),
    Print(String, Color),
}

impl Drawable {
    pub fn ghost_ball(loc: Point3<f32>) -> Self {
        Drawable::GhostBall(loc, color::WHITE)
    }

    pub fn ghost_car_ground(loc: Point2<f32>, rot: Rotation3<f32>) -> Self {
        Drawable::GhostCar(loc.to_3d(rl::OCTANE_NEUTRAL_Z), rot)
    }

    pub fn print(text: impl Into<String>, color: Color) -> Self {
        Drawable::Print(text.into(), color)
    }
}
