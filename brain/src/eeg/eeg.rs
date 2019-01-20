use crate::eeg::{color, window::Window};
use common::{prelude::*, rl, Angle, Distance, PrettyPrint, Time};
use graphics::types::Color;
use nalgebra::{Point2, Point3, Rotation3};
use std::{collections::HashSet, mem};

pub struct EEG {
    log_to_stdout: bool,
    window: Option<Window>,
    current_packet_time: f32,
    draw_list: DrawList,
    pub events: Option<HashSet<Event>>,
    // I added quick-chat here only for convenience before a tournament, but it should really be
    // somewhere else…
    pub quick_chat: Option<rlbot::flat::QuickChatSelection>,
}

#[derive(Eq, PartialEq, Hash)]
pub enum Event {
    Defense,
    Retreat,
    HitToOwnCorner,
    PushFromLeftToRight,
    PushFromRightToLeft,
    RetreatingSave,
    Offense,
    TepidHitTowardEnemyGoal,
    TepidHitBlockAngleToGoal,
    TepidHitAwayFromOwnGoal,
    PanicDefense,
    WallHitFinishedWithoutJump,
    WallHitNotFacingTarget,
}

impl EEG {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        EEG {
            log_to_stdout: false,
            window: None,
            current_packet_time: 0.0,
            draw_list: DrawList::new(),
            events: None,
            quick_chat: None,
        }
    }

    pub fn log_to_stdout(&mut self) {
        self.log_to_stdout = true;
    }

    pub fn show_window(&mut self) {
        self.window = Some(Window::new());
    }

    pub fn track_events(&mut self) {
        self.events = Some(HashSet::new());
    }
}

impl EEG {
    /// Call this at the start of each frame.
    pub fn begin(&mut self, packet: &rlbot::ffi::LiveDataPacket) {
        self.current_packet_time = packet.GameInfo.TimeSeconds;
        assert!(self.draw_list.drawables.is_empty());
        self.quick_chat = None;
    }

    /// Call this at the end of each frame.
    pub fn show(&mut self, packet: &rlbot::ffi::LiveDataPacket) {
        let drawables = mem::replace(&mut self.draw_list.drawables, Vec::new());
        if let Some(window) = &self.window {
            window.draw(packet.clone(), drawables);
        }
    }

    pub fn quick_chat(&mut self, selection: rlbot::flat::QuickChatSelection) {
        self.quick_chat = Some(selection);
    }

    pub fn draw(&mut self, drawable: Drawable) {
        self.draw_list.draw(drawable);
    }

    pub fn print_value(&mut self, label: &str, value: impl PrettyPrint) {
        self.draw_list.print_value(label, value);
    }

    pub fn print_time(&mut self, label: &str, time: f32) {
        self.draw_list.print_value(label, Time(time));
    }

    pub fn print_angle(&mut self, label: &str, angle: f32) {
        self.draw_list.print_value(label, Angle(angle));
    }

    pub fn print_distance(&mut self, label: &str, distance: f32) {
        self.draw_list.print_value(label, Distance(distance));
    }

    pub fn log(&mut self, tag: &str, message: impl Into<String>) {
        if !self.log_to_stdout {
            return;
        }
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
