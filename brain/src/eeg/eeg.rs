// This file is a hot mess, don't look at it please :)

use common::{prelude::*, rl, PrettyPrint};
use crossbeam_channel;
use graphics::{types::Color, Transformed};
use nalgebra::{Point2, Point3, Rotation3};
use piston_window::{
    circle_arc, clear, ellipse, line, rectangle, text, AdvancedWindow, Ellipse, Glyphs, OpenGL,
    PistonWindow, Position, Rectangle, TextureSettings, WindowSettings,
};
use rlbot;
use std::{collections::VecDeque, mem, path::PathBuf, thread};
use strategy::Team;

pub struct EEG {
    tx: Option<crossbeam_channel::Sender<ThreadMessage>>,
    join_handle: Option<thread::JoinHandle<()>>,
    current_packet_time: f32,
    draw_list: DrawList,
    pub log: VecDeque<String>,
}

impl EEG {
    pub fn new() -> EEG {
        let (tx, rx) = crossbeam_channel::unbounded();
        let join_handle = thread::spawn(|| thread(rx));
        EEG {
            tx: Some(tx),
            join_handle: Some(join_handle),
            current_packet_time: 0.0,
            draw_list: DrawList::new(),
            log: VecDeque::new(),
        }
    }
}

impl Drop for EEG {
    fn drop(&mut self) {
        drop(self.tx.take().unwrap());
        self.join_handle.take().unwrap().join().unwrap();
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

    pub fn log(&mut self, message: impl Into<String>) {
        let message = message.into();

        println!("{:>8.3} {}", self.current_packet_time, message);

        self.log.push_back(message);

        // Limit RAM usage
        while self.log.len() > 250 {
            self.log.pop_front();
        }
    }

    pub fn show(&mut self, packet: &rlbot::ffi::LiveDataPacket) {
        let drawables = mem::replace(&mut self.draw_list.drawables, Vec::new());
        self.tx
            .as_ref()
            .unwrap()
            .send(ThreadMessage::Draw(packet.clone(), drawables));
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
    GhostBall(Point3<f32>),
    GhostBall2(Point3<f32>, Color),
    GhostCar(Point3<f32>, Rotation3<f32>),
    Crosshair(Point2<f32>),
    Line(Point2<f32>, Point2<f32>, Color),
    Arc(Point2<f32>, f32, f32, f32, Color),
    Print(String, Color),
}

impl Drawable {
    pub fn ghost_car_ground(loc: Point2<f32>, rot: Rotation3<f32>) -> Drawable {
        Drawable::GhostCar(loc.to_3d(rl::OCTANE_NEUTRAL_Z), rot)
    }

    pub fn print(text: impl Into<String>, color: Color) -> Drawable {
        Drawable::Print(text.into(), color)
    }
}

#[allow(dead_code)]
pub mod color {
    use graphics::types::Color;
    use strategy::Team;

    pub const TRANSPARENT: Color = [0.0, 0.0, 0.0, 0.0];
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
}

enum ThreadMessage {
    Draw(rlbot::ffi::LiveDataPacket, Vec<Drawable>),
}

fn thread(rx: crossbeam_channel::Receiver<ThreadMessage>) {
    let mut window: PistonWindow = WindowSettings::new("Formula nOne", (660, 640))
        .opengl(OpenGL::V3_2)
        .build()
        .unwrap();
    window.set_position(Position { x: 1912, y: 190 });

    let path = PathBuf::from(r"C:\Windows\Fonts\calibri.ttf");
    let factory = window.factory.clone();
    let mut glyphs = Glyphs::new(path, factory, TextureSettings::new()).unwrap();

    // Try to make the window start drawing more quickly.
    for _ in 0..10 {
        let e = window.next().unwrap();
        window.draw_2d(&e, |_c, g| clear(color::BLACK, g));
    }

    loop {
        let event = match window.next() {
            Some(e) => e,
            None => break,
        };

        let mut message = rx.recv();
        // Only process the latest message
        while let Some(m) = rx.try_recv() {
            message = Some(m);
        }

        match message {
            None => break, // The channel was closed, so exit the thread.
            Some(ThreadMessage::Draw(packet, drawables)) => {
                window.draw_2d(&event, |c, g| {
                    const GOAL_DEPTH: f64 = 900.0; // This was just estimated visually.
                    let car_rect = rectangle::rectangle_by_corners(-100.0, -50.0, 100.0, 50.0);
                    let ball_rect = ellipse::circle(0.0, 0.0, 92.0);

                    const SCALE: f64 = 0.05;
                    const OUTLINE_RADIUS: f64 = 0.5 / SCALE;

                    clear(color::BLACK, g);

                    let transform = c.transform.scale(SCALE, SCALE).trans(4200.0, 6200.0);

                    rectangle(
                        color::PITCH,
                        rectangle::rectangle_by_corners(
                            -rl::FIELD_MAX_X as f64,
                            -rl::FIELD_MAX_Y as f64,
                            rl::FIELD_MAX_X as f64,
                            rl::FIELD_MAX_Y as f64,
                        ),
                        transform,
                        g,
                    );
                    rectangle(
                        color::BLUE_DARK,
                        rectangle::rectangle_by_corners(
                            -rl::GOALPOST_X as f64,
                            -rl::FIELD_MAX_Y as f64,
                            rl::GOALPOST_X as f64,
                            -rl::FIELD_MAX_Y as f64 - GOAL_DEPTH,
                        ),
                        transform,
                        g,
                    );
                    rectangle(
                        color::ORANGE_DARK,
                        rectangle::rectangle_by_corners(
                            -rl::GOALPOST_X as f64,
                            rl::FIELD_MAX_Y as f64,
                            rl::GOALPOST_X as f64,
                            rl::FIELD_MAX_Y as f64 + GOAL_DEPTH,
                        ),
                        transform,
                        g,
                    );

                    for boost in &[
                        Point2::new(-3072.0, -4096.0),
                        Point2::new(3072.0, -4096.0),
                        Point2::new(-3584.0, 0.0),
                        Point2::new(3584.0, 0.0),
                        Point2::new(-3072.0, 4096.0),
                        Point2::new(3072.0, 4096.0),
                    ] {
                        ellipse(
                            color::YELLOW,
                            ellipse::circle(0.0, 0.0, 40.0),
                            transform.trans(boost.x as f64, boost.y as f64),
                            g,
                        );
                    }

                    for car in packet.cars() {
                        rectangle(
                            color::for_team(Team::from_ffi(car.Team)),
                            car_rect,
                            transform
                                .trans(car.Physics.Location.X as f64, car.Physics.Location.Y as f64)
                                .rot_rad(car.Physics.Rotation.Yaw as f64),
                            g,
                        );
                    }

                    ellipse(
                        color::WHITE,
                        ball_rect,
                        transform.trans(
                            packet.GameBall.Physics.Location.X as f64,
                            packet.GameBall.Physics.Location.Y as f64,
                        ),
                        g,
                    );

                    let mut prints = Vec::new();

                    for drawable in drawables.into_iter() {
                        match drawable {
                            Drawable::GhostBall(loc) => {
                                Ellipse::new_border(color::WHITE, OUTLINE_RADIUS).draw(
                                    ball_rect,
                                    &Default::default(),
                                    transform.trans(loc.x as f64, loc.y as f64),
                                    g,
                                );
                            }
                            Drawable::GhostBall2(loc, color) => {
                                Ellipse::new_border(color, OUTLINE_RADIUS).draw(
                                    ball_rect,
                                    &Default::default(),
                                    transform.trans(loc.x as f64, loc.y as f64),
                                    g,
                                );
                            }
                            Drawable::GhostCar(loc, rot) => {
                                Rectangle::new_border(color::WHITE, OUTLINE_RADIUS).draw(
                                    car_rect,
                                    &Default::default(),
                                    transform
                                        .trans(loc.x as f64, loc.y as f64)
                                        .rot_rad(rot.yaw() as f64),
                                    g,
                                );
                            }
                            Drawable::Crosshair(loc) => {
                                line(
                                    color::YELLOW,
                                    OUTLINE_RADIUS,
                                    [
                                        loc.x as f64 - 100.0,
                                        loc.y as f64 - 100.0,
                                        loc.x as f64 + 100.0,
                                        loc.y as f64 + 100.0,
                                    ],
                                    transform,
                                    g,
                                );
                                line(
                                    color::YELLOW,
                                    OUTLINE_RADIUS,
                                    [
                                        loc.x as f64 - 100.0,
                                        loc.y as f64 + 100.0,
                                        loc.x as f64 + 100.0,
                                        loc.y as f64 - 100.0,
                                    ],
                                    transform,
                                    g,
                                );
                            }
                            Drawable::Line(start, end, color) => {
                                let pts =
                                    [start.x as f64, start.y as f64, end.x as f64, end.y as f64];
                                line(color, OUTLINE_RADIUS, pts, transform, g);
                            }
                            Drawable::Arc(center, radius, start, end, color) => {
                                circle_arc(
                                    color,
                                    OUTLINE_RADIUS,
                                    start as f64,
                                    end as f64,
                                    rectangle::centered_square(
                                        center.x as f64,
                                        center.y as f64,
                                        radius as f64,
                                    ),
                                    transform,
                                    g,
                                );
                            }
                            Drawable::Print(txt, color) => {
                                prints.push((txt, color));
                            }
                        }
                    }

                    let mut y = 20.0;
                    for (txt, color) in prints.into_iter() {
                        text(color, 14, &txt, &mut glyphs, c.transform.trans(420.0, y), g).unwrap();
                        y += 20.0;
                    }
                });
            }
        }
    }
}
