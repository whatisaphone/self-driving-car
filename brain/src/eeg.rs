// This file is a hot mess, don't look at it please :)

use collect::ExtendRotation3;
use crossbeam_channel;
use graphics::types::Color;
use graphics::Transformed;
use nalgebra::{Rotation3, Vector3};
use piston_window::{
    clear, ellipse, rectangle, text, AdvancedWindow, Ellipse, Glyphs, OpenGL, PistonWindow,
    Position, Rectangle, TextureSettings, WindowSettings,
};
use rlbot;
use std::mem;
use std::path::PathBuf;
use std::thread;

pub struct EEG {
    tx: Option<crossbeam_channel::Sender<ThreadMessage>>,
    join_handle: Option<thread::JoinHandle<()>>,
    draw_list: Vec<Drawable>,
}

impl EEG {
    pub fn new() -> EEG {
        let (tx, rx) = crossbeam_channel::unbounded();
        let join_handle = thread::spawn(|| thread(rx));
        EEG {
            tx: Some(tx),
            join_handle: Some(join_handle),
            draw_list: Vec::new(),
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
        self.draw_list.push(drawable);
    }

    pub fn show(&mut self, packet: &rlbot::LiveDataPacket) {
        let draw_list = mem::replace(&mut self.draw_list, Vec::new());
        self.tx.as_ref().unwrap().send(ThreadMessage::Draw(
            packet.clone(),
            draw_list.into_boxed_slice(),
        ));
    }
}

pub enum Drawable {
    GhostBall(Vector3<f32>),
    GhostCar(Vector3<f32>, Rotation3<f32>),
    Print(String, Color),
}

impl Drawable {
    pub fn print(text: impl Into<String>, color: Color) -> Drawable {
        Drawable::Print(text.into(), color)
    }
}

pub mod color {
    pub const TRANSPARENT: [f32; 4] = [0.0, 0.0, 0.0, 0.0];
    pub const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
    pub const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
    pub const PITCH: [f32; 4] = [0.0, 0.2, 0.0, 1.0];
    pub const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
    pub const ORANGE: [f32; 4] = [1.0, 0.5, 0.0, 1.0];
    pub const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
    pub const YELLOW: [f32; 4] = [1.0, 1.0, 0.0, 1.0];
    pub const BLUE: [f32; 4] = [0.5, 0.5, 1.0, 1.0];
}

enum ThreadMessage {
    Draw(rlbot::LiveDataPacket, Box<[Drawable]>),
}

fn thread(rx: crossbeam_channel::Receiver<ThreadMessage>) {
    let mut window: PistonWindow = WindowSettings::new("Formula nOne", (640, 640))
        .opengl(OpenGL::V3_2)
        .build()
        .unwrap();
    window.set_position(Position { x: 1920, y: 10 });

    let path = PathBuf::from(r"C:\Windows\Fonts\calibri.ttf");
    let factory = window.factory.clone();
    let mut glyphs = Glyphs::new(path, factory, TextureSettings::new()).unwrap();

    loop {
        let event = match window.next() {
            Some(e) => e,
            None => break,
        };

        match rx.recv() {
            None => break, // The channel was closed, so exit the thread.
            Some(ThreadMessage::Draw(packet, drawables)) => {
                window.draw_2d(&event, |c, g| {
                    const SCALE: f64 = 0.05;
                    const OUTLINE_RADIUS: f64 = 0.5 / SCALE;
                    let car_rect = rectangle::rectangle_by_corners(-100.0, -50.0, 100.0, 50.0);
                    let ball_rect = ellipse::circle(0.0, 0.0, 92.0);

                    clear(color::BLACK, g);

                    let transform = c.transform.scale(SCALE, SCALE).trans(4200.0, 6200.0);
                    rectangle(
                        color::PITCH,
                        rectangle::rectangle_by_corners(-4000.0, -5000.0, 4000.0, 5000.0),
                        transform,
                        g,
                    );

                    for car in packet.cars() {
                        rectangle(
                            if car.Team == 0 {
                                color::BLUE
                            } else {
                                color::ORANGE
                            },
                            car_rect,
                            transform
                                .trans(car.Physics.Location.X.into(), car.Physics.Location.Y.into())
                                .rot_rad(car.Physics.Rotation.Yaw.into()),
                            g,
                        );
                    }

                    ellipse(
                        color::WHITE,
                        ball_rect,
                        transform.trans(
                            packet.GameBall.Physics.Location.X.into(),
                            packet.GameBall.Physics.Location.Y.into(),
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
                                    transform.trans(loc.x.into(), loc.y.into()),
                                    g,
                                );
                            }
                            Drawable::GhostCar(loc, rot) => {
                                Rectangle::new_border(color::WHITE, OUTLINE_RADIUS).draw(
                                    car_rect,
                                    &Default::default(),
                                    transform
                                        .trans(loc.x.into(), loc.y.into())
                                        .rot_rad(rot.yaw().into()),
                                    g,
                                );
                            }
                            Drawable::Print(txt, color) => {
                                prints.push((txt, *color));
                            }
                        }
                    }

                    let mut y = 20.0;
                    for (txt, color) in prints.into_iter() {
                        text(color, 16, &txt, &mut glyphs, c.transform.trans(420.0, y), g).unwrap();
                        y += 20.0;
                    }
                });
            }
        }
    }
}
