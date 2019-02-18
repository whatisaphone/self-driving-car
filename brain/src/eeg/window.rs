// This file is a hot mess, don't look at it please :)

use crate::{
    eeg::{color, eeg::Drawable},
    strategy::Team,
};
use common::{prelude::*, rl};
use graphics::{
    circle_arc, clear, ellipse, line, rectangle, text, Ellipse, Rectangle, Transformed,
};
use nalgebra::Point2;
use piston_window::{
    AdvancedWindow, Glyphs, OpenGL, PistonWindow, Position, TextureSettings, WindowSettings,
};
use std::{path::PathBuf, thread};

pub struct Window {
    tx: Option<crossbeam_channel::Sender<ThreadMessage>>,
    join_handle: Option<thread::JoinHandle<()>>,
}

impl Window {
    pub fn new() -> Self {
        let (tx, rx) = crossbeam_channel::unbounded();
        let join_handle = thread::spawn(|| thread(rx));
        Self {
            tx: Some(tx),
            join_handle: Some(join_handle),
        }
    }

    pub fn draw(&self, packet: common::halfway_house::LiveDataPacket, drawables: Vec<Drawable>) {
        self.tx
            .as_ref()
            .unwrap()
            .send(ThreadMessage::Draw(packet, drawables));
    }
}

impl Drop for Window {
    fn drop(&mut self) {
        drop(self.tx.take().unwrap());
        self.join_handle.take().unwrap().join().unwrap();
    }
}

enum ThreadMessage {
    Draw(common::halfway_house::LiveDataPacket, Vec<Drawable>),
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

    while let Some(event) = window.next() {
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
                            f64::from(-rl::FIELD_MAX_X),
                            f64::from(-rl::FIELD_MAX_Y),
                            f64::from(rl::FIELD_MAX_X),
                            f64::from(rl::FIELD_MAX_Y),
                        ),
                        transform,
                        g,
                    );
                    rectangle(
                        color::BLUE_DARK,
                        rectangle::rectangle_by_corners(
                            f64::from(-rl::GOALPOST_X),
                            f64::from(-rl::FIELD_MAX_Y),
                            f64::from(rl::GOALPOST_X),
                            f64::from(-rl::FIELD_MAX_Y) - GOAL_DEPTH,
                        ),
                        transform,
                        g,
                    );
                    rectangle(
                        color::ORANGE_DARK,
                        rectangle::rectangle_by_corners(
                            f64::from(-rl::GOALPOST_X),
                            f64::from(rl::FIELD_MAX_Y),
                            f64::from(rl::GOALPOST_X),
                            f64::from(rl::FIELD_MAX_Y) + GOAL_DEPTH,
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
                            transform.trans(boost.x, boost.y),
                            g,
                        );
                    }

                    for car in packet.cars() {
                        rectangle(
                            color::for_team(Team::from_ffi(car.Team)),
                            car_rect,
                            transform
                                .trans(
                                    f64::from(car.Physics.Location.X),
                                    f64::from(car.Physics.Location.Y),
                                )
                                .rot_rad(f64::from(car.Physics.Rotation.Yaw)),
                            g,
                        );
                    }

                    ellipse(
                        color::WHITE,
                        ball_rect,
                        transform.trans(
                            f64::from(packet.GameBall.Physics.Location.X),
                            f64::from(packet.GameBall.Physics.Location.Y),
                        ),
                        g,
                    );

                    let mut prints = Vec::new();

                    for drawable in drawables.into_iter() {
                        match drawable {
                            Drawable::GhostBall(loc, color) => {
                                Ellipse::new_border(color, OUTLINE_RADIUS).draw(
                                    ball_rect,
                                    &Default::default(),
                                    transform.trans(f64::from(loc.x), f64::from(loc.y)),
                                    g,
                                );
                            }
                            Drawable::GhostCar(loc, rot) => {
                                Rectangle::new_border(color::WHITE, OUTLINE_RADIUS).draw(
                                    car_rect,
                                    &Default::default(),
                                    transform
                                        .trans(f64::from(loc.x), f64::from(loc.y))
                                        .rot_rad(f64::from(rot.yaw())),
                                    g,
                                );
                            }
                            Drawable::Crosshair(loc) => {
                                line(
                                    color::YELLOW,
                                    OUTLINE_RADIUS,
                                    [
                                        f64::from(loc.x) - 100.0,
                                        f64::from(loc.y) - 100.0,
                                        f64::from(loc.x) + 100.0,
                                        f64::from(loc.y) + 100.0,
                                    ],
                                    transform,
                                    g,
                                );
                                line(
                                    color::YELLOW,
                                    OUTLINE_RADIUS,
                                    [
                                        f64::from(loc.x) - 100.0,
                                        f64::from(loc.y) + 100.0,
                                        f64::from(loc.x) + 100.0,
                                        f64::from(loc.y) - 100.0,
                                    ],
                                    transform,
                                    g,
                                );
                            }
                            Drawable::Line(start, end, color) => {
                                let pts = [
                                    f64::from(start.x),
                                    f64::from(start.y),
                                    f64::from(end.x),
                                    f64::from(end.y),
                                ];
                                line(color, OUTLINE_RADIUS, pts, transform, g);
                            }
                            Drawable::Arc(center, radius, start, end, color) => {
                                circle_arc(
                                    color,
                                    OUTLINE_RADIUS,
                                    f64::from(start),
                                    f64::from(end),
                                    rectangle::centered_square(
                                        f64::from(center.x),
                                        f64::from(center.y),
                                        f64::from(radius),
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
