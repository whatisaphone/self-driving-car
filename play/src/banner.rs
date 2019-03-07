use crate::built;
use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
};

pub struct Banner {
    drain_bad_packets: i32,
    start_time: Option<f32>,
}

impl Banner {
    pub fn new() -> Self {
        Self {
            drain_bad_packets: 0,
            start_time: None,
        }
    }

    pub fn run(&mut self, rlbot: &rlbot::RLBot, packet: &common::halfway_house::LiveDataPacket) {
        if self.drain_bad_packets < 10 {
            self.drain_bad_packets += 1;
            return;
        }

        let now = packet.GameInfo.TimeSeconds;
        let start_time = *self.start_time.get_or_insert(now);
        let elapsed = now - start_time;

        let id = {
            let mut hasher = DefaultHasher::new();
            "self-driving car banner".hash(&mut hasher);
            hasher.finish()
        };
        let mut group = rlbot.begin_render_group(id as i32);

        if elapsed < 3.0 {
            let white = group.color_rgb(255, 255, 255);
            let black = group.color_rgb(0, 0, 0);

            let lines = &["Self-driving car", &format!("built {}", built::BUILD_DATE)];
            for (i, line) in lines.iter().enumerate() {
                shadowed(
                    &mut group,
                    (25.0, (25 + i * 50) as f32),
                    (4, 4),
                    line,
                    white,
                    black,
                );
            }
        } else if elapsed < 6.0 {
            let a = (255 - ((elapsed - 4.0) * 255.0) as i32).max(0).min(255) as u8;
            let white = group.color_argb(a, 255, 255, 255);
            let black = group.color_argb(a, 0, 0, 0);
            shadowed(&mut group, (25.0, 25.0), (4, 4), "glhf!", white, black);
        }

        // Ignore errors
        let _ = group.render();
    }
}

fn shadowed(
    group: &mut rlbot::RenderGroup<'_>,
    (x, y): (f32, f32),
    (scale_x, scale_y): (i32, i32),
    text: impl AsRef<str>,
    color1: rlbot::Color<'_>,
    color2: rlbot::Color<'_>,
) {
    group.draw_string_2d(
        (x + scale_x as f32, y + scale_y as f32),
        (scale_x, scale_y),
        &text,
        color2,
    );
    group.draw_string_2d((x, y), (scale_x, scale_y), &text, color1);
}
