use crate::strategy::Context;
use common::prelude::*;
use nalgebra::Point3;

pub struct PodiumQuickChat {
    start_time: Option<f32>,
    gg: bool,
    nice_moves: bool,
}

impl PodiumQuickChat {
    pub fn new() -> Self {
        Self {
            start_time: None,
            gg: false,
            nice_moves: false,
        }
    }

    pub fn run(&mut self, ctx: &mut Context<'_>) {
        let start_time = *self
            .start_time
            .get_or_insert(ctx.packet.GameInfo.TimeSeconds);
        let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;

        if !self.gg && elapsed >= 1.3 {
            ctx.quick_chat(1.0, &[rlbot::flat::QuickChatSelection::PostGame_Gg]);
            self.gg = true;
        }

        if !self.nice_moves && elapsed >= 2.0 {
            ctx.quick_chat(1.0, &[rlbot::flat::QuickChatSelection::PostGame_NiceMoves]);
            self.nice_moves = true;
        }
    }
}

pub struct PodiumTimeTracker {
    start_loc: Option<Point3<f32>>,
    mobile_time: Option<f32>,
}

impl PodiumTimeTracker {
    pub fn new() -> Self {
        Self {
            start_loc: None,
            mobile_time: None,
        }
    }

    /// Returns how long we have been in podium phase. The game freezes us in
    /// place before the podium phase starts. Once it begins, we can move again
    /// and we have 5 sweet seconds to celebrate.
    pub fn update(&mut self, ctx: &mut Context<'_>) -> f32 {
        let now = ctx.packet.GameInfo.TimeSeconds;
        if let Some(mobile_time) = self.mobile_time {
            let elapsed = now - mobile_time;
            ctx.eeg.print_time("podium_elapsed", elapsed);
            return elapsed;
        }

        // If the car has moved, that is our signal that podium phase has begun.
        let me_loc = ctx.me().Physics.loc();
        let start_loc = *self.start_loc.get_or_insert_with(|| me_loc);
        if (me_loc - start_loc).norm() >= 1.0 {
            self.mobile_time = Some(now);
        }

        0.0
    }
}
