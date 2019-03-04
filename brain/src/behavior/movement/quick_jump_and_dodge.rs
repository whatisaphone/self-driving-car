use crate::{
    behavior::movement::Dodge,
    strategy::{Action, Behavior, Context, Priority},
};
use nalgebra::UnitComplex;
use nameof::name_of_type;

pub struct QuickJumpAndDodge {
    start_time: Option<f32>,
    dodge_time: f32,
    phase: Phase,
    dodge: Dodge,
}

#[derive(Eq, PartialEq)]
enum Phase {
    Jump,
    And,
    Dodge,
    FollowThrough,
    Finished,
}

impl QuickJumpAndDodge {
    const MIN_PHASE_TIME: f32 = 0.05;
    const MIN_DODGE_TIME: f32 = Self::MIN_PHASE_TIME * 2.0;
    const FOLLOW_THROUGH_TIME: f32 = 0.5;

    pub fn new() -> Self {
        Self {
            start_time: None,
            dodge_time: Self::MIN_DODGE_TIME,
            phase: Phase::Jump,
            dodge: Dodge::new(),
        }
    }

    pub fn jump_time(mut self, jump_time: f32) -> Self {
        assert!(jump_time >= Self::MIN_PHASE_TIME);
        self.dodge_time = jump_time + Self::MIN_PHASE_TIME;
        self
    }

    pub fn angle(mut self, angle: f32) -> Self {
        self.dodge = self.dodge.angle(UnitComplex::new(angle));
        self
    }

    pub fn towards_ball(mut self) -> Self {
        self.dodge = self.dodge.towards_ball();
        self
    }
}

impl Behavior for QuickJumpAndDodge {
    fn name(&self) -> &str {
        name_of_type!(QuickJumpAndDodge)
    }

    fn priority(&self) -> Priority {
        Priority::Force
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let start_time = *self
            .start_time
            .get_or_insert(ctx.packet.GameInfo.TimeSeconds);
        let elapsed = ctx.packet.GameInfo.TimeSeconds - start_time;

        ctx.eeg.print_time("dodge_time", self.dodge_time);
        ctx.eeg.print_time("elapsed", elapsed);

        if self.phase == Phase::Jump || elapsed < self.dodge_time - Self::MIN_PHASE_TIME {
            if self.phase == Phase::Jump && !ctx.me().OnGround {
                ctx.eeg.log(self.name(), "wheels must be on ground");
                return Action::Abort;
            }

            self.phase = Phase::And;

            Action::Yield(common::halfway_house::PlayerInput {
                Jump: true,
                ..Default::default()
            })
        } else if self.phase == Phase::And || elapsed < self.dodge_time {
            if ctx.me().DoubleJumped {
                ctx.eeg.log(self.name(), "must have air charge");
                return Action::Abort;
            }

            self.phase = Phase::Dodge;

            Action::Yield(Default::default())
        } else if self.phase == Phase::Dodge || elapsed < self.dodge_time + 0.1 {
            if ctx.me().OnGround {
                ctx.eeg.log(self.name(), "goomba stomped?");
                return Action::Abort;
            }

            self.phase = Phase::FollowThrough;

            self.dodge.execute_old(ctx)
        } else if self.phase == Phase::FollowThrough
            || elapsed < self.dodge_time + Self::FOLLOW_THROUGH_TIME
        {
            if ctx.me().OnGround {
                ctx.eeg.log(self.name(), "we landed early somehow");
                return Action::Return;
            }

            self.phase = Phase::Finished;

            Action::Yield(Default::default())
        } else {
            Action::Return
        }
    }
}
