use crate::strategy::{Action, Behavior, Context};
use common::prelude::*;
use nalgebra::Point3;
use nameof::name_of_type;

struct WallDrive {
    target_loc: Point3<f32>,
}

impl WallDrive {
    pub fn to(loc: Point3<f32>) -> Self {
        Self { target_loc: loc }
    }
}

impl Behavior for WallDrive {
    fn name(&self) -> &str {
        name_of_type!(WallDrive)
    }

    fn execute_old(&mut self, ctx: &mut Context<'_>) -> Action {
        let me = ctx.me();
        let me_forward = me.Physics.forward_axis();

        if !me.OnGround {
            ctx.eeg.log(self.name(), "not on ground");
            return Action::Abort;
        }

        let current_plane = ctx.game.pitch().closest_plane(&me.Physics.loc());
        let target_plane = ctx.game.pitch().closest_plane(&self.target_loc);

        if current_plane.distance_to_point(&me.Physics.loc()) >= 100.0 {
            ctx.eeg
                .log(self.name(), "not on the plane we think we are?");
            return Action::Abort;
        }

        let steer_target_loc = if current_plane.normal.dot(&target_plane.normal) < 0.95 {
            let unfold = some_or_else!(target_plane.unfold(current_plane).ok(), {
                ctx.eeg.log(self.name(), "can't unfold wall");
                return Action::Abort;
            });
            unfold * self.target_loc
        } else {
            self.target_loc
        };

        let rotation = me_forward.rotation_to(&(steer_target_loc - me.Physics.loc()).to_axis());
        let steer = rotation.project_2d(&me.Physics.roof_axis());

        Action::Yield(rlbot::ffi::PlayerInput {
            Throttle: 1.0,
            Steer: steer.angle(),
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod integration_tests {
    use crate::{
        behavior::movement::wall_drive::WallDrive,
        integration_tests::helpers::{TestRunner, TestScenario},
    };
    use common::rl;
    use nalgebra::Point3;

    #[test]
    #[ignore(note = "TODO")]
    fn simple() {
        TestRunner::new()
            .scenario(TestScenario {
                car_loc: Point3::new(3000.0, 0.0, 17.01),
                ..Default::default()
            })
            .behavior(WallDrive::to(Point3::new(rl::FIELD_MAX_X, 3000.0, 1000.0)))
            .run_for_millis(5000);
    }
}
