use common::prelude::*;
use nalgebra::UnitQuaternion;
use rlbot;
use std::error::Error;

const PHYSICS_TPS: f32 = 120.0;

pub fn get_packet_and_inject_rigid_body_tick(
    rlbot: &rlbot::RLBot,
    rigid_body_tick: rlbot::flat::RigidBodyTick,
) -> Result<rlbot::ffi::LiveDataPacket, Box<Error>> {
    let mut packet = unsafe { ::std::mem::uninitialized() };
    rlbot.update_live_data_packet(&mut packet)?;
    physicsify(&mut packet, rigid_body_tick);
    Ok(packet)
}

pub fn physicsify(packet: &mut rlbot::ffi::LiveDataPacket, physics: rlbot::flat::RigidBodyTick) {
    let ball = physics.ball().unwrap();
    let ball_state = ball.state().unwrap();
    packet.GameInfo.TimeSeconds = ball_state.frame() as f32 / PHYSICS_TPS;
    set_physics(&mut packet.GameBall.Physics, ball_state);
    for i in 0..packet.NumCars as usize {
        let player = physics.players().unwrap().get(i);
        set_physics(&mut packet.GameCars[i].Physics, player.state().unwrap());
    }
}

fn set_physics(dest: &mut rlbot::ffi::Physics, source: rlbot::flat::RigidBodyState) {
    dest.Location.X = source.location().unwrap().x();
    dest.Location.Y = source.location().unwrap().y();
    dest.Location.Z = source.location().unwrap().z();

    let (pitch, yaw, roll) = convert_quat_to_pyr(source.rotation().unwrap());
    dest.Rotation.Pitch = pitch;
    dest.Rotation.Yaw = yaw;
    dest.Rotation.Roll = roll;

    dest.Velocity.X = source.velocity().unwrap().x();
    dest.Velocity.Y = source.velocity().unwrap().y();
    dest.Velocity.Z = source.velocity().unwrap().z();

    dest.AngularVelocity.X = source.angularVelocity().unwrap().x();
    dest.AngularVelocity.Y = source.angularVelocity().unwrap().y();
    dest.AngularVelocity.Z = source.angularVelocity().unwrap().z();
}

fn convert_quat_to_pyr(quat: &rlbot::flat::Quaternion) -> (f32, f32, f32) {
    let quat = UnitQuaternion::xyzw(quat.x(), quat.y(), quat.z(), quat.w());
    quat.rocket_league_munge()
        .to_rotation_matrix()
        .to_unreal_angles()
}

#[cfg(test)]
mod tests {
    use rlbot;
    use rlbot_ext;

    #[test]
    fn rotation() {
        let cases = [
            (
                rlbot::flat::Quaternion::new(-0.0015563988, 0.0045613004, 0.32138819, 0.94693524),
                (-0.009683254, 0.65443456, 0.0),
            ),
            (
                rlbot::flat::Quaternion::new(0.37422496, 0.6927582, -0.34245488, 0.51260734),
                (-1.3085815, 2.421868, 2.7907903),
            ),
        ];

        for (quat, (approx_pitch, approx_yaw, approx_roll)) in cases.iter() {
            println!("{:?}", quat);
            println!("{:?} {:?} {:?}", approx_pitch, approx_yaw, approx_roll);
            let (pitch, yaw, roll) = rlbot_ext::convert_quat_to_pyr(&quat);
            assert!((pitch - approx_pitch).abs() < 0.02, "{}", pitch);
            assert!((yaw - approx_yaw).abs() < 0.02, "{}", yaw);
            assert!((roll - approx_roll).abs() < 0.02, "{}", roll);
        }
    }
}
