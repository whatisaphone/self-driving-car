use collect::ExtendRotation3;
use nalgebra::{Quaternion, UnitQuaternion};
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
    let quat =
        UnitQuaternion::from_quaternion(Quaternion::new(quat.w(), quat.x(), quat.y(), quat.z()));
    let (pitch, yaw, roll) = quat.to_rotation_matrix().to_unreal_angles();
    // I have no clue why two of these are negated, but it makes the numbers match
    // up…
    (-pitch, yaw, -roll)
}

#[cfg(test)]
mod tests {
    use rlbot;
    use utils::rlbot_ext;

    #[test]
    fn rotation() {
        let cases = [
            (
                rlbot::flat::Quaternion::new(0.18755347, -0.18754272, 0.68180066, 0.6817619),
                (0.52193695, 1.5708922, 0.0),
            ),
            (
                rlbot::flat::Quaternion::new(-0.25964448, 0.37656632, 0.26754785, 0.84805703),
                (-0.8697671, 0.4358423, 0.37342843),
            ),
        ];
        for (quat, (approx_pitch, approx_yaw, approx_roll)) in cases.iter() {
            println!("{:?}", quat);
            println!("{:?} {:?} {:?}", approx_pitch, approx_yaw, approx_roll);
            let (pitch, yaw, roll) = rlbot_ext::convert_quat_to_pyr(&quat);
            assert!((pitch - approx_pitch).abs() < 0.05, "{}", pitch);
            assert!((yaw - approx_yaw).abs() < 0.05, "{}", yaw);
            assert!((roll - approx_roll).abs() < 0.05, "{}", roll);
        }
    }
}
