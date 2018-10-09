use flatbuffers;
use rlbot;

#[derive(Default)]
pub struct Vector3Partial {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
}

impl Vector3Partial {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x: Some(x),
            y: Some(y),
            z: Some(z),
        }
    }

    fn serialize<'a>(
        &self,
        builder: &mut flatbuffers::FlatBufferBuilder<'a>,
    ) -> flatbuffers::WIPOffset<rlbot::flat::Vector3Partial<'a>> {
        let x = self.x.map(rlbot::flat::Float::new);
        let y = self.y.map(rlbot::flat::Float::new);
        let z = self.z.map(rlbot::flat::Float::new);
        let args = rlbot::flat::Vector3PartialArgs {
            x: x.as_ref(),
            y: y.as_ref(),
            z: z.as_ref(),
        };
        rlbot::flat::Vector3Partial::create(builder, &args)
    }
}

#[derive(Default)]
pub struct RotatorPartial {
    pub pitch: Option<f32>,
    pub yaw: Option<f32>,
    pub roll: Option<f32>,
}

impl RotatorPartial {
    pub fn new(pitch: f32, yaw: f32, roll: f32) -> Self {
        Self {
            pitch: Some(pitch),
            yaw: Some(yaw),
            roll: Some(roll),
        }
    }

    fn serialize<'a>(
        &self,
        builder: &mut flatbuffers::FlatBufferBuilder<'a>,
    ) -> flatbuffers::WIPOffset<rlbot::flat::RotatorPartial<'a>> {
        let pitch = self.pitch.map(rlbot::flat::Float::new);
        let yaw = self.yaw.map(rlbot::flat::Float::new);
        let roll = self.roll.map(rlbot::flat::Float::new);
        let args = rlbot::flat::RotatorPartialArgs {
            pitch: pitch.as_ref(),
            yaw: yaw.as_ref(),
            roll: roll.as_ref(),
        };
        rlbot::flat::RotatorPartial::create(builder, &args)
    }
}

#[derive(Default)]
pub struct DesiredPhysics {
    pub location: Option<Vector3Partial>,
    pub rotation: Option<RotatorPartial>,
    pub velocity: Option<Vector3Partial>,
    pub angular_velocity: Option<Vector3Partial>,
}

impl DesiredPhysics {
    fn serialize<'a>(
        &self,
        builder: &mut flatbuffers::FlatBufferBuilder<'a>,
    ) -> flatbuffers::WIPOffset<rlbot::flat::DesiredPhysics<'a>> {
        let args = rlbot::flat::DesiredPhysicsArgs {
            location: self.location.as_ref().map(|x| x.serialize(builder)),
            rotation: self.rotation.as_ref().map(|x| x.serialize(builder)),
            velocity: self.velocity.as_ref().map(|x| x.serialize(builder)),
            angularVelocity: self.angular_velocity.as_ref().map(|x| x.serialize(builder)),
        };
        rlbot::flat::DesiredPhysics::create(builder, &args)
    }
}

#[derive(Default)]
pub struct DesiredBallState {
    pub physics: Option<DesiredPhysics>,
}

impl DesiredBallState {
    fn serialize<'a>(
        &self,
        builder: &mut flatbuffers::FlatBufferBuilder<'a>,
    ) -> flatbuffers::WIPOffset<rlbot::flat::DesiredBallState<'a>> {
        let args = rlbot::flat::DesiredBallStateArgs {
            physics: self.physics.as_ref().map(|x| x.serialize(builder)),
        };
        rlbot::flat::DesiredBallState::create(builder, &args)
    }
}

#[derive(Default)]
pub struct DesiredCarState {
    pub physics: Option<DesiredPhysics>,
    pub boost_amount: Option<f32>,
    pub jumped: Option<bool>,
    pub double_jumped: Option<bool>,
}

impl DesiredCarState {
    fn serialize<'a>(
        &self,
        builder: &mut flatbuffers::FlatBufferBuilder<'a>,
    ) -> flatbuffers::WIPOffset<rlbot::flat::DesiredCarState<'a>> {
        let boost_amount = self.boost_amount.map(rlbot::flat::Float::new);
        let jumped = self.jumped.map(rlbot::flat::Bool::new);
        let double_jumped = self.double_jumped.map(rlbot::flat::Bool::new);
        let args = rlbot::flat::DesiredCarStateArgs {
            physics: self.physics.as_ref().map(|x| x.serialize(builder)),
            boostAmount: boost_amount.as_ref(),
            jumped: jumped.as_ref(),
            doubleJumped: double_jumped.as_ref(),
        };
        rlbot::flat::DesiredCarState::create(builder, &args)
    }
}

#[derive(Default)]
pub struct DesiredGameState {
    pub ball_state: Option<DesiredBallState>,
    pub car_states: Vec<DesiredCarState>,
}

impl DesiredGameState {
    pub fn serialize<'a>(&self) -> flatbuffers::FlatBufferBuilder<'a> {
        let mut builder = flatbuffers::FlatBufferBuilder::new_with_capacity(1024);

        let car_states = self
            .car_states
            .iter()
            .map(|x| x.serialize(&mut builder))
            .collect::<Vec<_>>();
        let args = rlbot::flat::DesiredGameStateArgs {
            ballState: self.ball_state.as_ref().map(|x| x.serialize(&mut builder)),
            carStates: Some(builder.create_vector(&car_states)),
            ..Default::default()
        };
        let root = rlbot::flat::DesiredGameState::create(&mut builder, &args);

        builder.finish(root, None);
        builder
    }
}
