extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2BodyCommands,
        b2BodyDef,
        b2BodyType::Dynamic,
        b2FixtureDef,
        b2MotorJoint,
        b2MotorJointDef,
        b2World,
        CreateMotorJoint,
    },
    plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin},
    schedule::PhysicsSchedule,
    utils::DebugDrawFixtures,
};

#[derive(States, Debug, Clone, PartialEq, Eq, Hash, Default)]
enum RunState {
    #[default]
    Paused,
    Playing,
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LiquidFunPlugin::default(),
            LiquidFunDebugDrawPlugin,
        ))
        .init_state::<RunState>()
        .add_systems(Startup, (setup_camera, setup_instructions))
        .add_systems(Startup, (setup_physics_world, setup_physics_bodies).chain())
        .add_systems(Update, toggle_state)
        .add_systems(
            PhysicsSchedule,
            update_motor_offsets.run_if(in_state(RunState::Playing)),
        )
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        projection: OrthographicProjection {
            scale: 0.05,
            far: 1000.,
            near: -1000.,
            ..OrthographicProjection::default()
        },
        transform: Transform::from_translation(Vec3::new(0., 10., 0.)),
        ..Camera2dBundle::default()
    });
}

fn setup_instructions(mut commands: Commands) {
    commands.spawn(
        TextBundle::from_section(
            "'S' Toggle Pause",
            TextStyle {
                font_size: 20.0,
                color: Color::WHITE,
                ..default()
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            left: Val::Px(15.0),
            ..default()
        }),
    );
}

fn setup_physics_world(mut commands: Commands) {
    let gravity = Vec2::new(0., -9.81);
    let b2_world = b2World::new(gravity);
    commands.insert_resource(b2_world);
}

fn setup_physics_bodies(mut commands: Commands) {
    let ground_entity = create_ground(&mut commands);
    let box_entity = create_box(&mut commands);

    let joint_def = b2MotorJointDef {
        linear_offset: Vec2::new(0., 8.),
        max_force: 1000.,
        max_torque: 1000.,
        ..default()
    };

    commands.spawn_empty().add(CreateMotorJoint::new(
        ground_entity,
        box_entity,
        true,
        &joint_def,
    ));
}

fn create_ground(commands: &mut Commands) -> Entity {
    let fixture_def = b2FixtureDef::new(
        b2Shape::EdgeTwoSided {
            v1: Vec2::new(-20., 0.),
            v2: Vec2::new(20., 0.),
        },
        0.,
    );
    commands
        .spawn_body(&b2BodyDef::default(), &fixture_def)
        .insert(DebugDrawFixtures::default_static())
        .id()
}

fn create_box(commands: &mut Commands) -> Entity {
    let body_def = b2BodyDef {
        body_type: Dynamic,
        position: Vec2::new(0., 1.),
        allow_sleep: false,
        ..default()
    };
    let shape = b2Shape::create_box(2.0, 0.5);
    let fixture_def = b2FixtureDef {
        shape,
        friction: 0.6,
        density: 2.0,
        ..default()
    };

    commands
        .spawn_body(&body_def, &fixture_def)
        .insert(DebugDrawFixtures::default_dynamic())
        .id()
}

fn toggle_state(
    input: Res<ButtonInput<KeyCode>>,
    state: Res<State<RunState>>,
    mut next_state: ResMut<NextState<RunState>>,
) {
    if input.just_pressed(KeyCode::KeyS) {
        next_state.set(match state.get() {
            RunState::Paused => RunState::Playing,
            RunState::Playing => RunState::Paused,
        });
    }
}

#[derive(Default)]
struct SimulationTime(f32);

fn update_motor_offsets(
    mut motor_joint: Query<&mut b2MotorJoint>,
    time: Res<Time>,
    mut simulation_time: Local<SimulationTime>,
) {
    simulation_time.0 += time.delta_seconds();

    let time = simulation_time.0;
    let linear_offset = Vec2::new(6. * f32::sin(2. * time), 8. + 4. * f32::sin(time));
    let angular_offset = 4. * time;

    let mut motor_joint = motor_joint.single_mut();
    motor_joint.linear_offset = linear_offset;
    motor_joint.angular_offset = angular_offset;
}
