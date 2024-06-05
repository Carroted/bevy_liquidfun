extern crate bevy;
extern crate bevy_liquidfun;

use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2Body,
        b2BodyCommands,
        b2BodyDef,
        b2BodyType::Dynamic,
        b2FixtureDef,
        b2Joint,
        b2RevoluteJoint,
        b2RevoluteJointDef,
        b2World,
        CreateRevoluteJoint,
    },
    particles::{
        b2ParticleFlags,
        b2ParticleGroup,
        b2ParticleGroupDef,
        b2ParticleSystem,
        b2ParticleSystemDef,
    },
    plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin},
    utils::{DebugDrawFixtures, DebugDrawParticleSystem},
};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LiquidFunPlugin::default(),
            LiquidFunDebugDrawPlugin,
        ))
        .add_systems(Startup, setup_camera)
        .add_systems(
            Startup,
            (setup_physics_world, setup_box, setup_particles).chain(),
        )
        .add_systems(FixedUpdate, set_motor_speed)
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        projection: OrthographicProjection {
            scale: 0.01,
            far: 1000.,
            near: -1000.,
            ..OrthographicProjection::default()
        },
        ..Camera2dBundle::default()
    });
}

fn setup_physics_world(mut commands: Commands) {
    let gravity = Vec2::new(0., -9.81);
    let b2_world = b2World::new(gravity);
    commands.insert_resource(b2_world);
}

fn setup_box(mut commands: Commands) {
    let ground_entity = commands.spawn(b2Body::new(&b2BodyDef::default())).id();

    let box_pos = Vec2::new(0., 1.);
    let box_def = b2BodyDef {
        body_type: Dynamic,
        position: box_pos,
        ..default()
    };

    let shapes = vec![
        b2Shape::create_box_with_offset(0.05, 1., Vec2::new(2.0, 0.0)),
        b2Shape::create_box_with_offset(0.05, 1., Vec2::new(-2.0, 0.0)),
        b2Shape::create_box_with_offset(2., 0.05, Vec2::new(0.0, 1.0)),
        b2Shape::create_box_with_offset(2., 0.05, Vec2::new(0.0, -1.0)),
    ];

    let box_entity = commands
        .spawn_multi_fixture_body(
            &box_def,
            &shapes
                .into_iter()
                .map(|shape| b2FixtureDef::new(shape, 5.))
                .collect(),
            |fixture| {
                fixture.insert(DebugDrawFixtures::default_static());
            },
        )
        .id();

    let joint_def = b2RevoluteJointDef {
        local_anchor_a: box_pos,
        motor_speed: 0.1 * PI,
        max_motor_torque: 1e7,
        enable_motor: true,
        ..default()
    };
    commands.spawn_empty().add(CreateRevoluteJoint::new(
        ground_entity,
        box_entity,
        true,
        &joint_def,
    ));
}
fn setup_particles(mut commands: Commands) {
    let particle_system_def = b2ParticleSystemDef {
        radius: 0.035,
        damping_strength: 0.2,
        ..default()
    };
    let particle_system = b2ParticleSystem::new(&particle_system_def);
    let particle_system_entity = commands
        .spawn((particle_system, DebugDrawParticleSystem {}))
        .id();

    let shape = b2Shape::create_box_with_offset(0.9, 0.9, Vec2::Y);
    let particle_group_def = b2ParticleGroupDef {
        flags: b2ParticleFlags::WaterParticle,
        shape,
        ..default()
    };
    let particle_group = b2ParticleGroup::new(particle_system_entity, &particle_group_def);
    commands.spawn(particle_group);
}

fn set_motor_speed(mut joints: Query<(&b2Joint, &mut b2RevoluteJoint)>, bodies: Query<&b2Body>) {
    let joint = joints.single();
    let box_entity = joint.0.body_b();
    let box_body = bodies.get(*box_entity).unwrap();

    if let Some(speed) = calculate_new_speed(box_body.angle) {
        let mut joint = joints.single_mut();
        joint.1.motor_speed = speed;
    }
}

fn calculate_new_speed(current_angle: f32) -> Option<f32> {
    const SPEED: f32 = 0.4;
    const MAX_ANGLE: f32 = PI / 8.;
    if current_angle > MAX_ANGLE {
        Some(-SPEED)
    } else if current_angle < -MAX_ANGLE {
        Some(SPEED)
    } else {
        None
    }
}
