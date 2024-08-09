extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2BodyCommands, b2BodyDef, b2BodyType::Dynamic, b2Fixture, b2RevoluteJointDef, b2World, CreateRevoluteJoint
    },
    plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin},
    utils::DebugDrawFixtures,
};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LiquidFunPlugin::default(),
            LiquidFunDebugDrawPlugin,
        ))
        .add_systems(Startup, setup_camera)
        .add_systems(Startup, (setup_physics_world, setup_physics_bodies).chain())
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

fn setup_physics_world(world: &mut World) {
    let gravity = Vec2::new(0., -9.81);
    let b2_world = b2World::new(gravity);
    world.insert_resource(b2_world);
}

fn setup_physics_bodies(mut commands: Commands) {
    let ground_fixture = b2Fixture::new(
        b2Shape::EdgeTwoSided {
            v1: Vec2::new(-40., 0.),
            v2: Vec2::new(40., 0.),
        },
        0.0,
    );
    let ground_entity = commands
        .spawn_body(&b2BodyDef::default(), ground_fixture)
        .insert(DebugDrawFixtures::default_static())
        .id();

    let box_shape = b2Shape::create_box(0.6, 0.125);
    let box_fixture = b2Fixture {
        shape: box_shape,
        density: 20.0,
        friction: 0.2,
        ..default()
    };
    const Y: f32 = 25.0;

    let mut body_entities = vec![Entity::PLACEHOLDER; 30];
    for i in 0..30 {
        let body_def = b2BodyDef {
            body_type: Dynamic,
            position: Vec2::new(0.5 + i as f32, Y),
            ..default()
        };

        body_entities[i] = commands
            .spawn_body(&body_def, box_fixture.clone())
            .insert(DebugDrawFixtures {
                draw_up_vector: false,
                draw_right_vector: false,
                ..DebugDrawFixtures::default_dynamic()
            })
            .id();
    }

    let joint_def = b2RevoluteJointDef {
        local_anchor_a: Vec2::new(0.5, 0.),
        local_anchor_b: Vec2::new(-0.5, 0.),
        ..default()
    };

    for body_pair in body_entities.windows(2) {
        let (body_a, body_b) = (body_pair[0], body_pair[1]);
        commands
            .spawn_empty()
            .add(CreateRevoluteJoint::new(body_a, body_b, false, &joint_def));
    }

    let joint_def = b2RevoluteJointDef {
        local_anchor_a: Vec2::new(0., Y),
        ..joint_def
    };

    commands.spawn_empty().add(CreateRevoluteJoint::new(
        ground_entity,
        body_entities[0],
        true,
        &joint_def,
    ));
}
