extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2BodyBundle,
        b2BodyCommands,
        b2BodyDef,
        b2BodyType::Dynamic,
        b2Fixture,
        b2FixtureDef,
        b2World,
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
            (
                setup_physics_world,
                setup_ground,
                setup_circle,
                setup_particles,
            )
                .chain(),
        )
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
        transform: Transform::from_translation(Vec3::new(0., 2., 0.)),
        ..Camera2dBundle::default()
    });
}

fn setup_physics_world(mut commands: Commands) {
    let gravity = Vec2::new(0., -9.81);
    let b2_world = b2World::new(gravity);
    commands.insert_resource(b2_world);
}

fn setup_ground(mut commands: Commands) {
    {
        let shapes = vec![
            b2Shape::Polygon {
                vertices: vec![
                    Vec2::new(-4., -1.),
                    Vec2::new(4., -1.),
                    Vec2::new(4., 0.),
                    Vec2::new(-4., 0.),
                ],
            },
            b2Shape::Polygon {
                vertices: vec![
                    Vec2::new(-4., -0.1),
                    Vec2::new(-2., -0.1),
                    Vec2::new(-2., 2.),
                    Vec2::new(-4., 3.),
                ],
            },
            b2Shape::Polygon {
                vertices: vec![
                    Vec2::new(2., -0.1),
                    Vec2::new(4., -0.1),
                    Vec2::new(4., 3.),
                    Vec2::new(2., 2.),
                ],
            },
        ];

        commands.spawn_multi_fixture_body(
            &b2BodyDef::default(),
            &shapes
                .into_iter()
                .map(|shape| b2FixtureDef::new(shape, 0.))
                .collect(),
            |e| {
                e.insert(DebugDrawFixtures::default_static());
            },
        );
    }
}

fn setup_circle(mut commands: Commands) {
    let body_def = b2BodyDef {
        body_type: Dynamic,
        position: Vec2::new(0., 8.),
        ..default()
    };
    let body_entity = commands.spawn(b2BodyBundle::new(&body_def)).id();

    let circle_shape = b2Shape::Circle {
        radius: 0.5,
        position: Vec2::default(),
    };
    let fixture_def = b2FixtureDef::new(circle_shape, 0.5);
    commands.spawn((
        b2Fixture::new(body_entity, &fixture_def),
        DebugDrawFixtures::default_dynamic(),
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

    let shape = b2Shape::Circle {
        radius: 2.,
        position: Vec2::new(0., 3.),
    };
    let particle_group_def = b2ParticleGroupDef {
        flags: b2ParticleFlags::WaterParticle,
        shape,
        ..default()
    };

    let particle_group = b2ParticleGroup::new(particle_system_entity, &particle_group_def);
    commands.spawn(particle_group);
}
