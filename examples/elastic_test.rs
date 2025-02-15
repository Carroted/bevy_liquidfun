extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;

use bevy_liquidfun::dynamics::{b2BodyBundle, b2Fixture, b2FixtureDef};
use bevy_liquidfun::particles::{
    b2ParticleFlags, b2ParticleGroup, b2ParticleGroupDef, b2ParticleSystem, b2ParticleSystemDef,
};
use bevy_liquidfun::plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin};
use bevy_liquidfun::utils::{DebugDrawFixtures, DebugDrawParticleSystem};
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{b2BodyDef, b2BodyType::Dynamic, b2World},
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
                setup_ground.after(setup_physics_world),
                setup_circle.after(setup_ground),
                setup_particles.after(setup_circle),
            ),
        )
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        projection: OrthographicProjection {
            scale: 0.007,
            far: 1000.,
            near: -1000.,
            ..OrthographicProjection::default()
        },
        transform: Transform::from_translation(Vec3::new(0., 2., 0.)),
        ..Camera2dBundle::default()
    });
}

fn setup_physics_world(world: &mut World) {
    let gravity = Vec2::new(0., -9.81);
    let b2_world = b2World::new(gravity);
    world.insert_non_send_resource(b2_world);
}

fn setup_ground(mut commands: Commands) {
    {
        let ground_entity = commands.spawn(b2BodyBundle::default()).id();

        {
            let shape = b2Shape::Polygon {
                vertices: vec![
                    Vec2::new(-4., -1.),
                    Vec2::new(4., -1.),
                    Vec2::new(4., 0.),
                    Vec2::new(-4., 0.),
                ],
            };

            let fixture_def = b2FixtureDef::new(shape, 0.);
            commands.spawn((
                b2Fixture::new(ground_entity, &fixture_def),
                DebugDrawFixtures::default_static(),
            ));
        }

        {
            let shape = b2Shape::Polygon {
                vertices: vec![
                    Vec2::new(-4., -0.1),
                    Vec2::new(-2., -0.1),
                    Vec2::new(-2., 2.),
                    Vec2::new(-4., 3.),
                ],
            };
            let fixture_def = b2FixtureDef::new(shape, 0.);
            commands.spawn((
                b2Fixture::new(ground_entity, &fixture_def),
                DebugDrawFixtures::default_static(),
            ));
        }

        {
            let shape = b2Shape::Polygon {
                vertices: vec![
                    Vec2::new(2., -0.1),
                    Vec2::new(4., -0.1),
                    Vec2::new(4., 3.),
                    Vec2::new(2., 2.),
                ],
            };
            let fixture_def = b2FixtureDef::new(shape, 0.);
            commands.spawn((
                b2Fixture::new(ground_entity, &fixture_def),
                DebugDrawFixtures::default_static(),
            ));
        }
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
        flags: b2ParticleFlags::ElasticParticle,
        shape,
    };
    let particle_group = b2ParticleGroup::new(particle_system_entity, &particle_group_def);
    commands.spawn(particle_group);
}
