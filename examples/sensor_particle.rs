extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{b2BodyCommands, b2BodyDef, b2FixtureDef, b2ParticlesInContact, b2World},
    particles::{
        b2ParticleFlags,
        b2ParticleGroup,
        b2ParticleGroupDef,
        b2ParticleSystem,
        b2ParticleSystemDef,
    },
    plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin},
    schedule::{PhysicsSchedule, PhysicsUpdateStep::UserCode},
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
            (setup_physics_world, setup, setup_particles).chain(),
        )
        .add_systems(PhysicsSchedule, apply_force.in_set(UserCode))
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        projection: OrthographicProjection {
            scale: 0.07,
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

fn setup(mut commands: Commands) {
    // ground
    {
        let fixture_def = b2FixtureDef::new(
            b2Shape::EdgeTwoSided {
                v1: Vec2::new(-40., 0.),
                v2: Vec2::new(40., 0.),
            },
            0.0,
        );
        commands
            .create_body(&b2BodyDef::default(), &fixture_def)
            .insert(DebugDrawFixtures::default_static());
    }

    // sensor
    {
        let shape = b2Shape::Circle {
            radius: 5.0,
            position: Vec2::new(0.0, 10.0),
        };

        let fixture_def = b2FixtureDef {
            shape,
            is_sensor: true,
            ..default()
        };
        commands
            .create_body(&b2BodyDef::default(), &fixture_def)
            .insert((
                DebugDrawFixtures::default_static(),
                b2ParticlesInContact::default(),
            ));
    }
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

    let shape = b2Shape::create_box_with_offset(0.9, 0.9, Vec2::new(0.0, 20.0));
    let particle_group_def = b2ParticleGroupDef {
        flags: b2ParticleFlags::WaterParticle,
        shape,
        ..default()
    };
    let particle_group = b2ParticleGroup::new(particle_system_entity, &particle_group_def);
    commands.spawn(particle_group);
}

fn apply_force(
    particle_query: Query<&b2ParticlesInContact>,
    mut particle_systems: Query<&mut b2ParticleSystem>,
) {
    let mut particle_system = particle_systems.get_single_mut().unwrap();

    for particles in &particle_query {
        for index in particles.contacts().iter() {
            let position = particle_system
                .get_positions()
                .get(*index as usize)
                .unwrap();
            let center = Vec2::new(0.0, 10.0);
            let direction = (center - *position).normalize();

            particle_system.particle_apply_force(*index, 0.05 * direction);
        }
    }
}
