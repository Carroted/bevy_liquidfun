extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2BodiesInContact,
        b2Body,
        b2BodyBundle,
        b2BodyDef,
        b2BodyType,
        b2Fixture,
        b2FixtureDef,
        b2World,
        ExternalForce,
    },
    plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin},
    schedule::{PhysicsSchedule, PhysicsUpdateStep::UserCode},
    utils::DebugDrawFixtures,
};

#[derive(Component)]
pub struct Sensor;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LiquidFunPlugin::default(),
            LiquidFunDebugDrawPlugin,
        ))
        .add_systems(Startup, setup_camera)
        .add_systems(Startup, (setup_physics_world, setup).chain())
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
    {
        let ground_entity = commands.spawn(b2BodyBundle::default()).id();

        let shape = b2Shape::EdgeTwoSided {
            v1: Vec2::new(-40., 0.),
            v2: Vec2::new(40., 0.),
        };
        let fixture_def = b2FixtureDef::new(shape, 0.0);
        commands.spawn((
            b2Fixture::new(ground_entity, &fixture_def),
            DebugDrawFixtures::default_static(),
        ));
    }

    let sensor_entity = {
        let body_entity = commands.spawn(b2BodyBundle::default()).id();

        let shape = b2Shape::Circle {
            radius: 5.0,
            position: Vec2::new(0.0, 10.0),
        };

        let fixture_def = b2FixtureDef {
            shape,
            is_sensor: true,
            ..default()
        };
        commands.spawn((
            b2Fixture::new(body_entity, &fixture_def),
            DebugDrawFixtures::default_static(),
            b2BodiesInContact::default(),
            Sensor,
        ))
    };

    {
        for i in 0..7 {
            let shape = b2Shape::Circle {
                radius: 1.0,
                position: Vec2::ZERO,
            };

            let body_def = b2BodyDef {
                body_type: b2BodyType::Dynamic,
                position: Vec2::new(-10.0 + 3.0 * i as f32, 20.0),
                ..default()
            };

            let body_entity = commands.spawn(b2BodyBundle::new(&body_def)).id();
            commands.spawn((
                b2Fixture::new(body_entity, &b2FixtureDef::new(shape, 1.0)),
                DebugDrawFixtures::default_static(),
            ));
        }
    }
}

fn apply_force(
    query: Query<(&b2Fixture, &b2BodiesInContact), With<Sensor>>,
    transforms: Query<(&Transform)>,
    mut external_forces: Query<&mut ExternalForce>,
) {
    for (fixture, bodies) in &query {
        for body in bodies.contacts() {
            let position = transforms.get(*body).unwrap().translation;
            let center = transforms.get(fixture.body()).unwrap().translation;
            let mut external_force = external_forces.get_mut(*body).unwrap();
            let direction = (center - position).normalize();

            let force = 100.0 * direction.xy();

            external_force.apply_force(force);
        }
    }
}
