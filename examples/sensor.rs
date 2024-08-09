extern crate bevy;
extern crate bevy_liquidfun;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2BodiesInContact,
        b2BodyCommands,
        b2BodyDef,
        b2BodyType,
        b2Fixture,
        b2FixtureBody,
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
    // ground
    {
        let fixture = b2Fixture::new(
            b2Shape::EdgeTwoSided {
                v1: Vec2::new(-40., 0.),
                v2: Vec2::new(40., 0.),
            },
            0.0,
        );
        commands
            .spawn_body(&b2BodyDef::default(), fixture)
            .insert(DebugDrawFixtures::default_static());
    }

    // sensor
    {
        let shape = b2Shape::Circle {
            radius: 5.0,
            position: Vec2::new(0.0, 10.0),
        };

        let fixture = b2Fixture {
            shape,
            is_sensor: true,
            ..default()
        };
        commands
            .spawn_body(&b2BodyDef::default(), fixture)
            .insert((
                DebugDrawFixtures::default_static(),
                b2BodiesInContact::default(),
                Sensor,
            ));
    }

    let shape = b2Shape::Circle {
        radius: 1.0,
        position: Vec2::ZERO,
    };
    for i in 0..7 {
        let body_def = b2BodyDef {
            body_type: b2BodyType::Dynamic,
            position: Vec2::new(-10.0 + 3.0 * i as f32, 20.0),
            ..default()
        };
        commands
            .spawn_body(&body_def, b2Fixture::new(shape.clone(), 1.0))
            .insert(DebugDrawFixtures::default_dynamic());
    }
}

fn apply_force(
    query: Query<(&b2FixtureBody, &b2BodiesInContact), With<Sensor>>,
    transforms: Query<&Transform>,
    mut external_forces: Query<&mut ExternalForce>,
) {
    for (fixture_body, bodies) in &query {
        for body in bodies.contacts() {
            let position = transforms.get(*body).unwrap().translation;
            let center = transforms.get(fixture_body.body()).unwrap().translation;
            let mut external_force = external_forces.get_mut(*body).unwrap();
            let direction = (center - position).normalize();

            let force = 100.0 * direction.xy();

            external_force.apply_force(force);
        }
    }
}
