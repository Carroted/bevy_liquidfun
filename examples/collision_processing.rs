extern crate bevy;
extern crate bevy_liquidfun;
extern crate rand;

use std::ops::RangeInclusive;

use bevy::prelude::*;
use bevy_liquidfun::{
    collision::b2Shape,
    dynamics::{
        b2BeginContactEvent, b2Body, b2BodyCommands, b2BodyDef, b2BodyType::Dynamic, b2Fixture, b2World
    },
    plugins::{LiquidFunDebugDrawPlugin, LiquidFunPlugin},
    schedule::{PhysicsSchedule, PhysicsUpdateStep},
    utils::DebugDrawFixtures,
};
use rand::prelude::*;

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
            (setup_physics_world, setup_bodies.after(setup_physics_world)),
        )
        .add_systems(
            PhysicsSchedule,
            process_collisions.in_set(PhysicsUpdateStep::UserCode),
        )
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
        transform: Transform::from_translation(Vec3::new(0., 10., 0.)),
        ..Camera2dBundle::default()
    });
}

fn setup_physics_world(mut commands: Commands) {
    let gravity = Vec2::new(0., -9.81);
    let b2_world = b2World::new(gravity);
    commands.insert_resource(b2_world);
}

fn setup_bodies(mut commands: Commands) {
    {
        let fixture = b2Fixture::new(
            b2Shape::EdgeTwoSided {
                v1: Vec2::new(-50., 0.),
                v2: Vec2::new(50., 0.),
            },
            0.,
        );
        commands
            .spawn_body(&b2BodyDef::default(), fixture)
            .insert(DebugDrawFixtures::default_static());
    }

    let x_range = -5.0..=5.0;
    let y_range = 2.0..=35.0;
    let mut rng = thread_rng();

    // small triangle
    {
        let shape = b2Shape::Polygon {
            vertices: vec![Vec2::new(-1., 0.), Vec2::new(1., 0.), Vec2::new(0., 2.)],
        };
        create_body_in_random_position(&mut commands, shape, &x_range, &y_range, &mut rng);
    }

    // large triangle
    {
        let shape = b2Shape::Polygon {
            vertices: vec![Vec2::new(-2., 0.), Vec2::new(2., 0.), Vec2::new(0., 4.)],
        };
        create_body_in_random_position(&mut commands, shape, &x_range, &y_range, &mut rng);
    }

    // small box
    {
        let shape = b2Shape::create_box(1., 0.5);
        create_body_in_random_position(&mut commands, shape, &x_range, &y_range, &mut rng);
    }

    // large box
    {
        let shape = b2Shape::create_box(2., 1.);
        create_body_in_random_position(&mut commands, shape, &x_range, &y_range, &mut rng);
    }

    // small circle
    {
        let shape = b2Shape::Circle {
            radius: 1.,
            position: Vec2::ZERO,
        };
        create_body_in_random_position(&mut commands, shape, &x_range, &y_range, &mut rng);
    }

    // large circle
    {
        let shape = b2Shape::Circle {
            radius: 2.,
            position: Vec2::ZERO,
        };

        create_body_in_random_position(&mut commands, shape, &x_range, &y_range, &mut rng);
    }
}

fn create_body_in_random_position(
    commands: &mut Commands,
    shape: b2Shape,
    x_range: &RangeInclusive<f32>,
    y_range: &RangeInclusive<f32>,
    rng: &mut ThreadRng,
) {
    commands
        .spawn_body(
            &b2BodyDef {
                body_type: Dynamic,
                position: Vec2::new(
                    rng.gen_range(x_range.clone()),
                    rng.gen_range(y_range.clone()),
                ),
                ..default()
            },
            b2Fixture::new(shape, 1.0),
        )
        .insert(DebugDrawFixtures::default_dynamic());
}

fn process_collisions(
    mut commands: Commands,
    bodies: Query<&b2Body>,
    mut event_reader: EventReader<b2BeginContactEvent>,
) {
    for contact_event in event_reader.read() {
        let contact = contact_event.0;
        let body_a = bodies.get(contact.body_a);
        let body_b = bodies.get(contact.body_b);
        if body_a.is_err() || body_b.is_err() {
            continue;
        }

        let (body_a, body_b) = (body_a.unwrap(), body_b.unwrap());
        if body_a.mass() != 0.0 && body_b.mass() != 0.0 {
            if body_a.mass() > body_b.mass() {
                commands.entity(contact.body_b).despawn_recursive();
            } else {
                commands.entity(contact.body_a).despawn_recursive();
            }
        }
    }
}
