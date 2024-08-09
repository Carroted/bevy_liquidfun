use bevy::{
    ecs::{entity::MapEntities, system::EntityCommands},
    prelude::*,
};
use libliquidfun_sys::box2d::{
    ffi,
    ffi::b2BodyType::{b2_dynamicBody, b2_kinematicBody, b2_staticBody},
};

use super::b2Fixture;
use crate::{
    dynamics::b2WorldImpl,
    internal::{to_Vec2, to_b2Vec2},
};

#[allow(non_camel_case_types)]
#[derive(Debug, Default, Copy, Clone, Reflect, PartialEq)]
#[cfg_attr(
    feature = "bevy-inspector-egui",
    derive(bevy_inspector_egui::inspector_options::InspectorOptions)
)]
#[type_path = "bevy_liquidfun"]
pub enum b2BodyType {
    #[default]
    Static,
    Kinematic,
    Dynamic,
}

impl From<ffi::b2BodyType> for b2BodyType {
    fn from(value: ffi::b2BodyType) -> Self {
        match value {
            b2_staticBody => b2BodyType::Static,
            b2_kinematicBody => b2BodyType::Kinematic,
            b2_dynamicBody => b2BodyType::Dynamic,
        }
    }
}

impl Into<ffi::b2BodyType> for b2BodyType {
    fn into(self) -> ffi::b2BodyType {
        match self {
            b2BodyType::Static => b2_staticBody,
            b2BodyType::Kinematic => b2_kinematicBody,
            b2BodyType::Dynamic => b2_dynamicBody,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct b2Body {
    pub body_type: b2BodyType,
    pub position: Vec2,
    pub angle: f32,
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
    pub awake: bool,
    pub allow_sleep: bool,
    pub fixed_rotation: bool,

    mass: f32,
}

impl Default for b2Body {
    fn default() -> Self {
        Self {
            body_type: b2BodyType::default(),
            position: Vec2::ZERO,
            angle: 0.,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.,
            awake: true,
            allow_sleep: true,
            fixed_rotation: false,
            mass: 0.,
        }
    }
}

impl b2Body {
    pub fn new(body_def: &b2BodyDef) -> Self {
        b2Body {
            body_type: body_def.body_type,
            position: body_def.position,
            angle: body_def.angle,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.,
            mass: 0.,
            awake: true,
            allow_sleep: body_def.allow_sleep,
            fixed_rotation: body_def.fixed_rotation,
        }
    }

    pub fn sync_with_world(&mut self, entity: Entity, world: &b2WorldImpl) {
        let Some(body_ptr) = world.body_ptr(entity) else {
            return;
        };

        self.body_type = From::from(body_ptr.as_ref().GetType());
        self.position = to_Vec2(body_ptr.as_ref().GetPosition());
        self.angle = body_ptr.as_ref().GetAngle();
        self.linear_velocity = to_Vec2(body_ptr.as_ref().GetLinearVelocity());
        self.angular_velocity = body_ptr.as_ref().GetAngularVelocity();
        self.fixed_rotation = body_ptr.as_ref().IsFixedRotation();
        self.mass = body_ptr.as_ref().GetMass();
        self.awake = body_ptr.as_ref().IsAwake();
    }

    pub fn sync_to_world(&self, entity: Entity, world: &mut b2WorldImpl) {
        let Some(mut body_ptr) = world.body_ptr_mut(entity) else {
            return;
        };

        body_ptr
            .as_mut()
            .SetTransform(&to_b2Vec2(&self.position), self.angle);
        body_ptr
            .as_mut()
            .SetLinearVelocity(&to_b2Vec2(&self.linear_velocity));
        body_ptr.as_mut().SetAngularVelocity(self.angular_velocity);
        body_ptr.as_mut().SetAwake(self.awake);
        body_ptr.as_mut().SetSleepingAllowed(self.allow_sleep);
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }
}

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct b2BodyFixtures {
    pub(crate) fixtures: Vec<Entity>,
}

impl b2BodyFixtures {
    pub(crate) fn new() -> Self {
        Self {
            fixtures: Vec::new(),
        }
    }

    pub fn fixtures(&self) -> &Vec<Entity> {
        &self.fixtures
    }
}

impl MapEntities for b2BodyFixtures {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        let mut new_fixtures = Vec::with_capacity(self.fixtures.len());
        new_fixtures.extend(
            self.fixtures
                .iter()
                .map(|entity| entity_mapper.map_entity(*entity)),
        );
        self.fixtures = new_fixtures;
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, Default, Clone)]
pub struct b2BodyDef {
    pub body_type: b2BodyType,
    pub position: Vec2,
    pub angle: f32,
    pub allow_sleep: bool,
    pub fixed_rotation: bool,
}

#[allow(non_camel_case_types)]
#[derive(Bundle)]
pub struct b2BodyBundle {
    pub transform: TransformBundle,
    pub body: b2Body,
    pub external_force: ExternalForce,
    pub external_impulse: ExternalImpulse,
    pub external_torque: ExternalTorque,
}

impl b2BodyBundle {
    pub fn new(def: &b2BodyDef) -> Self {
        Self {
            transform: TransformBundle {
                local: Transform {
                    translation: def.position.extend(0.),
                    rotation: Quat::from_rotation_z(def.angle),
                    ..default()
                },
                ..default()
            },
            body: b2Body::new(def),
            external_force: ExternalForce::default(),
            external_impulse: ExternalImpulse::default(),
            external_torque: ExternalTorque::default(),
        }
    }
}

impl Default for b2BodyBundle {
    fn default() -> Self {
        b2BodyBundle::new(&b2BodyDef::default())
    }
}

#[derive(Component, Debug, Default, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct ExternalForce {
    force: Vec2,
    pub should_wake: bool,
    torque: f32,
}

impl ExternalForce {
    pub const ZERO: Self = Self {
        force: Vec2::ZERO,
        should_wake: false,
        torque: 0.,
    };

    pub fn new(force: Vec2) -> Self {
        Self { force, ..default() }
    }

    pub fn set_force(&mut self, force: Vec2) -> &mut Self {
        self.force = force;
        self
    }

    pub fn apply_force(&mut self, force: Vec2) -> &mut Self {
        self.force += force;
        self
    }

    pub fn apply_force_at_point(
        &mut self,
        force: Vec2,
        point: Vec2,
        center_of_mass: Vec2,
    ) -> &mut Self {
        self.force += force;
        self.torque += (point - center_of_mass).perp_dot(force);
        self
    }

    pub fn force(&self) -> Vec2 {
        self.force
    }

    pub fn torque(&self) -> f32 {
        self.torque
    }

    pub fn clear(&mut self) {
        self.force = Vec2::ZERO;
        self.torque = 0.;
    }
}

#[derive(Component, Debug, Default, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct ExternalImpulse {
    impulse: Vec2,
    pub should_wake: bool,
    angular_impulse: f32,
}

impl ExternalImpulse {
    pub const ZERO: Self = Self {
        impulse: Vec2::ZERO,
        should_wake: false,
        angular_impulse: 0.,
    };

    pub fn new(impulse: Vec2) -> Self {
        Self {
            impulse,
            ..default()
        }
    }

    pub fn set_impulse(&mut self, impulse: Vec2) -> &mut Self {
        self.impulse = impulse;
        self
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) -> &mut Self {
        self.impulse += impulse;
        self
    }

    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vec2,
        point: Vec2,
        center_of_mass: Vec2,
    ) -> &mut Self {
        self.impulse += impulse;
        self.angular_impulse += (point - center_of_mass).perp_dot(impulse);
        self
    }

    pub fn impulse(&self) -> Vec2 {
        self.impulse
    }

    pub fn angular_impulse(&self) -> f32 {
        self.angular_impulse
    }

    pub fn clear(&mut self) {
        self.impulse = Vec2::ZERO;
        self.angular_impulse = 0.;
    }
}

#[derive(Component, Debug, Default, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct ExternalTorque {
    pub torque: f32,
    pub should_wake: bool,
}

impl ExternalTorque {
    pub const ZERO: Self = Self {
        torque: 0.,
        should_wake: false,
    };
}

#[derive(Component, Debug, Deref, DerefMut)]
pub struct GravityScale(pub f32);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.)
    }
}

impl GravityScale {
    pub const ZERO: Self = Self(0.);
}

#[allow(non_camel_case_types)]
pub trait b2BodyCommands {
    fn spawn_body(&mut self, body_def: &b2BodyDef, fixture_def: b2Fixture) -> EntityCommands<'_>;

    fn spawn_multi_fixture_body(
        &mut self,
        body_def: &b2BodyDef,
        fixture_defs: &Vec<b2Fixture>,
        fixture_builder: fn(&mut EntityCommands),
    ) -> EntityCommands<'_>;
}

impl b2BodyCommands for Commands<'_, '_> {
    fn spawn_body(&mut self, body_def: &b2BodyDef, fixture_def: b2Fixture) -> EntityCommands<'_> {
        self.spawn((b2BodyBundle::new(body_def), fixture_def))
    }

    fn spawn_multi_fixture_body(
        &mut self,
        body_def: &b2BodyDef,
        fixture_defs: &Vec<b2Fixture>,
        fixture_builder: fn(&mut EntityCommands),
    ) -> EntityCommands<'_> {
        let mut entity_commands = self.spawn(b2BodyBundle::new(body_def));
        entity_commands.with_children(|builder| {
            for fixture in fixture_defs {
                fixture_builder(&mut builder.spawn(fixture.clone()));
            }
        });
        entity_commands
    }
}
