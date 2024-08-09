use std::pin::Pin;

use autocxx::WithinBox;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectComponent,
    },
    prelude::{Component, Entity},
    reflect::Reflect,
    utils::default,
};
use libliquidfun_sys::box2d::{
    ffi,
    ffi::{int16, uint16},
};

use crate::collision::b2Shape;

#[allow(non_camel_case_types)]
#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct b2Fixture {
    pub shape: b2Shape,
    pub density: f32,
    pub friction: f32,
    pub restitution: f32,
    pub restitution_threshold: f32,
    pub is_sensor: bool,
    pub filter: b2Filter,
}

impl b2Fixture {
    pub fn new(shape: b2Shape, density: f32) -> Self {
        b2Fixture {
            shape,
            density,
            ..default()
        }
    }

    pub(crate) fn to_ffi(&self) -> Pin<Box<ffi::b2FixtureDef>> {
        let mut b2fixture_def = ffi::b2FixtureDef::new().within_box();
        b2fixture_def.shape = self.shape.to_ffi();
        b2fixture_def.density = self.density;
        b2fixture_def.friction = self.friction;
        b2fixture_def.restitution = self.restitution;
        b2fixture_def.restitutionThreshold = self.restitution_threshold;
        b2fixture_def.isSensor = self.is_sensor;
        b2fixture_def.filter.categoryBits = uint16::from(self.filter.category);
        b2fixture_def.filter.maskBits = uint16::from(self.filter.mask);
        b2fixture_def.filter.groupIndex = int16::from(self.filter.group_index);
        return b2fixture_def;
    }
}

impl Default for b2Fixture {
    fn default() -> Self {
        b2Fixture {
            shape: b2Shape::default(),
            density: 0.,
            friction: 0.2,
            restitution: 0.,
            restitution_threshold: 1.,
            is_sensor: false,
            filter: b2Filter::default(),
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct b2FixtureBody {
    pub(crate) body: Entity,
}

impl b2FixtureBody {
    pub(crate) fn new(body: Entity) -> Self {
        Self { body }
    }

    pub fn body(&self) -> Entity {
        self.body
    }
}

impl MapEntities for b2FixtureBody {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.body = entity_mapper.map_entity(self.body);
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, Reflect)]
#[type_path = "bevy_liquidfun"]
pub struct b2Filter {
    pub category: u16,
    pub mask: u16,
    pub group_index: i16,
}

impl Default for b2Filter {
    fn default() -> Self {
        Self {
            category: 0x0001,
            mask: 0xFFFF,
            group_index: 0,
        }
    }
}
