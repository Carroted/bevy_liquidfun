use std::os::raw::c_uint;

use bevy::{
    math::Vec2,
    prelude::{Component, Entity},
};
use libliquidfun_sys::box2d::{ffi, ffi::uint32};

use crate::{collision::b2Shape, internal::to_b2Vec2, particles::particle::b2ParticleFlags};

#[allow(non_camel_case_types)]
#[derive(Debug, Clone)]
pub struct b2ParticleGroupDef {
    pub flags: b2ParticleFlags,
    pub shape: b2Shape,
    pub position: Vec2,
    pub angle: f32,
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
}

impl Default for b2ParticleGroupDef {
    fn default() -> Self {
        Self {
            flags: b2ParticleFlags::WaterParticle,
            shape: b2Shape::Circle {
                radius: 1.0,
                position: Vec2::default(),
            },
            position: Vec2::default(),
            angle: 0.0,
            linear_velocity: Vec2::default(),
            angular_velocity: 0.0,
        }
    }
}

impl b2ParticleGroupDef {
    pub(crate) fn to_ffi<'a>(&self) -> &ffi::b2ParticleGroupDef {
        let ffi_shape = self.shape.to_ffi();
        let flags = self.flags.bits();
        let flags: c_uint = flags as c_uint;
        let flags = uint32::from(flags);
        unsafe {
            return ffi::CreateParticleGroupDef(
                flags,
                uint32::from(0),
                to_b2Vec2(&self.position),
                self.angle,
                to_b2Vec2(&self.linear_velocity),
                self.angular_velocity,
                1.,
                ffi_shape,
                0.,
                0.,
            )
            .as_ref()
            .unwrap();
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Component, Debug)]
pub struct b2ParticleGroup {
    particle_system_entity: Entity,
    definition: b2ParticleGroupDef,
}

impl b2ParticleGroup {
    pub fn new(particle_system_entity: Entity, def: &b2ParticleGroupDef) -> b2ParticleGroup {
        b2ParticleGroup {
            particle_system_entity,
            definition: def.clone(),
        }
    }

    pub fn get_particle_system_entity(&self) -> Entity {
        self.particle_system_entity
    }

    pub fn get_definition(&self) -> &b2ParticleGroupDef {
        &self.definition
    }
}
