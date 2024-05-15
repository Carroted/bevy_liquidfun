use crate::dynamics::{b2WorldImpl, JointPtr};
use crate::internal::to_b2Vec2;
use bevy::ecs::system::EntityCommand;
use bevy::prelude::*;
use libliquidfun_sys::box2d::ffi;
use std::pin::Pin;

use super::{b2Joint, b2JointType, SyncJointToWorld, ToJointPtr};

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
pub struct b2DistanceJoint {
    /// The local anchor point relative to bodyA's origin.
    pub local_anchor_a: Vec2,

    /// The local anchor point relative to bodyB's origin.
    pub local_anchor_b: Vec2,

    /// The rest length of this joint. Clamped to a stable minimum value.
    pub length: f32,

    /// Minimum length. Clamped to a stable minimum value.
    pub min_length: f32,

    /// Maximum length. Must be greater than or equal to the minimum length.
    pub max_length: f32,

    /// The linear stiffness in N/m.
    pub stiffness: f32,

    /// The linear damping in N*s/m.
    pub damping: f32,
}

impl b2DistanceJoint {
    pub fn new(def: &b2DistanceJointDef) -> Self {
        Self {
            local_anchor_a: def.local_anchor_a,
            local_anchor_b: def.local_anchor_b,
            length: def.length,
            min_length: def.min_length,
            max_length: def.max_length,
            stiffness: def.stiffness,
            damping: def.damping,
        }
    }
}

impl ToJointPtr for b2DistanceJoint {
    fn create_ffi_joint(
        &self,
        b2_world: &mut b2WorldImpl,
        body_a: Entity,
        body_b: Entity,
        collide_connected: bool,
    ) -> JointPtr {
        unsafe {
            let body_a = b2_world.body_ptr_mut(body_a).unwrap();
            let body_a = body_a.get_unchecked_mut() as *mut ffi::b2Body;
            let body_b = b2_world.body_ptr_mut(body_b).unwrap();
            let body_b = body_b.get_unchecked_mut() as *mut ffi::b2Body;
            let ffi_world = b2_world.get_world_ptr().as_mut();
            let ffi_joint = ffi::CreateDistanceJoint(
                ffi_world,
                body_a,
                body_b,
                collide_connected,
                to_b2Vec2(&self.local_anchor_a),
                to_b2Vec2(&self.local_anchor_b),
                self.length,
                self.min_length,
                self.max_length,
                self.stiffness,
                self.damping,
            );
            JointPtr::Distance(ffi_joint)
        }
    }
}

impl SyncJointToWorld for b2DistanceJoint {
    fn sync_to_world(&self, joint_ptr: &mut JointPtr) {
        let JointPtr::Distance(joint_ptr) = joint_ptr else {
            panic!("Expected joint of type b2DistanceJoint")
        };
        let mut joint_ptr = unsafe { Pin::new_unchecked(joint_ptr.as_mut().unwrap()) };
        joint_ptr.as_mut().SetLength(self.length);
        joint_ptr.as_mut().SetMinLength(self.min_length);
        joint_ptr.as_mut().SetMaxLength(self.max_length);
        joint_ptr.as_mut().SetStiffness(self.stiffness);
        joint_ptr.as_mut().SetDamping(self.damping);
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Debug, Default)]
pub struct b2DistanceJointDef {
    /// The local anchor point relative to bodyA's origin.
    pub local_anchor_a: Vec2,

    /// The local anchor point relative to bodyB's origin.
    pub local_anchor_b: Vec2,

    /// The rest length of this joint. Clamped to a stable minimum value.
    pub length: f32,

    /// Minimum length. Clamped to a stable minimum value.
    pub min_length: f32,

    /// Maximum length. Must be greater than or equal to the minimum length.
    pub max_length: f32,

    /// The linear stiffness in N/m.
    pub stiffness: f32,

    /// The linear damping in N*s/m.
    pub damping: f32,
}

pub struct CreateDistanceJoint {
    body_a: Entity,
    body_b: Entity,
    collide_connected: bool,
    def: b2DistanceJointDef,
}

impl CreateDistanceJoint {
    pub fn new(
        body_a: Entity,
        body_b: Entity,
        collide_connected: bool,
        def: &b2DistanceJointDef,
    ) -> Self {
        Self {
            body_a,
            body_b,
            collide_connected,
            def: def.clone(),
        }
    }
}

impl EntityCommand for CreateDistanceJoint {
    fn apply(self, id: Entity, world: &mut World) {
        let joint = b2Joint::new(
            b2JointType::Distance,
            self.body_a,
            self.body_b,
            self.collide_connected,
        );
        let distance_joint = b2DistanceJoint::new(&self.def);
        world.entity_mut(id).insert((joint, distance_joint));
    }
}
