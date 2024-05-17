use std::pin::Pin;

use bevy::{ecs::system::EntityCommand, prelude::*};
use libliquidfun_sys::box2d::ffi;

use super::{b2Joint, b2JointType, SyncJointToWorld, ToJointPtr};
use crate::{
    dynamics::{b2WorldImpl, JointPtr},
    internal::to_b2Vec2,
};

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
pub struct b2WeldJoint {
    /// The local anchor point relative to bodyA's origin.
    pub local_anchor_a: Vec2,

    /// The local anchor point relative to bodyB's origin.
    pub local_anchor_b: Vec2,

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    pub reference_angle: f32,

    /// The rotational stiffness in N*m
    /// Disable softness with a value of 0
    pub stiffness: f32,

    /// The rotational damping in N*m*s
    pub damping: f32,
}

impl b2WeldJoint {
    pub fn new(def: &b2WeldJointDef) -> Self {
        Self {
            local_anchor_a: def.local_anchor_a,
            local_anchor_b: def.local_anchor_b,
            reference_angle: def.reference_angle,
            stiffness: def.stiffness,
            damping: def.damping,
        }
    }
}

impl ToJointPtr for b2WeldJoint {
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
            let ffi_joint = ffi::CreateWeldJoint(
                ffi_world,
                body_a,
                body_b,
                collide_connected,
                to_b2Vec2(&self.local_anchor_a),
                to_b2Vec2(&self.local_anchor_b),
                self.reference_angle,
                self.stiffness,
                self.damping,
            );
            JointPtr::Weld(ffi_joint)
        }
    }
}

impl SyncJointToWorld for b2WeldJoint {
    fn sync_to_world(&self, joint_ptr: &mut JointPtr) {
        let JointPtr::Weld(joint_ptr) = joint_ptr else {
            panic!("Expected joint of type b2WeldJoint")
        };
        let mut joint_ptr = unsafe { Pin::new_unchecked(joint_ptr.as_mut().unwrap()) };
        joint_ptr.as_mut().SetStiffness(self.stiffness);
        joint_ptr.as_mut().SetDamping(self.damping);
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Debug, Default)]
pub struct b2WeldJointDef {
    /// The local anchor point relative to bodyA's origin.
    pub local_anchor_a: Vec2,

    /// The local anchor point relative to bodyB's origin.
    pub local_anchor_b: Vec2,

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    pub reference_angle: f32,

    /// The rotational stiffness in N*m
    /// Disable softness with a value of 0
    pub stiffness: f32,

    /// The rotational damping in N*m*s
    pub damping: f32,
}

pub struct CreateWeldJoint {
    body_a: Entity,
    body_b: Entity,
    collide_connected: bool,
    def: b2WeldJointDef,
}

impl CreateWeldJoint {
    pub fn new(
        body_a: Entity,
        body_b: Entity,
        collide_connected: bool,
        def: &b2WeldJointDef,
    ) -> Self {
        Self {
            body_a,
            body_b,
            collide_connected,
            def: def.clone(),
        }
    }
}

impl EntityCommand for CreateWeldJoint {
    fn apply(self, id: Entity, world: &mut World) {
        let joint = b2Joint::new(
            b2JointType::Weld,
            self.body_a,
            self.body_b,
            self.collide_connected,
        );
        let weld_joint = b2WeldJoint::new(&self.def);
        world.entity_mut(id).insert((joint, weld_joint));
    }
}
