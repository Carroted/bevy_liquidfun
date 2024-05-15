use crate::dynamics::{b2Joint, b2JointType, b2WorldImpl, JointPtr};
use crate::internal::to_b2Vec2;
use bevy::ecs::system::EntityCommand;
use bevy::prelude::*;
use libliquidfun_sys::box2d::ffi;
use std::pin::Pin;

use super::{SyncJointToWorld, ToJointPtr};

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
pub struct b2MotorJoint {
    /// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
    pub linear_offset: Vec2,

    /// The bodyB angle minus bodyA angle in radians.
    pub angular_offset: f32,

    /// The maximum motor force in N.
    pub max_force: f32,

    /// The maximum motor torque in N-m.
    pub max_torque: f32,

    /// Position correction factor in the range [0,1].
    pub correction_factor: f32,
}

impl b2MotorJoint {
    pub fn new(def: &b2MotorJointDef) -> Self {
        Self {
            linear_offset: def.linear_offset,
            angular_offset: def.angular_offset,
            max_force: def.max_force,
            max_torque: def.max_torque,
            correction_factor: def.correction_factor,
        }
    }
}

impl SyncJointToWorld for b2MotorJoint {
    fn sync_to_world(&self, joint_ptr: &mut JointPtr) {
        let JointPtr::Motor(joint_ptr) = joint_ptr else { panic!("Expected joint of type b2MotorJoint") };
        let mut joint_ptr = unsafe { Pin::new_unchecked(joint_ptr.as_mut().unwrap()) };
        joint_ptr
            .as_mut()
            .SetLinearOffset(&to_b2Vec2(&self.linear_offset));
        joint_ptr.as_mut().SetAngularOffset(self.angular_offset);
        joint_ptr.as_mut().SetMaxForce(self.max_force);
        joint_ptr.as_mut().SetMaxTorque(self.max_torque);
        joint_ptr
            .as_mut()
            .SetCorrectionFactor(self.correction_factor);
    }
}

impl ToJointPtr for b2MotorJoint {
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
            let ffi_joint = ffi::CreateMotorJoint(
                ffi_world,
                body_a,
                body_b,
                collide_connected,
                to_b2Vec2(&self.linear_offset),
                self.angular_offset,
                self.max_force,
                self.max_torque,
                self.correction_factor,
            );
            JointPtr::Motor(ffi_joint)
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Debug)]
pub struct b2MotorJointDef {
    /// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
    pub linear_offset: Vec2,

    /// The bodyB angle minus bodyA angle in radians.
    pub angular_offset: f32,

    /// The maximum motor force in N.
    pub max_force: f32,

    /// The maximum motor torque in N-m.
    pub max_torque: f32,

    /// Position correction factor in the range [0,1].
    pub correction_factor: f32,
}

impl Default for b2MotorJointDef {
    fn default() -> Self {
        Self {
            linear_offset: Vec2::ZERO,
            angular_offset: 0.,
            max_force: 1.0,
            max_torque: 1.0,
            correction_factor: 0.3,
        }
    }
}

pub struct CreateMotorJoint {
    body_a: Entity,
    body_b: Entity,
    collide_connected: bool,
    def: b2MotorJointDef,
}

impl CreateMotorJoint {
    pub fn new(
        body_a: Entity,
        body_b: Entity,
        collide_connected: bool,
        def: &b2MotorJointDef,
    ) -> Self {
        Self {
            body_a,
            body_b,
            collide_connected,
            def: def.clone(),
        }
    }
}

impl EntityCommand for CreateMotorJoint {
    fn apply(self, id: Entity, world: &mut World) {
        let joint = b2Joint::new(
            b2JointType::Motor,
            self.body_a,
            self.body_b,
            self.collide_connected,
        );
        let motor_joint = b2MotorJoint::new(&self.def);
        world.entity_mut(id).insert((joint, motor_joint));
    }
}
