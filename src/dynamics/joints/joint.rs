use bevy::{ecs::reflect::ReflectComponent, prelude::{Component, Entity}, reflect::Reflect};
use libliquidfun_sys::box2d::ffi;

use crate::dynamics::b2WorldImpl;

#[allow(non_camel_case_types)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct b2Joint {
    joint_type: b2JointType,
    body_a: Entity,
    body_b: Entity,
    collide_connected: bool,
}

impl b2Joint {
    pub fn new(
        joint_type: b2JointType,
        body_a: Entity,
        body_b: Entity,
        collide_connected: bool,
    ) -> Self {
        Self {
            joint_type,
            body_a,
            body_b,
            collide_connected,
        }
    }

    pub fn joint_type(&self) -> &b2JointType {
        &self.joint_type
    }

    pub fn body_a(&self) -> &Entity {
        &self.body_a
    }

    pub fn body_b(&self) -> &Entity {
        &self.body_b
    }

    pub fn collide_connected(&self) -> bool {
        self.collide_connected
    }
}

pub(crate) trait ToJointPtr {
    fn create_ffi_joint(
        &self,
        b2_world: &mut b2WorldImpl,
        body_a: Entity,
        body_b: Entity,
        collide_connected: bool,
    ) -> JointPtr;
}

pub(crate) trait SyncJointToWorld {
    fn sync_to_world(&self, joint_ptr: &mut JointPtr);
}

#[allow(non_camel_case_types)]
#[derive(Debug, Reflect)]
#[type_path = "bevy_liquidfun"]
pub enum b2JointType {
    Revolute,
    Prismatic,
    Distance,
    Weld,
    Motor,
    _Pulley, // TODO
    _Mouse,
    _Gear,
    _Wheel,
    _Friction,
    _Area,
}

pub(crate) enum JointPtr {
    Revolute(*mut ffi::b2RevoluteJoint),
    Prismatic(*mut ffi::b2PrismaticJoint),
    Distance(*mut ffi::b2DistanceJoint),
    Weld(*mut ffi::b2WeldJoint),
    Motor(*mut ffi::b2MotorJoint),
    _Pulley, // TODO
    _Mouse,
    _Gear,
    _Wheel,
    _Friction,
    _Area,
}
