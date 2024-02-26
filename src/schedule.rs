use std::time::Duration;

use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;
use bevy::transform::TransformSystem;

use crate::dynamics::b2WorldSettings;

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct PhysicsUpdate;

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone, Default)]
pub enum PhysicsUpdateStep {
    ClearEvents,
    #[default]
    UserCode,
    SyncToPhysicsWorld,
    ApplyForces,
    Step,
    SyncFromPhysicsWorld,
}

#[derive(Default, Debug)]
pub struct PhysicsTime;

#[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
pub struct PhysicsSchedule;

pub struct LiquidFunSchedulePlugin;

impl Plugin for LiquidFunSchedulePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Time::new_with(PhysicsTime))
            .insert_resource(PhysicsTimeAccumulator(0.));

        app.configure_sets(
            Update,
            PhysicsUpdate.before(TransformSystem::TransformPropagate),
        );

        let mut physics_schedule = Schedule::new(PhysicsSchedule);
        physics_schedule.configure_sets(
            (
                PhysicsUpdateStep::ClearEvents,
                PhysicsUpdateStep::UserCode,
                PhysicsUpdateStep::SyncToPhysicsWorld,
                PhysicsUpdateStep::ApplyForces,
                PhysicsUpdateStep::Step,
                PhysicsUpdateStep::SyncFromPhysicsWorld,
            )
                .chain(),
        );
        app.add_schedule(physics_schedule);
        app.add_systems(Update, run_liquidfun_schedule.in_set(PhysicsUpdate));
    }
}

#[derive(Resource, Default)]
pub(crate) struct PhysicsTimeAccumulator(pub f32);

fn run_liquidfun_schedule(world: &mut World) {
    let _ = world.try_schedule_scope(PhysicsSchedule, |world, schedule| {
        *world.resource_mut::<Time>() = world.resource::<Time<PhysicsTime>>().as_generic();

        let real_delta = world.resource::<Time<Real>>().delta_seconds();
        let max_frame_delta = world.resource::<b2WorldSettings>().max_frame_delta;
        let real_delta = real_delta.min(max_frame_delta);
        let wanted_time_step = world.resource::<b2WorldSettings>().time_step;
        world.resource_scope(
            |world, mut physics_time_accumulator: Mut<PhysicsTimeAccumulator>| {
                physics_time_accumulator.0 += real_delta;

                while physics_time_accumulator.0 >= wanted_time_step {
                    world
                        .resource_mut::<Time<PhysicsTime>>()
                        .advance_by(Duration::from_secs_f32(wanted_time_step));
                    schedule.run(world);
                    physics_time_accumulator.0 -= wanted_time_step;
                }
            },
        );

        *world.resource_mut::<Time>() = world.resource::<Time<Virtual>>().as_generic();
    });
}
