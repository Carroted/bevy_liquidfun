use std::time::Duration;

use bevy::{
    ecs::schedule::{InternedSystemSet, ScheduleLabel},
    prelude::*,
    transform::TransformSystem,
};

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

#[derive(SystemSet, Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
struct DefaultSystemSet;

pub struct LiquidFunSchedulePlugin {
    pub set_to_run_in: InternedSystemSet,
}

impl LiquidFunSchedulePlugin {
    pub fn new(set_to_run_in: Option<InternedSystemSet>) -> Self {
        Self {
            set_to_run_in: set_to_run_in.unwrap_or(DefaultSystemSet.intern()),
        }
    }
}

impl Plugin for LiquidFunSchedulePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Time::new_with(PhysicsTime))
            .insert_resource(PhysicsTimeAccumulator(0.));

        app.configure_sets(
            Update,
            PhysicsUpdate
                .in_set(self.set_to_run_in)
                .before(TransformSystem::TransformPropagate),
        );

        app.edit_schedule(PhysicsSchedule, |physics_schedule| {
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
        });

        app.add_systems(Update, run_liquidfun_schedule.in_set(PhysicsUpdate));
    }
}

#[derive(Resource, Default)]
pub(crate) struct PhysicsTimeAccumulator(pub f32);

fn run_liquidfun_schedule(world: &mut World) {
    let _ = world.try_schedule_scope(PhysicsSchedule, |world, schedule| {
        *world.resource_mut::<Time>() = world.resource::<Time<PhysicsTime>>().as_generic();

        let real_delta = world.resource::<Time<Virtual>>().delta_seconds();
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
