use std::{borrow::BorrowMut, ops::Deref, pin::Pin};

use bevy::{
    color::palettes::css::{GREEN, RED},
    ecs::{component::Tick, schedule::InternedSystemSet},
    prelude::*,
    transform::TransformSystem,
    utils::{HashMap, HashSet},
};
use libliquidfun_sys::box2d::ffi::int32;

use crate::{
    collision::b2Shape,
    dynamics::{
        b2BeginContactEvent,
        b2BodiesInContact,
        b2Body,
        b2BodyType,
        b2Contact,
        b2Contacts,
        b2DistanceJoint,
        b2EndContactEvent,
        b2Fixture,
        b2FixturesInContact,
        b2Joint,
        b2MotorJoint,
        b2ParticleBodyContact,
        b2ParticlesInContact,
        b2PrismaticJoint,
        b2RevoluteJoint,
        b2WeldJoint,
        b2World,
        b2WorldSettings,
        ExternalForce,
        ExternalImpulse,
        ExternalTorque,
        GravityScale,
        SyncJointToWorld,
        ToJointPtr,
    },
    internal::to_b2Vec2,
    particles::{b2ParticleGroup, b2ParticleSystem, b2ParticleSystemContacts},
    schedule::{
        LiquidFunSchedulePlugin,
        PhysicsSchedule,
        PhysicsTimeAccumulator,
        PhysicsUpdate,
        PhysicsUpdateStep,
    },
    utils::{DebugDrawFixtures, DebugDrawParticleSystem},
};

#[derive(Default)]
pub struct LiquidFunPlugin {
    settings: b2WorldSettings,
    set_to_run_in: Option<InternedSystemSet>,
}

impl LiquidFunPlugin {
    pub fn new(settings: b2WorldSettings) -> Self {
        Self {
            settings,
            set_to_run_in: None,
        }
    }

    pub fn new_with_set(settings: b2WorldSettings, set_to_run_in: impl SystemSet) -> Self {
        Self {
            settings,
            set_to_run_in: Some(set_to_run_in.intern()),
        }
    }
}

impl Plugin for LiquidFunPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(LiquidFunSchedulePlugin::new(self.set_to_run_in))
            .insert_resource(self.settings.clone())
            .init_resource::<b2Contacts>()
            .register_type::<b2Body>()
            .register_type::<b2BodyType>()
            .register_type::<HashSet<Entity>>()
            .register_type::<b2Fixture>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalImpulse>()
            .register_type::<ExternalTorque>()
            .register_type::<b2Joint>()
            .register_type::<b2DistanceJoint>()
            .register_type::<b2MotorJoint>()
            .register_type::<b2PrismaticJoint>()
            .register_type::<b2RevoluteJoint>()
            .register_type::<b2WeldJoint>()
            .register_type::<b2WorldSettings>()
            .register_type::<DebugDrawFixtures>()
            .add_systems(
                PhysicsSchedule,
                (
                    (
                        clear_forces,
                        clear_impulses,
                        clear_torques,
                        clear_events::<b2BeginContactEvent>,
                        clear_events::<b2EndContactEvent>,
                        (
                            destroy_removed_joints,
                            destroy_removed_fixtures,
                            destroy_removed_bodies,
                        )
                            .chain(),
                    )
                        .run_if(resource_exists::<b2World>)
                        .in_set(PhysicsUpdateStep::ClearEvents),
                    (
                        create_bodies,
                        create_fixtures,
                        (
                            create_joints::<b2DistanceJoint>,
                            create_joints::<b2MotorJoint>,
                            create_joints::<b2PrismaticJoint>,
                            create_joints::<b2RevoluteJoint>,
                            create_joints::<b2WeldJoint>,
                        )
                            .chain(),
                        create_particle_systems,
                        create_particle_groups,
                        create_queued_particles,
                        destroy_removed_joints,
                        destroy_removed_fixtures,
                        destroy_removed_bodies,
                        destroy_queued_particles,
                        apply_particle_forces,
                        apply_deferred,
                        sync_bodies_to_world,
                        (
                            sync_joints_to_world::<b2DistanceJoint>,
                            sync_joints_to_world::<b2MotorJoint>,
                            sync_joints_to_world::<b2PrismaticJoint>,
                            sync_joints_to_world::<b2RevoluteJoint>,
                            sync_joints_to_world::<b2WeldJoint>,
                        )
                            .chain(),
                    )
                        .chain()
                        .in_set(PhysicsUpdateStep::SyncToPhysicsWorld),
                    (
                        apply_forces,
                        apply_impulses,
                        apply_torques,
                        apply_gravity_scale,
                    )
                        .chain()
                        .in_set(PhysicsUpdateStep::ApplyForces),
                    step_physics.in_set(PhysicsUpdateStep::Step),
                    (
                        sync_bodies_from_world,
                        sync_particle_systems_from_world,
                        send_contact_events,
                        copy_particle_system_contacts,
                        update_particle_body_contacts_components
                            .after(copy_particle_system_contacts),
                        copy_contacts,
                        update_bodies_in_contact_components.after(copy_contacts),
                        update_fixtures_in_contact_components.after(copy_contacts),
                    )
                        .in_set(PhysicsUpdateStep::SyncFromPhysicsWorld),
                )
                    .run_if(resource_exists::<b2World>),
            )
            .add_systems(
                Update,
                update_transforms
                    .after(PhysicsUpdate)
                    .before(TransformSystem::TransformPropagate)
                    .run_if(resource_exists::<b2World>),
            )
            .add_systems(
                Last,
                (
                    destroy_removed_joints,
                    destroy_removed_fixtures,
                    destroy_removed_bodies,
                )
                    .run_if(resource_exists::<b2World>),
            )
            .init_resource::<Events<b2BeginContactEvent>>()
            .init_resource::<Events<b2EndContactEvent>>()
            .init_resource::<BodyChangeTracker>();
    }
}

#[derive(Resource, Debug, Default)]
struct BodyChangeTracker {
    last_sync_ticks: HashMap<Entity, Tick>,
}

fn step_physics(mut b2_world: ResMut<b2World>, settings: Res<b2WorldSettings>) {
    b2_world.inner().step(
        settings.time_step,
        settings.velocity_iterations,
        settings.position_iterations,
        settings.particle_iterations,
    );
}

fn clear_forces(mut external_forces: Query<&mut ExternalForce>) {
    for mut force in external_forces.iter_mut() {
        force.clear()
    }
}

fn clear_impulses(mut external_impulses: Query<&mut ExternalImpulse>) {
    for mut impulse in external_impulses.iter_mut() {
        impulse.clear()
    }
}

fn clear_torques(mut external_torques: Query<&mut ExternalTorque>) {
    for mut external_torques in external_torques.iter_mut() {
        external_torques.torque = 0.;
    }
}

fn clear_events<T: 'static + Send + Sync + Event>(mut events: ResMut<Events<T>>) {
    events.clear();
}

fn create_bodies(
    mut b2_world: ResMut<b2World>,
    mut added: Query<(Entity, &mut b2Body), Added<b2Body>>,
    mut body_change_tracker: ResMut<BodyChangeTracker>,
) {
    for (entity, mut body) in added.iter_mut() {
        b2_world.inner().create_body(entity, &mut body);
        body_change_tracker
            .last_sync_ticks
            .insert(entity, body.last_changed());
    }
}

fn create_fixtures(
    mut b2_world: ResMut<b2World>,
    mut added: Query<(Entity, &mut b2Fixture), Added<b2Fixture>>,
    mut bodies: Query<(Entity, &mut b2Body)>,
) {
    for (fixture_entity, mut fixture) in added.iter_mut() {
        let (body_entity, mut body) = bodies.get_mut(fixture.body()).unwrap();
        b2_world
            .inner()
            .create_fixture((fixture_entity, &mut fixture), (body_entity, &mut body));
    }
}

fn create_joints<T: Component + ToJointPtr>(
    mut b2_world: ResMut<b2World>,
    mut added: Query<(Entity, &b2Joint, &T), Added<T>>,
    mut bodies: Query<(Entity, &mut b2Body)>,
) {
    for (joint_entity, joint, revolute_joint) in added.iter_mut() {
        let [mut body_a, mut body_b] = bodies
            .get_many_mut([*joint.body_a(), *joint.body_b()])
            .unwrap();
        let mut b2_world_impl = b2_world.inner();
        let joint_ptr = revolute_joint.create_ffi_joint(
            b2_world_impl.borrow_mut(),
            body_a.0,
            body_b.0,
            joint.collide_connected(),
        );
        b2_world_impl.register_joint(
            (joint_entity, &joint, joint_ptr),
            (body_a.0, &mut body_a.1),
            (body_b.0, &mut body_b.1),
        );
    }
}

fn create_particle_systems(
    mut commands: Commands,
    mut b2_world: ResMut<b2World>,
    mut added: Query<(Entity, &mut b2ParticleSystem), Added<b2ParticleSystem>>,
) {
    for (entity, mut particle_system) in added.iter_mut() {
        b2_world
            .inner()
            .create_particle_system(entity, &mut particle_system);
        commands
            .entity(entity)
            .insert(b2ParticleSystemContacts::default());
    }
}

fn create_particle_groups(
    mut b2_world: ResMut<b2World>,
    mut added_groups: Query<(Entity, &mut b2ParticleGroup), Added<b2ParticleGroup>>,
) {
    for (entity, mut particle_group) in added_groups.iter_mut() {
        b2_world.inner().create_particle_group(
            particle_group.get_particle_system_entity(),
            entity,
            &mut particle_group,
        );
    }
}

fn create_queued_particles(
    mut b2_world: ResMut<b2World>,
    mut query: Query<(Entity, &mut b2ParticleSystem)>,
) {
    for (entity, mut particle_system) in &mut query {
        let mut particle_system_ptr = b2_world.inner().particle_system_ptr_mut(entity).unwrap();
        particle_system.process_creation_queue(particle_system_ptr.as_mut());
    }
}

fn destroy_removed_bodies(
    mut b2_world: ResMut<b2World>,
    mut removed: RemovedComponents<b2Body>,
    mut commands: Commands,
    mut body_change_tracker: ResMut<BodyChangeTracker>,
) {
    let mut b2_world_impl = b2_world.inner();
    for entity in removed.read() {
        let fixture_entities = b2_world_impl.get_fixtures_attached_to_entity(&entity);
        if let Some(fixture_entities) = fixture_entities {
            fixture_entities.iter().for_each(|fixture_entity| {
                if *fixture_entity == entity {
                    // fixture was on same entity as body, this is ok
                } else if let Some(fixture_entity) = commands.get_entity(*fixture_entity) {
                    fixture_entity.despawn_recursive();
                } else {
                    warn!("Destroyed fixture entity found: {fixture_entity:?}");
                }
            });
        }

        b2_world_impl.destroy_body_for_entity(entity);
        body_change_tracker.last_sync_ticks.remove(&entity);
    }
}

fn destroy_removed_joints(mut b2_world: ResMut<b2World>, mut removed: RemovedComponents<b2Joint>) {
    let mut b2_world_impl = b2_world.inner();
    for entity in removed.read() {
        b2_world_impl.destroy_joint(entity);
    }
}

fn destroy_queued_particles(
    mut b2_world: ResMut<b2World>,
    mut query: Query<(Entity, &mut b2ParticleSystem)>,
) {
    for (entity, mut particle_system) in &mut query {
        let mut particle_system_ptr = b2_world.inner().particle_system_ptr_mut(entity).unwrap();
        particle_system.process_destruction_queue(particle_system_ptr.as_mut());
    }
}

fn destroy_removed_fixtures(
    mut b2_world: ResMut<b2World>,
    mut removed: RemovedComponents<b2Fixture>,
) {
    for entity in removed.read() {
        b2_world.inner().destroy_fixture_for_entity(entity);
    }
}

fn sync_bodies_to_world(
    mut b2_world: ResMut<b2World>,
    bodies: Query<(Entity, Ref<b2Body>), Changed<b2Body>>,
    change_tracker: Res<BodyChangeTracker>,
) {
    let mut b2_world_impl = b2_world.inner();
    for (entity, body) in bodies.iter() {
        if let Some(last_changed_by_box2d) = change_tracker.last_sync_ticks.get(&entity) {
            if *last_changed_by_box2d == body.last_changed() {
                continue;
            }
        }

        body.as_ref()
            .sync_to_world(entity, b2_world_impl.borrow_mut());
    }
}

fn sync_joints_to_world<T: Component + SyncJointToWorld>(
    mut b2_world: ResMut<b2World>,
    joints: Query<(Entity, &T), Changed<T>>,
) {
    let mut b2_world_impl = b2_world.inner();
    for (entity, joint) in joints.iter() {
        let joint_ptr = b2_world_impl.joint_ptr_mut(&entity).unwrap();
        joint.sync_to_world(joint_ptr);
    }
}

fn apply_forces(mut b2_world: ResMut<b2World>, external_forces: Query<(Entity, &ExternalForce)>) {
    let mut b2_world_impl = b2_world.inner();
    for (entity, external_force) in external_forces.iter() {
        let body_ptr = b2_world_impl.body_ptr_mut(entity);
        if let Some(mut body_ptr) = body_ptr {
            body_ptr.as_mut().ApplyForceToCenter(
                &to_b2Vec2(&external_force.force()),
                external_force.should_wake,
            );
            body_ptr
                .as_mut()
                .ApplyTorque(external_force.torque(), false);
        } else {
            warn!(
                "Encountered ExternalForce component on an Entity without a matching b2Body: {:?}",
                entity
            );
        }
    }
}

fn apply_impulses(
    mut b2_world: ResMut<b2World>,
    external_impulses: Query<(Entity, &ExternalImpulse)>,
) {
    let mut b2_world_impl = b2_world.inner();
    for (entity, external_impulse) in external_impulses.iter() {
        let body_ptr = b2_world_impl.body_ptr_mut(entity);
        if let Some(mut body_ptr) = body_ptr {
            body_ptr.as_mut().ApplyLinearImpulseToCenter(
                &to_b2Vec2(&external_impulse.impulse()),
                external_impulse.should_wake,
            );
            body_ptr
                .as_mut()
                .ApplyAngularImpulse(external_impulse.angular_impulse(), false);
        } else {
            warn!(
                "Encountered ExternalImpulse component on an Entity without a matching b2Body: {:?}",
                entity
            );
        }
    }
}

fn apply_torques(
    mut b2_world: ResMut<b2World>,
    external_torques: Query<(Entity, &ExternalTorque)>,
) {
    let mut b2_world_impl = b2_world.inner();
    for (entity, external_torque) in external_torques.iter() {
        let body_ptr = b2_world_impl.body_ptr_mut(entity);
        if let Some(mut body_ptr) = body_ptr {
            body_ptr
                .as_mut()
                .ApplyTorque(external_torque.torque, external_torque.should_wake);
        } else {
            warn!(
                "Encountered ExternalTorque component on an Entity without a matching b2Body: {:?}",
                entity
            );
        }
    }
}

fn apply_gravity_scale(
    mut b2_world: ResMut<b2World>,
    gravity_scales: Query<(Entity, &GravityScale)>,
) {
    let mut b2_world_impl = b2_world.inner();
    for (entity, gravity_scale) in gravity_scales.iter() {
        let body_ptr = b2_world_impl.body_ptr_mut(entity);
        if let Some(mut body_ptr) = body_ptr {
            body_ptr.as_mut().SetGravityScale(gravity_scale.0);
        } else {
            warn!(
                "Encountered GravityScale component on an Entity without a matching b2Body: {:?}",
                entity
            );
        }
    }
}

fn sync_bodies_from_world(
    mut b2_world: ResMut<b2World>,
    mut bodies: Query<(Entity, &mut b2Body)>,
    mut body_change_tracker: ResMut<BodyChangeTracker>,
) {
    let b2_world_impl = b2_world.inner();
    for (entity, mut body) in bodies.iter_mut() {
        body.sync_with_world(entity, &b2_world_impl);
        body_change_tracker
            .last_sync_ticks
            .insert(entity, body.last_changed());
    }
}

fn sync_particle_systems_from_world(
    mut b2_world: ResMut<b2World>,
    mut particle_systems: Query<(Entity, &mut b2ParticleSystem)>,
) {
    let b2_world_impl = b2_world.inner();
    for (entity, mut particle_system) in particle_systems.iter_mut() {
        particle_system.sync_with_world(entity, &b2_world_impl);
    }
}

fn send_contact_events(
    mut begin_contact_events: EventWriter<b2BeginContactEvent>,
    mut end_contact_events: EventWriter<b2EndContactEvent>,
    mut b2_world: ResMut<b2World>,
) {
    let mut b2_world_impl = b2_world.inner();
    let contact_listener = b2_world_impl.contact_listener();

    {
        let contact_listener = contact_listener.borrow();
        let fixture_contacts = contact_listener.fixture_contacts();
        let ended_contacts = contact_listener.ended_fixture_contacts();
        for key in contact_listener.begun_fixture_contacts() {
            // if the contact is not available in fixture contacts anymore, the contact has ended during the same frame
            let contact = fixture_contacts.get(key).or(ended_contacts.get(key));
            if let Some(contact) = contact {
                begin_contact_events.send(b2BeginContactEvent(contact.clone()));
            }
        }

        for contact in ended_contacts.values() {
            end_contact_events.send(b2EndContactEvent(contact.clone()));
        }
    }

    let mut contact_listener = contact_listener.deref().borrow_mut();
    contact_listener.clear_contact_changes();
}

fn copy_contacts(mut b2_world: ResMut<b2World>, mut contacts: ResMut<b2Contacts>) {
    let contacts = contacts.contacts_mut();
    contacts.clear();

    let mut b2_world_impl = b2_world.inner();
    let world_ptr = b2_world_impl.get_world_ptr();
    let contact_count = i32::from(int32::from(world_ptr.as_ref().GetContactCount())) as usize;

    if contact_count == 0 {
        return;
    }
    let mut ffi_contact = unsafe {
        let Some(ffi_contacts) = world_ptr.as_mut().GetContactList().as_mut() else {
            return;
        };

        ffi_contacts
    };

    for _ in 0..contact_count {
        let contact = b2Contact::from_ffi_contact(ffi_contact);
        contacts.push(contact);

        unsafe {
            if let Some(next) = Pin::new_unchecked(ffi_contact).as_mut().GetNext().as_mut() {
                ffi_contact = next;
            } else {
                break;
            };
        }
    }
}

fn copy_particle_system_contacts(
    mut b2_world: ResMut<b2World>,
    mut particle_systems: Query<(Entity, &mut b2ParticleSystemContacts)>,
) {
    for (entity, mut particle_system_contacts) in &mut particle_systems {
        let new_body_contacts = particle_system_contacts.body_contacts_mut();
        new_body_contacts.clear();

        let b2_world_impl = b2_world.inner();
        let particle_system_ptr = b2_world_impl.particle_system_ptr(entity).unwrap();
        let body_contacts = unsafe {
            let body_contacts = particle_system_ptr.as_ref().GetBodyContacts();
            let count = i32::from(int32::from(
                particle_system_ptr.as_ref().GetBodyContactCount(),
            )) as usize;
            if body_contacts.is_null() || count == 0 {
                continue;
            }
            std::slice::from_raw_parts(body_contacts, count)
        };

        new_body_contacts.extend(
            body_contacts
                .iter()
                .map(|c| b2ParticleBodyContact::from_ffi_contact(c)),
        );
    }
}

fn update_fixtures_in_contact_components(
    mut fixtures_in_contact_components: Query<
        (Entity, &mut b2FixturesInContact),
        Or<(With<b2Body>, With<b2Fixture>)>,
    >,
    contacts: Res<b2Contacts>,
) {
    for (entity, mut fixtures_in_contact) in &mut fixtures_in_contact_components {
        let fixtures = fixtures_in_contact.contacts_mut();
        fixtures.clear();
        for contact in contacts.contacts() {
            if contact.fixture_a == entity || contact.body_a == entity {
                fixtures.insert(contact.fixture_b);
            } else if contact.fixture_b == entity || contact.body_b == entity {
                fixtures.insert(contact.fixture_a);
            }
        }
    }
}

fn update_bodies_in_contact_components(
    mut bodies_in_contact_components: Query<
        (Entity, &mut b2BodiesInContact),
        Or<(With<b2Body>, With<b2Fixture>)>,
    >,
    contacts: Res<b2Contacts>,
) {
    for (entity, mut bodies_in_contact) in &mut bodies_in_contact_components {
        let bodies = bodies_in_contact.contacts_mut();
        bodies.clear();
        for contact in contacts.contacts() {
            if contact.fixture_a == entity || contact.body_a == entity {
                bodies.insert(contact.body_b);
            } else if contact.fixture_b == entity || contact.body_b == entity {
                bodies.insert(contact.body_a);
            }
        }
    }
}

fn update_particle_body_contacts_components(
    mut particle_contact_components: Query<(Entity, &mut b2ParticlesInContact)>,
    particle_system_contacts: Query<&b2ParticleSystemContacts>,
) {
    for (_, mut particle_contact_component) in &mut particle_contact_components {
        particle_contact_component.contacts_mut().clear();
    }

    for particle_system in &particle_system_contacts {
        for contact in particle_system.body_contacts() {
            for (entity, mut particle_contact_component) in &mut particle_contact_components {
                if contact.body == entity || contact.fixture == entity {
                    particle_contact_component
                        .contacts_mut()
                        .insert(contact.particle_index);
                }
            }
        }
    }
}

fn apply_particle_forces(
    mut b2_world: ResMut<b2World>,
    mut query: Query<(Entity, &mut b2ParticleSystem)>,
) {
    for (entity, mut particle_system) in &mut query {
        let mut particle_system_ptr = b2_world.inner().particle_system_ptr_mut(entity).unwrap();
        particle_system.apply_particle_forces(particle_system_ptr.as_mut());
    }
}

fn update_transforms(
    mut bodies: Query<(&b2Body, &mut Transform)>,
    physics_time_accumulator: Res<PhysicsTimeAccumulator>,
) {
    let extrapolation_time = physics_time_accumulator.0;
    for (body, mut transform) in bodies.iter_mut() {
        let extrapolated_position = body.position + body.linear_velocity * extrapolation_time;
        transform.translation = extrapolated_position.extend(0.);
        let extrapolated_rotation = body.angle + body.angular_velocity * extrapolation_time;
        transform.rotation = Quat::from_rotation_z(extrapolated_rotation);
        transform.translation = body.position.extend(0.);
        transform.rotation = Quat::from_rotation_z(body.angle);
    }
}

pub struct LiquidFunDebugDrawPlugin;

impl Plugin for LiquidFunDebugDrawPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Last,
            (
                draw_fixtures
                    .after(TransformSystem::TransformPropagate)
                    .after(destroy_removed_bodies),
                draw_particle_systems.after(TransformSystem::TransformPropagate),
            ),
        );
    }
}

fn draw_fixtures(
    fixtures: Query<(&b2Fixture, &DebugDrawFixtures)>,
    bodies: Query<(&b2Body, &GlobalTransform)>,
    mut gizmos: Gizmos,
) {
    let to_global =
        |transform: &GlobalTransform, p: Vec2| transform.transform_point(p.extend(0.)).truncate();
    for (fixture, debug_draw_fixtures) in fixtures.iter() {
        let body_entity = fixture.body();
        let Ok((body, transform)) = bodies.get(body_entity) else {
            continue;
        };
        let color = if body.awake {
            debug_draw_fixtures.awake_color
        } else {
            debug_draw_fixtures.asleep_color
        };
        let shape = &fixture.def().shape;
        match shape {
            b2Shape::Circle { radius, position } => {
                gizmos.circle_2d(to_global(transform, *position), *radius, color);
            }
            b2Shape::EdgeTwoSided { v1, v2 } => {
                gizmos.line_2d(to_global(transform, *v1), to_global(transform, *v2), color);
            }
            b2Shape::Polygon { vertices } | b2Shape::ChainLoop { vertices } => {
                gizmos.linestrip_2d(
                    vertices
                        .iter()
                        .chain(vertices.iter().take(1))
                        .map(|v| to_global(transform, *v)),
                    color,
                );
            }
            b2Shape::Chain {
                vertices,
                prev_vertex: _p,
                next_vertex: _n,
            } => {
                gizmos.linestrip_2d(vertices.iter().map(|v| to_global(transform, *v)), color);
            }
        }

        if debug_draw_fixtures.draw_pivot {
            gizmos.circle_2d(body.position, debug_draw_fixtures.pivot_scale, Color::WHITE);
        }

        if debug_draw_fixtures.draw_up_vector {
            gizmos.line_2d(
                body.position,
                body.position + transform.up().truncate() * debug_draw_fixtures.vector_scale,
                GREEN,
            );
        }

        if debug_draw_fixtures.draw_right_vector {
            gizmos.line_2d(
                body.position,
                body.position + transform.right().truncate() * debug_draw_fixtures.vector_scale,
                RED,
            );
        }
    }
}

fn draw_particle_systems(
    particle_systems: Query<(&b2ParticleSystem, &DebugDrawParticleSystem)>,
    mut gizmos: Gizmos,
) {
    for (particle_system, _debug_draw) in particle_systems.iter() {
        let radius = particle_system.get_definition().radius;
        particle_system.get_positions().iter().for_each(|p| {
            gizmos.circle_2d(*p, radius, Color::WHITE);
        });
    }
}
