/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"

#include "BLI_array_utils.hh"
#include "BLI_assert.h"
#include "BLI_implicit_sharing.hh"
#include "BLI_index_mask.hh"
#include "BLI_mempool.h"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <LinearMath/btMotionState.h>
#include <LinearMath/btTransform.h>
#include <functional>
#include <mutex>
#include <shared_mutex>

#include "attribute_access_intern.hh"
#include "physics_geometry_impl.hh"

#ifdef WITH_BULLET
#  include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#  include <BulletCollision/Gimpact/btGImpactShape.h>
#  include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#  include <BulletDynamics/Dynamics/btRigidBody.h>
#  include <LinearMath/btDefaultMotionState.h>
#  include <btBulletDynamicsCommon.h>
#endif

namespace blender::bke {

template<typename T, int chunk_size, bool allow_iterator> class MemPool {
 private:
  BLI_mempool *pool_;

 public:
  MemPool(const int size = 0)
  {
    const int flag = allow_iterator ? BLI_MEMPOOL_ALLOW_ITER : BLI_MEMPOOL_NOP;
    pool_ = BLI_mempool_create(sizeof(T), size, chunk_size, flag);
  }

  ~MemPool()
  {
    BLI_mempool_destroy(pool_);
  }

  template<typename... Args> T *alloc(Args &&...args)
  {
    return new (BLI_mempool_alloc(pool_)) T(std::forward<Args>(args)...);
  }

  void free(T *ptr)
  {
    BLI_mempool_free(pool_, ptr);
  }

  int size() const
  {
    return BLI_mempool_len(pool_);
  }
};

#ifdef WITH_BULLET

inline float to_blender(const btScalar v)
{
  return float(v);
}

inline float3 to_blender(const btVector3 &v)
{
  return float3(v.x(), v.y(), v.z());
}

inline math::Quaternion to_blender(const btQuaternion &q)
{
  return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

inline btScalar to_bullet(const float v)
{
  return btScalar(v);
}

inline btVector3 to_bullet(const float3 &v)
{
  return btVector3(v.x, v.y, v.z);
}

inline btQuaternion to_bullet(const math::Quaternion &q)
{
  return btQuaternion(q.x, q.y, q.z, q.w);
}

int activation_state_to_bullet(bke::PhysicsGeometry::BodyActivationState state)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;
  switch (state) {
    case BodyActivationState::AlwaysActive:
      return DISABLE_DEACTIVATION;
    case BodyActivationState::Active:
      return ACTIVE_TAG;
    case BodyActivationState::WantsSleep:
      return WANTS_DEACTIVATION;
    case BodyActivationState::Sleeping:
      return ISLAND_SLEEPING;
    case BodyActivationState::AlwaysSleeping:
      return DISABLE_SIMULATION;
  }
  return DISABLE_SIMULATION;
}

bke::PhysicsGeometry::BodyActivationState activation_state_to_blender(int bt_state)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;
  switch (bt_state) {
    case ACTIVE_TAG:
      return BodyActivationState::Active;
    case ISLAND_SLEEPING:
      return BodyActivationState::Sleeping;
    case WANTS_DEACTIVATION:
      return BodyActivationState::WantsSleep;
    case DISABLE_DEACTIVATION:
      return BodyActivationState::AlwaysActive;
    case DISABLE_SIMULATION:
      return BodyActivationState::AlwaysSleeping;
    default:
      BLI_assert_unreachable();
      return BodyActivationState::AlwaysSleeping;
  }
}

inline btTypedConstraintType to_bullet(bke::PhysicsGeometry::ConstraintType type)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;
  switch (type) {
    case ConstraintType::None:
      return FIXED_CONSTRAINT_TYPE;
    case ConstraintType::Fixed:
      return FIXED_CONSTRAINT_TYPE;
    case ConstraintType::Point:
      return POINT2POINT_CONSTRAINT_TYPE;
    case ConstraintType::Hinge:
      return HINGE_CONSTRAINT_TYPE;
    case ConstraintType::Slider:
      return SLIDER_CONSTRAINT_TYPE;
    case ConstraintType::ConeTwist:
      return CONETWIST_CONSTRAINT_TYPE;
    case ConstraintType::SixDoF:
      return D6_CONSTRAINT_TYPE;
    case ConstraintType::SixDoFSpring:
      return D6_SPRING_CONSTRAINT_TYPE;
    case ConstraintType::SixDoFSpring2:
      return D6_SPRING_2_CONSTRAINT_TYPE;
    case ConstraintType::Contact:
      return CONTACT_CONSTRAINT_TYPE;
    case ConstraintType::Gear:
      return GEAR_CONSTRAINT_TYPE;
  }
  return FIXED_CONSTRAINT_TYPE;
}

inline bke::PhysicsGeometry::ConstraintType to_blender(btTypedConstraintType type)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;
  switch (type) {
    case FIXED_CONSTRAINT_TYPE:
      return ConstraintType::Fixed;
    case POINT2POINT_CONSTRAINT_TYPE:
      return ConstraintType::Point;
    case HINGE_CONSTRAINT_TYPE:
      return ConstraintType::Hinge;
    case SLIDER_CONSTRAINT_TYPE:
      return ConstraintType::Slider;
    case CONETWIST_CONSTRAINT_TYPE:
      return ConstraintType::ConeTwist;
    case D6_CONSTRAINT_TYPE:
      return ConstraintType::SixDoF;
    case D6_SPRING_CONSTRAINT_TYPE:
      return ConstraintType::SixDoFSpring;
    case D6_SPRING_2_CONSTRAINT_TYPE:
      return ConstraintType::SixDoFSpring2;
    case CONTACT_CONSTRAINT_TYPE:
      return ConstraintType::Contact;
    case GEAR_CONSTRAINT_TYPE:
      return ConstraintType::Gear;

    default:
      return ConstraintType::None;
  }
}

struct DefaultOverlapFilter : public btOverlapFilterCallback {
  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
  }
};

struct OverlapFilterWrapper : public btOverlapFilterCallback {
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  OverlapFilterFn fn;

  OverlapFilterWrapper(OverlapFilterFn fn) : fn(fn) {}

  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    const int64_t body0 = int64_t(proxy0->m_clientObject);
    const int64_t body1 = int64_t(proxy1->m_clientObject);
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) && fn(body0, body1);
  }
};

/* Extra flags stored in btRigidBody. */
enum class RigidBodyUserFlag : int {
  /* The body gets added to the simulation world. */
  IsSimulated = (1 << 0),
};
ENUM_OPERATORS(RigidBodyUserFlag, RigidBodyUserFlag::IsSimulated)

static RigidBodyUserFlag get_body_user_flags(const btRigidBody &body)
{
  return RigidBodyUserFlag(body.getUserIndex2());
}

static void set_body_user_flags(btRigidBody &body, const RigidBodyUserFlag flag, bool enable)
{
  RigidBodyUserFlag current = RigidBodyUserFlag(body.getUserIndex2());
  SET_FLAG_FROM_TEST(current, enable, flag);
  return body.setUserIndex2(int(current));
}

static int get_body_index(const btRigidBody &body)
{
  return body.getUserIndex3();
}

static void set_body_index(btRigidBody &body, const int index)
{
  body.setUserIndex3(index);
}

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

/* Make sure any body flagged for simulation is actually in the world. */
static void ensure_bodies_simulated(PhysicsGeometry &physics)
{
  PhysicsGeometryImpl &impl = physics.impl_for_write();
  /* TODO there are threadsafe versions of Bullet world that could allow this in parallel. */
  btDynamicsWorld *world = impl.world;
  if (world == nullptr) {
    return;
  }

  for (btRigidBody *body : impl.rigid_bodies) {
    const bool should_be_simulated = (get_body_user_flags(*body) &
                                      RigidBodyUserFlag::IsSimulated) != RigidBodyUserFlag(0);
    if (should_be_simulated) {
      if (!body->isInWorld()) {
        world->addRigidBody(body);
      }
    }
    else {
      if (body->isInWorld()) {
        world->removeRigidBody(body);
      }
    }
  }
}

static void create_world(PhysicsGeometryImpl &impl)
{
  impl.config = new btDefaultCollisionConfiguration();
  impl.dispatcher = new btCollisionDispatcher(impl.config);
  btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)impl.dispatcher);

  impl.broadphase = new btDbvtBroadphase();
  impl.overlap_filter = new DefaultOverlapFilter();
  impl.broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl.overlap_filter);

  impl.constraint_solver = new btSequentialImpulseConstraintSolver();

  impl.world = new btDiscreteDynamicsWorld(
      impl.dispatcher, impl.broadphase, impl.constraint_solver, impl.config);
}

static void destroy_world(PhysicsGeometryImpl &impl)
{
  delete impl.world;
  delete impl.constraint_solver;
  delete impl.broadphase;
  delete impl.dispatcher;
  delete impl.config;
  delete impl.overlap_filter;

  impl.world = nullptr;
  impl.constraint_solver = nullptr;
  impl.broadphase = nullptr;
  impl.dispatcher = nullptr;
  impl.config = nullptr;
  impl.overlap_filter = nullptr;
}

static void move_world(PhysicsGeometryImpl &from, PhysicsGeometryImpl &to)
{
  to.world = from.world;
  to.constraint_solver = from.constraint_solver;
  to.broadphase = from.broadphase;
  to.dispatcher = from.dispatcher;
  to.config = from.config;
  to.overlap_filter = from.overlap_filter;

  from.world = nullptr;
  from.constraint_solver = nullptr;
  from.broadphase = nullptr;
  from.dispatcher = nullptr;
  from.config = nullptr;
  from.overlap_filter = nullptr;
}

PhysicsGeometryImpl::PhysicsGeometryImpl() {}

PhysicsGeometryImpl::~PhysicsGeometryImpl()
{
  if (this->world) {
    destroy_world(*this);
  }
  for (const int i : this->rigid_bodies.index_range()) {
    delete this->rigid_bodies[i];
  }
  for (const int i : this->motion_states.index_range()) {
    delete this->motion_states[i];
  }
  for (const int i : this->constraints.index_range()) {
    delete this->constraints[i];
  }
}

void PhysicsGeometryImpl::delete_self()
{
  delete this;
}

void PhysicsGeometryImpl::tag_body_topology_changed()
{
  this->body_index_cache_.tag_dirty();
}

void PhysicsGeometryImpl::ensure_body_indices() const
{
  this->body_index_cache_.ensure([&]() {
    for (const int i : this->rigid_bodies.index_range()) {
      /* Note: Technically the btRigidBody is not mutable here! We're just using it as a cache with
       * exclusive write access, so it's fine. */
      btRigidBody &body = const_cast<btRigidBody &>(*this->rigid_bodies[i]);
      set_body_index(body, i);
    }
  });
}

const PhysicsGeometry::BuiltinAttributes PhysicsGeometry::builtin_attributes = {
    "id",
    "simulated",
    "static",
    "kinematic",
    "mass",
    "inertia",
    "position",
    "rotation",
    "velocity",
    "angular_velocity",
    "activation_state",
    "friction",
    "rolling_friction",
    "spinning_friction",
    "restitution",
    "linear_damping",
    "angular_damping",
    "linear_sleeping_threshold",
    "angular_sleeping_threshold",
    "constraint_body1",
    "constraint_body2"};

static void create_bodies(MutableSpan<btRigidBody *> rigid_bodies,
                          MutableSpan<btMotionState *> motion_states)
{
  for (const int i : rigid_bodies.index_range()) {
    const float mass = 1.0f;
    const float3 local_inertia = float3(0.0f);
    btMotionState *motion_state = motion_states[i] = new btDefaultMotionState();
    btCollisionShape *collision_shape = nullptr;
    rigid_bodies[i] = new btRigidBody(
        mass, motion_state, collision_shape, to_bullet(local_inertia));
  }
}

PhysicsGeometry::PhysicsGeometry()
{
  impl_ = new PhysicsGeometryImpl();
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  impl_ = other.impl_;
  if (impl_) {
    impl_->add_user();
  }
  shapes_ = other.shapes_;
}

PhysicsGeometry::PhysicsGeometry(int bodies_num, int constraints_num)
{
  PhysicsGeometryImpl *impl = new PhysicsGeometryImpl();
  impl->rigid_bodies.reinitialize(bodies_num);
  impl->motion_states.reinitialize(bodies_num);
  create_bodies(impl->rigid_bodies, impl->motion_states);
  impl->constraints.reinitialize(constraints_num);
  for (const int i : impl->constraints.index_range()) {
    impl->constraints[i] = nullptr;
  }
  impl_ = impl;

  this->tag_body_topology_changed();
}

PhysicsGeometry::~PhysicsGeometry()
{
  BLI_assert(impl_ && impl_->strong_users() > 0);
  impl_->remove_user_and_delete_if_last();
}

const PhysicsGeometryImpl &PhysicsGeometry::impl() const
{
  return *impl_;
}

PhysicsGeometryImpl &PhysicsGeometry::impl_for_write()
{
  if (impl_->is_cached) {
    /* Dummy impl for stub write access on cached physics. */
    impl_ = new PhysicsGeometryImpl();
  }
  else if (!impl_->is_mutable()) {
    PhysicsGeometryImpl *new_impl = new PhysicsGeometryImpl();
    new_impl->rigid_bodies.reinitialize(impl_->rigid_bodies.size());
    new_impl->motion_states.reinitialize(impl_->motion_states.size());
    move_physics_impl_data(*impl_, *new_impl, true, 0, 0);

    impl_ = new_impl;
  }

  return *const_cast<PhysicsGeometryImpl *>(impl_);
}

void move_physics_data(const PhysicsGeometry &from,
                       PhysicsGeometry &to,
                       const bool use_world,
                       int bodies_offset,
                       int constraints_offset)
{
  PhysicsGeometryImpl &to_impl = to.impl_for_write();
  const PhysicsGeometryImpl &from_impl = from.impl();
  move_physics_impl_data(from_impl, to_impl, use_world, bodies_offset, constraints_offset);
}

void move_physics_impl_data(const PhysicsGeometryImpl &from,
                            PhysicsGeometryImpl &to,
                            const bool use_world,
                            const int bodies_offset,
                            const int constraints_offset)
{
  BLI_assert(to.is_mutable());

  /* Early check before locking. */
  if (from.is_cached) {
    return;
  }

  const IndexRange body_range = IndexRange(bodies_offset, from.rigid_bodies.size());
  const IndexRange constraint_range = IndexRange(constraints_offset, from.constraints.size());
  /* Make sure target has enough space. */
  BLI_assert(body_range.intersect(to.rigid_bodies.index_range()) == body_range);
  BLI_assert(constraint_range.intersect(to.constraints.index_range()) == constraint_range);

  std::unique_lock lock(from.data_mutex);
  if (from.is_cached) {
    /* This may have changed before locking the mutex. */
    return;
  }
  PhysicsGeometryImpl &from_mutable = const_cast<PhysicsGeometryImpl &>(from);

  btDynamicsWorld *from_world = from_mutable.world;
  if (use_world) {
    if (to.world) {
      destroy_world(to);
    }
    move_world(from_mutable, to);
  }
  btDynamicsWorld *to_world = to.world;

  for (const int i_body : body_range.index_range()) {
    btRigidBody *body = from_mutable.rigid_bodies[i_body];
    btMotionState *motion_state = from_mutable.motion_states[i_body];

    const bool add_to_world = (get_body_user_flags(*body) & RigidBodyUserFlag::IsSimulated) !=
                              RigidBodyUserFlag(0);
    if (add_to_world && to_world != from_world) {
      /* Move all to new world. */
      if (from_world != nullptr) {
        from_world->removeRigidBody(body);
      }

      if (to_world != nullptr) {
        to_world->addRigidBody(body);
      }
    }

    to.rigid_bodies[body_range[i_body]] = body;
    to.motion_states[body_range[i_body]] = motion_state;
  }

  /* Clear source pointers. */
  from_mutable.world = nullptr;
  from_mutable.rigid_bodies.reinitialize(0);
  from_mutable.motion_states.reinitialize(0);
}

bool PhysicsGeometry::has_world() const
{
  return this->impl().world != nullptr;
}

void PhysicsGeometry::set_world_enabled(const bool enabled)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  if (enabled) {
    if (this->impl().world == nullptr) {
      create_world(impl);
    }
  }
  else {
    if (this->impl().world != nullptr) {
      destroy_world(impl);
    }
  }
  // ensure_bodies_simulated(*this);
}

void PhysicsGeometry::set_overlap_filter(OverlapFilterFn fn)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  impl.overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl.broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl.overlap_filter);
}

void PhysicsGeometry::clear_overlap_filter()
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  if (impl.overlap_filter) {
    delete impl.overlap_filter;
    impl.overlap_filter = new DefaultOverlapFilter();
  }
  impl.broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl.overlap_filter);
}

float3 PhysicsGeometry::gravity() const
{
  if (this->impl().world) {
    return to_blender(this->impl().world->getGravity());
  }
  return float3(0.0f);
}

void PhysicsGeometry::set_gravity(const float3 &gravity)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  impl.world->setGravity(to_bullet(gravity));
}

void PhysicsGeometry::set_solver_iterations(const int num_solver_iterations)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  btContactSolverInfo &info = impl.world->getSolverInfo();
  info.m_numIterations = num_solver_iterations;
}

void PhysicsGeometry::set_split_impulse(const bool split_impulse)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  btContactSolverInfo &info = impl.world->getSolverInfo();
  /* Note: Bullet stores this as int, but it's used as a bool. */
  info.m_splitImpulse = int(split_impulse);
}

void PhysicsGeometry::step_simulation(float delta_time)
{
  constexpr const float fixed_time_step = 1.0f / 120.0f;

  PhysicsGeometryImpl &impl = this->impl_for_write();
  impl.world->stepSimulation(delta_time, 100, fixed_time_step);
}

int PhysicsGeometry::bodies_num() const
{
  return impl().rigid_bodies.size();
}

int PhysicsGeometry::constraints_num() const
{
  return impl().constraints.size();
}

int PhysicsGeometry::shapes_num() const
{
  return shapes_.size();
}

IndexRange PhysicsGeometry::bodies_range() const
{
  return impl().rigid_bodies.index_range();
}

IndexRange PhysicsGeometry::constraints_range() const
{
  return impl().constraints.index_range();
}

IndexRange PhysicsGeometry::shapes_range() const
{
  return shapes_.index_range();
}

void PhysicsGeometry::tag_collision_shapes_changed() {}

void PhysicsGeometry::tag_body_transforms_changed() {}

void PhysicsGeometry::tag_body_topology_changed()
{
  this->impl_for_write().tag_body_topology_changed();
}

void PhysicsGeometry::tag_physics_changed()
{
  this->tag_body_topology_changed();
}

Span<CollisionShape::Ptr> PhysicsGeometry::shapes() const
{
  return shapes_.as_span();
}

std::optional<int> PhysicsGeometry::find_shape_handle(const CollisionShape &shape)
{
  for (const int i : shapes_.index_range()) {
    const CollisionShapePtr &ptr = shapes_[i];
    if (ptr.get() == &shape) {
      return i;
    }
  }
  return std::nullopt;
}

int PhysicsGeometry::add_shape(const CollisionShapePtr &shape)
{
  if (const std::optional<int> handle = find_shape_handle(*shape)) {
    return *handle;
  }

  return shapes_.append_and_get_index(shape);
}

void PhysicsGeometry::set_body_shapes(const IndexMask &selection,
                                      const Span<int> shape_handles,
                                      const bool update_local_inertia)
{
  BLI_assert(selection.bounds().intersect(impl_->rigid_bodies.index_range()) ==
             selection.bounds());
  selection.foreach_index([&](const int index) {
    const int handle = shape_handles[index];
    if (!shapes_.index_range().contains(handle)) {
      return;
    }
    const CollisionShapePtr &shape_ptr = shapes_[handle];
    const btCollisionShape *bt_shape = &shape_ptr->impl().as_bullet_shape();
    const bool is_moveable_shape = !bt_shape->isNonMoving();

    btRigidBody *body = impl_->rigid_bodies[index];
    if (body->getCollisionShape() == bt_shape) {
      /* Shape is correct, nothing to do. */
      return;
    }

    // XXX is const_cast safe here? not sure why Bullet wants a mutable shape.
    body->setCollisionShape(const_cast<btCollisionShape *>(bt_shape));
    if (impl_->broadphase && body->isInWorld()) {
      impl_->broadphase->getOverlappingPairCache()->cleanProxyFromPairs(body->getBroadphaseProxy(),
                                                                        impl_->dispatcher);
    }
    if (is_moveable_shape) {
      if (update_local_inertia) {
        btVector3 bt_local_inertia;
        bt_shape->calculateLocalInertia(body->getMass(), bt_local_inertia);
        body->setMassProps(body->getMass(), bt_local_inertia);
      }
    }
    else {
      body->setMassProps(0.0f, btVector3(0.0f, 0.0f, 0.0f));
    }
  });
}

VArray<int> PhysicsGeometry::body_ids() const
{
  return attributes().lookup(builtin_attributes.id).varray.typed<int>();
}

AttributeWriter<int> PhysicsGeometry::body_ids_for_write()
{
  return attributes_for_write().lookup_for_write<int>(builtin_attributes.id);
}

VArray<bool> PhysicsGeometry::body_is_simulated() const
{
  return attributes().lookup(builtin_attributes.is_simulated).varray.typed<bool>();
}

AttributeWriter<bool> PhysicsGeometry::body_is_simulated_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(builtin_attributes.is_simulated);
}

VArray<bool> PhysicsGeometry::body_is_static() const
{
  return attributes().lookup(builtin_attributes.is_static).varray.typed<bool>();
}

VArray<float> PhysicsGeometry::body_masses() const
{
  return attributes().lookup(builtin_attributes.mass).varray.typed<float>();
}

AttributeWriter<float> PhysicsGeometry::body_masses_for_write()
{
  return attributes_for_write().lookup_for_write<float>(builtin_attributes.mass);
}

VArray<float3> PhysicsGeometry::body_inertias() const
{
  return attributes().lookup(builtin_attributes.inertia).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_inertias_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.inertia);
}

VArray<float3> PhysicsGeometry::body_positions() const
{
  return attributes().lookup(builtin_attributes.position).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_positions_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.position);
}

VArray<math::Quaternion> PhysicsGeometry::body_rotations() const
{
  return attributes().lookup(builtin_attributes.rotation).varray.typed<math::Quaternion>();
}

AttributeWriter<math::Quaternion> PhysicsGeometry::body_rotations_for_write()
{
  return attributes_for_write().lookup_for_write<math::Quaternion>(builtin_attributes.rotation);
}

VArray<float3> PhysicsGeometry::body_velocities() const
{
  return attributes().lookup(builtin_attributes.velocity).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.velocity);
}

VArray<float3> PhysicsGeometry::body_angular_velocities() const
{
  return attributes().lookup(builtin_attributes.angular_velocity).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_angular_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.angular_velocity);
}

VArray<int> PhysicsGeometry::body_activation_states() const
{
  return attributes().lookup<int>(builtin_attributes.activation_state).varray;
}

AttributeWriter<int> PhysicsGeometry::body_activation_states_for_write()
{
  return attributes_for_write().lookup_for_write<int>(builtin_attributes.activation_state);
}

VArray<int> PhysicsGeometry::constraint_body_1() const
{
  return attributes().lookup<int>(builtin_attributes.constraint_body1).varray;
}

AttributeWriter<int> PhysicsGeometry::constraint_body_1_for_write()
{
  return attributes_for_write().lookup_for_write<int>(builtin_attributes.constraint_body1);
}

VArray<int> PhysicsGeometry::constraint_body_2() const
{
  return attributes().lookup<int>(builtin_attributes.constraint_body2).varray;
}

AttributeWriter<int> PhysicsGeometry::constraint_body_2_for_write()
{
  return attributes_for_write().lookup_for_write<int>(builtin_attributes.constraint_body2);
}

/**
 * Utility to group together multiple functions that are used to access custom data on geometry
 * components in a generic way.
 */
struct PhysicsAccessInfo {
  using PhysicsGetter = PhysicsGeometry *(*)(void *owner);
  using ConstPhysicsGetter = const PhysicsGeometry *(*)(const void *owner);

  PhysicsGetter get_physics;
  ConstPhysicsGetter get_const_physics;
};

template<typename T> using RigidBodyGetCacheFn = Span<T> (*)(const PhysicsGeometryImpl &impl);
template<typename T>
using RigidBodyGetMutableCacheFn = MutableSpan<T> (*)(PhysicsGeometryImpl &impl);
template<typename T> using ConstraintGetCacheFn = Span<T> (*)(const PhysicsGeometryImpl &impl);
template<typename T>
using ConstraintGetMutableCacheFn = MutableSpan<T> (*)(PhysicsGeometryImpl &impl);

/**
 * Provider for builtin rigid body attributes.
 */
template<typename T,
         RigidBodyGetFn<T> GetFn,
         RigidBodySetFn<T> SetFn = nullptr,
         RigidBodyGetCacheFn<T> GetCacheFn = nullptr,
         RigidBodyGetMutableCacheFn<T> GetMutableCacheFn = nullptr>
class BuiltinRigidBodyAttributeProvider final : public bke::BuiltinAttributeProvider {
  using UpdateOnChange = void (*)(void *owner);
  const PhysicsAccessInfo physics_access_;
  const UpdateOnChange update_on_change_;

 public:
  BuiltinRigidBodyAttributeProvider(std::string attribute_name,
                                    const AttrDomain domain,
                                    const DeletableEnum deletable,
                                    const PhysicsAccessInfo physics_access,
                                    const UpdateOnChange update_on_change,
                                    const AttributeValidator validator = {})
      : BuiltinAttributeProvider(std::move(attribute_name),
                                 domain,
                                 cpp_type_to_custom_data_type(CPPType::get<T>()),
                                 deletable,
                                 validator),
        physics_access_(physics_access),
        update_on_change_(update_on_change)
  {
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    const PhysicsGeometry *physics = physics_access_.get_const_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    GVArray varray;
    if constexpr (GetCacheFn == nullptr) {
      varray = VArray_For_PhysicsBodies<T, GetFn>(physics, T());
    }
    else {
      varray = VArray_For_PhysicsBodies<T, GetFn>(physics, GetCacheFn(physics->impl()));
    }

    return {std::move(varray), domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    if constexpr (SetFn == nullptr) {
      return {};
    }
    PhysicsGeometry *physics = physics_access_.get_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    // GVMutableArray varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics);
    PhysicsGeometryImpl &impl = physics->impl_for_write();

    GVMutableArray varray;
    if constexpr (GetMutableCacheFn == nullptr) {
      varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics, T());
    }
    else {
      varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics, GetMutableCacheFn(impl));
    }

    std::function<void()> tag_modified_fn;
    if (update_on_change_ != nullptr) {
      tag_modified_fn = [owner, update = update_on_change_]() { update(owner); };
    }

    return {std::move(varray), domain_, std::move(tag_modified_fn)};
  }

  bool try_delete(void * /*owner*/) const final
  {
    return false;
  }

  bool try_create(void * /*owner*/, const AttributeInit & /*initializer*/) const final
  {
    return false;
  }

  bool exists(const void * /*owner*/) const final
  {
    return true;
  }
};

/**
 * Provider for builtin constraint attributes.
 */
template<typename T,
         ConstraintGetFn<T> GetFn,
         ConstraintSetFn<T> SetFn = nullptr,
         ConstraintGetCacheFn<T> GetCacheFn = nullptr,
         ConstraintGetMutableCacheFn<T> GetMutableCacheFn = nullptr>
class BuiltinConstraintAttributeProvider final : public bke::BuiltinAttributeProvider {
  using UpdateOnChange = void (*)(void *owner);
  using EnsureOnAccess = void (*)(const void *owner);
  const PhysicsAccessInfo physics_access_;
  const UpdateOnChange update_on_change_;
  const EnsureOnAccess ensure_on_access_;

 public:
  BuiltinConstraintAttributeProvider(std::string attribute_name,
                                     const AttrDomain domain,
                                     const DeletableEnum deletable,
                                     const PhysicsAccessInfo physics_access,
                                     const UpdateOnChange update_on_change,
                                     const AttributeValidator validator = {},
                                     const EnsureOnAccess ensure_on_access = nullptr)
      : BuiltinAttributeProvider(std::move(attribute_name),
                                 domain,
                                 cpp_type_to_custom_data_type(CPPType::get<T>()),
                                 deletable,
                                 validator),
        physics_access_(physics_access),
        update_on_change_(update_on_change),
        ensure_on_access_(ensure_on_access)
  {
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    const PhysicsGeometry *physics = physics_access_.get_const_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    if (ensure_on_access_) {
      ensure_on_access_(owner);
    }

    GVArray varray;
    if constexpr (GetCacheFn == nullptr) {
      varray = VArray_For_PhysicsConstraints<T, GetFn>(physics, T());
    }
    else {
      varray = VArray_For_PhysicsConstraints<T, GetFn>(physics, GetCacheFn(physics->impl()));
    }

    return {std::move(varray), domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    if constexpr (SetFn == nullptr) {
      return {};
    }
    PhysicsGeometry *physics = physics_access_.get_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    if (ensure_on_access_) {
      ensure_on_access_(owner);
    }

    // GVMutableArray varray = VMutableArray_For_PhysicsBodies<T, GetFn, SetFn>(physics);
    PhysicsGeometryImpl &impl = physics->impl_for_write();

    GVMutableArray varray;
    if constexpr (GetMutableCacheFn == nullptr) {
      varray = VMutableArray_For_PhysicsConstraints<T, GetFn, SetFn>(physics, T());
    }
    else {
      varray = VMutableArray_For_PhysicsConstraints<T, GetFn, SetFn>(physics,
                                                                     GetMutableCacheFn(impl));
    }

    std::function<void()> tag_modified_fn;
    if (update_on_change_ != nullptr) {
      tag_modified_fn = [owner, update = update_on_change_]() { update(owner); };
    }

    return {std::move(varray), domain_, std::move(tag_modified_fn)};
  }

  bool try_delete(void * /*owner*/) const final
  {
    return false;
  }

  bool try_create(void * /*owner*/, const AttributeInit & /*initializer*/) const final
  {
    return false;
  }

  bool exists(const void * /*owner*/) const final
  {
    return true;
  }
};

static ComponentAttributeProviders create_attribute_providers_for_physics()
{
  static PhysicsAccessInfo physics_access = {
      [](void *owner) -> PhysicsGeometry * { return static_cast<PhysicsGeometry *>(owner); },
      [](const void *owner) -> const PhysicsGeometry * {
        return static_cast<const PhysicsGeometry *>(owner);
      }};

  constexpr auto id_get_fn = [](const btRigidBody &body) -> int { return body.getUserIndex(); };
  constexpr auto id_set_fn = [](btRigidBody &body, int value) { body.setUserIndex(value); };
  static BuiltinRigidBodyAttributeProvider<int, id_get_fn, id_set_fn> body_id(
      PhysicsGeometry::builtin_attributes.id,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto simulated_get_fn = [](const btRigidBody &body) -> bool {
    return (get_body_user_flags(body) & RigidBodyUserFlag::IsSimulated) != RigidBodyUserFlag(0);
  };
  constexpr auto simulated_set_fn = [](btRigidBody &body, bool value) {
    set_body_user_flags(body, RigidBodyUserFlag::IsSimulated, value);
  };
  static BuiltinRigidBodyAttributeProvider<bool, simulated_get_fn, simulated_set_fn>
      body_simulated(
          PhysicsGeometry::builtin_attributes.is_simulated,
          AttrDomain::Point,
          BuiltinAttributeProvider::NonDeletable,
          physics_access,
          [](void *owner) { ensure_bodies_simulated(*static_cast<PhysicsGeometry *>(owner)); });

  constexpr auto static_get_fn = [](const btRigidBody &body) -> bool {
    return body.isStaticObject();
  };
  constexpr auto static_set_fn = [](btRigidBody &body, bool value) {
    const bool is_moveable_shape = !body.getCollisionShape() ||
                                   !body.getCollisionShape()->isNonMoving();
    if (is_moveable_shape) {
      if (value) {
        body.setMassProps(0.0f, to_bullet(float3(0.0f)));
      }
      else if (body.isStaticObject()) {
        body.setMassProps(1.0f, to_bullet(float3(1.0f)));
      }
    }
  };
  static BuiltinRigidBodyAttributeProvider<bool, static_get_fn, static_set_fn> body_static(
      PhysicsGeometry::builtin_attributes.is_static,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto kinematic_get_fn = [](const btRigidBody &body) -> bool {
    return body.isKinematicObject();
  };
  constexpr auto kinematic_set_fn = [](btRigidBody &body, bool value) {
    int bt_collision_flags = body.getCollisionFlags();
    SET_FLAG_FROM_TEST(bt_collision_flags, value, btCollisionObject::CF_KINEMATIC_OBJECT);
    body.setCollisionFlags(bt_collision_flags);
  };
  static BuiltinRigidBodyAttributeProvider<bool, kinematic_get_fn, kinematic_set_fn>
      body_kinematic(PhysicsGeometry::builtin_attributes.is_kinematic,
                     AttrDomain::Point,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     nullptr);

  constexpr auto mass_get_fn = [](const btRigidBody &body) -> float { return body.getMass(); };
  constexpr auto mass_set_fn = [](btRigidBody &body, float value) {
    const bool is_moveable_shape = !body.getCollisionShape() ||
                                   !body.getCollisionShape()->isNonMoving();
    if (is_moveable_shape) {
      body.setMassProps(value, body.getLocalInertia());
    }
  };
  static BuiltinRigidBodyAttributeProvider<float, mass_get_fn, mass_set_fn> body_mass(
      PhysicsGeometry::builtin_attributes.mass,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto inertia_get_fn = [](const btRigidBody &body) -> float3 {
    return to_blender(body.getLocalInertia());
  };
  constexpr auto inertia_set_fn = [](btRigidBody &body, float3 value) {
    const bool is_moveable_shape = !body.getCollisionShape() ||
                                   !body.getCollisionShape()->isNonMoving();
    if (is_moveable_shape) {
      if (math::is_zero(value) && body.getCollisionShape()) {
        btVector3 bt_inertia;
        body.getCollisionShape()->calculateLocalInertia(body.getMass(), bt_inertia);
        body.setMassProps(body.getMass(), bt_inertia);
      }
      else {
        body.setMassProps(body.getMass(), to_bullet(value));
      }
    }
  };
  static BuiltinRigidBodyAttributeProvider<float3, inertia_get_fn, inertia_set_fn> body_inertia(
      PhysicsGeometry::builtin_attributes.inertia,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto position_get_fn = [](const btRigidBody &body) -> float3 {
    return to_blender(body.getWorldTransform().getOrigin());
  };
  constexpr auto position_set_fn = [](btRigidBody &body, float3 value) {
    body.getWorldTransform().setOrigin(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3, position_get_fn, position_set_fn> body_position(
      PhysicsGeometry::builtin_attributes.position,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto rotation_get_fn = [](const btRigidBody &body) -> math::Quaternion {
    return to_blender(body.getWorldTransform().getRotation());
  };
  constexpr auto rotation_set_fn = [](btRigidBody &body, math::Quaternion value) {
    body.getWorldTransform().setRotation(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<math::Quaternion, rotation_get_fn, rotation_set_fn>
      body_rotation(PhysicsGeometry::builtin_attributes.rotation,
                    AttrDomain::Point,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    nullptr);

  constexpr auto velocity_get_fn = [](const btRigidBody &body) -> float3 {
    return to_blender(body.getLinearVelocity());
  };
  constexpr auto velocity_set_fn = [](btRigidBody &body, float3 value) {
    body.setLinearVelocity(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3, velocity_get_fn, velocity_set_fn> body_velocity(
      PhysicsGeometry::builtin_attributes.velocity,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto angular_velocity_get_fn = [](const btRigidBody &body) -> float3 {
    return to_blender(body.getAngularVelocity());
  };
  constexpr auto angular_velocity_set_fn = [](btRigidBody &body, float3 value) {
    body.setAngularVelocity(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3,
                                           angular_velocity_get_fn,
                                           angular_velocity_set_fn>
      body_angular_velocity(PhysicsGeometry::builtin_attributes.angular_velocity,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);

  constexpr auto activation_state_get_fn = [](const btRigidBody &body) -> int {
    return int(activation_state_to_blender(body.getActivationState()));
  };
  constexpr auto activation_state_set_fn = [](btRigidBody &body, int value) {
    /* Note: there is also setActivationState, but that only sets if the state is not always-active
     * or always-sleeping. This check can be performed on the caller side if the "always-x" state
     * must be retained. */
    body.forceActivationState(
        activation_state_to_bullet(bke::PhysicsGeometry::BodyActivationState(value)));
  };
  static BuiltinRigidBodyAttributeProvider<int, activation_state_get_fn, activation_state_set_fn>
      body_activation_state(PhysicsGeometry::builtin_attributes.activation_state,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);

  constexpr auto friction_get_fn = [](const btRigidBody &body) -> float {
    return body.getFriction();
  };
  constexpr auto friction_set_fn = [](btRigidBody &body, float value) { body.setFriction(value); };
  static BuiltinRigidBodyAttributeProvider<float, friction_get_fn, friction_set_fn> body_friction(
      PhysicsGeometry::builtin_attributes.friction,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto rolling_friction_get_fn = [](const btRigidBody &body) -> float {
    return body.getRollingFriction();
  };
  constexpr auto rolling_friction_set_fn = [](btRigidBody &body, float value) {
    body.setRollingFriction(value);
  };
  static BuiltinRigidBodyAttributeProvider<float, rolling_friction_get_fn, rolling_friction_set_fn>
      body_rolling_friction(PhysicsGeometry::builtin_attributes.rolling_friction,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);

  constexpr auto spinning_friction_get_fn = [](const btRigidBody &body) -> float {
    return body.getSpinningFriction();
  };
  constexpr auto spinning_friction_set_fn = [](btRigidBody &body, float value) {
    body.setSpinningFriction(value);
  };
  static BuiltinRigidBodyAttributeProvider<float,
                                           spinning_friction_get_fn,
                                           spinning_friction_set_fn>
      body_spinning_friction(PhysicsGeometry::builtin_attributes.spinning_friction,
                             AttrDomain::Point,
                             BuiltinAttributeProvider::NonDeletable,
                             physics_access,
                             nullptr);

  constexpr auto restitution_get_fn = [](const btRigidBody &body) -> float {
    return body.getRestitution();
  };
  constexpr auto restitution_set_fn = [](btRigidBody &body, float value) {
    body.setRestitution(value);
  };
  static BuiltinRigidBodyAttributeProvider<float, restitution_get_fn, restitution_set_fn>
      body_restitution(PhysicsGeometry::builtin_attributes.restitution,
                       AttrDomain::Point,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       nullptr);

  constexpr auto linear_damping_get_fn = [](const btRigidBody &body) -> float {
    return body.getLinearSleepingThreshold();
  };
  constexpr auto linear_damping_set_fn = [](btRigidBody &body, float value) {
    body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
  };
  static BuiltinRigidBodyAttributeProvider<float, linear_damping_get_fn, linear_damping_set_fn>
      body_linear_damping(PhysicsGeometry::builtin_attributes.linear_damping,
                          AttrDomain::Point,
                          BuiltinAttributeProvider::NonDeletable,
                          physics_access,
                          nullptr);

  constexpr auto angular_damping_get_fn = [](const btRigidBody &body) -> float {
    return body.getAngularSleepingThreshold();
  };
  constexpr auto angular_damping_set_fn = [](btRigidBody &body, float value) {
    body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
  };
  static BuiltinRigidBodyAttributeProvider<float, angular_damping_get_fn, angular_damping_set_fn>
      body_angular_damping(PhysicsGeometry::builtin_attributes.angular_damping,
                           AttrDomain::Point,
                           BuiltinAttributeProvider::NonDeletable,
                           physics_access,
                           nullptr);

  constexpr auto linear_sleeping_threshold_get_fn = [](const btRigidBody &body) -> float {
    return body.getLinearSleepingThreshold();
  };
  constexpr auto linear_sleeping_threshold_set_fn = [](btRigidBody &body, float value) {
    body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
  };
  static BuiltinRigidBodyAttributeProvider<float,
                                           linear_sleeping_threshold_get_fn,
                                           linear_sleeping_threshold_set_fn>
      body_linear_sleeping_threshold(PhysicsGeometry::builtin_attributes.linear_sleeping_threshold,
                                     AttrDomain::Point,
                                     BuiltinAttributeProvider::NonDeletable,
                                     physics_access,
                                     nullptr);

  constexpr auto angular_sleeping_threshold_get_fn = [](const btRigidBody &body) -> float {
    return body.getAngularSleepingThreshold();
  };
  constexpr auto angular_sleeping_threshold_set_fn = [](btRigidBody &body, float value) {
    body.setSleepingThresholds(body.getLinearSleepingThreshold(), value);
  };
  static BuiltinRigidBodyAttributeProvider<float,
                                           angular_sleeping_threshold_get_fn,
                                           angular_sleeping_threshold_set_fn>
      body_angular_sleeping_threshold(
          PhysicsGeometry::builtin_attributes.angular_sleeping_threshold,
          AttrDomain::Point,
          BuiltinAttributeProvider::NonDeletable,
          physics_access,
          nullptr);

  constexpr auto constraint_body1_get_fn = [](const btTypedConstraint &constraint) -> int {
    return get_body_index(constraint.getRigidBodyA());
  };
  static BuiltinConstraintAttributeProvider<int, constraint_body1_get_fn> constraint_body1(
      PhysicsGeometry::builtin_attributes.constraint_body1,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr,
      {},
      [](const void *owner) {
        const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
        physics->impl().ensure_body_indices();
      });

  constexpr auto constraint_body2_get_fn = [](const btTypedConstraint &constraint) -> int {
    return get_body_index(constraint.getRigidBodyB());
  };
  static BuiltinConstraintAttributeProvider<int, constraint_body2_get_fn> constraint_body2(
      PhysicsGeometry::builtin_attributes.constraint_body2,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr,
      {},
      [](const void *owner) {
        const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
        physics->impl().ensure_body_indices();
      });

  return ComponentAttributeProviders({&body_id,
                                      &body_simulated,
                                      &body_static,
                                      &body_kinematic,
                                      &body_mass,
                                      &body_inertia,
                                      &body_position,
                                      &body_rotation,
                                      &body_velocity,
                                      &body_angular_velocity,
                                      &body_activation_state,
                                      &body_friction,
                                      &body_rolling_friction,
                                      &body_spinning_friction,
                                      &body_restitution,
                                      &body_linear_damping,
                                      &body_angular_damping,
                                      &body_linear_sleeping_threshold,
                                      &body_angular_sleeping_threshold,
                                      &constraint_body1,
                                      &constraint_body2},
                                     {});
}

static GVArray adapt_physics_attribute_domain(const PhysicsGeometry & /*physics*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_physics_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_physics();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return int(physics.bodies_num());
      case AttrDomain::Edge:
        return int(physics.constraints_num());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
    return adapt_physics_attribute_domain(physics, varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_physics_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions();
  return fn;
}

AttributeAccessor PhysicsGeometry::attributes() const
{
  return AttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

/** \} */

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::bke
