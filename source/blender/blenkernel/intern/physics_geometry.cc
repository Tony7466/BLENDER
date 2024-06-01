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

#include "BLI_implicit_sharing.hh"
#include "BLI_mempool.h"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btMotionState.h>
#include <LinearMath/btTransform.h>
#include <functional>

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

/* -------------------------------------------------------------------- */
/** \name Physics World
 * \{ */

PhysicsWorldImpl::PhysicsWorldImpl()
{
  this->config = new btDefaultCollisionConfiguration();
  this->dispatcher = new btCollisionDispatcher(this->config);
  btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)this->dispatcher);

  this->broadphase = new btDbvtBroadphase();
  this->overlap_filter = new DefaultOverlapFilter();
  this->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(this->overlap_filter);

  this->constraint_solver = new btSequentialImpulseConstraintSolver();

  this->world = new btDiscreteDynamicsWorld(
      this->dispatcher, this->broadphase, this->constraint_solver, this->config);
}

PhysicsWorldImpl::~PhysicsWorldImpl()
{
  delete this->world;
  delete this->constraint_solver;
  delete this->broadphase;
  delete this->dispatcher;
  delete this->config;
  delete this->overlap_filter;

  this->world = 0;
  this->constraint_solver = 0;
  this->broadphase = 0;
  this->dispatcher = 0;
  this->config = 0;
  this->overlap_filter = 0;
}

PhysicsWorldImpl *PhysicsWorldImpl::copy() const
{
  PhysicsWorldImpl *result = new PhysicsWorldImpl();
  // TODO copy settings
  return result;
}

PhysicsWorld::PhysicsWorld(PhysicsWorldImpl *impl) : impl_(impl)
{
}

PhysicsWorld::~PhysicsWorld()
{
}

PhysicsWorldImpl &PhysicsWorld::impl()
{
  return *impl_;
}

const PhysicsWorldImpl &PhysicsWorld::impl() const
{
  return *impl_;
}

void PhysicsWorld::set_overlap_filter(OverlapFilterFn fn)
{
  impl_->overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

void PhysicsWorld::clear_overlap_filter()
{
  if (impl_->overlap_filter) {
    delete impl_->overlap_filter;
    impl_->overlap_filter = new DefaultOverlapFilter();
  }
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

float3 PhysicsWorld::gravity() const
{
  return to_blender(impl_->world->getGravity());
}

void PhysicsWorld::set_gravity(const float3 &gravity)
{
  impl_->world->setGravity(to_bullet(gravity));
}

void PhysicsWorld::set_solver_iterations(const int num_solver_iterations)
{
  btContactSolverInfo &info = impl_->world->getSolverInfo();
  info.m_numIterations = num_solver_iterations;
}

void PhysicsWorld::set_split_impulse(const bool split_impulse)
{
  btContactSolverInfo &info = impl_->world->getSolverInfo();
  /* Note: Bullet stores this as int, but it's used as a bool. */
  info.m_splitImpulse = int(split_impulse);
}

void PhysicsWorld::step_simulation(float delta_time)
{
  constexpr const float fixed_time_step = 1.0f / 60.0f;
  impl_->world->stepSimulation(delta_time, fixed_time_step);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

static void remove_all_bodies_from_world(PhysicsGeometry &physics)
{
  if (physics.world() == nullptr) {
    return;
  }
  btDynamicsWorld *world = physics.world_for_write()->impl().world;
  for (btRigidBody *body : physics.impl_for_write().rigid_bodies) {
    if (body->isInWorld()) {
      world->removeRigidBody(body);
    }
  }
}

/* Make sure any body flagged for simulation is actually in the world. */
static void ensure_bodies_simulated(PhysicsGeometry &physics)
{
  if (physics.world() == nullptr) {
    return;
  }
  /* TODO there are threadsafe versions of Bullet world that could allow this in parallel. */
  btDynamicsWorld *world = physics.world_for_write()->impl().world;
  for (btRigidBody *body : physics.impl_for_write().rigid_bodies) {
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

PhysicsGeometryImpl::PhysicsGeometryImpl() {}

PhysicsGeometryImpl::~PhysicsGeometryImpl()
{
  if (this->world_impl) {
    delete this->world_impl;
  }
  for (const int i : this->rigid_bodies.index_range()) {
    delete this->rigid_bodies[i];
  }

  for (const int i : this->motion_states.index_range()) {
    delete this->motion_states[i];
  }
}

void PhysicsGeometryImpl::delete_self()
{
  delete this;
}

PhysicsGeometryImpl *PhysicsGeometryImpl::copy() const
{
  PhysicsGeometryImpl *result = new PhysicsGeometryImpl{};
  if (this->world_impl) {
    result->world_impl = new PhysicsWorldImpl(*this->world_impl);
  }
  // TODO copy all the rest: motion states, bodies, constraints, shapes, ...
  return result;
}

const PhysicsGeometry::BuiltinAttributes PhysicsGeometry::builtin_attributes = {
    "id", "simulated", "mass", "inertia", "position", "rotation", "velocity", "angular_velocity"};

PhysicsGeometry::PhysicsGeometry() : PhysicsGeometry(0) {}

PhysicsGeometry::PhysicsGeometry(int rigid_bodies_num)
{
  PhysicsGeometryImpl *impl = new PhysicsGeometryImpl{};
  impl->rigid_bodies.reinitialize(rigid_bodies_num);
  impl->motion_states.reinitialize(rigid_bodies_num);
  for (const int i : IndexRange(rigid_bodies_num)) {
    const float mass = 1.0f;
    const float3 inertia = float3(0.0f);

    impl->motion_states[i] = new btDefaultMotionState();
    impl->rigid_bodies[i] = new btRigidBody(
        mass, impl->motion_states[i], nullptr, to_bullet(inertia));
  }
  impl_ = impl;
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  impl_ = other.impl_;
  if (impl_) {
    impl_->add_user();
  }
}

PhysicsGeometry::~PhysicsGeometry()
{
  impl_->remove_user_and_delete_if_last();
  if (world_) {
    delete world_;
  }
}

PhysicsGeometryImpl &PhysicsGeometry::impl_for_write()
{
  if (!impl_->is_mutable()) {
    impl_ = impl_->copy();
    if (world_) {
      world_->impl_ = impl_->world_impl;
    }
  }
  return *const_cast<PhysicsGeometryImpl *>(impl_);
}

const PhysicsGeometryImpl &PhysicsGeometry::impl() const
{
  return *impl_;
}

PhysicsWorld *PhysicsGeometry::world_for_write()
{
  if (world_ == nullptr) {
    return nullptr;
  }
  /* Ensure mutable. */
  impl_for_write();
  return world_;
}

const PhysicsWorld *PhysicsGeometry::world() const
{
  return world_;
}

void PhysicsGeometry::set_world_enabled(const bool enabled)
{
  if (enabled) {
    if (world_ == nullptr) {
      BLI_assert(impl_->world_impl == nullptr);
      PhysicsWorldImpl *world_impl = new PhysicsWorldImpl();
      impl_for_write().world_impl = world_impl;
      world_ = new PhysicsWorld(world_impl);
    }
  }
  else {
    if (world_ != nullptr) {
      BLI_assert(impl_->world_impl != nullptr);
      delete world_;
      world_ = nullptr;
      delete impl_for_write().world_impl;
      impl_for_write().world_impl = nullptr;
    }
  }
  ensure_bodies_simulated(*this);
}

int PhysicsGeometry::rigid_bodies_num() const
{
  return impl_->rigid_bodies.size();
}

int PhysicsGeometry::constraints_num() const
{
  return 0;
}

int PhysicsGeometry::shapes_num() const
{
  return 0;
}

IndexRange PhysicsGeometry::rigid_bodies_range() const
{
  return impl_->rigid_bodies.index_range();
}

IndexRange PhysicsGeometry::constraints_range() const
{
  return IndexRange();
}

IndexRange PhysicsGeometry::shapes_range() const
{
  return IndexRange();
}

VArray<const CollisionShape *> PhysicsGeometry::body_collision_shapes() const
{
  auto get_fn = [](const btRigidBody *const &body) -> const CollisionShape * {
    return static_cast<CollisionShape *>(body->getCollisionShape()->getUserPointer());
  };
  return VArray<const CollisionShape *>::ForDerivedSpan<const btRigidBody *, get_fn>(
      impl_->rigid_bodies);
}

VMutableArray<CollisionShape *> PhysicsGeometry::body_collision_shapes_for_write()
{
  BLI_assert(impl_->is_mutable());
  auto get_fn = [](btRigidBody *const &body) -> CollisionShape * {
    return static_cast<CollisionShape *>(body->getCollisionShape()->getUserPointer());
  };
  auto set_fn = [](btRigidBody *&body, CollisionShape *value) {
    body->setCollisionShape(&value->impl().as_bullet_shape());
  };
  return VMutableArray<CollisionShape *>::ForDerivedSpan<btRigidBody *, get_fn, set_fn>(
      impl_for_write().rigid_bodies);
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
  return attributes().lookup(builtin_attributes.simulated).varray.typed<bool>();
}

AttributeWriter<bool> PhysicsGeometry::body_is_simulated_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(builtin_attributes.simulated);
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

void PhysicsGeometry::tag_collision_shapes_changed() {}

void PhysicsGeometry::tag_body_transforms_changed() {}

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

template<typename T> static void dummy_set_fn(btRigidBody *& /*body*/, T /*value*/) {}

/**
 * Provider for builtin rigid body attributes.
 */
template<typename T,
         T (*GetFn)(const btRigidBody *const &),
         void (*SetFn)(btRigidBody *&, T) = dummy_set_fn>
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

  /* Wrapper around GetFn that takes a const pointer.
   * Necessary because VArray::ForDerivedSpan expects a callback for const pointers,
   * but VMutableArray::ForDerivedSpan wants a non-const pointer instead.
   * This wrapper allows us to use the same callback for both cases.
   */
  static T get_fn_mutable(btRigidBody *const &body)
  {
    return GetFn(body);
  }

  GAttributeReader try_get_for_read(const void *owner) const final
  {
    const PhysicsGeometry *physics = physics_access_.get_const_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    GVArray varray = VArray<T>::template ForDerivedSpan<const btRigidBody *, GetFn>(
        physics->impl().rigid_bodies);

    return {varray, domain_, nullptr};
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    PhysicsGeometry *physics = physics_access_.get_physics(owner);
    if (physics == nullptr) {
      return {};
    }

    std::function<void()> tag_modified_fn;
    if (update_on_change_ != nullptr) {
      tag_modified_fn = [owner, update = update_on_change_]() { update(owner); };
    }

    GVMutableArray varray =
        VMutableArray<T>::template ForDerivedSpan<btRigidBody *, get_fn_mutable, SetFn>(
            physics->impl_for_write().rigid_bodies);

    return {varray, domain_, std::move(tag_modified_fn)};
  }

  bool try_delete(void * /*owner*/) const final
  {
    if (deletable_ != Deletable) {
      return false;
    }
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

  constexpr auto id_get_fn = [](const btRigidBody *const &body) -> int {
    return body->getUserIndex();
  };
  constexpr auto id_set_fn = [](btRigidBody *&body, int value) { body->setUserIndex(value); };
  static BuiltinRigidBodyAttributeProvider<int, id_get_fn, id_set_fn> body_id(
      PhysicsGeometry::builtin_attributes.id,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto simulated_get_fn = [](const btRigidBody *const &body) -> bool {
    return (get_body_user_flags(*body) & RigidBodyUserFlag::IsSimulated) != RigidBodyUserFlag(0);
  };
  constexpr auto simulated_set_fn = [](btRigidBody *&body, bool value) {
    set_body_user_flags(*body, RigidBodyUserFlag::IsSimulated, value);
  };
  static BuiltinRigidBodyAttributeProvider<bool, simulated_get_fn, simulated_set_fn>
      body_simulated(
          PhysicsGeometry::builtin_attributes.simulated,
          AttrDomain::Point,
          BuiltinAttributeProvider::NonDeletable,
          physics_access,
          [](void *owner) { ensure_bodies_simulated(*static_cast<PhysicsGeometry *>(owner)); });

  constexpr auto mass_get_fn = [](const btRigidBody *const &body) -> float {
    return body->getMass();
  };
  constexpr auto mass_set_fn = [](btRigidBody *&body, float value) {
    body->setMassProps(value, body->getLocalInertia());
  };
  static BuiltinRigidBodyAttributeProvider<float, mass_get_fn, mass_set_fn> body_mass(
      PhysicsGeometry::builtin_attributes.mass,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto inertia_get_fn = [](const btRigidBody *const &body) -> float3 {
    return to_blender(body->getLocalInertia());
  };
  constexpr auto inertia_set_fn = [](btRigidBody *&body, float3 value) {
    body->setMassProps(body->getMass(), to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3, inertia_get_fn, inertia_set_fn> body_inertia(
      PhysicsGeometry::builtin_attributes.inertia,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto position_get_fn = [](const btRigidBody *const &body) -> float3 {
    return to_blender(body->getWorldTransform().getOrigin());
  };
  constexpr auto position_set_fn = [](btRigidBody *&body, float3 value) {
    body->getWorldTransform().setOrigin(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3, position_get_fn, position_set_fn> body_position(
      PhysicsGeometry::builtin_attributes.position,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto rotation_get_fn = [](const btRigidBody *const &body) -> math::Quaternion {
    return to_blender(body->getWorldTransform().getRotation());
  };
  constexpr auto rotation_set_fn = [](btRigidBody *&body, math::Quaternion value) {
    body->getWorldTransform().setRotation(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<math::Quaternion, rotation_get_fn, rotation_set_fn>
      body_rotation(PhysicsGeometry::builtin_attributes.rotation,
                    AttrDomain::Point,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    nullptr);

  constexpr auto velocity_get_fn = [](const btRigidBody *const &body) -> float3 {
    return to_blender(body->getLinearVelocity());
  };
  constexpr auto velocity_set_fn = [](btRigidBody *&body, float3 value) {
    body->setLinearVelocity(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3, velocity_get_fn, velocity_set_fn> body_velocity(
      PhysicsGeometry::builtin_attributes.velocity,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  constexpr auto angular_velocity_get_fn = [](const btRigidBody *const &body) -> float3 {
    return to_blender(body->getAngularVelocity());
  };
  constexpr auto angular_velocity_set_fn = [](btRigidBody *&body, float3 value) {
    body->setAngularVelocity(to_bullet(value));
  };
  static BuiltinRigidBodyAttributeProvider<float3,
                                           angular_velocity_get_fn,
                                           angular_velocity_set_fn>
      body_angular_velocity(PhysicsGeometry::builtin_attributes.angular_velocity,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);

  return ComponentAttributeProviders({&body_id,
                                      &body_mass,
                                      &body_inertia,
                                      &body_position,
                                      &body_rotation,
                                      &body_velocity,
                                      &body_angular_velocity},
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
        return int(physics.rigid_bodies_num());
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

void PhysicsGeometry::steal_data_from(PhysicsGeometry &src_physics, bool use_world)
{
  PhysicsGeometryImpl &src_impl = src_physics.impl_for_write();
  PhysicsGeometryImpl &dst_impl = this->impl_for_write();

  if (use_world) {
    if (world_ != nullptr) {
      BLI_assert(dst_impl.world_impl != nullptr);
      delete world_;
      world_ = nullptr;
      delete dst_impl.world_impl;
      dst_impl.world_impl = nullptr;
    }

    if (src_physics.world_ != nullptr) {
      BLI_assert(src_impl.world_impl != nullptr);

      /* Steal world impl from the source. */
      PhysicsWorldImpl *world_impl = src_impl.world_impl;
      src_impl.world_impl = nullptr;
      delete src_physics.world_;
      src_physics.world_ = nullptr;

      dst_impl.world_impl = world_impl;
      world_ = new PhysicsWorld(world_impl);
    }
  }
}

/** \} */

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::bke
