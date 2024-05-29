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

#include "BLI_mempool.h"
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
  using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;

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

const PhysicsGeometry::BuiltinAttributes PhysicsGeometry::builtin_attributes = {
    "id", "mass", "inertia", "position", "rotation"};

PhysicsGeometry::PhysicsGeometry() : PhysicsGeometry(0) {}

PhysicsGeometry::PhysicsGeometry(int rigid_bodies_num)
{
  impl_ = new PhysicsImpl{};

  impl_->rigid_bodies.reinitialize(rigid_bodies_num);
  impl_->motion_states.reinitialize(rigid_bodies_num);
  for (const int i : IndexRange(rigid_bodies_num)) {
    const float mass = 1.0f;
    const float3 inertia = float3(0.0f);

    impl_->motion_states[i] = new btDefaultMotionState();
    impl_->rigid_bodies[i] = new btRigidBody(
        mass, impl_->motion_states[i], nullptr, to_bullet(inertia));
  }
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  impl_ = new PhysicsImpl{};

  if (other.has_world()) {
    this->set_world(true);
  }

  // TODO is this possible and/or necessary?

  // impl_->rigid_bodies.reinitialize(other.impl_->rigid_bodies.size());
  // impl_->motion_states.reinitialize(other.impl_->motion_states.size());
  // for (const int i : other.impl_->rigid_bodies.index_range()) {
  //   const btRigidBody &src_body = *other.impl_->rigid_bodies[i];
  //   const btMotionState &src_motion_state = *other.impl_->motion_states[i];
  //   btTransform start_transform, center_of_mass;
  //   src_motion_state.getWorldTransform(start_transform);
  //   btMotionState *dst_motion_state = new btDefaultMotionState(start_transform);
  //   const float3 local_inertia = float3(0.0f);

  //   const CollisionShapeID shape_id = src_body.getCollisionShape()->getUserIndex();
  //   std::shared_ptr<CollisionShape> shape = impl_->collision_shapes.lookup(shape_id);
  //   btCollisionShape *bt_shape = shape ? &shape->impl_->as_bullet_shape() : nullptr;

  //   btRigidBody::btRigidBodyConstructionInfo constructionInfo(
  //       src_body.getMass(), dst_motion_state, bt_shape, to_bullet(local_inertia));

  //   impl_->rigid_bodies[i] = other.impl_->rigid_bodies[i];
  // }
}

PhysicsGeometry::~PhysicsGeometry()
{
  // clear_rigid_bodies();
  set_world(false);
}

PhysicsImpl &PhysicsGeometry::impl()
{
  return *impl_;
}

const PhysicsImpl &PhysicsGeometry::impl() const
{
  return *impl_;
}

bool PhysicsGeometry::has_world() const
{
  return impl_->world != nullptr;
}

void PhysicsGeometry::set_world(bool enable)
{
  if (enable) {
    if (impl_->world == nullptr) {
      impl_->config = new btDefaultCollisionConfiguration();
      impl_->dispatcher = new btCollisionDispatcher(impl_->config);
      btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)impl_->dispatcher);

      impl_->broadphase = new btDbvtBroadphase();
      impl_->overlap_filter = new DefaultOverlapFilter();
      impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(
          impl_->overlap_filter);

      impl_->constraint_solver = new btSequentialImpulseConstraintSolver();

      impl_->world = new btDiscreteDynamicsWorld(
          impl_->dispatcher, impl_->broadphase, impl_->constraint_solver, impl_->config);
    }
  }
  else {
    if (impl_->world != nullptr) {
      delete impl_->world;
      delete impl_->constraint_solver;
      delete impl_->broadphase;
      delete impl_->dispatcher;
      delete impl_->config;
      delete impl_->overlap_filter;
    }
  }
}

void PhysicsGeometry::set_overlap_filter(OverlapFilterFn fn)
{
  if (impl_->world == nullptr) {
    return;
  }
  BLI_assert(impl_->broadphase);

  impl_->overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

void PhysicsGeometry::clear_overlap_filter()
{
  if (impl_->world == nullptr) {
    return;
  }
  BLI_assert(impl_->broadphase);

  if (impl_->overlap_filter) {
    delete impl_->overlap_filter;
    impl_->overlap_filter = new DefaultOverlapFilter();
  }
  impl_->broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl_->overlap_filter);
}

float3 PhysicsGeometry::gravity() const
{
  if (impl_->world) {
    return to_blender(impl_->world->getGravity());
  }
  return float3(0.0f);
}

void PhysicsGeometry::set_gravity(const float3 &gravity)
{
  if (impl_->world) {
    impl_->world->setGravity(to_bullet(gravity));
  }
}

void PhysicsGeometry::set_solver_iterations(const int num_solver_iterations)
{
  if (impl_->world) {
    btContactSolverInfo &info = impl_->world->getSolverInfo();
    info.m_numIterations = num_solver_iterations;
  }
}

void PhysicsGeometry::set_split_impulse(const bool split_impulse)
{
  if (impl_->world) {
    btContactSolverInfo &info = impl_->world->getSolverInfo();
    /* Note: Bullet stores this as int, but it's used as a bool. */
    info.m_splitImpulse = int(split_impulse);
  }
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
  auto get_fn = [](btRigidBody *const &body) -> CollisionShape * {
    return static_cast<CollisionShape *>(body->getCollisionShape()->getUserPointer());
  };
  auto set_fn = [](btRigidBody *&body, CollisionShape *value) {
    body->setCollisionShape(&value->impl().as_bullet_shape());
  };
  return VMutableArray<CollisionShape *>::ForDerivedSpan<btRigidBody *, get_fn, set_fn>(
      impl_->rigid_bodies);
}

VArray<RigidBodyID> PhysicsGeometry::body_ids() const
{
  return attributes().lookup(builtin_attributes.id).varray.typed<RigidBodyID>();
}

VArray<float> PhysicsGeometry::body_masses() const
{
  return attributes().lookup(builtin_attributes.mass).varray.typed<float>();
}

VMutableArray<float> PhysicsGeometry::body_masses_for_write()
{
  return attributes_for_write().lookup_for_write(builtin_attributes.mass).varray.typed<float>();
}

VArray<float3> PhysicsGeometry::body_inertias() const
{
  return attributes().lookup(builtin_attributes.inertia).varray.typed<float3>();
}

VMutableArray<float3> PhysicsGeometry::body_inertias_for_write()
{
  return attributes_for_write()
      .lookup_for_write(builtin_attributes.inertia)
      .varray.typed<float3>();
}

VArray<float3> PhysicsGeometry::body_positions() const
{
  return attributes().lookup(builtin_attributes.position).varray.typed<float3>();
}

VMutableArray<float3> PhysicsGeometry::body_positions_for_write()
{
  return attributes_for_write().lookup_for_write(builtin_attributes.position).varray.typed<float3>();
}

VArray<math::Quaternion> PhysicsGeometry::body_rotations() const
{
  return attributes().lookup(builtin_attributes.rotation).varray.typed<math::Quaternion>();
}

VMutableArray<math::Quaternion> PhysicsGeometry::body_rotations_for_write()
{
  return attributes_for_write()
      .lookup_for_write(builtin_attributes.rotation)
      .varray.typed<math::Quaternion>();
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
            physics->impl().rigid_bodies);

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

  constexpr auto id_get_fn = [](const btRigidBody *const &body) -> RigidBodyID {
    return body->getUserIndex();
  };
  static BuiltinRigidBodyAttributeProvider<RigidBodyID, id_get_fn> body_id(
      PhysicsGeometry::builtin_attributes.id, AttrDomain::Point, BuiltinAttributeProvider::NonDeletable, physics_access, nullptr);

  constexpr auto mass_get_fn = [](const btRigidBody *const &body) -> float {
    return body->getMass();
  };
  constexpr auto mass_set_fn = [](btRigidBody *&body, float value) {
    body->setMassProps(value, body->getLocalInertia());
  };
  static BuiltinRigidBodyAttributeProvider<float, mass_get_fn, mass_set_fn> body_mass(
      PhysicsGeometry::builtin_attributes.mass, AttrDomain::Point, BuiltinAttributeProvider::NonDeletable, physics_access, nullptr);

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
  static BuiltinRigidBodyAttributeProvider<math::Quaternion, rotation_get_fn, rotation_set_fn> body_rotation(
      PhysicsGeometry::builtin_attributes.rotation,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);

  return ComponentAttributeProviders({&body_id, &body_mass, &body_inertia, &body_position, &body_rotation}, {});
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

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::bke
