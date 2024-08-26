/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include <functional>
#include <mutex>

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"
#include "BKE_type_conversions.hh"

#include "BLI_array_utils.hh"
#include "BLI_assert.h"
#include "BLI_cpp_type.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "DNA_customdata_types.h"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_intern.hh"
#include "physics_geometry_world.hh"

namespace blender::bke {

#ifdef WITH_BULLET

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

/* Similar to CacheMutex, but with supplied mutex and flag. */
static void ensure_cache(std::mutex &mutex,
                         std::atomic<bool> &flag,
                         const FunctionRef<void()> compute_cache)
{
  if (flag.load(std::memory_order_acquire)) {
    return;
  }
  std::scoped_lock lock{mutex};
  /* Double checked lock. */
  if (flag.load(std::memory_order_relaxed)) {
    return;
  }
  /* Use task isolation because a mutex is locked and the cache computation might use
   * multi-threading. */
  threading::isolate_task(compute_cache);

  flag.store(true, std::memory_order_release);
}

/* Same as ensure_cache but updates if any flag is dirty. */
static void ensure_cache_any(std::mutex &mutex,
                             Span<std::atomic<bool> *> flags,
                             Span<const FunctionRef<void()>> compute_caches)
{
  bool all_valid = true;
  for (const int i : flags.index_range()) {
    if (!flags[i]->load(std::memory_order_acquire)) {
      all_valid = false;
      break;
    }
  }
  if (all_valid) {
    return;
  }
  std::scoped_lock lock{mutex};
  /* Double checked lock. */
  for (const int i : flags.index_range()) {
    if (!flags[i]->load(std::memory_order_acquire)) {
      /* Use task isolation because a mutex is locked and the cache computation might use
       * multi-threading. */
      threading::isolate_task(compute_caches[i]);

      flags[i]->store(true, std::memory_order_release);
    }
  }
}

static void tag_cache_dirty(std::atomic<bool> &flag)
{
  flag.store(false);
}

static bool is_cache_dirty(const std::atomic<bool> &flag)
{
  return !flag.load(std::memory_order_relaxed);
}

PhysicsWorldState::PhysicsWorldState()
    : body_num_(0), constraint_num_(0), body_data_({}), constraint_data_({})
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
  this->tag_read_cache_changed();
}

PhysicsWorldState::PhysicsWorldState(int body_num, int constraint_num, int shape_num)
    : body_num_(body_num), constraint_num_(constraint_num), body_data_({}), constraint_data_({})
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
  CustomData_realloc(&body_data_, 0, body_num);
  CustomData_realloc(&constraint_data_, 0, constraint_num);
  shapes_.reinitialize(shape_num);
  this->tag_read_cache_changed();
}

PhysicsWorldState::PhysicsWorldState(const PhysicsWorldState &other)
{
  *this = other;
  this->tag_read_cache_changed();
}

PhysicsWorldState::~PhysicsWorldState()
{
  CustomData_free(&body_data_, body_num_);
  CustomData_free(&constraint_data_, constraint_num_);

  /* World data is owned by the geometry when it's mutable (always the case on destruction). */
  delete world_data_;
}

PhysicsWorldState &PhysicsWorldState::operator=(const PhysicsWorldState &other)
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
  body_num_ = other.body_num_;
  constraint_num_ = other.constraint_num_;
  CustomData_copy(&other.body_data_, &body_data_, CD_MASK_ALL, other.body_num_);
  CustomData_copy(&other.constraint_data_, &constraint_data_, CD_MASK_ALL, other.constraint_num_);

  shapes_ = other.shapes_;

  if (other.world_data_) {
    try_move_data(other,
                  body_num_,
                  constraint_num_,
                  IndexRange(body_num_),
                  IndexRange(constraint_num_),
                  0,
                  0);
  }

  this->tag_read_cache_changed();
  return *this;
}

void PhysicsWorldState::delete_self()
{
  delete this;
}

int PhysicsWorldState::bodies_num() const
{
  return body_num_;
}

int PhysicsWorldState::constraints_num() const
{
  return constraint_num_;
}

int PhysicsWorldState::shapes_num() const
{
  return shapes_.size();
}

IndexRange PhysicsWorldState::bodies_range() const
{
  return IndexRange(body_num_);
}

IndexRange PhysicsWorldState::constraints_range() const
{
  return IndexRange(constraint_num_);
}

IndexRange PhysicsWorldState::shapes_range() const
{
  return shapes_.index_range();
}

bool PhysicsWorldState::has_world_data() const
{
  return world_data_ != nullptr;
}

void PhysicsWorldState::tag_read_cache_changed()
{
  /* Cache only becomes invalid if there is world data that can change. */
  if (world_data_ != nullptr) {
    tag_cache_dirty(custom_data_read_cache_valid_);
  }
}

void PhysicsWorldState::tag_body_topology_changed()
{
  this->tag_constraint_disable_collision_changed();
}

void PhysicsWorldState::tag_body_collision_shape_changed()
{
  tag_cache_dirty(body_collision_shapes_valid_);
}

void PhysicsWorldState::tag_body_is_static_changed()
{
  tag_cache_dirty(body_is_static_valid_);
}

void PhysicsWorldState::tag_body_mass_changed()
{
  tag_cache_dirty(body_mass_valid_);
}

void PhysicsWorldState::tag_constraints_changed()
{
  tag_cache_dirty(constraints_valid_);
}

void PhysicsWorldState::tag_constraint_disable_collision_changed()
{
  tag_cache_dirty(constraint_disable_collision_valid_);
}

void PhysicsWorldState::tag_shapes_changed()
{
  /* Update same as if shape indices had been changed. */
  this->tag_body_collision_shape_changed();
}

bool PhysicsWorldState::has_builtin_attribute_custom_data_layer(
    PhysicsWorldState::BodyAttribute attribute) const
{
  AttributeAccessor dst_attributes = this->custom_data_attributes();
  StringRef name = physics_attribute_name(attribute);
  return bool(dst_attributes.lookup_meta_data(name));
}

bool PhysicsWorldState::has_builtin_attribute_custom_data_layer(
    PhysicsWorldState::ConstraintAttribute attribute) const
{
  AttributeAccessor dst_attributes = this->custom_data_attributes();
  StringRef name = physics_attribute_name(attribute);
  return bool(dst_attributes.lookup_meta_data(name));
}

void PhysicsWorldState::ensure_read_cache() const
{
  ensure_cache(world_data_mutex_, custom_data_read_cache_valid_, [&]() {
    this->ensure_read_cache_no_lock();
  });
}

void PhysicsWorldState::ensure_read_cache_no_lock() const
{
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  const static StringRef collision_shape_id = physics_attribute_name(
      BodyAttribute::collision_shape);
  const static StringRef is_static_id = physics_attribute_name(BodyAttribute::is_static);
  const static StringRef mass_id = physics_attribute_name(BodyAttribute::mass);
  const static StringRef constraint_type_id = physics_attribute_name(
      ConstraintAttribute::constraint_type);
  const static StringRef constraint_body1_id = physics_attribute_name(
      ConstraintAttribute::constraint_body1);
  const static StringRef constraint_body2_id = physics_attribute_name(
      ConstraintAttribute::constraint_body2);
  const static StringRef disable_collision_id = physics_attribute_name(
      ConstraintAttribute::disable_collision);

  PhysicsWorldState &dst = *const_cast<PhysicsWorldState *>(this);

  if (world_data_ == nullptr) {
    for (const BodyAttribute attribute : all_body_attributes()) {
      dst.ensure_custom_data_attribute_no_lock(attribute);
    }
    for (const ConstraintAttribute attribute : all_constraint_attributes()) {
      dst.ensure_custom_data_attribute_no_lock(attribute);
    }
    return;
  }

  /* Some attributes require other updates before valid world data can be read. */
  dst.ensure_motion_type_no_lock();
  dst.ensure_constraints_no_lock();

  /* Write to cache attributes. */
  MutableAttributeAccessor dst_attributes = dst.custom_data_attributes_for_write();

  /* Read from world data and ignore the cache.
   * Important! This also prevents deadlock caused by re-entering this function. */
  const AttributeAccessor src_attributes = this->world_data_attributes();
  Set<std::string> skip_attributes = {collision_shape_id,
                                      is_static_id,
                                      mass_id,
                                      constraint_type_id,
                                      constraint_body1_id,
                                      constraint_body2_id,
                                      disable_collision_id};
  /* Only use builtin attributes, dynamic attributes are already in custom data. */
  src_attributes.for_all(
      [&](const AttributeIDRef &id, const AttributeMetaData & /*meta_data*/) -> bool {
        if (!src_attributes.is_builtin(id)) {
          skip_attributes.add(id.name());
        }
        return true;
      });

  gather_attributes(src_attributes,
                    bke::AttrDomain::Point,
                    {},
                    skip_attributes,
                    IndexRange(body_num_),
                    dst_attributes);
  gather_attributes(src_attributes,
                    bke::AttrDomain::Edge,
                    {},
                    skip_attributes,
                    IndexRange(constraint_num_),
                    dst_attributes);
}

void PhysicsWorldState::ensure_motion_type()
{
  ensure_cache_any(world_data_mutex_,
                   {&body_collision_shapes_valid_, &body_is_static_valid_, &body_mass_valid_},
                   {[&]() { this->ensure_body_collision_shapes_no_lock(); },
                    [&]() { this->ensure_body_is_static_no_lock(); },
                    [&]() { this->ensure_body_masses_no_lock(); }});
}

void PhysicsWorldState::ensure_motion_type_no_lock()
{
  this->ensure_body_collision_shapes_no_lock();
  this->ensure_body_is_static_no_lock();
  this->ensure_body_masses_no_lock();
}

void PhysicsWorldState::ensure_body_collision_shapes_no_lock()
{
  const static StringRef collision_shape_id = physics_attribute_name(
      PhysicsBodyAttribute::collision_shape);

  if (!is_cache_dirty(body_collision_shapes_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArraySpan<int> body_shapes = *custom_data_attributes.lookup_or_default(
      collision_shape_id, AttrDomain::Point, -1);

  const IndexMask selection = world_data_->bodies().index_range();
  world_data_->set_body_shapes(selection, shapes_, body_shapes);
}

void PhysicsWorldState::ensure_body_is_static_no_lock()
{
  const static StringRef is_static_id = physics_attribute_name(PhysicsBodyAttribute::is_static);

  if (!is_cache_dirty(body_is_static_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArraySpan<bool> is_static = *custom_data_attributes.lookup_or_default<bool>(
      is_static_id, AttrDomain::Point, -1);

  const IndexMask selection = world_data_->bodies().index_range();
  world_data_->set_body_static(selection, is_static);
}

void PhysicsWorldState::ensure_body_masses_no_lock()
{
  const static StringRef mass_id = physics_attribute_name(PhysicsBodyAttribute::mass);

  if (!is_cache_dirty(body_mass_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArraySpan<float> masses = *custom_data_attributes.lookup_or_default<float>(
      mass_id, AttrDomain::Point, 0.0f);

  const IndexMask selection = world_data_->bodies().index_range();
  world_data_->set_body_mass(selection, masses);
}

void PhysicsWorldState::ensure_constraints()
{
  ensure_cache_any(world_data_mutex_,
                   {&constraints_valid_, &constraint_disable_collision_valid_},
                   {[&]() { this->ensure_constraints_no_lock(); },
                    [&]() { this->ensure_constraint_disable_collision_no_lock(); }});
}

void PhysicsWorldState::ensure_constraints_no_lock()
{
  using ConstraintType = PhysicsConstraintType;

  const static StringRef constraint_type_id = physics_attribute_name(
      PhysicsConstraintAttribute::constraint_type);
  const static StringRef constraint_body1_id = physics_attribute_name(
      PhysicsConstraintAttribute::constraint_body1);
  const static StringRef constraint_body2_id = physics_attribute_name(
      PhysicsConstraintAttribute::constraint_body2);

  if (!is_cache_dirty(constraints_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArray<int> types = *custom_data_attributes.lookup_or_default<int>(
      constraint_type_id, AttrDomain::Edge, int(ConstraintType::Fixed));
  const VArray<int> body1 = *custom_data_attributes.lookup_or_default<int>(
      constraint_body1_id, AttrDomain::Edge, -1);
  const VArray<int> body2 = *custom_data_attributes.lookup_or_default<int>(
      constraint_body2_id, AttrDomain::Edge, -1);

  const IndexMask selection = world_data_->constraints().index_range();
  world_data_->create_constraints(selection, types, body1, body2);
}

void PhysicsWorldState::ensure_constraint_disable_collision()
{
  ensure_cache(world_data_mutex_, constraint_disable_collision_valid_, [&]() {
    this->ensure_constraint_disable_collision_no_lock();
  });
}

void PhysicsWorldState::ensure_constraint_disable_collision_no_lock()
{
  const static StringRef disable_collision_id = physics_attribute_name(
      PhysicsConstraintAttribute::disable_collision);

  if (!is_cache_dirty(constraint_disable_collision_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArray<bool> disable_collision = *custom_data_attributes.lookup_or_default<bool>(
      disable_collision_id, AttrDomain::Edge, false);

  const IndexMask selection = world_data_->constraints().index_range();
  world_data_->set_disable_collision(selection, disable_collision);
}

void PhysicsWorldState::ensure_custom_data_attribute(
    PhysicsWorldState::BodyAttribute attribute) const
{
  if (has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }

  PhysicsWorldState &dst = *const_cast<PhysicsWorldState *>(this);
  dst.ensure_custom_data_attribute_no_lock(attribute);
}

void PhysicsWorldState::ensure_custom_data_attribute(
    PhysicsWorldState::ConstraintAttribute attribute) const
{
  if (has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }

  PhysicsWorldState &dst = *const_cast<PhysicsWorldState *>(this);
  dst.ensure_custom_data_attribute_no_lock(attribute);
}

void PhysicsWorldState::ensure_custom_data_attribute_no_lock(BodyAttribute attribute)
{
  MutableAttributeAccessor dst_attributes = this->custom_data_attributes_for_write();
  StringRef name = physics_attribute_name(attribute);
  const CPPType &type = physics_attribute_type(attribute);
  GVArray varray = GVArray::ForSingle(type, body_num_, physics_attribute_default_value(attribute));
  eCustomDataType data_type = cpp_type_to_custom_data_type(type);
  dst_attributes.add(name, AttrDomain::Point, data_type, AttributeInitVArray(varray));
}

void PhysicsWorldState::ensure_custom_data_attribute_no_lock(ConstraintAttribute attribute)
{
  MutableAttributeAccessor dst_attributes = this->custom_data_attributes_for_write();
  StringRef name = physics_attribute_name(attribute);
  const CPPType &type = physics_attribute_type(attribute);
  GVArray varray = GVArray::ForSingle(
      type, constraint_num_, physics_attribute_default_value(attribute));
  eCustomDataType data_type = cpp_type_to_custom_data_type(type);
  dst_attributes.add(name, AttrDomain::Edge, data_type, AttributeInitVArray(varray));
}

void PhysicsWorldState::remove_attributes_from_customdata()
{
  /* Force use of cache for writing. */
  MutableAttributeAccessor attributes = this->custom_data_attributes_for_write();
  attributes.for_all(
      [&](const AttributeIDRef &attribute_id, const AttributeMetaData &meta_data) -> bool {
        CustomData *custom_data = nullptr;
        int totelem = 0;
        switch (meta_data.domain) {
          case AttrDomain::Point:
            custom_data = &body_data_;
            totelem = body_num_;
            break;
          case AttrDomain::Edge:
            custom_data = &constraint_data_;
            totelem = constraint_num_;
            break;
          case AttrDomain::Instance:
            break;
          default:
            BLI_assert_unreachable();
            break;
        }
        if (custom_data == nullptr) {
          return true;
        }
        CustomData_free_layer_named(custom_data, attribute_id.name(), totelem);
        return true;
      });
}

void PhysicsWorldState::create_world()
{
  // using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using ConstraintType = PhysicsConstraintType;

  /* Avoid locking later on. */
  this->ensure_read_cache();

  if (world_data_) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_) {
    return;
  }

  /* Read from custom data, write to world data. */
  const AttributeAccessor src_attributes = this->custom_data_attributes();

  const IndexRange body_range = IndexRange(body_num_);
  const IndexRange constraint_range = IndexRange(constraint_num_);

  const VArraySpan<int> body_shapes = *src_attributes.lookup_or_default(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
  const VArray<int> constraint_types = *src_attributes.lookup_or_default(
      physics_attribute_name(ConstraintAttribute::constraint_type),
      AttrDomain::Edge,
      int(ConstraintType::Fixed));
  const VArray<int> constraint_bodies1 = *src_attributes.lookup_or_default(
      physics_attribute_name(ConstraintAttribute::constraint_body1), AttrDomain::Edge, -1);
  const VArray<int> constraint_bodies2 = *src_attributes.lookup_or_default(
      physics_attribute_name(ConstraintAttribute::constraint_body2), AttrDomain::Edge, -1);

  world_data_ = new PhysicsWorldData(body_num_, constraint_num_);
  world_data_->set_body_shapes(body_range, shapes_, body_shapes);
  world_data_->create_constraints(
      constraint_range, constraint_types, constraint_bodies1, constraint_bodies2);
}

void PhysicsWorldState::destroy_world()
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  this->ensure_read_cache();
  delete world_data_;
  world_data_ = nullptr;
}

static void remap_bodies(const int src_bodies_num,
                         const IndexMask &bodies_mask,
                         const IndexMask &constraints_mask,
                         const Span<int> src_constraint_types,
                         const Span<int> src_constraint_body1,
                         const Span<int> src_constraint_body2,
                         MutableSpan<int> dst_constraint_types,
                         MutableSpan<int> dst_constraint_body1,
                         MutableSpan<int> dst_constraint_body2)
{
  Array<int> map(src_bodies_num);
  const IndexRange body_range = map.index_range();
  index_mask::build_reverse_map<int>(bodies_mask, map);
  constraints_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
    dst_constraint_types[dst_i] = src_constraint_types[src_i];

    const int body1 = src_constraint_body1[src_i];
    const int body2 = src_constraint_body2[src_i];
    dst_constraint_body1[dst_i] = body_range.contains(body1) ? map[body1] : -1;
    dst_constraint_body2[dst_i] = body_range.contains(body2) ? map[body2] : -1;
  });
}

void PhysicsWorldState::move_or_copy_selection(
    const PhysicsWorldState &src,
    const IndexMask &src_body_mask,
    const IndexMask &src_constraint_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  this->try_move_data(src, body_num_, constraint_num_, src_body_mask, src_constraint_mask, 0, 0);

  const bke::AttributeAccessor src_attributes = src.attributes();
  bke::MutableAttributeAccessor dst_attributes = this->attributes_for_write();

  Set<std::string> ignored_attributes = {};
  if (world_data_) {
    /* Don't copy builtin attributes when there is world data. */
    ignored_attributes.add_multiple(all_body_attribute_names());
    ignored_attributes.add_multiple(all_constraint_attribute_names());
  }
  else {
    /* Map index references in custom data that is no longer based on world data. */
    static const StringRef constraint_type_id = physics_attribute_name(
        PhysicsConstraintAttribute::constraint_type);
    static const StringRef constraint_body1_id = physics_attribute_name(
        PhysicsConstraintAttribute::constraint_body1);
    static const StringRef constraint_body2_id = physics_attribute_name(
        PhysicsConstraintAttribute::constraint_body2);

    ignored_attributes.add_multiple(
        {constraint_type_id, constraint_body1_id, constraint_body2_id});

    const VArraySpan<int> src_types = *src_attributes.lookup<int>(constraint_type_id);
    const VArraySpan<int> src_body1 = *src_attributes.lookup<int>(constraint_body1_id);
    const VArraySpan<int> src_body2 = *src_attributes.lookup<int>(constraint_body2_id);

    SpanAttributeWriter<int> dst_types = dst_attributes.lookup_for_write_span<int>(
        constraint_type_id);
    SpanAttributeWriter<int> dst_body1 = dst_attributes.lookup_for_write_span<int>(
        constraint_body1_id);
    SpanAttributeWriter<int> dst_body2 = dst_attributes.lookup_for_write_span<int>(
        constraint_body2_id);

    remap_bodies(src.body_num_,
                 src_body_mask,
                 src_constraint_mask,
                 src_types,
                 src_body1,
                 src_body2,
                 dst_types.span,
                 dst_body1.span,
                 dst_body2.span);
  }

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         propagation_info,
                         ignored_attributes,
                         src_body_mask,
                         dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         propagation_info,
                         ignored_attributes,
                         src_constraint_mask,
                         dst_attributes);
}

bool PhysicsWorldState::try_move_data(const PhysicsWorldState &src,
                                      const int body_num,
                                      const int constraint_num,
                                      const IndexMask &src_body_mask,
                                      const IndexMask &src_constraint_mask,
                                      int dst_body_offset,
                                      int dst_constraint_offset)
{
  BLI_assert(this->is_mutable());

  if (src.world_data_ == nullptr) {
    return false;
  }
  std::scoped_lock src_lock(src.world_data_mutex_);
  if (src.world_data_ == nullptr) {
    return false;
  }

  std::scoped_lock dst_lock(world_data_mutex_);
  if (world_data_) {
    delete world_data_;
    world_data_ = nullptr;
  }

  world_data_ = src.world_data_;
  src.world_data_ = nullptr;

  world_data_->resize(body_num,
                      constraint_num,
                      src_body_mask,
                      src_constraint_mask,
                      dst_body_offset,
                      dst_constraint_offset);

  return true;
}

Span<CollisionShapePtr> PhysicsWorldState::shapes() const
{
  return shapes_;
}

MutableSpan<CollisionShapePtr> PhysicsWorldState::shapes_for_write()
{
  return shapes_;
}

void PhysicsWorldState::set_overlap_filter(OverlapFilterFn fn)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  world_data_->set_overlap_filter(fn);
}

void PhysicsWorldState::clear_overlap_filter()
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  world_data_->clear_overlap_filter();
}

float3 PhysicsWorldState::gravity() const
{
  /* TODO add output caches for single values too, to avoid locking. */
  if (world_data_ == nullptr) {
    return float3(0.0f);
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return float3(0.0f);
  }

  return world_data_->gravity();
}

void PhysicsWorldState::set_gravity(const float3 &gravity)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  world_data_->set_gravity(gravity);
}

void PhysicsWorldState::set_solver_iterations(const int num_solver_iterations)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  world_data_->set_solver_iterations(num_solver_iterations);
}

void PhysicsWorldState::set_split_impulse(const bool split_impulse)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  world_data_->set_split_impulse(split_impulse);
}

void PhysicsWorldState::step_simulation(float delta_time)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  this->ensure_motion_type_no_lock();
  this->ensure_constraints_no_lock();
  world_data_->step_simulation(delta_time);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::compute_local_inertia(const IndexMask &selection)
{
  using BodyAttribute = PhysicsBodyAttribute;

  MutableAttributeAccessor attributes = this->attributes_for_write();
  const VArray<int> body_shapes = *attributes.lookup_or_default<int>(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
  const VArray<float> masses = *attributes.lookup_or_default<float>(
      physics_attribute_name(BodyAttribute::mass), AttrDomain::Point, 1.0f);
  AttributeWriter<float3> local_inertias = attributes.lookup_or_add_for_write<float3>(
      physics_attribute_name(BodyAttribute::inertia), AttrDomain::Point);

  selection.foreach_index([&](const int body_i) {
    const int shape_index = body_shapes[body_i];
    if (!shapes_.index_range().contains(shape_index)) {
      local_inertias.varray.set(body_i, float3(1.0f));
      return;
    }
    const CollisionShapePtr &shape_ptr = shapes_[shape_index];
    if (shape_ptr == nullptr) {
      local_inertias.varray.set(body_i, float3(1.0f));
      return;
    }

    local_inertias.varray.set(body_i, shape_ptr->calculate_local_inertia(masses[body_i]));
  });

  local_inertias.finish();
}

void PhysicsWorldState::apply_force(const IndexMask &selection,
                                    const VArray<float3> &forces,
                                    const VArray<float3> &relative_positions)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_force(selection, forces, relative_positions);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_torque(selection, torques);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_impulse(const IndexMask &selection,
                                      const VArray<float3> &impulses,
                                      const VArray<float3> &relative_positions)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_impulse(selection, impulses, relative_positions);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_angular_impulse(const IndexMask &selection,
                                              const VArray<float3> &angular_impulses)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_angular_impulse(selection, angular_impulses);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::clear_forces(const IndexMask &selection)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->clear_forces(selection);

  this->tag_read_cache_changed();
}

[[maybe_unused]] static bool has_constraint_ref(const btRigidBody &body,
                                                const btTypedConstraint &constraint)
{
  for (const int i_ref : IndexRange(body.getNumConstraintRefs())) {
    const btTypedConstraint *ref_constraint = const_cast<btRigidBody &>(body).getConstraintRef(
        i_ref);
    if (ref_constraint == &constraint) {
      return true;
    }
  }
  return false;
}

bool PhysicsWorldState::validate_world_data()
{
  bool ok = true;

  if (world_data_ == nullptr) {
    return ok;
  }

  this->ensure_motion_type();
  this->ensure_constraints();
  this->ensure_read_cache();
  world_data_->ensure_body_and_constraint_indices();
  world_data_->ensure_bodies_and_constraints_in_world();

  AttributeAccessor cached_attributes = custom_data_attributes();
  const Span<btRigidBody *> rigid_bodies = world_data_->bodies();
  const Span<btTypedConstraint *> constraints = world_data_->constraints();
  const btCollisionObjectArray &bt_collision_objects =
      world_data_->world().getCollisionObjectArray();

  for (const int i : rigid_bodies.index_range()) {
    const btRigidBody *body = rigid_bodies[i];

    /* Bodies should always be allocated. */
    if (body == nullptr) {
      BLI_assert_unreachable();
      ok = false;
    }
    if (get_body_index(*body) != i) {
      BLI_assert_unreachable();
      ok = false;
    }

    /* All bodies must be in the world, except if they don't have a collision shape. */
    if (body->getCollisionShape() != nullptr) {
      if (!body->isInWorld()) {
        BLI_assert_unreachable();
        ok = false;
      }
      if (bt_collision_objects.findLinearSearch(const_cast<btRigidBody *>(body)) >=
          bt_collision_objects.size())
      {
        BLI_assert_unreachable();
        ok = false;
      }
    }

    /* Bodies with a non-moving collision shape must be static and zero-mass. */
    if (body->getCollisionShape() != nullptr && body->getCollisionShape()->isNonMoving()) {
      if (!body->isStaticObject() || body->getMass() != 0.0f) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    /* Static bodies must have zero mass. */
    if (body->isStaticObject()) {
      if (body->getMass() != 0.0f) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    /* Zero mass bodies must be static. */
    if (body->getMass() == 0.0f) {
      if (!body->isStaticObject()) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
  }

  const VArray<int> cached_constraint_types = *cached_attributes.lookup<int>(
      physics_attribute_name(ConstraintAttribute::constraint_type), AttrDomain::Edge);
  const VArray<int> cached_constraint_body1 = *cached_attributes.lookup<int>(
      physics_attribute_name(ConstraintAttribute::constraint_body1), AttrDomain::Edge);
  const VArray<int> cached_constraint_body2 = *cached_attributes.lookup<int>(
      physics_attribute_name(ConstraintAttribute::constraint_body2), AttrDomain::Edge);
  const VArray<bool> cached_constraint_disable_collision = *cached_attributes.lookup<bool>(
      physics_attribute_name(ConstraintAttribute::disable_collision), AttrDomain::Edge);
  for (const int i : constraints.index_range()) {
    const btTypedConstraint *constraint = constraints[i];
    if (constraint == nullptr) {
      BLI_assert_unreachable();
      ok = false;
    }
    /* Constraint class should match the type enum. */
    if (!validate_bullet_constraint_type(PhysicsConstraintType(cached_constraint_types[i]),
                                         constraint))
    {
      BLI_assert_unreachable();
      ok = false;
    }
    if (get_constraint_index(*constraint) != i) {
      BLI_assert_unreachable();
      ok = false;
    }

    const int cached_body1 = cached_constraint_body1[i];
    const int cached_body2 = cached_constraint_body2[i];
    const btRigidBody *bt_body1_expected = (rigid_bodies.index_range().contains(cached_body1) ?
                                                rigid_bodies[cached_body1] :
                                                &btTypedConstraint::getFixedBody());
    const btRigidBody *bt_body2_expected = (rigid_bodies.index_range().contains(cached_body2) ?
                                                rigid_bodies[cached_body2] :
                                                &btTypedConstraint::getFixedBody());
    if (&constraint->getRigidBodyA() != bt_body1_expected) {
      BLI_assert_unreachable();
      ok = false;
    }
    if (&constraint->getRigidBodyB() != bt_body2_expected) {
      BLI_assert_unreachable();
      ok = false;
    }

    /* Bullet does not keep track of which constraints have been added to the world. This flag is
     * set by us to know when to add or remove a constraint. */
    if (!is_constraint_in_world(*constraint)) {
      BLI_assert_unreachable();
      ok = false;
    }
    /* Constraints with body1 == body2 are not actually added to the world (Bullet crashes
     * otherwise). */
    if (&constraint->getRigidBodyA() != &constraint->getRigidBodyB()) {
      bool world_has_constraint_ref = false;
      for (const int i : IndexRange(world_data_->world().getNumConstraints())) {
        if (world_data_->world().getConstraint(i) == constraint) {
          world_has_constraint_ref = true;
        }
      }
      if (!world_has_constraint_ref) {
        BLI_assert_unreachable();
        ok = false;
      }
    }

    /* Constraints that disable the collision between its bodies add a reference on each body. */
    const btRigidBody &body1 = constraint->getRigidBodyA();
    const btRigidBody &body2 = constraint->getRigidBodyB();
    const bool is_fixed1 = (&body1 == &btTypedConstraint::getFixedBody());
    const bool is_fixed2 = (&body2 == &btTypedConstraint::getFixedBody());
    const bool found_ref_in_body1 = has_constraint_ref(body1, *constraint);
    const bool found_ref_in_body2 = has_constraint_ref(body2, *constraint);

    if (cached_constraint_disable_collision[i]) {
      if ((!is_fixed1 && !found_ref_in_body1) || (!is_fixed2 && !found_ref_in_body2)) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    else {
      if ((!is_fixed1 && found_ref_in_body1) || (!is_fixed2 && found_ref_in_body2)) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
  }

  const VArray<int> cached_body_shapes = *cached_attributes.lookup<int>(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point);
  const IndexRange shape_range = shapes_.index_range();
  if (rigid_bodies.size() != cached_body_shapes.size()) {
    BLI_assert_unreachable();
    ok = false;
  }
  for (const int i : cached_body_shapes.index_range()) {
    /* Internal shape pointers must match the body shape index attribute. */
    const int shape_index = cached_body_shapes[i];
    if (shape_range.contains(shape_index)) {
      /* Shape pointer must match indicated shape. */
      const CollisionShapePtr shape_ptr = shapes_[shape_index];
      if (!validate_bullet_body_shape(*rigid_bodies[i], shape_ptr)) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    else {
      const btCollisionShape *bt_shape = rigid_bodies[i]->getCollisionShape();
      if (bt_shape != nullptr) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
  }

  return ok;
}

AttributeAccessor PhysicsWorldState::attributes() const
{
  return AttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldState::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsWorldState::custom_data_attributes() const
{
  return AttributeAccessor(this, bke::get_physics_custom_data_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldState::custom_data_attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_custom_data_accessor_functions_ref());
}

AttributeAccessor PhysicsWorldState::world_data_attributes() const
{
  return AttributeAccessor(this, bke::get_physics_world_data_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldState::world_data_attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_world_data_accessor_functions_ref());
}

const PhysicsWorldDataAccessInfo &PhysicsWorldState::world_data_access_info()
{
  static PhysicsWorldDataAccessInfo world_data_access = {
      [](void *owner) -> PhysicsWorldData * {
        auto &state = *static_cast<PhysicsWorldState *>(owner);
        return state.world_data_;
      },
      [](const void *owner) -> const PhysicsWorldData * {
        const auto &state = *static_cast<const PhysicsWorldState *>(owner);
        return state.world_data_;
      }};
  return world_data_access;
}

const CustomDataAccessInfo &PhysicsWorldState::body_custom_data_access_info()
{
  static CustomDataAccessInfo body_custom_data_access = {
      [](void *owner) -> CustomData * {
        auto &state = *static_cast<PhysicsWorldState *>(owner);
        return &state.body_data_;
      },
      [](const void *owner) -> const CustomData * {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return &state->body_data_;
      },
      [](const void *owner) -> int {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return state->body_num_;
      }};
  return body_custom_data_access;
}

const CustomDataAccessInfo &PhysicsWorldState::constraint_custom_data_access_info()
{
  static CustomDataAccessInfo constraint_custom_data_access = {
      [](void *owner) -> CustomData * {
        auto &state = *static_cast<PhysicsWorldState *>(owner);
        return &state.constraint_data_;
      },
      [](const void *owner) -> const CustomData * {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return &state->constraint_data_;
      },
      [](const void *owner) -> int {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return state->constraint_num_;
      }};
  return constraint_custom_data_access;
}

PhysicsGeometry::PhysicsGeometry()
{
  world_state_ = new PhysicsWorldState();
}

PhysicsGeometry::PhysicsGeometry(int bodies_num, int constraints_num, int shapes_num)
{
  world_state_ = new PhysicsWorldState(bodies_num, constraints_num, shapes_num);
  this->state_for_write().tag_body_topology_changed();
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  world_state_ = other.world_state_;
  world_state_->add_user();
}

PhysicsGeometry::~PhysicsGeometry()
{
  BLI_assert(world_state_ && world_state_->strong_users() > 0);
  world_state_->remove_user_and_delete_if_last();
}

const PhysicsWorldState &PhysicsGeometry::state() const
{
  return *world_state_;
}

PhysicsWorldState &PhysicsGeometry::state_for_write()
{
  if (world_state_->is_mutable()) {
    return *const_cast<PhysicsWorldState *>(world_state_);
  }

  PhysicsWorldState *new_state = new PhysicsWorldState(*world_state_);

  new_state->try_move_data(*world_state_,
                           world_state_->bodies_num(),
                           world_state_->constraints_num(),
                           world_state_->bodies_range(),
                           world_state_->constraints_range(),
                           0,
                           0);

  world_state_->remove_user_and_delete_if_last();
  world_state_ = new_state;

  return *const_cast<PhysicsWorldState *>(world_state_);
}

int PhysicsGeometry::bodies_num() const
{
  return world_state_->bodies_num();
}

int PhysicsGeometry::constraints_num() const
{
  return world_state_->constraints_num();
}

int PhysicsGeometry::shapes_num() const
{
  return world_state_->shapes_num();
}

IndexRange PhysicsGeometry::bodies_range() const
{
  return world_state_->bodies_range();
}

IndexRange PhysicsGeometry::constraints_range() const
{
  return world_state_->constraints_range();
}

IndexRange PhysicsGeometry::shapes_range() const
{
  return world_state_->shapes_range();
}

VArray<int> PhysicsGeometry::body_ids() const
{
  return *attributes().lookup<int>(body_attribute_name(BodyAttribute::id));
}

AttributeWriter<int> PhysicsGeometry::body_ids_for_write()
{
  return attributes_for_write().lookup_for_write<int>(body_attribute_name(BodyAttribute::id));
}

VArray<int> PhysicsGeometry::body_shapes() const
{
  return *attributes().lookup_or_default<int>(
      body_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
}

AttributeWriter<int> PhysicsGeometry::body_shapes_for_write()
{
  const GVArray init_varray = VArray<int>::ForSingle(-1, this->state().bodies_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      body_attribute_name(BodyAttribute::collision_shape),
      AttrDomain::Point,
      AttributeInitVArray(init_varray));
}

VArray<bool> PhysicsGeometry::body_is_static() const
{
  return *attributes().lookup<bool>(body_attribute_name(BodyAttribute::is_static));
}

AttributeWriter<bool> PhysicsGeometry::body_is_static_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<bool>(
      body_attribute_name(BodyAttribute::is_static), AttrDomain::Point);
}

VArray<bool> PhysicsGeometry::body_is_kinematic() const
{
  return *attributes().lookup<bool>(body_attribute_name(BodyAttribute::is_kinematic));
}

AttributeWriter<bool> PhysicsGeometry::body_is_kinematic_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(
      body_attribute_name(BodyAttribute::is_kinematic));
}

VArray<float> PhysicsGeometry::body_masses() const
{
  return *attributes().lookup<float>(body_attribute_name(BodyAttribute::mass));
}

AttributeWriter<float> PhysicsGeometry::body_masses_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<float>(
      body_attribute_name(BodyAttribute::mass), AttrDomain::Point);
}

VArray<float3> PhysicsGeometry::body_inertias() const
{
  return *attributes().lookup<float3>(body_attribute_name(BodyAttribute::inertia));
}

AttributeWriter<float3> PhysicsGeometry::body_inertias_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::inertia));
}

VArray<float3> PhysicsGeometry::body_positions() const
{
  return *attributes().lookup<float3>(body_attribute_name(BodyAttribute::position));
}

AttributeWriter<float3> PhysicsGeometry::body_positions_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::position));
}

VArray<math::Quaternion> PhysicsGeometry::body_rotations() const
{
  return *attributes().lookup<math::Quaternion>(body_attribute_name(BodyAttribute::rotation));
}

AttributeWriter<math::Quaternion> PhysicsGeometry::body_rotations_for_write()
{
  return attributes_for_write().lookup_for_write<math::Quaternion>(
      body_attribute_name(BodyAttribute::rotation));
}

VArray<float3> PhysicsGeometry::body_velocities() const
{
  return *attributes().lookup<float3>(body_attribute_name(BodyAttribute::velocity));
}

AttributeWriter<float3> PhysicsGeometry::body_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::velocity));
}

VArray<float3> PhysicsGeometry::body_angular_velocities() const
{
  return *attributes().lookup<float3>(body_attribute_name(BodyAttribute::angular_velocity));
}

AttributeWriter<float3> PhysicsGeometry::body_angular_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::angular_velocity));
}

VArray<int> PhysicsGeometry::body_activation_states() const
{
  return *attributes().lookup<int>(body_attribute_name(BodyAttribute::activation_state));
}

AttributeWriter<int> PhysicsGeometry::body_activation_states_for_write()
{
  return attributes_for_write().lookup_for_write<int>(
      body_attribute_name(BodyAttribute::activation_state));
}

VArray<float3> PhysicsGeometry::body_total_force() const
{
  return *attributes().lookup<float3>(body_attribute_name(BodyAttribute::total_force));
}

VArray<float3> PhysicsGeometry::body_total_torque() const
{
  return *attributes().lookup<float3>(body_attribute_name(BodyAttribute::total_torque));
}

VArray<bool> PhysicsGeometry::constraint_enabled() const
{
  return *attributes().lookup<bool>(
      constraint_attribute_name(ConstraintAttribute::constraint_enabled));
}

AttributeWriter<bool> PhysicsGeometry::constraint_enabled_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(
      constraint_attribute_name(ConstraintAttribute::constraint_enabled));
}

VArray<int> PhysicsGeometry::constraint_types() const
{
  return *attributes().lookup<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_type));
}

AttributeWriter<int> PhysicsGeometry::constraint_types_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(
      int(PhysicsGeometry::ConstraintType::Fixed), this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_type),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<int> PhysicsGeometry::constraint_body1() const
{
  return *attributes().lookup<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_body1));
}

AttributeWriter<int> PhysicsGeometry::constraint_body1_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(-1, this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_body1),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<int> PhysicsGeometry::constraint_body2() const
{
  return *attributes().lookup<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_body2));
}

AttributeWriter<int> PhysicsGeometry::constraint_body2_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(-1, this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_body2),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<float4x4> PhysicsGeometry::constraint_frame1() const
{
  return *attributes().lookup<float4x4>(
      constraint_attribute_name(ConstraintAttribute::constraint_frame1));
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame1_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      constraint_attribute_name(ConstraintAttribute::constraint_frame1));
}

VArray<float4x4> PhysicsGeometry::constraint_frame2() const
{
  return *attributes().lookup<float4x4>(
      constraint_attribute_name(ConstraintAttribute::constraint_frame2));
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame2_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      constraint_attribute_name(ConstraintAttribute::constraint_frame2));
}

VArray<float> PhysicsGeometry::constraint_applied_impulse() const
{
  return *attributes().lookup<float>(
      constraint_attribute_name(ConstraintAttribute::applied_impulse));
}

VArray<float> PhysicsGeometry::constraint_breaking_impulse_threshold_impulse() const
{
  return *attributes().lookup<float>(
      constraint_attribute_name(ConstraintAttribute::breaking_impulse_threshold));
}

AttributeWriter<float> PhysicsGeometry::constraint_breaking_impulse_threshold_for_write()
{
  return attributes_for_write().lookup_for_write<float>(
      constraint_attribute_name(ConstraintAttribute::breaking_impulse_threshold));
}

VArray<bool> PhysicsGeometry::constraint_disable_collision() const
{
  return *attributes().lookup<bool>(
      constraint_attribute_name(ConstraintAttribute::disable_collision));
}

AttributeWriter<bool> PhysicsGeometry::constraint_disable_collision_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<bool>(
      constraint_attribute_name(ConstraintAttribute::disable_collision), AttrDomain::Edge);
}

StringRef PhysicsGeometry::body_attribute_name(BodyAttribute attribute)
{
  return bke::physics_attribute_name(attribute);
}

StringRef PhysicsGeometry::constraint_attribute_name(ConstraintAttribute attribute)
{
  return bke::physics_attribute_name(attribute);
}

bool PhysicsGeometry::validate_world_data()
{
  return state_for_write().validate_world_data();
}

AttributeAccessor PhysicsGeometry::attributes() const
{
  return AttributeAccessor(&this->state(), bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(&this->state_for_write(),
                                  bke::get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsGeometry::dummy_attributes()
{
  return AttributeAccessor(nullptr, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::dummy_attributes_for_write()
{
  return MutableAttributeAccessor(nullptr, bke::get_physics_accessor_functions_ref());
}

/** \} */

bool validate_bullet_constraint_type(const PhysicsConstraintType type,
                                     const btTypedConstraint *constraint)
{
  using ConstraintType = PhysicsConstraintType;

  switch (type) {
    case ConstraintType::Fixed:
      return dynamic_cast<const btFixedConstraint *>(constraint) != nullptr;
    case ConstraintType::Point:
      return dynamic_cast<const btPoint2PointConstraint *>(constraint) != nullptr;
    case ConstraintType::Hinge:
      return dynamic_cast<const btHinge2Constraint *>(constraint) != nullptr;
    case ConstraintType::Slider:
      return dynamic_cast<const btSliderConstraint *>(constraint) != nullptr;
    case ConstraintType::ConeTwist:
      return dynamic_cast<const btConeTwistConstraint *>(constraint) != nullptr;
    case ConstraintType::SixDoF:
      return dynamic_cast<const btGeneric6DofConstraint *>(constraint) != nullptr;
    case ConstraintType::SixDoFSpring:
      return dynamic_cast<const btGeneric6DofSpringConstraint *>(constraint) != nullptr;
    case ConstraintType::SixDoFSpring2:
      return dynamic_cast<const btGeneric6DofSpring2Constraint *>(constraint) != nullptr;
    case ConstraintType::Contact:
      /* XXX Currently unsupported. */
      return false;
    case ConstraintType::Gear:
      return dynamic_cast<const btGearConstraint *>(constraint) != nullptr;
  }
  BLI_assert_unreachable();
  return false;
}

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::bke
