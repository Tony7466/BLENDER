/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_physics_geometry.hh"

#include "BLI_virtual_array.hh"
#include "GEO_physics_selection.hh"

namespace blender::geometry {

IndexMask body_selection_from_constraint(const Span<int> constraint_body1,
                                         const Span<int> constraint_body2,
                                         const IndexMask &constraint_mask,
                                         const int bodies_num,
                                         IndexMaskMemory &memory)
{
  const IndexRange body_range = IndexRange(bodies_num);
  Array<bool> array(bodies_num, false);
  constraint_mask.foreach_index_optimized<int>(GrainSize(4096), [&](const int i) {
    const int body1 = constraint_body1[i];
    const int body2 = constraint_body2[i];
    if (body_range.contains(body1)) {
      array[body1] = true;
    }
    if (body_range.contains(body2)) {
      array[body2] = true;
    }
  });
  return IndexMask::from_bools(array, memory);
}

IndexMask constraint_selection_from_body(Span<int> constraint_body1,
                                         Span<int> constraint_body2,
                                         Span<bool> body_selection,
                                         const int bodies_num,
                                         IndexMaskMemory &memory)
{
  const IndexRange body_range = IndexRange(bodies_num);
  return IndexMask::from_predicate(
      constraint_body1.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const int body1 = constraint_body1[i];
        const int body2 = constraint_body2[i];
        return (body_range.contains(body1) ? body_selection[body1] : false) &&
               (body_range.contains(body2) ? body_selection[body2] : false);
      });
}

IndexMask shape_selection_from_body(Span<int> body_shapes,
                                    const IndexMask &body_mask,
                                    const int shapes_num,
                                    IndexMaskMemory &memory)
{
  const IndexRange shape_range = IndexRange(shapes_num);
  Array<bool> array(shapes_num, false);
  body_mask.foreach_index_optimized<int>(GrainSize(4096), [&](const int i) {
    const int shape = body_shapes[i];
    if (shape_range.contains(shape)) {
      array[shape] = true;
    }
  });
  return IndexMask::from_bools(array, memory);
}

IndexMask body_selection_from_shape(Span<int> body_shapes,
                                    Span<bool> shape_selection,
                                    const int shapes_num,
                                    IndexMaskMemory &memory)
{
  const IndexRange shape_range = IndexRange(shapes_num);
  return IndexMask::from_predicate(
      body_shapes.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const int shape = body_shapes[i];
        return shape_range.contains(shape) ? shape_selection[shape] : false;
      });
}

std::optional<bke::PhysicsGeometry *> physics_copy_selection(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    const bke::AttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const VArraySpan<int> src_body_shapes = src_physics.body_shapes();
  const VArraySpan<int> src_body1 = src_physics.constraint_body1();
  const VArraySpan<int> src_body2 = src_physics.constraint_body2();

  if (selection.is_empty()) {
    return std::nullopt;
  }
  if (const std::optional<bool> single = selection.get_if_single()) {
    return *single ? std::nullopt : std::make_optional<bke::PhysicsGeometry *>(nullptr);
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  IndexMask body_mask;
  IndexMask constraint_mask;
  IndexMask shape_mask;
  switch (selection_domain) {
    case bke::AttrDomain::Point: {
      const VArraySpan<bool> span(selection);
      body_mask = IndexMask::from_bools(span, memory.local());
      threading::parallel_invoke(
          src_physics.bodies_num() > 1024,
          [&]() {
            constraint_mask = constraint_selection_from_body(
                src_body1, src_body2, span, src_physics.bodies_num(), memory.local());
          },
          [&]() {
            shape_mask = shape_selection_from_body(
                src_body_shapes, body_mask, src_physics.shapes_num(), memory.local());
          });
      break;
    }
    case bke::AttrDomain::Edge: {
      const VArraySpan<bool> span(selection);
      constraint_mask = IndexMask::from_bools(span, memory.local());
      body_mask = body_selection_from_constraint(
          src_body1, src_body2, constraint_mask, src_physics.bodies_num(), memory.local());
      shape_mask = shape_selection_from_body(
          src_body_shapes, body_mask, src_physics.shapes_num(), memory.local());
      break;
    }
    case bke::AttrDomain::Instance: {
      const VArraySpan<bool> span(selection);
      shape_mask = IndexMask::from_bools(span, memory.local());
      body_mask = body_selection_from_shape(
          src_body_shapes, span, src_physics.shapes_num(), memory.local());
      constraint_mask = constraint_selection_from_body(
          src_body1, src_body2, span, src_physics.bodies_num(), memory.local());
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  if (body_mask.is_empty()) {
    return nullptr;
  }
  const bool same_bodies = body_mask.size() == src_physics.bodies_num();
  const bool same_constraints = constraint_mask.size() == src_physics.constraints_num();
  if (same_bodies && same_constraints) {
    return std::nullopt;
  }

  bke::PhysicsGeometry *dst_physics = new bke::PhysicsGeometry(
      body_mask.size(), constraint_mask.size(), shape_mask.size());

  dst_physics->state_for_write().move_or_copy_selection(
      src_physics.state(), body_mask, constraint_mask, propagation_info);

  return dst_physics;
}

std::optional<bke::PhysicsGeometry *> physics_copy_selection_keep_bodies(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    const bke::AttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const VArraySpan<int> src_body_shapes = src_physics.body_shapes();
  const VArraySpan<int> src_body1 = src_physics.constraint_body1();
  const VArraySpan<int> src_body2 = src_physics.constraint_body2();

  if (selection.is_empty()) {
    return std::nullopt;
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  const IndexRange body_mask = src_physics.bodies_range();
  IndexMask constraint_mask;
  IndexMask shape_mask;
  switch (selection_domain) {
    case bke::AttrDomain::Point: {
      const VArraySpan<bool> span(selection);
      constraint_mask = constraint_selection_from_body(
          src_body1, src_body2, span, src_physics.bodies_num(), memory.local());
      shape_mask = shape_selection_from_body(
          src_body_shapes, body_mask, src_physics.shapes_num(), memory.local());
      break;
    }
    case bke::AttrDomain::Edge: {
      const VArraySpan<bool> span(selection);
      constraint_mask = IndexMask::from_bools(span, memory.local());
      shape_mask = src_physics.shapes_range();
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  const bool same_constraints = constraint_mask.size() == src_physics.constraints_num();
  if (same_constraints) {
    return std::nullopt;
  }

  bke::PhysicsGeometry *dst_physics = new bke::PhysicsGeometry(
      src_physics.bodies_num(), constraint_mask.size(), src_physics.shapes_num());

  dst_physics->state_for_write().move_or_copy_selection(
      src_physics.state(), body_mask, constraint_mask, propagation_info);

  return dst_physics;
}

}  // namespace blender::geometry
