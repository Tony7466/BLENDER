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
  Array<bool> array(bodies_num, false);
  constraint_mask.foreach_index_optimized<int>(GrainSize(4096), [&](const int i) {
    array[constraint_body1[i]] = true;
    array[constraint_body2[i]] = true;
  });
  return IndexMask::from_bools(array, memory);
}

IndexMask constraint_selection_from_body(Span<int> constraint_body1,
                                         Span<int> constraint_body2,
                                         Span<bool> body_selection,
                                         IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(
      constraint_body1.index_range(), GrainSize(1024), memory, [&](const int64_t i) {
        const int body1 = constraint_body1[i];
        const int body2 = constraint_body2[i];
        return body_selection[body1] && body_selection[body2];
      });
}

std::optional<bke::PhysicsGeometry *> physics_copy_selection(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    const bke::AttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
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
  switch (selection_domain) {
    case bke::AttrDomain::Point: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_physics.bodies_num() > 1024,
          [&]() { body_mask = IndexMask::from_bools(span, memory.local()); },
          [&]() {
            constraint_mask = constraint_selection_from_body(
                src_body1, src_body2, span, memory.local());
          });
      break;
    }
    case bke::AttrDomain::Edge: {
      const VArraySpan<bool> span(selection);
      constraint_mask = IndexMask::from_bools(span, memory.local());
      body_mask = body_selection_from_constraint(
          src_body1, src_body2, constraint_mask, src_physics.bodies_num(), memory.local());
      break;
    }
    case bke::AttrDomain::Face: {
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

  bke::PhysicsGeometry *dst_physics = new bke::PhysicsGeometry(body_mask.size(),
                                                               constraint_mask.size());

  dst_physics->move_or_copy_selection(
      src_physics, true, body_mask, constraint_mask, propagation_info);

  return dst_physics;
}

std::optional<bke::PhysicsGeometry *> physics_copy_selection_keep_bodies(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    const bke::AttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const VArraySpan<int> src_body1 = src_physics.constraint_body1();
  const VArraySpan<int> src_body2 = src_physics.constraint_body2();

  if (selection.is_empty()) {
    return std::nullopt;
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  IndexMask constraint_mask;
  switch (selection_domain) {
    case bke::AttrDomain::Point: {
      const VArraySpan<bool> span(selection);
      constraint_mask = constraint_selection_from_body(src_body1, src_body2, span, memory.local());
      break;
    }
    case bke::AttrDomain::Edge: {
      const VArraySpan<bool> span(selection);
      constraint_mask = IndexMask::from_bools(span, memory.local());
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

  bke::PhysicsGeometry *dst_physics = new bke::PhysicsGeometry(src_physics.bodies_num(),
                                                               constraint_mask.size());

  dst_physics->move_or_copy_selection(
      src_physics, true, src_physics.bodies_range(), constraint_mask, propagation_info);

  return dst_physics;
}

}  // namespace blender::geometry
