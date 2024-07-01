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
  index_mask::build_reverse_map<int>(bodies_mask, map);
  constraints_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
    dst_constraint_types[dst_i] = map[src_constraint_types[src_i]];
    dst_constraint_body1[dst_i] = map[src_constraint_body1[src_i]];
    dst_constraint_body2[dst_i] = map[src_constraint_body2[src_i]];
  });
}

std::optional<bke::PhysicsGeometry *> physics_copy_selection(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    const bke::AttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const VArraySpan<int> src_types = src_physics.constraint_types();
  const VArraySpan<int> src_body1 = src_physics.constraint_body1();
  const VArraySpan<int> src_body2 = src_physics.constraint_body2();
  const bke::AttributeAccessor src_attributes = src_physics.attributes();

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
      src_physics, true, body_mask, constraint_mask, 0, 0, propagation_info);

  return dst_physics;
}

std::optional<bke::PhysicsGeometry *> physics_copy_selection_keep_bodies(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    const bke::AttrDomain selection_domain,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const VArraySpan<int> src_types = src_physics.constraint_types();
  const VArraySpan<int> src_body1 = src_physics.constraint_body1();
  const VArraySpan<int> src_body2 = src_physics.constraint_body2();
  const bke::AttributeAccessor src_attributes = src_physics.attributes();

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
  // BKE_physics_copy_parameters_for_eval(dst_physics, &src_physics);
  bke::MutableAttributeAccessor dst_attributes = dst_physics->attributes_for_write();
  Array<int> dst_types(dst_physics->constraints_num());
  Array<int> dst_body1(dst_physics->constraints_num());
  Array<int> dst_body2(dst_physics->constraints_num());

  remap_bodies(src_physics.bodies_num(),
               src_physics.bodies_range(),
               constraint_mask,
               src_types,
               src_body1,
               src_body2,
               dst_types,
               dst_body1,
               dst_body2);

  dst_physics->create_constraints(dst_physics->constraints_range(),
                                  VArray<int>::ForSpan(dst_types),
                                  VArray<int>::ForSpan(dst_body1),
                                  VArray<int>::ForSpan(dst_body2));

  bke::copy_attributes(
      src_attributes, bke::AttrDomain::Point, propagation_info, {}, dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         propagation_info,
                         {},
                         constraint_mask,
                         dst_attributes);

  return dst_physics;
}

}  // namespace blender::geometry
