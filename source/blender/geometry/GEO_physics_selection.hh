/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_span.hh"
#include "BLI_virtual_array_fwd.hh"

#include <cstdint>
#include <optional>

namespace blender {
namespace fn {
template<typename T> class Field;
}
namespace bke {
enum class AttrDomain : int8_t;
class AnonymousAttributePropagationInfo;
class PhysicsGeometry;
}  // namespace bke
}  // namespace blender

namespace blender::geometry {

/** A body is selected if it's used by a selected constraint. */
IndexMask body_selection_from_constraint(Span<int> constraint_body1,
                                         Span<int> constraint_body2,
                                         const IndexMask &constraint_mask,
                                         int bodies_num,
                                         IndexMaskMemory &memory);

/** A constraint is selected if both of its bodies are selected. */
IndexMask constraint_selection_from_body(Span<int> constraint_body1,
                                         Span<int> constraint_body2,
                                         Span<bool> body_selection,
                                         int bodies_num,
                                         IndexMaskMemory &memory);

IndexMask shape_selection_from_body(Span<int> body_shapes,
                                    const IndexMask &body_mask,
                                    int shapes_num,
                                    IndexMaskMemory &memory);

IndexMask body_selection_from_shape(Span<int> body_shapes,
                                    Span<bool> shape_selection,
                                    int shapes_num,
                                    IndexMaskMemory &memory);

std::optional<bke::PhysicsGeometry *> physics_copy_selection(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    bke::AttrDomain selection_domain,
    const bke::AttributeFilter &attribute_filter);

std::optional<bke::PhysicsGeometry *> physics_copy_selection_keep_bodies(
    const bke::PhysicsGeometry &src_physics,
    const VArray<bool> &selection,
    bke::AttrDomain selection_domain,
    const bke::AttributeFilter &attribute_filter);

}  // namespace blender::geometry
