/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_vector.hh"

#include "BKE_attribute_filter.hh"

struct GreasePencil;

namespace blender::geometry {

GreasePencil *merge_layers(const GreasePencil &src_grease_pencil,
                           Span<Vector<int>> layers_to_merge,
                           const bke::AttributeFilter &attribute_filter = {});

}
