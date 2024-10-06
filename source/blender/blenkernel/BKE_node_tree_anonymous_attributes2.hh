
/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_group_vector.hh"
#include "BLI_linear_allocator_chunked_list.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"

namespace blender::bke::anonymous_attribute_inferencing2 {

struct AttributeSetSource {
  enum class Type {
    /** Geometry outputs may require attributes. */
    GroupOutput,
    /** Field inputs may require attributes. */
    GroupInput,
    /** Local fields may require attributes.  */
    Local,
  };

  Type type;
  union {
    /** Used for group interface sockets. */
    int index;
    /** Used for local sockets. */
    const bNodeSocket *socket;
  };

  linear_allocator::ChunkedList<const bNodeSocket *> potential_data_origins;

  AttributeSetSource(Type type, const int index) : type(type), index(index)
  {
    BLI_assert(ELEM(type, Type::GroupInput, Type::GroupOutput));
  }

  AttributeSetSource(Type type, const bNodeSocket *socket) : type(type), socket(socket)
  {
    BLI_assert(ELEM(type, Type::Local));
  }

  friend std::ostream &operator<<(std::ostream &stream, const AttributeSetSource &source);
};

struct AnonymousAttributesInfo {
  Vector<AttributeSetSource> attribute_sets;

  BoundedBitSpan required_attribute_sets(const bNodeSocket &socket) const;
};

void analyse(const bNodeTree &tree);

}  // namespace blender::bke::anonymous_attribute_inferencing2
