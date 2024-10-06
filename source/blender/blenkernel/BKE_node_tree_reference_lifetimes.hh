
/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_group_vector.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"

#include "NOD_node_declaration.hh"

namespace blender::bke::node_tree_reference_lifetimes {

struct ReferenceSetInfo {
  enum class Type {
    /**
     * Corresponds to geometry outputs that may contain attributes that are propagated from a group
     * input. In such cases, the caller may provide a set of attributes that should be propagated.
     */
    GroupOutputData,
    /** Field inputs may require attributes. */
    GroupInputReferenceSet,
    /** Local fields may require attributes, e.g. the output of the Capture Attribute node.  */
    LocalReferenceSet,
  };

  Type type;
  union {
    /** Used for group interface sockets. */
    int index;
    /** Used for local sockets. */
    const bNodeSocket *socket;
  };

  /** Sockets that may contain the referenced data. */
  Vector<const bNodeSocket *> potential_data_origins;

  ReferenceSetInfo(Type type, const int index) : type(type), index(index)
  {
    BLI_assert(ELEM(type, Type::GroupInputReferenceSet, Type::GroupOutputData));
  }

  ReferenceSetInfo(Type type, const bNodeSocket *socket) : type(type), socket(socket)
  {
    BLI_assert(ELEM(type, Type::LocalReferenceSet));
  }

  friend std::ostream &operator<<(std::ostream &stream, const ReferenceSetInfo &source);
};

struct ReferenceLifetimesInfo {
  Vector<ReferenceSetInfo> reference_sets;
  BitGroupVector<> required_data_by_socket;
  nodes::aal::RelationsInNode tree_relations;
};

bool analyse_reference_lifetimes(bNodeTree &tree);

}  // namespace blender::bke::node_tree_reference_lifetimes
