/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"
#include "BLI_map.hh"
#include "BLI_sub_frame.hh"

#include "BKE_geometry_set.hh"

namespace blender::bke {

class BakeItem {
 public:
  virtual ~BakeItem() = default;
};

class GeometryBakeItem : public BakeItem {
 public:
  GeometryBakeItem(GeometrySet geometry);
  GeometrySet geometry;
};

/**
 * References a field input/output that becomes an attribute as part of the simulation state.
 * The attribute is actually stored in a #GeometryBakeItem, so this just references
 * the attribute's name.
 */
class AttributeBakeItem : public BakeItem {
 private:
  std::string name_;

 public:
  AttributeBakeItem(std::string name) : name_(std::move(name)) {}

  StringRefNull name() const
  {
    return name_;
  }
};

/** Storage for a single value of a trivial type like `float`, `int`, etc. */
class PrimitiveBakeItem : public BakeItem {
 private:
  const CPPType &type_;
  void *value_;

 public:
  PrimitiveBakeItem(const CPPType &type, const void *value);
  ~PrimitiveBakeItem();

  const void *value() const
  {
    return value_;
  }

  const CPPType &type() const
  {
    return type_;
  }
};

class StringBakeItem : public BakeItem {
 private:
  std::string value_;

 public:
  StringBakeItem(std::string value);

  StringRefNull value() const
  {
    return value_;
  }
};

class BakeNodeState {
 public:
  std::optional<GeometrySet> geometry;
};

class BakeNodeStateAtFrame {
 public:
  SubFrame frame;
  std::unique_ptr<BakeNodeState> state;
};

class BakeNodeStorage {
 public:
  Vector<BakeNodeStateAtFrame> states;
  std::unique_ptr<BakeNodeState> current_bake_state;
};

class GeometryNodesModifierBakes {
 public:
  Map<int32_t, std::unique_ptr<BakeNodeStorage>> storage_by_id;
  Set<int32_t> requested_bake_ids;

  BakeNodeStorage *get_storage(const int32_t nested_node_id)
  {
    std::unique_ptr<BakeNodeStorage> *storage = this->storage_by_id.lookup_ptr(nested_node_id);
    if (storage) {
      return storage->get();
    }
    return nullptr;
  }
};

}  // namespace blender::bke
