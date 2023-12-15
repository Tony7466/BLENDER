/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <mutex>

#include "BLI_color.hh"
#include "BLI_cpp_type.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_math_base.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

#include "BKE_volume_enums.hh"

#ifdef WITH_OPENVDB
#  include "openvdb_fwd.hh"
#endif

struct VolumeFileCacheEntry;

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Grid Type Converter
 *
 * Converter between Blender types and OpenVDB types.
 * \{ */

#ifdef WITH_OPENVDB
namespace grids {

template<typename T> struct Converter {
  using AttributeValueType = T;
  using GridValueType = void;
};
template<> struct Converter<bool> {
  using AttributeValueType = bool;
  using GridValueType = bool;
  static GridValueType to_openvdb(const AttributeValueType &value)
  {
    return value;
  }
  static AttributeValueType to_blender(const GridValueType &value)
  {
    return value;
  }
};
template<> struct Converter<int> {
  using AttributeValueType = int;
  using GridValueType = int;
  static GridValueType to_openvdb(const AttributeValueType &value)
  {
    return value;
  }
  static AttributeValueType to_blender(const GridValueType &value)
  {
    return value;
  }
};
template<> struct Converter<float> {
  using AttributeValueType = float;
  using GridValueType = float;
  static GridValueType to_openvdb(const AttributeValueType &value)
  {
    return value;
  }
  static AttributeValueType to_blender(const GridValueType &value)
  {
    return value;
  }
};
template<> struct Converter<float3> {
  using AttributeValueType = float3;
  using GridValueType = openvdb::Vec3f;
  static GridValueType to_openvdb(const AttributeValueType &value)
  {
    return openvdb::Vec3f(*value);
  }
  static AttributeValueType to_blender(const GridValueType &value)
  {
    return float3(value.asV());
  }
};
template<> struct Converter<math::Quaternion> {
  using AttributeValueType = math::Quaternion;
  using GridValueType = openvdb::Vec4f;
  static GridValueType to_openvdb(const AttributeValueType &value)
  {
    return openvdb::Vec4f(value.w, value.x, value.y, value.z);
  }
  static AttributeValueType to_blender(const GridValueType &value)
  {
    return math::Quaternion(value.asV());
  }
};

}  // namespace grids
#endif /* WITH_OPENVDB */

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shared Grid Data
 *
 * Wraps around an OpenVDB grid to manage it with implicit sharing.
 * \{ */

struct VolumeGrid : public ImplicitSharingMixin {
#ifdef WITH_OPENVDB
 public:
  using GridBase = openvdb::GridBase;
  using GridBasePtr = std::shared_ptr<GridBase>;
  using GridBaseConstPtr = std::shared_ptr<const GridBase>;

 protected:
  mutable GridBasePtr grid_;
  /* Tree data has been loaded from file. */
  mutable VolumeTreeSource tree_source_;
  /* Mutex for on-demand reading of tree. */
  mutable std::mutex mutex_;

 public:
  VolumeGrid(const GridBasePtr &grid, VolumeTreeSource tree_source);
  ~VolumeGrid();

  VolumeGrid *copy() const;

  const char *name() const;

  GridBaseConstPtr grid() const;
  GridBasePtr grid_for_write();

  /* Source of the tree data stored in the grid. */
  VolumeTreeSource tree_source() const;
  /* Make sure the tree data has been loaded if the grid has a file source. */
  bool ensure_tree_loaded(
      StringRef filepath, FunctionRef<void(StringRef)> error_fn = [](StringRef) {}) const;
  /* Unload the tree data but keep metadata. */
  void unload_tree() const;

 protected:
  GridBasePtr main_grid() const;
  void clear_cache_entry();
#endif

  void delete_self() override;
  void delete_data_only() override;
};

/** \} */

}  // namespace blender::bke

/* Root namespace alias for older code. */
using VolumeGrid = blender::bke::VolumeGrid;
