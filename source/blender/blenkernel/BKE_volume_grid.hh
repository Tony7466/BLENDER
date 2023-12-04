/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

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

template<typename T> struct AttributeConverter {
  using Type = T;
  using GridValueType = void;
};
template<> struct AttributeConverter<float> {
  using Type = float;
  using GridValueType = float;
  static GridValueType convert(const Type &value)
  {
    return value;
  }
};
template<> struct AttributeConverter<float3> {
  using Type = float3;
  using GridValueType = openvdb::Vec3f;
  static GridValueType convert(const Type &value)
  {
    return openvdb::Vec3f(*value);
  }
};

template<typename T> struct GridConverter {
  using Type = T;
  using AttributeValueType = void;
};
template<> struct GridConverter<bool> {
  using Type = bool;
  using GridValueType = bool;
  static Type convert(const GridValueType &value)
  {
    return value;
  }
};
template<> struct GridConverter<int> {
  using Type = int;
  using GridValueType = int;
  static Type convert(const GridValueType &value)
  {
    return value;
  }
};
template<> struct GridConverter<float> {
  using Type = float;
  using GridValueType = float;
  static Type convert(const GridValueType &value)
  {
    return value;
  }
};
template<> struct GridConverter<float3> {
  using Type = float3;
  using GridValueType = openvdb::Vec3f;
  static Type convert(const GridValueType &value)
  {
    return float3(value.asV());
  }
};
template<> struct GridConverter<math::Quaternion> {
  using Type = math::Quaternion;
  using GridValueType = openvdb::Vec4f;
  static Type convert(const GridValueType &value)
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
  /* OpenVDB grid used when there is no file cache entry. */
  GridBasePtr local_grid_;
  /* File cache entry when grid comes directly from a file and may be shared
   * with other volume datablocks. */
  VolumeFileCacheEntry *entry_;
  /* If this volume grid is in the global file cache, we can reference a simplified version of it,
   * instead of the original high resolution grid. */
  int simplify_level_;
  /**
   * Indicates if the tree has been loaded for this grid. Note that vdb.tree()
   * may actually be loaded by another user while this is false. But only after
   * calling load() and is_loaded changes to true is it safe to access.
   *
   * `const` write access to this must be protected by `entry->mutex`.
   */
  mutable bool is_loaded_;

 public:
  VolumeGrid(const GridBasePtr &grid);
  VolumeGrid(const VolumeFileCacheEntry &template_entry, int simplify_level = 0);
  VolumeGrid(const char *template_file_path,
             const GridBasePtr &template_grid,
             int simplify_level = 0);
  VolumeGrid(const VolumeGrid &other);
  ~VolumeGrid();

  const char *name() const;

  const char *error_message() const;

  bool grid_is_loaded() const;

  void load(const char *volume_name, const char *filepath) const;
  void unload(const char *volume_name) const;

  void set_simplify_level(int simplify_level);

  void clear_reference(const char *volume_name);
  void duplicate_reference(const char *volume_name, const char *filepath);

  GridBaseConstPtr grid() const;
  GridBasePtr grid_for_write();

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
