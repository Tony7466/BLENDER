/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <mutex>
#include <unordered_set>

#include "BLI_color.hh"
#include "BLI_cpp_type.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_math_base.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

#include "BKE_volume_types.hh"

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
namespace detail {

template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_ReinterpretCast {
  static GridValueType single_value_to_grid(const FieldValueType &value)
  {
    return reinterpret_cast<const GridValueType &>(value);
  }

  static FieldValueType single_value_to_attribute(const GridValueType &value)
  {
    return reinterpret_cast<const FieldValueType &>(value);
  }

  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<FieldValueType *>(values.begin()), values.size()};
  }
};

template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_CopyConstruct {
  static GridValueType single_value_to_grid(const FieldValueType &value)
  {
    return GridValueType(value);
  }

  static FieldValueType single_value_to_attribute(const GridValueType &value)
  {
    return FieldValueType(value);
  }

  // XXX can't do this, will only work as a DerivedSpan varray.
  //  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
  //      const MutableSpan<LeafBufferValueType> values)
  //  {
  //    return {reinterpret_cast<FieldValueType *>(values.begin()), values.size()};
  //  }
};

template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_Vector3CopyConstruct {
  using GridBaseType = typename GridValueType::ValueType;
  using FieldBaseType = typename FieldValueType::base_type;

  static GridValueType single_value_to_grid(const FieldValueType &value)
  {
    return GridValueType(GridBaseType(value[0]), GridBaseType(value[1]), GridBaseType(value[2]));
  }

  static FieldValueType single_value_to_attribute(const GridValueType &value)
  {
    return FieldValueType(
        FieldBaseType(value[0]), FieldBaseType(value[1]), FieldBaseType(value[2]));
  }

  // XXX can't do this, will only work as a DerivedSpan varray.
  //  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
  //      const MutableSpan<LeafBufferValueType> values)
  //  {
  //    return {reinterpret_cast<FieldValueType *>(values.begin()), values.size()};
  //  }
};

/* Return default values without trying to convert. */
template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_Dummy {
  static GridValueType single_value_to_grid(const FieldValueType & /*value*/)
  {
    return GridValueType();
  }

  static FieldValueType single_value_to_attribute(const GridValueType & /*value*/)
  {
    return FieldValueType();
  }

  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> /*values*/)
  {
    return {};
  }
};

}  // namespace detail

/* Default implementation, only used when grid and attribute types are exactly the same. */
template<typename T> struct GridConverter : public detail::GridConverter_ReinterpretCast<T, T, T> {
  using FieldValueType = T;
};

/* TODO Some base types (double, int3, ...) don't have a CPPType registered yet, and even then
 * there are other layers like attributes, customdata, node sockets, which need to support these
 * types. In the meantime the converters will use smaller base types for fields and attributes. */

template<>
struct GridConverter<float3>
    : public detail::GridConverter_ReinterpretCast<float3, openvdb::Vec3f, openvdb::Vec3f> {
  using FieldValueType = float3;
};

template<>
struct GridConverter<math::Quaternion>
    : public detail::
          GridConverter_ReinterpretCast<math::Quaternion, openvdb::Vec4f, openvdb::Vec4f> {
  using FieldValueType = math::Quaternion;
};

template<>
struct GridConverter<blender::ColorGeometry4f>
    : public detail::
          GridConverter_ReinterpretCast<blender::ColorGeometry4f, openvdb::Vec4f, openvdb::Vec4f> {
  using FieldValueType = blender::ColorGeometry4f;
};

template<>
struct GridConverter<std::string>
    : public detail::GridConverter_Dummy<std::string, openvdb::ValueMask, uint64_t> {
  using FieldValueType = std::string;
};
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
  VolumeGrid(bool is_loaded, int simplify_level = 0);
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
  GridBasePtr grid_for_write() const;

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
