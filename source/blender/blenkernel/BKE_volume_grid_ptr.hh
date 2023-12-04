/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BLI_implicit_sharing_ptr.hh"

#include "BKE_volume_grid.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke {

template<typename T> struct VolumeGridPtr;

/* -------------------------------------------------------------------- */
/** \name Common Grid Wrapper
 *
 * Base class for #GVolumeGridPtr and #VolumeGridPtr.
 * \{ */

struct VolumeGridPtrCommon : ImplicitSharingPtr<VolumeGrid> {
  VolumeGridPtrCommon() = default;
  VolumeGridPtrCommon(const VolumeGridPtrCommon &other) = default;
  /* Enable implicit conversion from nullptr. */
  VolumeGridPtrCommon(std::nullptr_t) : ImplicitSharingPtr<VolumeGrid>(nullptr) {}
  using ImplicitSharingPtr<VolumeGrid>::ImplicitSharingPtr;
  ~VolumeGridPtrCommon() = default;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Generic Volume Grid
 *
 * Owning pointer to a #VolumeGrid.
 * \{ */

struct GVolumeGridPtr : public VolumeGridPtrCommon {
#ifdef WITH_OPENVDB
  using GridType = openvdb::GridBase;
  using GridPtr = std::shared_ptr<GridType>;
  using GridConstPtr = std::shared_ptr<const GridType>;
#endif

  template<typename T>
  GVolumeGridPtr(const VolumeGridPtr<T> &typed_grid) : VolumeGridPtrCommon(typed_grid)
  {
  }
  using VolumeGridPtrCommon::VolumeGridPtrCommon;

  template<typename T> VolumeGridPtr<T> typed() const
  {
    // TODO type check
    //    using GridType = typename VolumeGridPtr<T>::GridType;
    //    BLI_assert(openvdb::GridBase::grid<GridType>(data->grid));
    /* Points to same data, increment user count. */
    if (*this) {
      this->get()->add_user();
    }
    return VolumeGridPtr<T>(*this);
  }

#ifdef WITH_OPENVDB
  GridConstPtr grid() const;
  GridPtr grid_for_write();
#endif
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Volume Grid
 *
 * Owning pointer to a #VolumeGrid of a known type.
 * \{ */

namespace detail {

#ifdef WITH_OPENVDB
/**
 * Tree types for commonly used values.
 */
template<typename T> struct VolumeGridTraits;

template<> struct VolumeGridTraits<bool> {
  using TreeType = openvdb::BoolTree;
};
template<> struct VolumeGridTraits<float> {
  using TreeType = openvdb::FloatTree;
};
template<> struct VolumeGridTraits<float2> {
  using TreeType = openvdb::Vec2STree;
};
template<> struct VolumeGridTraits<float3> {
  using TreeType = openvdb::Vec3STree;
};
template<> struct VolumeGridTraits<double> {
  using TreeType = openvdb::DoubleTree;
};
template<> struct VolumeGridTraits<double3> {
  using TreeType = openvdb::Vec3DTree;
};
template<> struct VolumeGridTraits<int8_t> {
  using TreeType = openvdb::Int8Grid;
};
template<> struct VolumeGridTraits<int32_t> {
  using TreeType = openvdb::Int32Tree;
};
template<> struct VolumeGridTraits<int64_t> {
  using TreeType = openvdb::Int64Tree;
};
template<> struct VolumeGridTraits<int2> {
  using TreeType = openvdb::Vec2ITree;
};
template<> struct VolumeGridTraits<int3> {
  using TreeType = openvdb::Vec3ITree;
};
template<> struct VolumeGridTraits<uint32_t> {
  using TreeType = openvdb::UInt32Tree;
};
template<> struct VolumeGridTraits<ColorGeometry4f> {
  using TreeType = openvdb::Vec4fTree;
};
template<> struct VolumeGridTraits<blender::ColorGeometry4b> {
  using TreeType = openvdb::UInt32Tree;
};
template<> struct VolumeGridTraits<math::Quaternion> {
  using TreeType = openvdb::Vec4fTree;
};
/* Stub class for string attributes, not supported. */
template<> struct VolumeGridTraits<std::string> {
  using TreeType = void;
};

template<typename T> using VolumeGridType = openvdb::Grid<typename VolumeGridTraits<T>::TreeType>;
#else  /* WITH_OPENVDB */
template<typename T> struct VolumeGridType {
};
#endif /* WITH_OPENVDB */

}  // namespace detail

#ifdef WITH_OPENVDB
template<typename T> struct VolumeGridPtr : public VolumeGridPtrCommon {
  using GridType = detail::VolumeGridType<T>;
  using GridPtr = std::shared_ptr<GridType>;
  using GridConstPtr = std::shared_ptr<const GridType>;

  VolumeGridPtr(const GVolumeGridPtr &other) : VolumeGridPtrCommon(other.get()) {}
  using VolumeGridPtrCommon::VolumeGridPtrCommon;

  GridConstPtr grid() const
  {
    return openvdb::GridBase::grid<GridType>(this->get()->grid());
  }

  GridPtr grid_for_write() const
  {
    const VolumeGrid *data = this->get();
    if (data->is_mutable()) {
      data->tag_ensured_mutable();
      return openvdb::GridBase::grid<GridType>(const_cast<VolumeGrid *>(data)->grid_for_write());
    }
    return openvdb::GridBase::grid<GridType>(data->grid()->deepCopyGrid());
  }
};
#else
template<typename T> struct VolumeGridPtr : public VolumeGridPtrCommon {
};
#endif

/** \} */

/* -------------------------------------------------------------------- */
/** \name Implicit Sharing Pointer Constructors
 * \{ */

template<typename... Args> inline GVolumeGridPtr make_volume_grid_ptr(Args &&...args)
{
  return GVolumeGridPtr(new VolumeGrid(std::forward<Args>(args)...));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Grid Utility Functions
 * \{ */

namespace grid_utils {

template<typename T> std::optional<T> get_background_value(const VolumeGridPtr<T> &grid)
{
#ifdef WITH_OPENVDB
  return grids::Converter<T>::to_blender(grid.grid()->background());
#else
  return std::nullopt;
#endif /* WITH_OPENVDB */
}

template<typename T> VolumeGridPtr<T> make_empty_grid(const T background_value)
{
#ifdef WITH_OPENVDB
  using GridType = typename VolumeGridPtr<T>::GridType;

  std::shared_ptr<GridType> grid;
  if constexpr (std::is_same_v<T, std::string>) {
    grid = nullptr;
  }
  else {
    grid = GridType::create(grids::Converter<T>::to_openvdb(background_value));
  }
  return make_volume_grid_ptr(grid).template typed<T>();
#else
  return nullptr;
#endif /* WITH_OPENVDB */
}

}  // namespace grid_utils

/** \} */

}  // namespace blender::bke
