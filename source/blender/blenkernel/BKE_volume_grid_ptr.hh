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
 * Base class for both generic and typed grid wrapper classes.
 * \{ */

struct VolumeGridPtrCommon {
#ifdef WITH_OPENVDB
  using GridPtr = std::shared_ptr<openvdb::GridBase>;
  using GridConstPtr = std::shared_ptr<const openvdb::GridBase>;
#endif

  virtual ~VolumeGridPtrCommon();
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Generic Volume Grid
 *
 * Wrapper around a generic OpenVDB grid.
 * Grids loaded from OpenVDB files are always stored in the global cache.
 * Procedurally generated grids are not.
 * \{ */

struct GVolumeGridPtr : public VolumeGridPtrCommon {
  using SharedDataPtr = ImplicitSharingPtr<VolumeGrid>;
#ifdef WITH_OPENVDB
  using GridType = openvdb::GridBase;
  using GridPtr = std::shared_ptr<GridType>;
  using GridConstPtr = std::shared_ptr<const GridType>;
#endif

  /* OpenVDB grid. */
  SharedDataPtr data;

  GVolumeGridPtr() = default;
  GVolumeGridPtr(const GVolumeGridPtr &) = default;
  GVolumeGridPtr(const SharedDataPtr data) : data(data) {}
  template<typename T> GVolumeGridPtr(const VolumeGridPtr<T> &typed_grid) : data(typed_grid.data)
  {
  }
  GVolumeGridPtr(std::nullptr_t) : data(nullptr) {}

  inline const VolumeGrid *get() const
  {
    return data.get();
  }

  template<typename T> VolumeGridPtr<T> typed() const
  {
    // TODO type check
    //    using GridType = typename VolumeGridPtr<T>::GridType;

    if (!*this) {
      return {};
    }
    //    BLI_assert(openvdb::GridBase::grid<GridType>(data->grid));
    return VolumeGridPtr<T>(data);
  }

#ifdef WITH_OPENVDB
  GridConstPtr grid() const;
  GridPtr grid_for_write();
#endif

  const VolumeGrid *operator->() const
  {
    BLI_assert(data);
    return data.get();
  }
  VolumeGrid *operator->()
  {
    BLI_assert(data && data->is_mutable());
    return const_cast<VolumeGrid *>(data.get());
  }

  const VolumeGrid &operator*() const
  {
    BLI_assert(data);
    return *data;
  }
  VolumeGrid &operator*()
  {
    BLI_assert(data && data->is_mutable());
    return const_cast<VolumeGrid &>(*data);
  }

  bool operator==(const GVolumeGridPtr &other) const
  {
    return data == other.data;
  }
  bool operator!=(const GVolumeGridPtr &other) const
  {
    return data != other.data;
  }
  bool operator==(const std::nullptr_t &) const
  {
    return data == nullptr;
  }
  bool operator!=(const std::nullptr_t &) const
  {
    return data != nullptr;
  }
  template<typename U> bool operator==(const VolumeGridPtr<U> &other) const
  {
    return data == other.data;
  }
  template<typename U> bool operator!=(const VolumeGridPtr<U> &other) const
  {
    return data != other.data;
  }
  operator bool() const;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Volume Grid
 *
 * Wrapper around OpenVDB grids using a Blender type parameter.
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
  using TreeType = openvdb::MaskTree;
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
  using SharedDataPtr = ImplicitSharingPtr<VolumeGrid>;

  /* OpenVDB grid. */
  SharedDataPtr data;

  VolumeGridPtr() = default;
  VolumeGridPtr(const VolumeGridPtr<T> &) = default;
  VolumeGridPtr(const SharedDataPtr &data) : data(data) {}

  GridConstPtr grid() const
  {
    return data ? openvdb::GridBase::grid<GridType>(data->grid()) : nullptr;
  }

  GridPtr grid_for_write() const
  {
    if (!data) {
      return nullptr;
    }
    if (data->is_mutable()) {
      return openvdb::GridBase::grid<GridType>(const_cast<VolumeGrid &>(*data).grid_for_write());
    }
    return openvdb::GridBase::grid<GridType>(data->grid())->deepCopy();
  }

  bool operator==(const GVolumeGridPtr &other) const
  {
    return data == other.data;
  }
  bool operator!=(const GVolumeGridPtr &other) const
  {
    return data != other.data;
  }
  template<typename U> bool operator==(const VolumeGridPtr<U> &other) const
  {
    return data == other.data;
  }
  template<typename U> bool operator!=(const VolumeGridPtr<U> &other) const
  {
    return data != other.data;
  }

  operator bool() const
  {
    return bool(data);
  }
};
#else
template<typename T> struct VolumeGridPtr : public VolumeGridPtrCommon {
};
#endif

/** \} */

/* -------------------------------------------------------------------- */
/** \name Grid Utility Functions
 * \{ */

namespace grid_utils {

template<typename T> std::optional<T> get_background_value(const VolumeGridPtr<T> &grid)
{
#ifdef WITH_OPENVDB
  if constexpr (std::is_same_v<T, std::string>) {
    return std::nullopt;
  }
  else {
    return grids::GridConverter<T>::convert(grid.grid()->background());
  }
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
    grid = GridType::create(grids::AttributeConverter<T>::convert(background_value));
  }
  return {make_implicit_shared<VolumeGrid>(grid)};
#else
  return nullptr;
#endif /* WITH_OPENVDB */
}

}  // namespace grid_utils

/** \} */

}  // namespace blender::bke
