/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_cpp_type.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_parameter_pack_utils.hh"

#include "BKE_volume.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke {

#ifdef WITH_OPENVDB

namespace detail {

template<typename Fn> struct TypeTagExecutor {
  const Fn &fn;

  template<typename T> void operator()() const
  {
    fn(TypeTag<T>{});
  }

  void operator()() const
  {
    fn(TypeTag<void>{});
  }
};

}  // namespace detail

/**
 * Call operator for static type of the form:
 *   void MyOperator<GridType>();
 *
 * e.g.
 *   void MyOperator<openvdb::FloatGrid>();
 *   void MyOperator<openvdb::Vec3fGrid>();
 */
template<typename Fn> auto volume_grid_to_static_type(const VolumeGridType grid_type, Fn &&fn)
{
  switch (grid_type) {
    case VOLUME_GRID_FLOAT:
      return fn.template operator()<openvdb::FloatGrid>();
    case VOLUME_GRID_VECTOR_FLOAT:
      return fn.template operator()<openvdb::Vec3fGrid>();
    case VOLUME_GRID_BOOLEAN:
      return fn.template operator()<openvdb::BoolGrid>();
    case VOLUME_GRID_DOUBLE:
      return fn.template operator()<openvdb::DoubleGrid>();
    case VOLUME_GRID_INT:
      return fn.template operator()<openvdb::Int32Grid>();
    case VOLUME_GRID_INT64:
      return fn.template operator()<openvdb::Int64Grid>();
    case VOLUME_GRID_VECTOR_INT:
      return fn.template operator()<openvdb::Vec3IGrid>();
    case VOLUME_GRID_VECTOR_DOUBLE:
      return fn.template operator()<openvdb::Vec3dGrid>();
    case VOLUME_GRID_MASK:
      return fn.template operator()<openvdb::MaskGrid>();
    case VOLUME_GRID_POINTS:
      return fn.template operator()<openvdb::points::PointDataGrid>();
    case VOLUME_GRID_UNKNOWN:
      break;
  }

  /* Should never be called. */
  BLI_assert_msg(0, "should never be reached");
  return fn.template operator()<openvdb::FloatGrid>();
}

/**
 * Call operator for static type with a dummy tag instance
 * (easier to use with lambdas), e.g :
 *   void MyOperator(TypeTag<GridType> dummy);
 *
 * e.g.
 *   void MyOperator(TypeTag<openvdb::FloatGrid>);
 *   void MyOperator(TypeTag<openvdb::Vec3fGrid>);
 */
template<typename Fn> auto volume_grid_to_static_type_tag(const VolumeGridType grid_type, Fn &&fn)
{
  detail::TypeTagExecutor<Fn> executor{fn};
  return volume_grid_to_static_type(grid_type, executor);
}

const CPPType &volume_grid_type_to_cpp_type(const VolumeGridType grid_type)
{
  const CPPType *cpptype = nullptr;
  volume_grid_to_static_type_tag(grid_type, [&cpptype](auto tag) {
    using GridType = typename decltype(tag)::type;
    using Converter = typename bke::template GridValueConverter<typename GridType::ValueType>;
    using AttributeType = typename Converter::AttributeType;
    cpptype = &CPPType::get<AttributeType>();
  });
  return *cpptype;
}

/* -------------------------------------------------------------------- */
/** \name Grid Value Converter
 * \{ */

template<typename GridValueT> struct GridValueConverter {
  using GridValueType = GridValueT;
  using AttributeType = GridValueT;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return value;
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return value;
  }
};

template<> struct GridValueConverter<double> {
  using GridValueType = double;
  using AttributeType = float;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return double(value);
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return value;
  }
};

template<> struct GridValueConverter<openvdb::Vec3d> {
  using GridValueType = openvdb::Vec3d;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(float(value.x()), float(value.y()), float(value.z()));
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3d(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::Vec3i> {
  using GridValueType = openvdb::Vec3i;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(float(value.x()), float(value.y()), float(value.z()));
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3i(int(value.x), int(value.y), int(value.z));
  }
};

template<> struct GridValueConverter<openvdb::Vec3f> {
  using GridValueType = openvdb::Vec3f;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(value.x(), value.y(), value.z());
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3f(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::PointDataIndex32> {
  using GridValueType = openvdb::PointDataIndex32;
  using AttributeType = int32_t;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return int32_t(value);
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return GridValueType(value);
  }
};

/** \} */

template<template<typename> typename VArrayImplT>
GVArray get_volume_varray(const openvdb::GridBase &grid)
{
  GVArray result;
  volume_grid_to_static_type_tag(BKE_volume_grid_type_openvdb(grid), [&grid, &result](auto tag) {
    using GridType = typename decltype(tag)::type;
    using VArrayImplType = VArrayImplT<GridType>;
    using ValueType = typename GridType::ValueType;
    using AttributeType = typename GridValueConverter<ValueType>::AttributeType;

    return VArray<AttributeType>::For<VArrayImplType>(static_cast<const GridType &>(grid));
  });
  return result;
}

#endif

}  // namespace blender::bke
