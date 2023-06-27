/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_OPENVDB

namespace blender::volume_openvdb {

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

}  // namespace blender::volume_openvdb

#endif  // WITH_OPENVDB
