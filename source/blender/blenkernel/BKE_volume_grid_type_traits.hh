/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

#ifdef WITH_OPENVDB
#  include "openvdb_fwd.hh"
#endif

#ifdef WITH_OPENVDB
namespace blender::bke {

template<typename T> struct VolumeGridTraits {
  using BlenderType = void;
  using PrimitiveType = void;
  using TreeType = void;
};

template<> struct VolumeGridTraits<bool> {
  using BlenderType = bool;
  using PrimitiveType = bool;
  using TreeType = openvdb::BoolTree;

  static bool to_openvdb(const bool &value)
  {
    return value;
  }
  static bool to_blender(const bool &value)
  {
    return value;
  }
};

template<> struct VolumeGridTraits<int> {
  using BlenderType = int;
  using PrimitiveType = int;
  using TreeType = openvdb::Int32Tree;

  static int to_openvdb(const int &value)
  {
    return value;
  }
  static int to_blender(const int &value)
  {
    return value;
  }
};

template<> struct VolumeGridTraits<float> {
  using BlenderType = float;
  using PrimitiveType = float;
  using TreeType = openvdb::FloatTree;

  static float to_openvdb(const float &value)
  {
    return value;
  }
  static float to_blender(const float &value)
  {
    return value;
  }
};

template<> struct VolumeGridTraits<float3> {
  using BlenderType = float3;
  using PrimitiveType = openvdb::Vec3f;
  using TreeType = openvdb::Vec3STree;

  static openvdb::Vec3f to_openvdb(const float3 &value)
  {
    return openvdb::Vec3f(*value);
  }
  static float3 to_blender(const openvdb::Vec3f &value)
  {
    return float3(value.asV());
  }
};

template<> struct VolumeGridTraits<math::Quaternion> {
  using BlenderType = math::Quaternion;
  using PrimitiveType = openvdb::Vec4f;
  using TreeType = openvdb::Vec4fTree;

  static openvdb::Vec4f to_openvdb(const math::Quaternion &value)
  {
    return openvdb::Vec4f(value.w, value.x, value.y, value.z);
  }
  static math::Quaternion to_blender(const openvdb::Vec4f &value)
  {
    return math::Quaternion(value.asV());
  }
};

}  // namespace blender::bke

#endif /* WITH_OPENVDB */
