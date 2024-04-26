/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "BLI_color.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_virtual_array.hh"

#include "DNA_customdata_types.h"

#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>

#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdUtils/sparseValueWriter.h>

#include <cstdint>
#include <optional>
#include <type_traits>

namespace blender::io::usd {

namespace detail {

/* Until we can use C++20, implement our own version of std::is_layout_compatible.
 * Types with compatible layouts can be exchanged much more efficiently than otherwise.
 */
template<class T, class U> struct is_layout_compatible : std::false_type {};

template<> struct is_layout_compatible<float2, pxr::GfVec2f> : std::true_type {};
template<> struct is_layout_compatible<float3, pxr::GfVec3f> : std::true_type {};

/* Conversion utilities to convert a Blender type to an USD type. */
template<typename BlenderT, typename USDT> inline USDT convert_value(const BlenderT value);

template<> inline bool convert_value(const bool value)
{
  return value;
}
template<> inline int convert_value(const int value)
{
  return value;
}
template<> inline float convert_value(const float value)
{
  return value;
}
template<> inline int32_t convert_value(const int8_t value)
{
  return int32_t(value);
}
template<> inline pxr::GfVec2f convert_value(const float2 value)
{
  return pxr::GfVec2f(value[0], value[1]);
}
template<> inline pxr::GfVec3f convert_value(const float3 value)
{
  return pxr::GfVec3f(value[0], value[1], value[2]);
}
template<> inline pxr::GfVec3f convert_value(const ColorGeometry4f value)
{
  return pxr::GfVec3f(value.r, value.g, value.b);
}
template<> inline pxr::GfQuatf convert_value(const math::Quaternion value)
{
  return pxr::GfQuatf(value.x, value.y, value.z, value.w);
}

}  // namespace detail

constexpr std::optional<pxr::SdfValueTypeName> convert_blender_type_to_usd(
    const eCustomDataType blender_type)
{
  switch (blender_type) {
    case CD_PROP_FLOAT:
      return pxr::SdfValueTypeNames->FloatArray;
    case CD_PROP_INT8:
    case CD_PROP_INT32:
      return pxr::SdfValueTypeNames->IntArray;
    case CD_PROP_FLOAT2:
      return pxr::SdfValueTypeNames->Float2Array;
    case CD_PROP_FLOAT3:
      return pxr::SdfValueTypeNames->Float3Array;
    case CD_PROP_STRING:
      return pxr::SdfValueTypeNames->StringArray;
    case CD_PROP_BOOL:
      return pxr::SdfValueTypeNames->BoolArray;
    case CD_PROP_QUATERNION:
      return pxr::SdfValueTypeNames->QuatfArray;
    default:
      return std::nullopt;
  }
}

/* Copy a typed Blender attribute array into a typed USD primvar attribute. */
template<typename BlenderT, typename USDT>
void copy_blender_buffer_to_primvar(const VArray<BlenderT> &buffer,
                                    const pxr::UsdTimeCode timecode,
                                    pxr::UsdGeomPrimvar primvar,
                                    pxr::UsdUtilsSparseValueWriter &value_writer)
{
  constexpr bool is_same = std::is_same_v<BlenderT, USDT>;
  constexpr bool is_compatible = detail::is_layout_compatible<BlenderT, USDT>::value;

  pxr::VtArray<USDT> usd_data;
  if (const std::optional<BlenderT> value = buffer.get_if_single()) {
    usd_data.assign(buffer.size(), detail::convert_value<BlenderT, USDT>(*value));
  }
  else if (buffer.is_span() && (is_same || is_compatible)) {
    Span<USDT> data = buffer.get_internal_span().cast<USDT>();
    usd_data.assign(data.begin(), data.end());

    // -- OR we could do the following
    // usd_data.resize(buffer.size());
    // MutableSpan<USDT> usd_dest(usd_data.data(), usd_data.size());
    // blender::array_utils::copy(data, usd_dest);
  }
  else {
    usd_data.resize(buffer.size());
    for (const int i : buffer.index_range()) {
      usd_data[i] = detail::convert_value<BlenderT, USDT>(buffer[i]);
    }
  }

  if (!primvar.HasValue() && timecode != pxr::UsdTimeCode::Default()) {
    primvar.Set(usd_data, pxr::UsdTimeCode::Default());
  }
  else {
    primvar.Set(usd_data, timecode);
  }

  value_writer.SetAttribute(primvar.GetAttr(), usd_data, timecode);
}

inline void copy_blender_attribute_to_primvar(const GVArray &attribute,
                                              const int data_type,
                                              const pxr::UsdTimeCode timecode,
                                              pxr::UsdGeomPrimvar primvar,
                                              pxr::UsdUtilsSparseValueWriter &value_writer)
{
  switch (data_type) {
    case CD_PROP_FLOAT:
      copy_blender_buffer_to_primvar<float, float>(
          attribute.typed<float>(), timecode, primvar, value_writer);
      break;
    case CD_PROP_INT8:
      copy_blender_buffer_to_primvar<int8_t, int32_t>(
          attribute.typed<int8_t>(), timecode, primvar, value_writer);
      break;
    case CD_PROP_INT32:
      copy_blender_buffer_to_primvar<int, int32_t>(
          attribute.typed<int>(), timecode, primvar, value_writer);
      break;
    case CD_PROP_FLOAT2:
      copy_blender_buffer_to_primvar<float2, pxr::GfVec2f>(
          attribute.typed<float2>(), timecode, primvar, value_writer);
      break;
    case CD_PROP_FLOAT3:
      copy_blender_buffer_to_primvar<float3, pxr::GfVec3f>(
          attribute.typed<float3>(), timecode, primvar, value_writer);
      break;
    case CD_PROP_BOOL:
      copy_blender_buffer_to_primvar<bool, bool>(
          attribute.typed<bool>(), timecode, primvar, value_writer);
      break;
    case CD_PROP_QUATERNION:
      copy_blender_buffer_to_primvar<math::Quaternion, pxr::GfQuatf>(
          attribute.typed<math::Quaternion>(), timecode, primvar, value_writer);
      break;
    default:
      BLI_assert_unreachable();
  }
}

}  // namespace blender::io::usd
