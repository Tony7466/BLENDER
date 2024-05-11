/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "usd_hash_types.hh"

#include "BLI_color.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_map.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"

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

template<> struct is_layout_compatible<pxr::GfVec2f, float2> : std::true_type {};
template<> struct is_layout_compatible<pxr::GfVec3f, float3> : std::true_type {};

/* Conversion utilities to convert a Blender type to an USD type. */
template<typename From, typename To> inline To convert_value(const From value)
{
  return value;
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

template<> inline float2 convert_value(const pxr::GfVec2f value)
{
  return float2(value[0], value[1]);
}
template<> inline float3 convert_value(const pxr::GfVec3f value)
{
  return float3(value[0], value[1], value[2]);
}
template<> inline ColorGeometry4f convert_value(const pxr::GfVec3f value)
{
  return ColorGeometry4f(value[0], value[1], value[2], 1.0f);
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
                                    const pxr::UsdGeomPrimvar &primvar,
                                    pxr::UsdUtilsSparseValueWriter &value_writer)
{
  constexpr bool is_same = std::is_same_v<BlenderT, USDT>;
  constexpr bool is_compatible = detail::is_layout_compatible<BlenderT, USDT>::value;

  pxr::VtArray<USDT> usd_data;
  if (const std::optional<BlenderT> value = buffer.get_if_single()) {
    usd_data.assign(buffer.size(), detail::convert_value<BlenderT, USDT>(*value));
  }
  else {
    const VArraySpan<BlenderT> data(buffer);
    if constexpr (is_same || is_compatible) {
      usd_data.assign(data.cast<USDT>().begin(), data.cast<USDT>().end());
    }
    else {
      usd_data.resize(data.size());
      for (const int i : data.index_range()) {
        usd_data[i] = detail::convert_value<BlenderT, USDT>(data[i]);
      }
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
                                              const eCustomDataType data_type,
                                              const pxr::UsdTimeCode timecode,
                                              const pxr::UsdGeomPrimvar &primvar,
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

inline std::optional<eCustomDataType> convert_usd_type_to_blender(
    const pxr::SdfValueTypeName usd_type)
{
  static const Map<pxr::SdfValueTypeName, eCustomDataType> type_map = []() {
    Map<pxr::SdfValueTypeName, eCustomDataType> map;
    map.add_new(pxr::SdfValueTypeNames->FloatArray, CD_PROP_FLOAT);
    map.add_new(pxr::SdfValueTypeNames->Double, CD_PROP_FLOAT);
    map.add_new(pxr::SdfValueTypeNames->IntArray, CD_PROP_INT32);
    map.add_new(pxr::SdfValueTypeNames->Float2Array, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->TexCoord2dArray, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->TexCoord2fArray, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->TexCoord2hArray, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->TexCoord3dArray, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->TexCoord3fArray, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->TexCoord3hArray, CD_PROP_FLOAT2);
    map.add_new(pxr::SdfValueTypeNames->Float3Array, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Point3fArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Point3dArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Point3hArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Normal3fArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Normal3dArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Normal3hArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Vector3fArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Vector3hArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Vector3dArray, CD_PROP_FLOAT3);
    map.add_new(pxr::SdfValueTypeNames->Color3fArray, CD_PROP_COLOR);
    map.add_new(pxr::SdfValueTypeNames->Color3hArray, CD_PROP_COLOR);
    map.add_new(pxr::SdfValueTypeNames->Color3dArray, CD_PROP_COLOR);
    map.add_new(pxr::SdfValueTypeNames->StringArray, CD_PROP_STRING);
    map.add_new(pxr::SdfValueTypeNames->BoolArray, CD_PROP_BOOL);
    map.add_new(pxr::SdfValueTypeNames->QuatfArray, CD_PROP_QUATERNION);
    map.add_new(pxr::SdfValueTypeNames->QuatdArray, CD_PROP_QUATERNION);
    map.add_new(pxr::SdfValueTypeNames->QuathArray, CD_PROP_QUATERNION);
    return map;
  }();

  const eCustomDataType *value = type_map.lookup_ptr(usd_type);
  if (value == nullptr) {
    return std::nullopt;
  }

  return *value;
}

template<typename T>
pxr::VtArray<T> get_primvar_array(const pxr::UsdGeomPrimvar &primvar,
                                  const pxr::UsdTimeCode timecode)
{
  pxr::VtValue primvar_val;
  if (!primvar.ComputeFlattened(&primvar_val, timecode)) {
    return {};
  }

  if (!primvar_val.CanCast<pxr::VtArray<T>>()) {
    return {};
  }

  return primvar_val.Cast<pxr::VtArray<T>>().template UncheckedGet<pxr::VtArray<T>>();
}

template<typename USDT, typename BlenderT>
void copy_primvar_to_blender_buffer(const pxr::UsdGeomPrimvar &primvar,
                                    const pxr::UsdTimeCode timecode,
                                    const OffsetIndices<int> faces,
                                    MutableSpan<BlenderT> attribute)
{
  pxr::VtArray<USDT> usd_data = get_primvar_array<USDT>(primvar, timecode);
  if (usd_data.empty()) {
    return;
  }

  constexpr bool is_same = std::is_same_v<USDT, BlenderT>;
  constexpr bool is_compatible = detail::is_layout_compatible<USDT, BlenderT>::value;

  const pxr::TfToken pv_interp = primvar.GetInterpolation();
  if (pv_interp == pxr::UsdGeomTokens->constant) {
    /* For situations where there's only a single item, flood fill the object. */
    attribute.fill(detail::convert_value<USDT, BlenderT>(usd_data[0]));
  }
  else if (pv_interp == pxr::UsdGeomTokens->faceVarying) {
    if (!faces.is_empty()) {
      /* Reverse the index order. */
      for (const int i : faces.index_range()) {
        const IndexRange face = faces[i];
        for (int j : face.index_range()) {
          const int rev_index = face.last(j);
          attribute[face.start() + j] = detail::convert_value<USDT, BlenderT>(usd_data[rev_index]);
        }
      }
    }
    else {
      if constexpr (is_same || is_compatible) {
        const Span<USDT> src(usd_data.data(), usd_data.size());
        attribute.copy_from(src.cast<BlenderT>());
      }
      else {
        for (const int64_t i : attribute.index_range()) {
          attribute[i] = detail::convert_value<USDT, BlenderT>(usd_data[i]);
        }
      }
    }
  }
  else {
    /* Assume direct one-to-one mapping. */
    if (usd_data.size() == attribute.size()) {
      if constexpr (is_same || is_compatible) {
        const Span<USDT> src(usd_data.data(), usd_data.size());
        attribute.copy_from(src.cast<BlenderT>());
      }
      else {
        for (const int64_t i : attribute.index_range()) {
          attribute[i] = detail::convert_value<USDT, BlenderT>(usd_data[i]);
        }
      }
    }
  }
}

inline void copy_primvar_to_blender_attribute(const pxr::UsdGeomPrimvar &primvar,
                                              const pxr::UsdTimeCode timecode,
                                              const eCustomDataType data_type,
                                              const bke::AttrDomain domain,
                                              const OffsetIndices<int> face_indices,
                                              bke::MutableAttributeAccessor attributes)
{
  const pxr::TfToken pv_name = pxr::UsdGeomPrimvar::StripPrimvarsName(primvar.GetPrimvarName());

  bke::GSpanAttributeWriter attribute = attributes.lookup_or_add_for_write_span(
      pv_name.GetText(), domain, data_type);

  switch (data_type) {
    case CD_PROP_FLOAT:
      copy_primvar_to_blender_buffer<float>(
          primvar, timecode, face_indices, attribute.span.typed<float>());
      break;
    case CD_PROP_INT32:
      copy_primvar_to_blender_buffer<int32_t>(
          primvar, timecode, face_indices, attribute.span.typed<int>());
      break;
    case CD_PROP_FLOAT2:
      copy_primvar_to_blender_buffer<pxr::GfVec2f>(
          primvar, timecode, face_indices, attribute.span.typed<float2>());
      break;
    case CD_PROP_FLOAT3:
      copy_primvar_to_blender_buffer<pxr::GfVec3f>(
          primvar, timecode, face_indices, attribute.span.typed<float3>());
      break;
    case CD_PROP_COLOR:
      copy_primvar_to_blender_buffer<pxr::GfVec3f>(
          primvar, timecode, face_indices, attribute.span.typed<ColorGeometry4f>());
      break;
    case CD_PROP_BOOL:
      copy_primvar_to_blender_buffer<bool>(
          primvar, timecode, face_indices, attribute.span.typed<bool>());
      break;

    default:
      BLI_assert_unreachable();
  }

  attribute.finish();
}

}  // namespace blender::io::usd
