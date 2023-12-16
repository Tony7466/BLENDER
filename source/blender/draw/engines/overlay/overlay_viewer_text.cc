/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 */

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_duplilist.h"
#include "BKE_geometry_set.hh"
#include "BKE_mesh.h"
#include "BKE_pointcloud.h"

#include "BLI_index_range.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_virtual_array.hh"

#include "DNA_curve_types.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "DRW_render.h"
#include "UI_resources.hh"
#include "draw_manager_text.h"

static void overlay_text_viewer_attribute_ex(char *numstr,
                                             size_t numstr_len,
                                             blender::float3 position)
{
  DRWTextStore *dt = DRW_text_cache_ensure();
  const short txt_flag = DRW_TEXT_CACHE_GLOBALSPACE;

  uchar col[4];
  UI_GetThemeColor4ubv(TH_DRAWEXTRA_FACEANG, col);

  DRW_text_cache_add(dt, position, numstr, numstr_len, 0, 0, txt_flag, col);
}

template<typename T>
static void overlay_text_viewer_attribute(const blender::VArray<T> &attribute_values,
                                          const blender::VArraySpan<blender::float3> &positions,
                                          const float4x4 modelMatrix);

template<>
void overlay_text_viewer_attribute<bool>(const blender::VArray<bool> &attribute_values,
                                         const blender::VArraySpan<blender::float3> &positions,
                                         const float4x4 modelMatrix)
{
  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(numstr, "%s", attribute_values.get(i) ? "True" : "False");
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}

template<>
void overlay_text_viewer_attribute<float>(const blender::VArray<float> &attribute_values,
                                          const blender::VArraySpan<blender::float3> &positions,
                                          const float4x4 modelMatrix)
{
  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(numstr, "%g", attribute_values.get(i));
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}

template<>
void overlay_text_viewer_attribute<int>(const blender::VArray<int> &attribute_values,
                                        const blender::VArraySpan<blender::float3> &positions,
                                        const float4x4 modelMatrix)
{

  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(numstr, "%d", attribute_values.get(i));
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}

template<>
void overlay_text_viewer_attribute<blender::float2>(
    const blender::VArray<blender::float2> &attribute_values,
    const blender::VArraySpan<blender::float3> &positions,
    const float4x4 modelMatrix)
{
  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(
        numstr, "(%g, %g)", attribute_values.get(i).x, attribute_values.get(i).y);
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}

template<>
void overlay_text_viewer_attribute<blender::float3>(
    const blender::VArray<blender::float3> &attribute_values,
    const blender::VArraySpan<blender::float3> &positions,
    const float4x4 modelMatrix)
{
  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(numstr,
                               "(%g, %g, %g)",
                               attribute_values.get(i).x,
                               attribute_values.get(i).y,
                               attribute_values.get(i).z);
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}

template<>
void overlay_text_viewer_attribute<blender::float4>(
    const blender::VArray<blender::float4> &attribute_values,
    const blender::VArraySpan<blender::float3> &positions,
    const float4x4 modelMatrix)
{
  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(numstr,
                               "(%.3f, %.3f, %.3f, %.3f)",
                               attribute_values.get(i).x,
                               attribute_values.get(i).y,
                               attribute_values.get(i).z,
                               attribute_values.get(i).w);
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}

template<>
void overlay_text_viewer_attribute<blender::math::Quaternion>(
    const blender::VArray<blender::math::Quaternion> &attribute_values,
    const blender::VArraySpan<blender::float3> &positions,
    const float4x4 modelMatrix)
{
  char numstr[32];
  size_t numstr_len;

  for (const int i : positions.index_range()) {
    float3 position = blender::math::transform_point(modelMatrix, positions[i]);
    numstr_len = SNPRINTF_RLEN(numstr,
                               "(%.3f, %.3f, %.3f, %.3f)",
                               attribute_values.get(i).x,
                               attribute_values.get(i).y,
                               attribute_values.get(i).z,
                               attribute_values.get(i).w);
    overlay_text_viewer_attribute_ex(numstr, numstr_len, position);
  }
}
static void add_data_based_on_type(const blender::GVArray &attributes,
                                   const blender::VArraySpan<float3> &positions,
                                   const float4x4 &modelMatrix)
{
  const eCustomDataType type = blender::bke::cpp_type_to_custom_data_type(attributes.type());
  if (type == CD_PROP_BOOL) {
    overlay_text_viewer_attribute(attributes.typed<bool>(), positions, modelMatrix);
  }
  if (type == CD_PROP_FLOAT) {
    overlay_text_viewer_attribute(attributes.typed<float>(), positions, modelMatrix);
  }
  if (type == CD_PROP_INT32) {
    overlay_text_viewer_attribute(attributes.typed<int>(), positions, modelMatrix);
  }
  if (type == CD_PROP_FLOAT2) {
    overlay_text_viewer_attribute(attributes.typed<float2>(), positions, modelMatrix);
  }
  if (type == CD_PROP_FLOAT3) {
    overlay_text_viewer_attribute(attributes.typed<float3>(), positions, modelMatrix);
  }
  if (type == CD_PROP_COLOR) {
    overlay_text_viewer_attribute(attributes.typed<float4>(), positions, modelMatrix);
  }
  if (type == CD_PROP_QUATERNION) {
    overlay_text_viewer_attribute(
        attributes.typed<blender::math::Quaternion>(), positions, modelMatrix);
  }
}

static void add_attributes_to_text_cache(blender::bke::AttributeAccessor attribute_accessor,
                                         float4x4 modelMatrix)
{
  using namespace blender;

  if (!attribute_accessor.contains(".viewer")) {
    return;
  }

  bke::GAttributeReader viewer_attribute_reader = attribute_accessor.lookup(".viewer");
  const GVArray viewer_attributes = *viewer_attribute_reader;

  const VArraySpan<float3> positions = *attribute_accessor.lookup<float3>(
      "position", viewer_attribute_reader.domain);

  add_data_based_on_type(viewer_attributes, positions, modelMatrix);
}

static void add_instance_attributes_to_text_cache(
    blender::bke::AttributeAccessor attribute_accessor,
    float4x4 modelMatrix,
    float3 position,
    int instance_index)
{
  using namespace blender;

  /* Data from instances are read as a single value from a given index. The data is converted back
   * to an array so one function can handle both instance and object data. */
  const GVArray viewer_attributes = attribute_accessor.lookup(".viewer").varray.slice(
      IndexRange{instance_index, 1});

  const blender::VArraySpan<float3> positions{VArray<float3>::ForSingle(position, 1)};

  add_data_based_on_type(viewer_attributes, positions, modelMatrix);
}

void OVERLAY_viewer_attribute_text(const Object &object)
{
  using namespace blender;
  float4x4 modelMatrix = float4x4(object.object_to_world);
  DupliObject *dupli_object = DRW_object_get_dupli(&object);

  if (dupli_object->preview_instance_index >= 0) {
    const auto &instances =
        *dupli_object->preview_base_geometry->get_component<bke::InstancesComponent>();

    if (instances.attributes()->contains(".viewer")) {
      add_instance_attributes_to_text_cache(*instances.attributes(),
                                            modelMatrix,
                                            dupli_object->ob->loc,
                                            dupli_object->preview_instance_index);

      return;
    }
  }

  switch (object.type) {
    case OB_MESH: {
      const Mesh *mesh = static_cast<Mesh *>(object.data);
      bke::AttributeAccessor attribute_accessor = mesh->attributes();
      add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      break;
    }
    case OB_POINTCLOUD: {
      const PointCloud *pointcloud = static_cast<PointCloud *>(object.data);
      bke::AttributeAccessor attribute_accessor = pointcloud->attributes();
      add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      break;
    }
    case OB_CURVES_LEGACY: {
      const Curve *curve = static_cast<Curve *>(object.data);
      if (curve->curve_eval) {
        const bke::CurvesGeometry &curves = curve->curve_eval->geometry.wrap();
        bke::AttributeAccessor attribute_accessor = curves.attributes();
        add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      }
      break;
    }
    case OB_CURVES: {
      const Curves *curves_id = static_cast<Curves *>(object.data);
      const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
      bke::AttributeAccessor attribute_accessor = curves.attributes();
      add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      break;
    }
  }
}
