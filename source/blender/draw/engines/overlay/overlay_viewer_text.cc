#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_duplilist.h"
#include "BKE_geometry_set.hh"
#include "BKE_mesh.h"
#include "BKE_pointcloud.h"

#include "DNA_curve_types.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "DRW_render.h"
#include "draw_manager_text.h"
#include <optional>

using namespace blender;
using namespace blender::bke;

static void add_data_based_on_type(GVArray attributes,
                                   int index,
                                   eCustomDataType type,
                                   blender::float3 pos)
{
  if (type == CD_PROP_BOOL) {
    DRW_text_viewer_attribute(attributes.get<bool>(index), pos);
  }
  if (type == CD_PROP_FLOAT) {
    DRW_text_viewer_attribute(attributes.get<float>(index), pos);
  }
  if (type == CD_PROP_INT32) {
    DRW_text_viewer_attribute(attributes.get<int>(index), pos);
  }
  if (type == CD_PROP_FLOAT2) {
    DRW_text_viewer_attribute(attributes.get<float2>(index), pos);
  }
  if (type == CD_PROP_FLOAT3) {
    DRW_text_viewer_attribute(attributes.get<float3>(index), pos);
  }
  if (CD_PROP_COLOR == type || CD_PROP_QUATERNION == type) {
    DRW_text_viewer_attribute(attributes.get<float4>(index), pos);
  }
}

// function for passing mesh, pointcloud, and curve data to text cache
static void add_attributes_to_text_cache(AttributeAccessor attribute_accessor,
                                         float4x4 modelMatrix)
{
  if (attribute_accessor.contains(".viewer")) {
    // get attribute values
    const GAttributeReader viewer_attribute_reader = attribute_accessor.lookup(".viewer");
    GVArray viewer_attributes = *viewer_attribute_reader;

    // get positions
    VArraySpan<float3> positions = *attribute_accessor.lookup<float3>(
        "position", viewer_attribute_reader.domain);

    eCustomDataType type = attribute_accessor.lookup_meta_data(".viewer")->data_type;

    // for each position
    for (const int i : positions.index_range()) {
      float3 pos = blender::math::transform_point(modelMatrix, positions[i]);

      // add the attribute to the text cache based on it's type
      add_data_based_on_type(viewer_attributes, i, type, pos);
    }
  }
}

static void add_instance_attributes_to_text_cache(AttributeAccessor attribute_accessor,
                                                  float4x4 modelMatrix,
                                                  float3 loc,
                                                  int instance_index)
{
  // get attribute values, position and type
  GVArray viewer_attributes = *attribute_accessor.lookup(".viewer");
  float3 pos = blender::math::transform_point(modelMatrix, loc);
  eCustomDataType type = attribute_accessor.lookup_meta_data(".viewer")->data_type;

  // and add that data to the text cache based on it's type
  add_data_based_on_type(viewer_attributes, instance_index, type, pos);
}

void OVERLAY_viewer_attribute_text(const Object &object)
{
  float4x4 modelMatrix = float4x4(object.object_to_world);

  DupliObject *dupli_object = DRW_object_get_dupli(&object);

  // check for instances
  if (dupli_object->preview_instance_index >= 0) {
    const auto &instances =
        *dupli_object->preview_base_geometry->get_component<InstancesComponent>();
    const AttributeAccessor instance_attributes = *instances.attributes();

    // if instances have the viewer attribute
    if (instance_attributes.contains(".viewer")) {
      // add single instance value to cache based on instance
      add_instance_attributes_to_text_cache(instance_attributes,
                                            modelMatrix,
                                            dupli_object->ob->loc,
                                            dupli_object->preview_instance_index);

      return;
    }
  }

  // if there's no instances, treat as regular object
  switch (object.type) {
    case OB_MESH: {
      Mesh *mesh = static_cast<Mesh *>(object.data);
      AttributeAccessor attribute_accessor = mesh->attributes();
      add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      break;
    }
    case OB_POINTCLOUD: {
      PointCloud *pointcloud = static_cast<PointCloud *>(object.data);
      AttributeAccessor attribute_accessor = pointcloud->attributes();
      add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      break;
    }
    case OB_CURVES_LEGACY: {
      Curve *curve = static_cast<Curve *>(object.data);
      if (curve->curve_eval) {
        const bke::CurvesGeometry &curves = curve->curve_eval->geometry.wrap();
        AttributeAccessor attribute_accessor = curves.attributes();
        add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      }
      break;
    }
    case OB_CURVES: {
      Curves *curves_id = static_cast<Curves *>(object.data);
      const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
      AttributeAccessor attribute_accessor = curves.attributes();
      add_attributes_to_text_cache(attribute_accessor, modelMatrix);
      break;
    }
  }
}
