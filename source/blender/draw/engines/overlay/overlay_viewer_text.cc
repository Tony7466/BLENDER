#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.h"

#include "BLI_math_matrix_types.hh"

#include "DNA_curve_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "draw_manager_text.h"

void OVERLAY_viewer_attribute_text(const Object &object)
{
  using namespace blender;
  using namespace blender::bke;

  float4x4 modelMatrix = float4x4(object.object_to_world);

  switch (object.type) {
    case OB_MESH: {
      Mesh *mesh = static_cast<Mesh *>(object.data);
      if (mesh->attributes().contains(".viewer")) {
        const AttributeReader viewer_attribute_reader = mesh->attributes().lookup<float>(
            ".viewer");
        VArray<float> viewer_attributes = *viewer_attribute_reader;
        printf("viewer_attributes.size() = %lld\n", viewer_attributes.size());
        VArraySpan<float3> positions = *mesh->attributes().lookup<float3>(
            "position", viewer_attribute_reader.domain);
        DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
      }
      break;
    }
    case OB_POINTCLOUD: {
      PointCloud *pointcloud = static_cast<PointCloud *>(object.data);
      if (pointcloud->attributes().contains(".viewer")) {
        const AttributeReader viewer_attribute_reader = pointcloud->attributes().lookup<float>(
            ".viewer");
        VArray<float> viewer_attributes = *viewer_attribute_reader;
        VArraySpan<float3> positions = *pointcloud->attributes().lookup<float3>(
            "position", viewer_attribute_reader.domain);
        DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
      }
      break;
    }
    case OB_CURVES_LEGACY: {
      Curve *curve = static_cast<Curve *>(object.data);
      if (curve->curve_eval) {
        const bke::CurvesGeometry &curves = curve->curve_eval->geometry.wrap();
        if (curves.attributes().contains(".viewer")) {
          const AttributeReader viewer_attribute_reader = curves.attributes().lookup<float>(
              ".viewer");
          VArray<float> viewer_attributes = *viewer_attribute_reader;
          VArraySpan<float3> positions = *curves.attributes().lookup<float3>(
              "position", viewer_attribute_reader.domain);
          DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
        }
      }
      break;
    }
    case OB_CURVES: {
      Curves *curves_id = static_cast<Curves *>(object.data);
      const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
      if (curves.attributes().contains(".viewer")) {
        const AttributeReader viewer_attribute_reader = curves.attributes().lookup<float>(
            ".viewer");
        VArray<float> viewer_attributes = *viewer_attribute_reader;
        VArraySpan<float3> positions = *curves.attributes().lookup<float3>(
            "position", viewer_attribute_reader.domain);
        DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
      }
      break;
    }
  }
}
