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
        VArray<float> viewer_attributes = *mesh->attributes().lookup<float>(".viewer");
        Span<float3> positions = mesh->vert_positions();
        DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
      }
      break;
    }
    case OB_POINTCLOUD: {
      PointCloud *pointcloud = static_cast<PointCloud *>(object.data);
      if (pointcloud->attributes().contains(".viewer")) {
        VArray<float> viewer_attributes = *pointcloud->attributes().lookup<float>(".viewer");
        Span<float3> positions = pointcloud->positions();
        DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
      }
      break;
    }
    case OB_CURVES_LEGACY: {
      Curve *curve = static_cast<Curve *>(object.data);
      if (curve->curve_eval) {
        const bke::CurvesGeometry &curves = curve->curve_eval->geometry.wrap();
        if (curves.attributes().contains(".viewer")) {
          VArray<float> viewer_attributes = *curves.attributes().lookup<float>(".viewer");
          Span<float3> positions = curves.positions();
          DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
        }
      }
      break;
    }
    case OB_CURVES: {
      Curves *curves_id = static_cast<Curves *>(object.data);
      const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
      if (curves.attributes().contains(".viewer")) {
        VArray<float> viewer_attributes = *curves.attributes().lookup<float>(".viewer");
        Span<float3> positions = curves.positions();
        DRW_text_viewer_attribute(viewer_attributes, positions, modelMatrix);
      }
      break;
    }
  }
}
