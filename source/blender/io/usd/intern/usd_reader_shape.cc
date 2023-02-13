
#include "BKE_lib_id.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"

#include "DNA_cachefile_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_space_types.h" /* for FILE_MAX */
#include "DNA_windowmanager_types.h"

#include "WM_api.h"

#include "usd_reader_shape.h"

#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cone.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usdImaging/usdImaging/capsuleAdapter.h>
#include <pxr/usdImaging/usdImaging/coneAdapter.h>
#include <pxr/usdImaging/usdImaging/cubeAdapter.h>
#include <pxr/usdImaging/usdImaging/cylinderAdapter.h>
#include <pxr/usdImaging/usdImaging/sphereAdapter.h>

namespace blender::io::usd {

USDShapeReader::USDShapeReader(const pxr::UsdPrim &prim,
                               const USDImportParams &import_params,
                               const ImportSettings &settings)
    : USDGeomReader(prim, import_params, settings)
{
}

void USDShapeReader::create_object(Main *bmain, double motionSampleTime)
{
  UNUSED_VARS(motionSampleTime);

  Mesh *mesh = BKE_mesh_add(bmain, name_.c_str());
  object_ = BKE_object_add_only_object(bmain, OB_MESH, name_.c_str());
  object_->data = mesh;
}

void USDShapeReader::read_object_data(Main *bmain, double motionSampleTime)
{
  Mesh *mesh = (Mesh *)object_->data;
  Mesh *read_mesh = this->read_mesh(
      mesh, motionSampleTime, import_params_.mesh_read_flag, nullptr);

  if (read_mesh != mesh) {
    /* FIXME: after 2.80; `mesh->flag` isn't copied by #BKE_mesh_nomain_to_mesh() */
    /* read_mesh can be freed by BKE_mesh_nomain_to_mesh(), so get the flag before that happens. */
    const uint16_t autosmooth = (read_mesh->flag & ME_AUTOSMOOTH);
    BKE_mesh_nomain_to_mesh(read_mesh, mesh, object_);
    mesh->flag |= autosmooth;

    if (is_time_varying()) {
      USDGeomReader::add_cache_modifier();
    }
  }

  USDXformReader::read_object_data(bmain, motionSampleTime);
}

template<typename Adapter>
void USDShapeReader::read_values(const double motionSampleTime,
                                 pxr::VtVec3fArray &positions,
                                 pxr::VtIntArray &face_indices,
                                 pxr::VtIntArray &face_counts) const
{
  Adapter adapter;
  pxr::VtValue points_val = adapter.GetPoints(prim_, motionSampleTime);

  if (points_val.IsHolding<pxr::VtVec3fArray>()) {
    positions = points_val.Get<pxr::VtVec3fArray>();
  }

  pxr::VtValue topology_val = adapter.GetTopology(prim_, pxr::SdfPath(), motionSampleTime);

  if (topology_val.IsHolding<pxr::HdMeshTopology>()) {
    const pxr::HdMeshTopology &topology = topology_val.Get<pxr::HdMeshTopology>();
    face_counts = topology.GetFaceVertexCounts();
    face_indices = topology.GetFaceVertexIndices();
  }
}

bool USDShapeReader::read_mesh_values(double motionSampleTime,
          pxr::VtVec3fArray &positions,
          pxr::VtIntArray &face_indices,
          pxr::VtIntArray &face_counts) const
{
  const bool is_capsule = prim_.IsA<pxr::UsdGeomCapsule>();
  const bool is_cylinder = prim_.IsA<pxr::UsdGeomCylinder>();
  const bool is_cone = prim_.IsA<pxr::UsdGeomCone>();
  const bool is_cube = prim_.IsA<pxr::UsdGeomCube>();
  const bool is_sphere = prim_.IsA<pxr::UsdGeomSphere>();

  if (is_capsule) {
    read_values<pxr::UsdImagingCapsuleAdapter>(motionSampleTime, positions, face_indices, face_counts);
    return true;
  }

  if (is_cylinder) {
    read_values<pxr::UsdImagingCylinderAdapter>(motionSampleTime, positions, face_indices, face_counts);
    return true;
  }

  if (is_cone) {
    read_values<pxr::UsdImagingConeAdapter>(motionSampleTime, positions, face_indices, face_counts);
    return true;
  }

  if (is_cube) {
    read_values<pxr::UsdImagingCubeAdapter>(motionSampleTime, positions, face_indices, face_counts);
    return true;
  }

  if (is_sphere) {
    read_values<pxr::UsdImagingSphereAdapter>(motionSampleTime, positions, face_indices, face_counts);
    return true;
  }

  WM_reportf(RPT_ERROR,
             "Unhandled Gprim type: %s (%s)",
             prim_.GetTypeName().GetText(),
             prim_.GetPath().GetText());
  return false;
}

struct Mesh *USDShapeReader::read_mesh(struct Mesh *existing_mesh,
                                       double motionSampleTime,
                                       int read_flag,
                                       const char **err_str)
{
  pxr::VtIntArray face_indices;
  pxr::VtIntArray face_counts;

  if (!prim_) {
    return existing_mesh;
  }

  UNUSED_VARS(read_flag);
  UNUSED_VARS(err_str);

  /* Should have a good set of data by this point-- copy over. */
  Mesh *active_mesh = mesh_from_prim(existing_mesh, motionSampleTime, face_indices, face_counts);

  MutableSpan<MPoly> polys = active_mesh->polys_for_write();
  MutableSpan<MLoop> loops = active_mesh->loops_for_write();

  int loop_index = 0;
  const char should_smooth = prim_.IsA<pxr::UsdGeomCube>() ? 0 : ME_SMOOTH;

  if (active_mesh != existing_mesh) {
    for (int i = 0; i < face_counts.size(); i++) {
      const int face_size = face_counts[i];

      MPoly &poly = polys[i];
      poly.loopstart = loop_index;
      poly.totloop = face_size;

      /* Don't smooth-shade cubes; we're not worrying about sharpness for Gprims. */
      poly.flag |= should_smooth;

      for (int f = 0; f < face_size; ++f, ++loop_index) {
        loops[loop_index].v = face_indices[loop_index];
      }
    }
  }

  BKE_mesh_calc_edges(active_mesh, false, false);
  BKE_mesh_normals_tag_dirty(active_mesh);

  return active_mesh;
}


Mesh *USDShapeReader::mesh_from_prim(Mesh *existing_mesh, double motionSampleTime,
                                     pxr::VtIntArray& face_indices, pxr::VtIntArray& face_counts) const
{
  pxr::VtVec3fArray positions;

  Mesh *active_mesh = existing_mesh;

  if (!read_mesh_values(motionSampleTime, positions, face_indices, face_counts)) {
    return active_mesh;
  }

  const bool poly_counts_match = existing_mesh ? face_counts.size() == existing_mesh->totpoly : false;
  const bool position_counts_match = existing_mesh ? positions.size() == existing_mesh->totvert : false;
  if (!position_counts_match || !poly_counts_match) {
    active_mesh = BKE_mesh_new_nomain_from_template(
        existing_mesh, positions.size(), 0, 0, face_indices.size(), face_counts.size());
  }

  MutableSpan<float3> verts = active_mesh->vert_positions_for_write();

  for (int i = 0; i < positions.size(); i++) {
   verts[i][0] = positions[i][0];
   verts[i][1] = positions[i][1];
   verts[i][2] = positions[i][2];
  }

  return active_mesh;
}

bool USDShapeReader::is_time_varying()
{
  const bool is_capsule = prim_.IsA<pxr::UsdGeomCapsule>();
  const bool is_cylinder = prim_.IsA<pxr::UsdGeomCylinder>();
  const bool is_cone = prim_.IsA<pxr::UsdGeomCone>();
  const bool is_cube = prim_.IsA<pxr::UsdGeomCube>();
  const bool is_sphere = prim_.IsA<pxr::UsdGeomSphere>();

  if (is_capsule) {
    pxr::UsdGeomCapsule geom(prim_);
    return (geom.GetAxisAttr().ValueMightBeTimeVarying() ||
              geom.GetHeightAttr().ValueMightBeTimeVarying() ||
              geom.GetRadiusAttr().ValueMightBeTimeVarying());
  }

  if (is_cylinder) {
    pxr::UsdGeomCylinder geom(prim_);
    return (geom.GetAxisAttr().ValueMightBeTimeVarying() ||
              geom.GetHeightAttr().ValueMightBeTimeVarying() ||
              geom.GetRadiusAttr().ValueMightBeTimeVarying());
  }

  if (is_cone) {
    pxr::UsdGeomCone geom(prim_);
    return (geom.GetAxisAttr().ValueMightBeTimeVarying() ||
              geom.GetHeightAttr().ValueMightBeTimeVarying() ||
              geom.GetRadiusAttr().ValueMightBeTimeVarying());
  }

  if (is_cube) {
    pxr::UsdGeomCube geom(prim_);
    return geom.GetSizeAttr().ValueMightBeTimeVarying();
  }

  if (is_sphere) {
    pxr::UsdGeomSphere geom(prim_);
    return geom.GetRadiusAttr().ValueMightBeTimeVarying();
  }

  WM_reportf(RPT_ERROR,
               "Unhandled Gprim type: %s (%s)",
               prim_.GetTypeName().GetText(),
               prim_.GetPath().GetText());
  return false;
}

}  // namespace blender::io::usd
