/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"
#include "BLI_task.hh"

#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.h"
#include "BKE_volume.h"

#include "GEO_mesh_to_volume.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/GridTransformer.h>
#  include <openvdb/tools/LevelSetUtil.h>
#  include <openvdb/tools/VolumeToMesh.h>

namespace blender::geometry {

/* This class follows the MeshDataAdapter interface from openvdb. */
class OpenVDBMeshAdapter {
 private:
  Span<float3> positions_;
  Span<int> corner_verts_;
  Span<MLoopTri> looptris_;
  float4x4 transform_;

 public:
  OpenVDBMeshAdapter(const Mesh &mesh, float4x4 transform);
  size_t polygonCount() const;
  size_t pointCount() const;
  size_t vertexCount(size_t /*polygon_index*/) const;
  void getIndexSpacePoint(size_t polygon_index, size_t vertex_index, openvdb::Vec3d &pos) const;
};

OpenVDBMeshAdapter::OpenVDBMeshAdapter(const Mesh &mesh, float4x4 transform)
    : positions_(mesh.vert_positions()),
      corner_verts_(mesh.corner_verts()),
      looptris_(mesh.looptris()),
      transform_(transform)
{
}

size_t OpenVDBMeshAdapter::polygonCount() const
{
  return size_t(looptris_.size());
}

size_t OpenVDBMeshAdapter::pointCount() const
{
  return size_t(positions_.size());
}

size_t OpenVDBMeshAdapter::vertexCount(size_t /*polygon_index*/) const
{
  /* All polygons are triangles. */
  return 3;
}

void OpenVDBMeshAdapter::getIndexSpacePoint(size_t polygon_index,
                                            size_t vertex_index,
                                            openvdb::Vec3d &pos) const
{
  const MLoopTri &looptri = looptris_[polygon_index];
  const float3 transformed_co = math::transform_point(
      transform_, positions_[corner_verts_[looptri.tri[vertex_index]]]);
  pos = &transformed_co.x;
}

float volume_compute_voxel_size(const MeshToVolumeSettings &settings,
                                FunctionRef<void(float3 &r_min, float3 &r_max)> bounds_fn,
                                const float4x4 &transform)
{
  if (settings.voxels <= 0) {
    return 0.0f;
  }

  float3 bb_min;
  float3 bb_max;
  bounds_fn(bb_min, bb_max);

  /* Compute the voxel size based on the desired number of voxels and the approximated bounding
   * box of the volume. */
  const float diagonal = math::distance(math::transform_point(transform, bb_max),
                                        math::transform_point(transform, bb_min));

  if (settings.use_world_space_units) {
    return (diagonal + settings.exterior_band_width * 2.0f) / float(settings.voxels);
  }
  return diagonal / std::max(1.0f, float(settings.voxels) - 2.0f * settings.exterior_band_width);
}

static openvdb::FloatGrid::Ptr mesh_to_volume_grid(const Mesh *mesh,
                                                   const float4x4 &mesh_to_volume_space_transform,
                                                   const MeshToVolumeSettings &settings)
{
  if (settings.voxel_size < 1e-5f) {
    return nullptr;
  }

  float4x4 mesh_to_index_space_transform = math::from_scale<float4x4>(
      float3(1.0f / (settings.voxel_size * settings.simplify)));
  mesh_to_index_space_transform *= mesh_to_volume_space_transform;
  /* Better align generated grid with the source mesh. */
  mesh_to_index_space_transform.location() -= 0.5f;

  OpenVDBMeshAdapter mesh_adapter{*mesh, mesh_to_index_space_transform};

  /* Convert the bandwidths from object in index space. */
  float exterior;
  float interior;

  if (settings.use_world_space_units) {
    exterior = std::max(1.0f, settings.exterior_band_width / settings.voxel_size);
    interior = std::max(1.0f, settings.interior_band_width / settings.voxel_size);
  }
  else {
    exterior = std::max(1.0f, settings.exterior_band_width);
    interior = std::max(1.0f, settings.interior_band_width);
  }

  openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(
      settings.voxel_size);

  /* Setting the interior bandwidth to FLT_MAX, will make it fill the entire volume. */
  openvdb::FloatGrid::Ptr new_grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
      mesh_adapter, *transform, exterior, settings.fill_volume ? FLT_MAX : interior);

  if (settings.convert_to_fog) {
    openvdb::tools::sdfToFogVolume(*new_grid);

    /* Take the desired density into account. */
    if (settings.density != 1.0f) {
      openvdb::tools::foreach (
          new_grid->beginValueOn(), [&](const openvdb::FloatGrid::ValueOnIter &iter) {
            iter.modifyValue([&](float &value) { value *= settings.density; });
          });
    }
  }
  return new_grid;
}

VolumeGrid *volume_grid_add_from_mesh(Volume *volume,
                                      const StringRefNull name,
                                      const Mesh *mesh,
                                      const float4x4 &mesh_to_volume_space_transform,
                                      const MeshToVolumeSettings &settings)
{
  openvdb::FloatGrid::Ptr mesh_grid = mesh_to_volume_grid(
      mesh, mesh_to_volume_space_transform, settings);
  return mesh_grid ? BKE_volume_grid_add_vdb(*volume, name, std::move(mesh_grid)) : nullptr;
}
}  // namespace blender::geometry
#endif
