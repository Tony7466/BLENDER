/* SPDX-FileCopyrightText: 2023 ????. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"
#include "BKE_modifier.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"

#include "DNA_cachefile_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_windowmanager_types.h"

#include "WM_api.hh"

#include "usd_reader_points.h"

namespace blender::io::usd {

USDPointsReader::USDPointsReader(const pxr::UsdPrim &prim,
                                 const USDImportParams &import_params,
                                 const ImportSettings &settings)
    : USDGeomReader(prim, import_params, settings), points_prim_(prim)
{
}

bool USDPointsReader::valid() const
{
  return bool(points_prim_);
}

void USDPointsReader::create_object(Main *bmain, double /*motionSampleTime*/)
{
  PointCloud *point_cloud = static_cast<PointCloud *>(BKE_pointcloud_add(bmain, name_.c_str()));
  object_ = BKE_object_add_only_object(bmain, OB_POINTCLOUD, name_.c_str());
  object_->data = point_cloud;
}

void USDPointsReader::read_object_data(Main *bmain, double motionSampleTime)
{
  if (!points_prim_) {
    /* Invalid prim, so we pass. */
    return;
  }

  const USDMeshReadParams params = create_mesh_read_params(motionSampleTime,
                                                          import_params_.mesh_read_flag);

  PointCloud *point_cloud = static_cast<PointCloud *>(object_->data);

  /* The code below is partly based on AbcPointsReader implementation,
   * but hasn't been tested. */

  bke::GeometrySet geometry_set = bke::GeometrySet::from_pointcloud(
      point_cloud, bke::GeometryOwnershipType::Editable);

  read_geometry(geometry_set, params, nullptr);

  PointCloud *read_point_cloud =
      geometry_set.get_component_for_write<bke::PointCloudComponent>().release();

  if (read_point_cloud != point_cloud) {
    BKE_pointcloud_nomain_to_pointcloud(read_point_cloud, point_cloud);
  }

  if (is_animated()) {
    /* If the point cloud has animated positions or attributes, we add the cache
     * modifier. */
    add_cache_modifier();
  }

  /* Update the transform. */
  USDXformReader::read_object_data(bmain, motionSampleTime);
}

void USDPointsReader::read_geometry(bke::GeometrySet &geometry_set,
                                    USDMeshReadParams params,
                                    const char ** /*err_str*/)
{
  if (!points_prim_) {
    /* Invalid prim, so we pass. */
    return;
  }

  /* Get the existing point cloud. */
  PointCloud *point_cloud = geometry_set.get_pointcloud_for_write();

  /* Read USD point positions. */
  pxr::VtVec3fArray positions;
  points_prim_.GetPointsAttr().Get(&positions, params.motion_sample_time);

  if (point_cloud->totpoint != positions.size()) {
    /* Size changed so we must reallocate. */
    point_cloud = BKE_pointcloud_new_nomain(positions.size());
  }

  /* TODO: Update point poistions and attributes here. */

  /* See AbcPointsReader::read_geometry() for an example of updating point
   * cloud geometry. */

  geometry_set.replace_pointcloud(point_cloud);
}

bool USDPointsReader::is_animated() const
{
  if (!points_prim_) {
    return false;
  }

  bool is_animated = false;

  is_animated |= points_prim_.GetPointsAttr().ValueMightBeTimeVarying();

  /* Will want to check if other attributes are time varying as well. */

  return is_animated;
}

}  // namespace blender::io::usd
