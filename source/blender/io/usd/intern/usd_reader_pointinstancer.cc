/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2021 NVIDIA Corporation.
 * All rights reserved.
 */

#include "usd_reader_pointinstancer.h"

#include "BKE_pointcloud.h"
#include "BKE_object.h"
#include "DNA_object_types.h"
#include "BKE_attribute.hh"
#include "BLI_math_quaternion.hh"

#include <iostream>

#include <pxr/usd/usdGeom/pointInstancer.h>

namespace blender::io::usd {

USDPointInstancerReader::USDPointInstancerReader(const pxr::UsdPrim &prim,
                                     const USDImportParams &import_params,
                                     const ImportSettings &settings)
    : USDXformReader(prim, import_params, settings)
{
}

bool USDPointInstancerReader::valid() const
{
  return prim_.IsValid() && prim_.IsA<pxr::UsdGeomPointInstancer>();
}

void USDPointInstancerReader::create_object(Main *bmain, const double /* motionSampleTime */)
{
  void* point_cloud = BKE_pointcloud_add(bmain, name_.c_str());
  this->object_ = BKE_object_add_only_object(bmain, OB_POINTCLOUD, name_.c_str());
  this->object_->data = point_cloud;
}

void USDPointInstancerReader::read_object_data(Main *bmain, const double motionSampleTime) {
    PointCloud *base_point_cloud = static_cast<PointCloud *>(object_->data);

    pxr::UsdGeomPointInstancer point_instancer_prim(prim_);

    if (!point_instancer_prim) {
        return;
    }

    pxr::VtArray<pxr::GfVec3f> positions;
    pxr::VtArray<pxr::GfVec3f> scales;
    pxr::VtArray<pxr::GfQuath> orientations;
    pxr::VtArray<int> proto_indices;
    point_instancer_prim.GetPositionsAttr().Get(&positions);
    point_instancer_prim.GetScalesAttr().Get(&scales);
    point_instancer_prim.GetOrientationsAttr().Get(&orientations);
    point_instancer_prim.GetProtoIndicesAttr().Get(&proto_indices);

    PointCloud *point_cloud = BKE_pointcloud_new_nomain(positions.size());

    auto positions_span = point_cloud->positions_for_write();

    for (int i = 0; i < positions.size(); i++) {
        positions_span[i] = float3(positions[i][0], positions[i][1], positions[i][2]);
    }

    auto scales_attribute = point_cloud->attributes_for_write().lookup_or_add_for_write_only_span<float3>("scales", ATTR_DOMAIN_POINT);

    for (int i = 0; i < scales.size(); i++) {
        scales_attribute.span[i] = float3(scales[i][0], scales[i][1], scales[i][2]);
    }

    auto orientations_attribute = point_cloud->attributes_for_write().lookup_or_add_for_write_only_span<math::Quaternion>("orientations", ATTR_DOMAIN_POINT);

    for (int i = 0; i < orientations.size(); i++) {
        orientations_attribute.span[i] = math::Quaternion(orientations[i].GetImaginary()[0], orientations[i].GetImaginary()[1], orientations[i].GetImaginary()[2], orientations[i].GetReal());
    }

    auto proto_indices_attribute = point_cloud->attributes_for_write().lookup_or_add_for_write_only_span<int>("proto_indices", ATTR_DOMAIN_POINT);

    for (int i = 0; i < proto_indices.size(); i++) {
        proto_indices_attribute.span[i] = proto_indices[i];
    }

    BKE_pointcloud_nomain_to_pointcloud(point_cloud, base_point_cloud);

    USDXformReader::read_object_data(bmain, motionSampleTime);
}

pxr::SdfPathVector USDPointInstancerReader::proto_paths() const {
    pxr::SdfPathVector paths;

    pxr::UsdGeomPointInstancer point_instancer_prim(prim_);

    if (!point_instancer_prim) {
        return paths;
    }

    point_instancer_prim.GetPrototypesRel().GetTargets(&paths);

    return paths;
}


}  // namespace blender::io::usd
