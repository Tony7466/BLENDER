/* SPDX-FileCopyrightText: 2023 ????. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_reader_points.hh"

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"
#include "BKE_modifier.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"

#include "BLI_color.hh"

#include "DNA_cachefile_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_windowmanager_types.h"

#include "WM_api.hh"

#include <iostream>

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

  // see if we can get a sense of what attributes are on the USD
  auto attrs = points_prim_.GetSchemaAttributeNames();
  // convert to strings for print out
  auto strings = TfToStringVector(attrs);

  for (const std::string& str : strings) {
        std::cout << "attr: " << str << std::endl;
  }

  // instead use the getprimvars
  pxr::UsdGeomPrimvarsAPI primvarsLoaded(points_prim_);
  std::vector<pxr::UsdGeomPrimvar> customAttrs=primvarsLoaded.GetPrimvars();

    std::cout << "Custom Attributes:" << std::endl;
    for (auto attr : customAttrs) {
        std::cout << attr.GetName() << " (" << attr.GetTypeName() << ")" << std::endl;
    }

  if (point_cloud->totpoint != positions.size()) {
    /* Size changed so we must reallocate. */
    point_cloud = BKE_pointcloud_new_nomain(positions.size());
  }

  /* TODO: Update point poistions and attributes here. */
  
  bke::SpanAttributeWriter<float3> positions_writer =
      point_cloud->attributes_for_write().lookup_or_add_for_write_span<float3>("position",
                                                                               bke::AttrDomain::Point);
  MutableSpan<float3> point_positions = positions_writer.span;

  // do things like iterate over the points and add their data via the point positions writerh
  // uses the copy_zup_from_yup?
  for (size_t i = 0 ; i < positions.size(); i++) {
    //blender::io::alembic::copy_zup_from_yup(point_positions[i],positions[i]);
    point_positions[i][0]=positions[i][0];
    point_positions[i][1]=positions[i][1];
    point_positions[i][2]=positions[i][2];
  }

  positions_writer.finish();
  // here we will use the same approach as above iterating over the prim vars to create custom spans for the attrs


  // not sure what other types I can use for the span attribute
  std::vector<pxr::UsdGeomPrimvar> primvars=primvarsLoaded.GetPrimvarsWithValues();
  for (auto pv : primvars) {
      std::cout << pv.GetName() << " (" << pv.GetTypeName() << ")" << std::endl;
      if (!pv.HasValue()) {
        continue;
      }
      const pxr::SdfValueTypeName type = pv.GetTypeName();
      const pxr::TfToken interp = pv.GetInterpolation();
      const pxr::TfToken name = pv.StripPrimvarsName(pv.GetPrimvarName());
    

    // I guess the hard part  is knowing which tempated pvibute writer goes with a particular primvar
      if (type == pxr::SdfValueTypeNames->Color3fArray && interp == pxr::UsdGeomTokens->vertex) {
        bke::SpanAttributeWriter<ColorGeometry4f> primvar_writer =
            point_cloud->attributes_for_write().lookup_or_add_for_write_span<ColorGeometry4f>(pv.GetName().GetText(), bke::AttrDomain::Point);
        if (!primvar_writer) {
          printf("couldn't make writer for color %s\n", name.GetText());
          continue;
        }
        pxr::VtVec3fArray colors;
        // where did the params value come from?
        if (!pv.ComputeFlattened(&colors,params.motion_sample_time)) {
          printf("coulnd't compute the flattened colors %s\n", name.GetText());
          continue;
        }

        for (int i =0; i < colors.size(); ++i) {
          const pxr::GfVec3f & usd_color = colors[i];
          primvar_writer.span[i] = ColorGeometry4f(
            usd_color[0],usd_color[1],usd_color[2],1.0f
          );
        };
        primvar_writer.finish();
      }
      if (type == pxr::SdfValueTypeNames->FloatArray) {
        printf("WE ARE IN THE FLOAT ARRAY PROCESSING BLOCK");
        bke::SpanAttributeWriter<float> primvar_writer =
            point_cloud->attributes_for_write().lookup_or_add_for_write_span<float>(name.GetText(), bke::AttrDomain::Point);
        if (!primvar_writer) {
          printf("couldn't make writer for float prop %s\n", name.GetText());
          continue;
        }
        pxr::VtArray<float> values;
        if (!pv.ComputeFlattened(&values,params.motion_sample_time)) {
          printf("couldn't compute the flattened colors %s\n", name.GetText());
          continue;
        }

        for (int i =0; i < values.size(); ++i) {
          primvar_writer.span[i] = values[i];
        };
        printf("finished writing values into the primvar writer");
        primvar_writer.finish();
    }
  }
  

  

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
