/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_reader_points.hh"

#include "BKE_geometry_set.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"

#include "BLI_color.hh"
#include "BLI_span.hh"

#include "DNA_cachefile_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"

#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

#include "CLG_log.h"
static CLG_LogRef LOG = {"io.usd"};

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

  PointCloud *point_cloud = geometry_set.get_pointcloud_for_write();

  /* Get the existing point cloud data. */
  pxr::VtVec3fArray usd_positions;
  points_prim_.GetPointsAttr().Get(&usd_positions, params.motion_sample_time);

  if (point_cloud->totpoint != usd_positions.size()) {
    /* Size changed so we must reallocate. */
    point_cloud = BKE_pointcloud_new_nomain(usd_positions.size());
  }

  /* Update point positions and radii */
  static_assert(sizeof(pxr::GfVec3f) == sizeof(float3));
  MutableSpan<float3> positions = point_cloud->positions_for_write();
  positions.copy_from(Span(usd_positions.data(), usd_positions.size()).cast<float3>());

  pxr::VtFloatArray usd_widths;
  points_prim_.GetWidthsAttr().Get(&usd_widths, params.motion_sample_time);

  if (!usd_widths.empty()) {
    bke::MutableAttributeAccessor attributes = point_cloud->attributes_for_write();
    bke::SpanAttributeWriter<float> radii = attributes.lookup_or_add_for_write_span<float>(
        "radius", bke::AttrDomain::Point);

    const pxr::TfToken widths_interp = points_prim_.GetWidthsInterpolation();
    if (widths_interp == pxr::UsdGeomTokens->constant) {
      radii.span.fill(usd_widths[0] / 2.0f);
    }
    else {
      for (int i_point = 0; i_point < usd_widths.size(); i_point++) {
        radii.span[i_point] = usd_widths[i_point] / 2.0f;
      }
    }

    radii.finish();
  }

  /* Read in velocity and generic data. */
  read_velocities(point_cloud, params.motion_sample_time);
  read_custom_data(point_cloud, params.motion_sample_time);

  geometry_set.replace_pointcloud(point_cloud);
}

void USDPointsReader::read_velocities(PointCloud *point_cloud, const double motionSampleTime) const
{
  pxr::VtVec3fArray velocities;
  points_prim_.GetVelocitiesAttr().Get(&velocities, motionSampleTime);

  if (!velocities.empty()) {
    bke::MutableAttributeAccessor attributes = point_cloud->attributes_for_write();
    bke::GSpanAttributeWriter attribute = attributes.lookup_or_add_for_write_span(
        "velocity", bke::AttrDomain::Point, CD_PROP_FLOAT3);

    Span<pxr::GfVec3f> usd_data(velocities.data(), velocities.size());
    attribute.span.typed<float3>().copy_from(usd_data.cast<float3>());
    attribute.finish();
  }
}

void USDPointsReader::read_custom_data(PointCloud *point_cloud,
                                       const double motionSampleTime) const
{
  pxr::UsdGeomPrimvarsAPI pv_api(points_prim_);

  std::vector<pxr::UsdGeomPrimvar> primvars = pv_api.GetPrimvarsWithValues();
  for (const pxr::UsdGeomPrimvar &pv : primvars) {
    if (!pv.HasValue()) {
      continue;
    }

    const pxr::SdfValueTypeName type = pv.GetTypeName();
    const pxr::TfToken interp = pv.GetInterpolation();
    const pxr::TfToken name = pv.StripPrimvarsName(pv.GetPrimvarName());

    if (type == pxr::SdfValueTypeNames->Color3fArray) {
      bke::SpanAttributeWriter<ColorGeometry4f> primvar_writer =
          point_cloud->attributes_for_write().lookup_or_add_for_write_span<ColorGeometry4f>(
              name.GetText(), bke::AttrDomain::Point);
      if (!primvar_writer) {
        CLOG_WARN(&LOG, "Couldn't make writer for color %s", name.GetText());
        continue;
      }

      pxr::VtVec3fArray colors;
      if (!pv.ComputeFlattened(&colors, motionSampleTime)) {
        CLOG_WARN(&LOG, "Couldn't compute the flattened colors %s", name.GetText());
        continue;
      }

      /* TODO: Do we need to handle any other interpolation modes? */
      if (interp == pxr::UsdGeomTokens->constant) {
        const pxr::GfVec3f &usd_color = colors[0];
        primvar_writer.span.fill(ColorGeometry4f(usd_color[0], usd_color[1], usd_color[2], 1.0f));
      }
      else if (interp == pxr::UsdGeomTokens->vertex) {
        for (int i = 0; i < colors.size(); ++i) {
          const pxr::GfVec3f &usd_color = colors[i];
          primvar_writer.span[i] = ColorGeometry4f(usd_color[0], usd_color[1], usd_color[2], 1.0f);
        }
      }

      primvar_writer.finish();
    }

    /* TODO: Need to handle the other data types. */
    if (type == pxr::SdfValueTypeNames->FloatArray) {
      bke::SpanAttributeWriter<float> primvar_writer =
          point_cloud->attributes_for_write().lookup_or_add_for_write_span<float>(
              name.GetText(), bke::AttrDomain::Point);
      if (!primvar_writer) {
        CLOG_WARN(&LOG, "Couldn't make writer for float prop %s", name.GetText());
        continue;
      }
      pxr::VtArray<float> values;
      if (!pv.ComputeFlattened(&values, motionSampleTime)) {
        CLOG_WARN(&LOG, "Couldn't compute the flattened float prop %s", name.GetText());
        continue;
      }

      /* TODO: Do we need to handle any other interpolation modes? */
      if (interp == pxr::UsdGeomTokens->constant) {
        primvar_writer.span.fill(values[0]);
      }
      else if (interp == pxr::UsdGeomTokens->vertex) {
        primvar_writer.span.copy_from(Span(values.data(), values.size()));
      }
      primvar_writer.finish();
    }
  }
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
