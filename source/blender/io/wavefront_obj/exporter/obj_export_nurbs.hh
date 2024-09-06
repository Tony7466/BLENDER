/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup obj
 */

#pragma once

#include "BLI_utility_mixins.hh"

#include "DNA_curve_types.h"

namespace blender::io::obj {

/**
 * Provides access to the a Curve Object's properties.
 * Only #CU_NURBS type is supported.
 *
 * \note Used for Curves to be exported in parameter form, and not converted to meshes.
 */
class OBJCurve : NonCopyable {
 private:
  const Object *export_object_eval_;
  const Curve *export_curve_;
  float world_axes_transform_[4][4];

 public:
  OBJCurve(const Depsgraph *depsgraph, const OBJExportParams &export_params, Object *curve_object);

  ObjectType get_object_type() const;
  const char *get_curve_name() const;
  short get_curve_type() const;
  int total_splines() const;
  const Nurb * get_nurb(int spline_idx) const;
  /**
   * \param spline_index: Zero-based index of spline of interest.
   * \return Total vertices in a spline.
   */
  int total_spline_vertices(int spline_index) const;
  /**
   * Get coordinates of the vertex at the given index on the given spline.
   */
  float4 vertex_coordinates(int spline_index, int vertex_index, float global_scale) const;
  /**
   * \param spline_index: Zero-based index of spline of interest.
   * \return: Number of control point on the requested axis of the surface.
   */
  int spline_control_points(const int spline_index, int uv) const;
  /**
   * Get total control points of the NURBS spline at the given index along a specific axis.
   * This is different than total vertices of a spline.
   */
  int total_spline_control_points(int spline_index, int uv) const;
  /**
   * Get the degree of the NURBS spline at the given index.
   */
  std::pair<int, int> get_nurbs_degree(int spline_index) const;
  /**
   * Get the U flags (CU_NURB_*) of the NURBS spline at the given index.
   */
  short get_nurbs_flags(int spline_index, int uv) const;

 private:
  /**
   * Set the final transform after applying axes settings and an Object's world transform.
   */
  void set_world_axes_transform(eIOAxis forward, eIOAxis up);
};

}  // namespace blender::io::obj
