/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup obj
 */

#include "BKE_lib_id.hh"
#include <algorithm>

#include "BKE_object.hh"

#include "BLI_listbase.h"
#include "BLI_math_vector.h"

#include "DNA_curve_types.h"

#include "IO_wavefront_obj.hh"
#include "importer_mesh_utils.hh"
#include "obj_import_nurbs.hh"
#include "obj_import_objects.hh"

namespace blender::io::obj {

static const std::string untitled = "Untitled";

Curve *blender::io::obj::CurveFromGeometry::create_curve()
{
  BLI_assert(!curve_geometry_.nurbs_element_.curv_indices.is_empty());

  Curve *curve = static_cast<Curve *>(BKE_id_new_nomain(ID_CU_LEGACY, nullptr));

  BKE_curve_init(curve, OB_CURVES_LEGACY);

  curve->flag = CU_3D;
  curve->resolu = curve->resolv = 12;
  /* Only one NURBS spline will be created in the curve object. */
  curve->actnu = 0;

  Nurb *nurb = static_cast<Nurb *>(MEM_callocN(sizeof(Nurb), __func__));
  BLI_addtail(BKE_curve_nurbs_get(curve), nurb);
  this->create_nurbs(curve);

  return curve;
}

Object *CurveFromGeometry::create_curve_object(Main *bmain, const OBJImportParams &import_params)
{
  std::string ob_name = get_geometry_name(curve_geometry_.geometry_name_,
                                          import_params.collection_separator);
  if (ob_name.empty() && !curve_geometry_.nurbs_element_.group_.empty()) {
    ob_name = curve_geometry_.nurbs_element_.group_;
  }
  if (ob_name.empty()) {
    ob_name = untitled;
  }
  BLI_assert(!curve_geometry_.nurbs_element_.curv_indices.is_empty());

  Curve *curve = BKE_curve_add(bmain, ob_name.c_str(), OB_CURVES_LEGACY);
  Object *obj = BKE_object_add_only_object(bmain, OB_CURVES_LEGACY, ob_name.c_str());

  curve->flag = CU_3D;
  curve->resolu = curve->resolv = 12;
  /* Only one NURBS spline will be created in the curve object. */
  curve->actnu = 0;

  Nurb *nurb = static_cast<Nurb *>(MEM_callocN(sizeof(Nurb), __func__));
  BLI_addtail(BKE_curve_nurbs_get(curve), nurb);
  this->create_nurbs(curve);

  obj->data = curve;
  transform_object(obj, import_params);

  return obj;
}

Object *CurveFromGeometry::create_surf(Main *bmain, const OBJImportParams &import_params)
{
  std::string ob_name = get_geometry_name(curve_geometry_.geometry_name_,
                                          import_params.collection_separator);
  if (ob_name.empty() && !curve_geometry_.nurbs_element_.group_.empty()) {
    ob_name = curve_geometry_.nurbs_element_.group_;
  }
  if (ob_name.empty()) {
    ob_name = untitled;
  }
  BLI_assert(!curve_geometry_.nurbs_element_.curv_indices.is_empty());

  Curve *curve = BKE_curve_add(bmain, ob_name.c_str(), OB_SURF);
  Object *obj = BKE_object_add_only_object(bmain, OB_SURF, ob_name.c_str());
  curve->flag |= CU_3D;
  curve->resolu = curve->resolv = 12;
  /* Only one NURBS spline will be created in the curve object. */
  curve->actnu = 0;

  Nurb *nurb = static_cast<Nurb *>(MEM_callocN(sizeof(Nurb), "OBJ import NURBS curve"));
  BLI_addtail(BKE_curve_nurbs_get(curve), nurb);
  create_nurbs(curve);

  obj->data = curve;
  transform_object(obj, import_params);

  return obj;
}

void CurveFromGeometry::create_nurbs(Curve *curve)
{
  const NurbsElement &nurbs_geometry = curve_geometry_.nurbs_element_;
  Nurb *nurb = static_cast<Nurb *>(curve->nurb.first);

  nurb->type = CU_NURBS;
  nurb->flag = CU_3D;
  nurb->next = nurb->prev = nullptr;
  nurb->resolu = curve->resolu;
  nurb->resolv = curve->resolv;
  nurb->orderu = nurbs_geometry.u.degree + 1;
  nurb->orderv = nurbs_geometry.v.degree + 1;

  const int64_t tot_vert{nurbs_geometry.curv_indices.size()};

  BKE_nurb_points_add(nurb, tot_vert);
  if(GEOM_SURF == curve_geometry_.geom_type_) {
    nurb->pntsu = nurbs_geometry.u.parms.size() - nurb->orderu;
    nurb->pntsv = 1;
  } else
  {
    /* pntsu was set in BKE_nurb_points_add. */
    nurb->pntsv = 1;
  }
  BLI_assert(tot_vert == nurb->pntsu * nurb->pntsv);

  for (int i = 0; i < tot_vert; i++) {
    BPoint &bpoint = nurb->bp[i];
    copy_v3_v3(bpoint.vec, global_vertices_.vertices[nurbs_geometry.curv_indices[i]]);
    bpoint.vec[3] = curve_geometry_.rational_ ?
                        global_vertices_.weights[nurbs_geometry.curv_indices[i]] :
                        1.f;
    bpoint.weight = 1.0f;
  }
  bool do_endpoints = false;
  struct {
    int degree;
    short &flag;
    const Vector<float> &parm;
    const float2 &range;
  } do_uv[] = {
      {nurbs_geometry.u.degree, nurb->flagu, nurbs_geometry.u.parms, nurbs_geometry.u.range},
      {nurbs_geometry.v.degree, nurb->flagv, nurbs_geometry.v.parms, nurbs_geometry.v.range}};

  BKE_nurb_knot_set_u(nurb, nurbs_geometry.u.parms);
  BKE_nurb_knot_set_v(nurb, nurbs_geometry.v.parms);
  /* Figure out whether curve should have U endpoint flag set:
   * the parameters should have at least (degree+1) values on each end,
   * and their values should match curve range. */
  for (int douv_i = 0; ARRAY_SIZE(do_uv) != douv_i; ++douv_i) {
    int deg1 = do_uv[douv_i].degree + 1;
    if (do_uv[douv_i].parm.size() >= deg1 * 2) {
      do_endpoints = true;
      const float2 &range = do_uv[douv_i].range;
      for (int i = 0; i < deg1; ++i) {
        if (abs(do_uv[douv_i].parm[i] - range.x) > 0.0001f) {
          do_endpoints = false;
          break;
        }
        if (abs(do_uv[douv_i].parm[do_uv[douv_i].parm.size() - 1 - i] - range.y) > 0.0001f) {
          do_endpoints = false;
          break;
        }
      }
    }
    if (do_endpoints) {
      do_uv[douv_i].flag = CU_NURB_ENDPOINT;
    }
  }
}

}  // namespace blender::io::obj
