/** \file
 * \ingroup edsculpt
 * Common helper methods and structures for gesture operations.
 */
#include "MEM_guardedalloc.h"

#include "DNA_vec_types.h"

#include "BLI_bit_vector.hh"
#include "BLI_bitmap_draw_2d.h"
#include "BLI_lasso_2d.h"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_paint.hh"

#include "ED_view3d.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "paint_intern.hh"
#include "sculpt_intern.hh"

namespace blender::ed::sculpt_paint::gesture {

void operator_properties(wmOperatorType *ot)
{
  RNA_def_boolean(ot->srna,
                  "use_front_faces_only",
                  false,
                  "Front Faces Only",
                  "Affect only faces facing towards the view");

  RNA_def_boolean(ot->srna,
                  "use_limit_to_segment",
                  false,
                  "Limit to Segment",
                  "Apply the gesture action only to the area that is contained within the "
                  "segment without extending its effect to the entire line");
}

static void init_common(bContext *C, wmOperator *op, Context *sgcontext)
{
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  sgcontext->vc = ED_view3d_viewcontext_init(C, depsgraph);
  Object *ob = sgcontext->vc.obact;

  /* Operator properties. */
  sgcontext->front_faces_only = RNA_boolean_get(op->ptr, "use_front_faces_only");
  sgcontext->line.use_side_planes = RNA_boolean_get(op->ptr, "use_limit_to_segment");

  /* SculptSession */
  sgcontext->ss = ob->sculpt;

  /* Symmetry. */
  sgcontext->symm = ePaintSymmetryFlags(SCULPT_mesh_symmetry_xyz_get(ob));

  /* View Normal. */
  float mat[3][3];
  float view_dir[3] = {0.0f, 0.0f, 1.0f};
  copy_m3_m4(mat, sgcontext->vc.rv3d->viewinv);
  mul_m3_v3(mat, view_dir);
  normalize_v3_v3(sgcontext->world_space_view_normal, view_dir);
  copy_m3_m4(mat, ob->world_to_object().ptr());
  mul_m3_v3(mat, view_dir);
  normalize_v3_v3(sgcontext->true_view_normal, view_dir);

  /* View Origin. */
  copy_v3_v3(sgcontext->world_space_view_origin, sgcontext->vc.rv3d->viewinv[3]);
  copy_v3_v3(sgcontext->true_view_origin, sgcontext->vc.rv3d->viewinv[3]);
}

static void lasso_px_cb(int x, int x_end, int y, void *user_data)
{
  Context *mcontext = static_cast<Context *>(user_data);
  LassoData *lasso = &mcontext->lasso;
  int index = (y * lasso->width) + x;
  int index_end = (y * lasso->width) + x_end;
  do {
    lasso->mask_px[index].set();
  } while (++index != index_end);
}

Context *init_from_lasso(bContext *C, wmOperator *op)
{
  Context *sgcontext = MEM_new<Context>(__func__);
  sgcontext->shape_type = SCULPT_GESTURE_SHAPE_LASSO;

  init_common(C, op, sgcontext);

  int mcoords_len;
  const int(*mcoords)[2] = WM_gesture_lasso_path_to_array(C, op, &mcoords_len);

  if (!mcoords) {
    return nullptr;
  }

  sgcontext->lasso.projviewobjmat = ED_view3d_ob_project_mat_get(sgcontext->vc.rv3d,
                                                                 sgcontext->vc.obact);
  BLI_lasso_boundbox(&sgcontext->lasso.boundbox, mcoords, mcoords_len);
  const int lasso_width = 1 + sgcontext->lasso.boundbox.xmax - sgcontext->lasso.boundbox.xmin;
  const int lasso_height = 1 + sgcontext->lasso.boundbox.ymax - sgcontext->lasso.boundbox.ymin;
  sgcontext->lasso.width = lasso_width;
  sgcontext->lasso.mask_px.resize(lasso_width * lasso_height);

  BLI_bitmap_draw_2d_poly_v2i_n(sgcontext->lasso.boundbox.xmin,
                                sgcontext->lasso.boundbox.ymin,
                                sgcontext->lasso.boundbox.xmax,
                                sgcontext->lasso.boundbox.ymax,
                                mcoords,
                                mcoords_len,
                                lasso_px_cb,
                                sgcontext);

  BoundBox bb;
  ED_view3d_clipping_calc(&bb,
                          sgcontext->true_clip_planes,
                          sgcontext->vc.region,
                          sgcontext->vc.obact,
                          &sgcontext->lasso.boundbox);

  sgcontext->gesture_points = static_cast<float(*)[2]>(
      MEM_malloc_arrayN(mcoords_len, sizeof(float[2]), "trim points"));
  sgcontext->tot_gesture_points = mcoords_len;
  for (int i = 0; i < mcoords_len; i++) {
    sgcontext->gesture_points[i][0] = mcoords[i][0];
    sgcontext->gesture_points[i][1] = mcoords[i][1];
  }

  MEM_freeN((void *)mcoords);

  return sgcontext;
}

Context *init_from_box(bContext *C, wmOperator *op)
{
  Context *sgcontext = MEM_new<Context>(__func__);
  sgcontext->shape_type = SCULPT_GESTURE_SHAPE_BOX;

  init_common(C, op, sgcontext);

  rcti rect;
  WM_operator_properties_border_to_rcti(op, &rect);

  BoundBox bb;
  ED_view3d_clipping_calc(
      &bb, sgcontext->true_clip_planes, sgcontext->vc.region, sgcontext->vc.obact, &rect);

  sgcontext->gesture_points = static_cast<float(*)[2]>(
      MEM_calloc_arrayN(4, sizeof(float[2]), "trim points"));
  sgcontext->tot_gesture_points = 4;

  sgcontext->gesture_points[0][0] = rect.xmax;
  sgcontext->gesture_points[0][1] = rect.ymax;

  sgcontext->gesture_points[1][0] = rect.xmax;
  sgcontext->gesture_points[1][1] = rect.ymin;

  sgcontext->gesture_points[2][0] = rect.xmin;
  sgcontext->gesture_points[2][1] = rect.ymin;

  sgcontext->gesture_points[3][0] = rect.xmin;
  sgcontext->gesture_points[3][1] = rect.ymax;
  return sgcontext;
}

static void line_plane_from_tri(float *r_plane,
                                Context *sgcontext,
                                const bool flip,
                                const float p1[3],
                                const float p2[3],
                                const float p3[3])
{
  float normal[3];
  normal_tri_v3(normal, p1, p2, p3);
  mul_v3_mat3_m4v3(normal, sgcontext->vc.obact->world_to_object().ptr(), normal);
  if (flip) {
    mul_v3_fl(normal, -1.0f);
  }
  float plane_point_object_space[3];
  mul_v3_m4v3(plane_point_object_space, sgcontext->vc.obact->world_to_object().ptr(), p1);
  plane_from_point_normal_v3(r_plane, plane_point_object_space, normal);
}

/* Creates 4 points in the plane defined by the line and 2 extra points with an offset relative to
 * this plane. */
static void line_calculate_plane_points(Context *sgcontext,
                                        float line_points[2][2],
                                        float r_plane_points[4][3],
                                        float r_offset_plane_points[2][3])
{
  float depth_point[3];
  add_v3_v3v3(depth_point, sgcontext->true_view_origin, sgcontext->true_view_normal);
  ED_view3d_win_to_3d(
      sgcontext->vc.v3d, sgcontext->vc.region, depth_point, line_points[0], r_plane_points[0]);
  ED_view3d_win_to_3d(
      sgcontext->vc.v3d, sgcontext->vc.region, depth_point, line_points[1], r_plane_points[3]);

  madd_v3_v3v3fl(depth_point, sgcontext->true_view_origin, sgcontext->true_view_normal, 10.0f);
  ED_view3d_win_to_3d(
      sgcontext->vc.v3d, sgcontext->vc.region, depth_point, line_points[0], r_plane_points[1]);
  ED_view3d_win_to_3d(
      sgcontext->vc.v3d, sgcontext->vc.region, depth_point, line_points[1], r_plane_points[2]);

  float normal[3];
  normal_tri_v3(normal, r_plane_points[0], r_plane_points[1], r_plane_points[2]);
  add_v3_v3v3(r_offset_plane_points[0], r_plane_points[0], normal);
  add_v3_v3v3(r_offset_plane_points[1], r_plane_points[3], normal);
}

Context *init_from_line(bContext *C, wmOperator *op)
{
  Context *sgcontext = MEM_new<Context>(__func__);
  sgcontext->shape_type = SCULPT_GESTURE_SHAPE_LINE;

  init_common(C, op, sgcontext);

  float line_points[2][2];
  line_points[0][0] = RNA_int_get(op->ptr, "xstart");
  line_points[0][1] = RNA_int_get(op->ptr, "ystart");
  line_points[1][0] = RNA_int_get(op->ptr, "xend");
  line_points[1][1] = RNA_int_get(op->ptr, "yend");

  sgcontext->line.flip = RNA_boolean_get(op->ptr, "flip");

  float plane_points[4][3];
  float offset_plane_points[2][3];
  line_calculate_plane_points(sgcontext, line_points, plane_points, offset_plane_points);

  /* Calculate line plane and normal. */
  const bool flip = sgcontext->line.flip ^ (!sgcontext->vc.rv3d->is_persp);
  line_plane_from_tri(sgcontext->line.true_plane,
                      sgcontext,
                      flip,
                      plane_points[0],
                      plane_points[1],
                      plane_points[2]);

  /* Calculate the side planes. */
  line_plane_from_tri(sgcontext->line.true_side_plane[0],
                      sgcontext,
                      false,
                      plane_points[1],
                      plane_points[0],
                      offset_plane_points[0]);
  line_plane_from_tri(sgcontext->line.true_side_plane[1],
                      sgcontext,
                      false,
                      plane_points[3],
                      plane_points[2],
                      offset_plane_points[1]);

  return sgcontext;
}

void free(Context *sgcontext)
{
  MEM_SAFE_FREE(sgcontext->gesture_points);
  MEM_SAFE_FREE(sgcontext->operation);
  MEM_delete(sgcontext);
}

static void flip_plane(float out[4], const float in[4], const char symm)
{
  if (symm & PAINT_SYMM_X) {
    out[0] = -in[0];
  }
  else {
    out[0] = in[0];
  }
  if (symm & PAINT_SYMM_Y) {
    out[1] = -in[1];
  }
  else {
    out[1] = in[1];
  }
  if (symm & PAINT_SYMM_Z) {
    out[2] = -in[2];
  }
  else {
    out[2] = in[2];
  }

  out[3] = in[3];
}

static void flip_for_symmetry_pass(Context *sgcontext, const ePaintSymmetryFlags symmpass)
{
  sgcontext->symmpass = symmpass;
  for (int j = 0; j < 4; j++) {
    flip_plane(sgcontext->clip_planes[j], sgcontext->true_clip_planes[j], symmpass);
  }

  negate_m4(sgcontext->clip_planes);

  flip_v3_v3(sgcontext->view_normal, sgcontext->true_view_normal, symmpass);
  flip_v3_v3(sgcontext->view_origin, sgcontext->true_view_origin, symmpass);
  flip_plane(sgcontext->line.plane, sgcontext->line.true_plane, symmpass);
  flip_plane(sgcontext->line.side_plane[0], sgcontext->line.true_side_plane[0], symmpass);
  flip_plane(sgcontext->line.side_plane[1], sgcontext->line.true_side_plane[1], symmpass);
}

static Vector<PBVHNode *> update_affected_nodes_by_line_plane(Context *sgcontext)
{
  SculptSession *ss = sgcontext->ss;
  float clip_planes[3][4];
  copy_v4_v4(clip_planes[0], sgcontext->line.plane);
  copy_v4_v4(clip_planes[1], sgcontext->line.side_plane[0]);
  copy_v4_v4(clip_planes[2], sgcontext->line.side_plane[1]);

  PBVHFrustumPlanes frustum{};
  frustum.planes = clip_planes;
  frustum.num_planes = sgcontext->line.use_side_planes ? 3 : 1;

  return sgcontext->nodes = bke::pbvh::search_gather(ss->pbvh, [&](PBVHNode &node) {
           return BKE_pbvh_node_frustum_contain_AABB(&node, &frustum);
         });
}

static void update_affected_nodes_by_clip_planes(Context *sgcontext)
{
  SculptSession *ss = sgcontext->ss;
  float clip_planes[4][4];
  copy_m4_m4(clip_planes, sgcontext->clip_planes);
  negate_m4(clip_planes);

  PBVHFrustumPlanes frustum{};
  frustum.planes = clip_planes;
  frustum.num_planes = 4;

  sgcontext->nodes = bke::pbvh::search_gather(ss->pbvh, [&](PBVHNode &node) {
    return BKE_pbvh_node_frustum_contain_AABB(&node, &frustum);
  });
}

static void update_affected_nodes(Context *sgcontext)
{
  switch (sgcontext->shape_type) {
    case SCULPT_GESTURE_SHAPE_BOX:
    case SCULPT_GESTURE_SHAPE_LASSO:
      update_affected_nodes_by_clip_planes(sgcontext);
      break;
    case SCULPT_GESTURE_SHAPE_LINE:
      update_affected_nodes_by_line_plane(sgcontext);
      break;
  }
}

static bool is_affected_lasso(Context *sgcontext, const float co[3])
{
  int scr_co_s[2];
  float co_final[3];

  flip_v3_v3(co_final, co, sgcontext->symmpass);

  /* First project point to 2d space. */
  const float2 scr_co_f = ED_view3d_project_float_v2_m4(
      sgcontext->vc.region, co_final, sgcontext->lasso.projviewobjmat);

  scr_co_s[0] = scr_co_f[0];
  scr_co_s[1] = scr_co_f[1];

  /* Clip against lasso boundbox. */
  LassoData *lasso = &sgcontext->lasso;
  if (!BLI_rcti_isect_pt(&lasso->boundbox, scr_co_s[0], scr_co_s[1])) {
    return false;
  }

  scr_co_s[0] -= lasso->boundbox.xmin;
  scr_co_s[1] -= lasso->boundbox.ymin;

  return lasso->mask_px[scr_co_s[1] * lasso->width + scr_co_s[0]].test();
}

bool is_affected(Context *sgcontext, const float3 &co, const float3 &vertex_normal)
{
  float dot = dot_v3v3(sgcontext->view_normal, vertex_normal);
  const bool is_effected_front_face = !(sgcontext->front_faces_only && dot < 0.0f);

  if (!is_effected_front_face) {
    return false;
  }

  switch (sgcontext->shape_type) {
    case SCULPT_GESTURE_SHAPE_BOX:
      return isect_point_planes_v3(sgcontext->clip_planes, 4, co);
    case SCULPT_GESTURE_SHAPE_LASSO:
      return is_affected_lasso(sgcontext, co);
    case SCULPT_GESTURE_SHAPE_LINE:
      if (sgcontext->line.use_side_planes) {
        return plane_point_side_v3(sgcontext->line.plane, co) > 0.0f &&
               plane_point_side_v3(sgcontext->line.side_plane[0], co) > 0.0f &&
               plane_point_side_v3(sgcontext->line.side_plane[1], co) > 0.0f;
      }
      return plane_point_side_v3(sgcontext->line.plane, co) > 0.0f;
  }
  return false;
}

void apply(bContext *C, Context *sgcontext, wmOperator *op)
{
  Operation *operation = sgcontext->operation;
  undo::push_begin(CTX_data_active_object(C), op);

  operation->begin(C, sgcontext);

  for (int symmpass = 0; symmpass <= sgcontext->symm; symmpass++) {
    if (SCULPT_is_symmetry_iteration_valid(symmpass, sgcontext->symm)) {
      flip_for_symmetry_pass(sgcontext, ePaintSymmetryFlags(symmpass));
      update_affected_nodes(sgcontext);

      operation->apply_for_symmetry_pass(C, sgcontext);
    }
  }

  operation->end(C, sgcontext);

  Object *ob = CTX_data_active_object(C);
  undo::push_end(ob);

  SCULPT_tag_update_overlays(C);
}
}  // namespace blender::ed::sculpt_paint::gesture
