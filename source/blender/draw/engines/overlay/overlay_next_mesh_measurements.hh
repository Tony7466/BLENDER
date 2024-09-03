/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BLI_math_rotation.h"
#include "BLI_math_vector_types.hh"
#include "BLI_memiter.h"
#include "BLI_string.h"

#include "BKE_editmesh.hh"
#include "BKE_editmesh_cache.hh"
#include "BKE_mesh_types.hh"
#include "BKE_mesh_wrapper.hh"
#include "BKE_unit.hh"

#include "DNA_screen_types.h"

#include "ED_view3d.hh"

#include "UI_interface.hh"

#include "draw_manager_text.hh"
#include "overlay_next_private.hh"

namespace blender::draw::overlay {

constexpr int overlay_edit_text = V3D_OVERLAY_EDIT_EDGE_LEN | V3D_OVERLAY_EDIT_FACE_AREA |
                                  V3D_OVERLAY_EDIT_FACE_ANG | V3D_OVERLAY_EDIT_EDGE_ANG |
                                  V3D_OVERLAY_EDIT_INDICES;

class MeshMeasurements {
 public:
  static void edit_object_sync(const ObjectRef &ob_ref, State &state)
  {
    if (!(DRW_state_show_text() && (state.overlay.edit_flag & overlay_edit_text))) {
      return;
    }
    /* Do not use ascii when using non-default unit system, some unit chars are utf8 (micro,
     * square, etc.). See bug #36090.
     */
    const View3D *v3d = state.v3d;
    const ARegion *region = state.region;
    const UnitSettings &unit = state.scene->unit;
    Object *ob = ob_ref.object;

    DRWTextStore *dt = DRW_text_cache_ensure();
    const short txt_flag = DRW_TEXT_CACHE_GLOBALSPACE;
    const Mesh *mesh = BKE_object_get_editmesh_eval_cage(ob);
    const BMEditMesh *em = mesh->runtime->edit_mesh.get();
    char numstr[32];             /* Stores the measurement display text here */
    const char *conv_float;      /* Use a float conversion matching the grid size */
    uchar4 col = {0, 0, 0, 255}; /* color of the text to draw */
    const float grid = unit.system ? unit.scale_length : v3d->grid;
    const bool do_global = (v3d->flag & V3D_GLOBAL_STATS) != 0;
    const bool do_moving = (G.moving & G_TRANSFORM_EDIT) != 0;
    float4x4 clip_planes;
    /* allow for displaying shape keys and deform mods */
    BMIter iter;
    const Span<float3> vert_positions = BKE_mesh_wrapper_vert_coords(mesh);
    const bool use_coords = !vert_positions.is_empty();

    /* when 2 or more edge-info options are enabled, space apart */
    short edge_tex_count = 0;
    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_EDGE_LEN) {
      edge_tex_count += 1;
    }
    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_EDGE_ANG) {
      edge_tex_count += 1;
    }
    if ((v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_INDICES) && (em->selectmode & SCE_SELECT_EDGE))
    {
      edge_tex_count += 1;
    }
    const short edge_tex_sep = short((edge_tex_count - 1) * 5.0f * UI_SCALE_FAC);

    /* Make the precision of the display value proportionate to the grid-size. */

    if (grid <= 0.01f) {
      conv_float = "%.6g";
    }
    else if (grid <= 0.1f) {
      conv_float = "%.5g";
    }
    else if (grid <= 1.0f) {
      conv_float = "%.4g";
    }
    else if (grid <= 10.0f) {
      conv_float = "%.3g";
    }
    else {
      conv_float = "%.2g";
    }

    if (v3d->overlay.edit_flag &
        (V3D_OVERLAY_EDIT_EDGE_LEN | V3D_OVERLAY_EDIT_EDGE_ANG | V3D_OVERLAY_EDIT_INDICES))
    {
      BoundBox bb;
      const rcti rect = {0, region->winx, 0, region->winy};

      ED_view3d_clipping_calc(&bb, clip_planes.ptr(), region, ob, &rect);
    }

    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_EDGE_LEN) {
      BMEdge *eed;

      UI_GetThemeColor3ubv(TH_DRAWEXTRA_EDGELEN, col);

      if (use_coords) {
        BM_mesh_elem_index_ensure(em->bm, BM_VERT);
      }

      BM_ITER_MESH (eed, &iter, em->bm, BM_EDGES_OF_MESH) {
        /* draw selected edges, or edges next to selected verts while dragging */
        if (BM_elem_flag_test(eed, BM_ELEM_SELECT) ||
            (do_moving && (BM_elem_flag_test(eed->v1, BM_ELEM_SELECT) ||
                           BM_elem_flag_test(eed->v2, BM_ELEM_SELECT))))
        {
          float3 v1, v2;
          float3 v1_clip, v2_clip;

          if (use_coords) {
            v1 = vert_positions[BM_elem_index_get(eed->v1)];
            v2 = vert_positions[BM_elem_index_get(eed->v2)];
          }
          else {
            v1 = eed->v1->co;
            v2 = eed->v2->co;
          }

          if (clip_segment_v3_plane_n(v1, v2, clip_planes.ptr(), 4, v1_clip, v2_clip)) {
            const float3 co = math::transform_point(ob->object_to_world(),
                                                    0.5 * (v1_clip + v2_clip));

            if (do_global) {
              v1 = ob->object_to_world().view<3, 3>() * v1;
              v2 = ob->object_to_world().view<3, 3>() * v2;
            }

            const size_t numstr_len = unit.system ?
                                          BKE_unit_value_as_string(numstr,
                                                                   sizeof(numstr),
                                                                   len_v3v3(v1, v2) *
                                                                       unit.scale_length,
                                                                   3,
                                                                   B_UNIT_LENGTH,
                                                                   &unit,
                                                                   false) :
                                          SNPRINTF_RLEN(numstr, conv_float, len_v3v3(v1, v2));

            DRW_text_cache_add(dt, co, numstr, numstr_len, 0, edge_tex_sep, txt_flag, col);
          }
        }
      }
    }

    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_EDGE_ANG) {
      const bool is_rad = (unit.system_rotation == USER_UNIT_ROT_RADIANS);
      BMEdge *eed;

      UI_GetThemeColor3ubv(TH_DRAWEXTRA_EDGEANG, col);

      Span<float3> face_normals;
      if (use_coords) {
        BM_mesh_elem_index_ensure(em->bm, BM_VERT | BM_FACE);
        /* TODO: This is not const correct for wrapper meshes, but it should be okay because
         * every evaluated object gets its own evaluated cage mesh (they are not shared). */
        face_normals = BKE_mesh_wrapper_face_normals(const_cast<Mesh *>(mesh));
      }

      BM_ITER_MESH (eed, &iter, em->bm, BM_EDGES_OF_MESH) {
        BMLoop *l_a, *l_b;
        if (BM_edge_loop_pair(eed, &l_a, &l_b)) {
          /* Draw selected edges, or edges next to selected verts while dragging. */
          if (BM_elem_flag_test(eed, BM_ELEM_SELECT) ||
              (do_moving && (BM_elem_flag_test(eed->v1, BM_ELEM_SELECT) ||
                             BM_elem_flag_test(eed->v2, BM_ELEM_SELECT) ||
                             /* Special case, this is useful to show when verts connected
                              * to this edge via a face are being transformed. */
                             BM_elem_flag_test(l_a->next->next->v, BM_ELEM_SELECT) ||
                             BM_elem_flag_test(l_a->prev->v, BM_ELEM_SELECT) ||
                             BM_elem_flag_test(l_b->next->next->v, BM_ELEM_SELECT) ||
                             BM_elem_flag_test(l_b->prev->v, BM_ELEM_SELECT))))
          {
            float3 v1, v2;
            float3 v1_clip, v2_clip;

            if (use_coords) {
              v1 = vert_positions[BM_elem_index_get(eed->v1)];
              v2 = vert_positions[BM_elem_index_get(eed->v2)];
            }
            else {
              v1 = eed->v1->co;
              v2 = eed->v2->co;
            }

            if (clip_segment_v3_plane_n(v1, v2, clip_planes.ptr(), 4, v1_clip, v2_clip)) {
              float3 no_a, no_b;

              const float3 co = math::transform_point(ob->object_to_world(),
                                                      0.5 * (v1_clip + v2_clip));

              if (use_coords) {
                no_a = face_normals[BM_elem_index_get(l_a->f)];
                no_b = face_normals[BM_elem_index_get(l_b->f)];
              }
              else {
                no_a = l_a->f->no;
                no_b = l_b->f->no;
              }

              if (do_global) {
                no_a = math::normalize(ob->object_to_world().view<3, 3>() * no_a);
                no_b = math::normalize(ob->object_to_world().view<3, 3>() * no_b);
              }

              const float angle = angle_normalized_v3v3(no_a, no_b);

              const size_t numstr_len = SNPRINTF_RLEN(numstr,
                                                      "%.3f%s",
                                                      (is_rad) ? angle : RAD2DEGF(angle),
                                                      (is_rad) ? "r" : BLI_STR_UTF8_DEGREE_SIGN);

              DRW_text_cache_add(dt, co, numstr, numstr_len, 0, -edge_tex_sep, txt_flag, col);
            }
          }
        }
      }
    }

    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_FACE_AREA) {
      /* would be nice to use BM_face_calc_area, but that is for 2d faces
       * so instead add up tessellation triangle areas */

      UI_GetThemeColor3ubv(TH_DRAWEXTRA_FACEAREA, col);

      int i;
      BMFace *f = nullptr;
      /* Alternative to using `poly_to_tri_count(i, BM_elem_index_get(f->l_first))`
       * without having to add an extra loop. */
      int tri_index = 0;
      BM_ITER_MESH_INDEX (f, &iter, em->bm, BM_FACES_OF_MESH, i) {
        const int f_corner_tris_len = f->len - 2;
        if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
          int n = 0;
          float area = 0; /* area of the face */
          float3 vmid(0.0f);
          const std::array<BMLoop *, 3> *ltri_array = &em->looptris[tri_index];
          for (int j = 0; j < f_corner_tris_len; j++) {
            float3 v1, v2, v3;

            if (use_coords) {
              v1 = vert_positions[BM_elem_index_get(ltri_array[j][0]->v)];
              v2 = vert_positions[BM_elem_index_get(ltri_array[j][1]->v)];
              v3 = vert_positions[BM_elem_index_get(ltri_array[j][2]->v)];
            }
            else {
              v1 = ltri_array[j][0]->v->co;
              v2 = ltri_array[j][1]->v->co;
              v3 = ltri_array[j][2]->v->co;
            }

            vmid += v1;
            vmid += v2;
            vmid += v3;
            n += 3;

            if (do_global) {
              v1 = ob->object_to_world().view<3, 3>() * v1;
              v2 = ob->object_to_world().view<3, 3>() * v2;
              v3 = ob->object_to_world().view<3, 3>() * v3;
            }

            area += area_tri_v3(v1, v2, v3);
          }

          vmid *= 1.0f / float(n);
          vmid = math::transform_point(ob->object_to_world(), vmid);

          const size_t numstr_len = unit.system ?
                                        BKE_unit_value_as_string(
                                            numstr,
                                            sizeof(numstr),
                                            double(area * unit.scale_length * unit.scale_length),
                                            3,
                                            B_UNIT_AREA,
                                            &unit,
                                            false) :
                                        SNPRINTF_RLEN(numstr, conv_float, area);

          DRW_text_cache_add(dt, vmid, numstr, numstr_len, 0, 0, txt_flag, col);
        }
        tri_index += f_corner_tris_len;
      }
    }

    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_FACE_ANG) {
      BMFace *efa;
      const bool is_rad = (unit.system_rotation == USER_UNIT_ROT_RADIANS);

      UI_GetThemeColor3ubv(TH_DRAWEXTRA_FACEANG, col);

      if (use_coords) {
        BM_mesh_elem_index_ensure(em->bm, BM_VERT);
      }

      BM_ITER_MESH (efa, &iter, em->bm, BM_FACES_OF_MESH) {
        const bool is_face_sel = BM_elem_flag_test_bool(efa, BM_ELEM_SELECT);

        if (is_face_sel || do_moving) {
          BMIter liter;
          BMLoop *loop;
          bool is_first = true;

          BM_ITER_ELEM (loop, &liter, efa, BM_LOOPS_OF_FACE) {
            if (is_face_sel || (do_moving && (BM_elem_flag_test(loop->v, BM_ELEM_SELECT) ||
                                              BM_elem_flag_test(loop->prev->v, BM_ELEM_SELECT) ||
                                              BM_elem_flag_test(loop->next->v, BM_ELEM_SELECT))))
            {
              float3 v1, v2, v3;
              float3 vmid;

              /* lazy init center calc */
              if (is_first) {
                if (use_coords) {
                  BM_face_calc_center_bounds_vcos(em->bm, efa, vmid, vert_positions);
                }
                else {
                  BM_face_calc_center_bounds(efa, vmid);
                }
                is_first = false;
              }
              if (use_coords) {
                v1 = vert_positions[BM_elem_index_get(loop->prev->v)];
                v2 = vert_positions[BM_elem_index_get(loop->v)];
                v3 = vert_positions[BM_elem_index_get(loop->next->v)];
              }
              else {
                v1 = loop->prev->v->co;
                v2 = loop->v->co;
                v3 = loop->next->v->co;
              }

              const float3 v2_local = v2;

              if (do_global) {
                v1 = ob->object_to_world().view<3, 3>() * v1;
                v2 = ob->object_to_world().view<3, 3>() * v2;
                v3 = ob->object_to_world().view<3, 3>() * v3;
              }

              const float angle = angle_v3v3v3(v1, v2, v3);

              const size_t numstr_len = SNPRINTF_RLEN(numstr,
                                                      "%.3f%s",
                                                      (is_rad) ? angle : RAD2DEGF(angle),
                                                      (is_rad) ? "r" : BLI_STR_UTF8_DEGREE_SIGN);
              const float3 co = math::transform_point(ob->object_to_world(),
                                                      math::interpolate(vmid, v2_local, 0.8f));
              DRW_text_cache_add(dt, co, numstr, numstr_len, 0, 0, txt_flag, col);
            }
          }
        }
      }
    }

    /* This option is for mesh ops and addons debugging; only available in UI if Blender starts
     * with
     * --debug */
    if (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_INDICES) {
      int i;

      /* For now, reuse an appropriate theme color */
      UI_GetThemeColor3ubv(TH_DRAWEXTRA_FACEANG, col);

      if (em->selectmode & SCE_SELECT_VERTEX) {
        BMVert *v;

        if (use_coords) {
          BM_mesh_elem_index_ensure(em->bm, BM_VERT);
        }
        BM_ITER_MESH_INDEX (v, &iter, em->bm, BM_VERTS_OF_MESH, i) {
          if (BM_elem_flag_test(v, BM_ELEM_SELECT)) {
            const float3 co = math::transform_point(
                ob->object_to_world(), use_coords ? vert_positions[BM_elem_index_get(v)] : v->co);

            const size_t numstr_len = SNPRINTF_RLEN(numstr, "%d", i);
            DRW_text_cache_add(dt, co, numstr, numstr_len, 0, 0, txt_flag, col);
          }
        }
      }

      if (em->selectmode & SCE_SELECT_EDGE) {
        BMEdge *eed;

        const bool use_edge_tex_sep = (edge_tex_count == 2);
        const bool use_edge_tex_len = (v3d->overlay.edit_flag & V3D_OVERLAY_EDIT_EDGE_LEN);

        BM_ITER_MESH_INDEX (eed, &iter, em->bm, BM_EDGES_OF_MESH, i) {
          if (BM_elem_flag_test(eed, BM_ELEM_SELECT)) {
            float3 v1, v2;
            float3 v1_clip, v2_clip;

            if (use_coords) {
              v1 = vert_positions[BM_elem_index_get(eed->v1)];
              v2 = vert_positions[BM_elem_index_get(eed->v2)];
            }
            else {
              v1 = eed->v1->co;
              v2 = eed->v2->co;
            }

            if (clip_segment_v3_plane_n(v1, v2, clip_planes.ptr(), 4, v1_clip, v2_clip)) {
              const float3 co = math::transform_point(ob->object_to_world(),
                                                      0.5 * (v1_clip + v2_clip));

              const size_t numstr_len = SNPRINTF_RLEN(numstr, "%d", i);
              DRW_text_cache_add(
                  dt,
                  co,
                  numstr,
                  numstr_len,
                  0,
                  (use_edge_tex_sep) ? (use_edge_tex_len) ? -edge_tex_sep : edge_tex_sep : 0,
                  txt_flag,
                  col);
            }
          }
        }
      }

      if (em->selectmode & SCE_SELECT_FACE) {
        BMFace *f;

        if (use_coords) {
          BM_mesh_elem_index_ensure(em->bm, BM_VERT);
        }

        BM_ITER_MESH_INDEX (f, &iter, em->bm, BM_FACES_OF_MESH, i) {
          if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
            float3 co;

            if (use_coords) {
              BM_face_calc_center_median_vcos(em->bm, f, co, vert_positions);
            }
            else {
              BM_face_calc_center_median(f, co);
            }

            co = math::transform_point(ob->object_to_world(), co);

            const size_t numstr_len = SNPRINTF_RLEN(numstr, "%d", i);
            DRW_text_cache_add(dt, co, numstr, numstr_len, 0, 0, txt_flag, col);
          }
        }
      }
    }
  }
};

}  // namespace blender::draw::overlay
