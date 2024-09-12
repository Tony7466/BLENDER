/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_vfont.hh"
#include "BLI_math_matrix.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class EditText {

 private:
  PassSimple ps_ = {"Selection&Cursor"};

  View view_edit_text = {"view_edit_text"};
  float view_dist = 0.0f;

  struct CallBuffers {
    StorageVectorBuffer<ObjectMatrices> text_selection_buf;
    StorageVectorBuffer<ObjectMatrices> text_cursor_buf;
  } call_buffers_;

  bool enabled_ = false;

 public:
  void begin_sync(const State &state)
  {
    enabled_ = state.v3d;
    call_buffers_.text_selection_buf.clear();
    call_buffers_.text_cursor_buf.clear();
  }

  void edit_object_sync(const ObjectRef &ob_ref)
  {
    if (!enabled_) {
      return;
    }

    gpu::Batch *geom = DRW_cache_text_edge_wire_get(ob_ref.object);
    if (geom) {
    }
    const Curve &cu = *static_cast<Curve *>(ob_ref.object->data);
    edit_text_add_select(cu, ob_ref.object->object_to_world());
    edit_text_add_cursor(cu, ob_ref.object->object_to_world());
  }

  void end_sync(Resources &res, const ShapeCache &shapes, const State &state)
  {
    ps_.init();
    {
      DRWState default_state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA;
      auto &sub = ps_.sub("text_selection");
      sub.state_set(default_state, state.clipping_plane_count);
      sub.shader_set(res.shaders.uniform_color_batch.get());
      float4 color;

      /* Selection Boxes. */
      {
        UI_GetThemeColor4fv(TH_WIDGET_TEXT_SELECTION, color);
        srgb_to_linearrgb_v4(color, color);
        sub.push_constant("ucolor", color);

        auto &buf = call_buffers_.text_selection_buf;
        buf.push_update();
        sub.bind_ssbo("matrix_buf", &buf);
        sub.draw(shapes.quad_solid.get(), buf.size());
      }

      /* Highlight text within selection boxes. */
      {
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA |
                          DRW_STATE_DEPTH_GREATER_EQUAL,
                      state.clipping_plane_count);
        UI_GetThemeColor4fv(TH_WIDGET_TEXT_HIGHLIGHT, color);
        srgb_to_linearrgb_v4(color, color);
        sub.push_constant("ucolor", color);

        auto &buf = call_buffers_.text_selection_buf;
        buf.push_update();
        sub.bind_ssbo("matrix_buf", &buf);
        sub.draw(shapes.quad_solid.get(), buf.size());
      }

      /* Cursor (text caret). */
      {
        sub.state_set(default_state, state.clipping_plane_count);
        UI_GetThemeColor4fv(TH_WIDGET_TEXT_CURSOR, color);
        srgb_to_linearrgb_v4(color, color);
        sub.push_constant("ucolor", color);
        auto &buf = call_buffers_.text_cursor_buf;
        buf.push_update();
        sub.bind_ssbo("matrix_buf", &buf);
        sub.draw(shapes.quad_solid.get(), buf.size());
      }
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }
    view_edit_text.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, -5.0f));
    manager.submit(ps_, view_edit_text);
  }

 private:
  /* Use 2D quad corners to create a matrix that set
   * a [-1..1] quad at the right position. */
  static void v2_quad_corners_to_mat4(const float4x2 &corners, float4x4 &r_mat)
  {
    r_mat = math::from_scale<float4x4>(float4(1.0f));
    auto r_mat_view = r_mat.view<4, 2>();
    r_mat_view[0] = corners[1] - corners[0];
    r_mat_view[1] = corners[3] - corners[0];
    r_mat_view[0] *= 0.5f;
    r_mat_view[1] *= 0.5f;
    r_mat_view[3] = corners[0];
    r_mat_view[3] += r_mat_view[0];
    r_mat_view[3] += r_mat_view[1];
  }

  void edit_text_add_select(const Curve &cu, const float4x4 &ob_to_world)
  {
    EditFont *ef = cu.editfont;
    float4x4 final_mat;
    float4x2 box;

    for (const int i : IndexRange(ef->selboxes_len)) {
      EditFontSelBox *sb = &ef->selboxes[i];

      float selboxw;
      if (i + 1 != ef->selboxes_len) {
        if (ef->selboxes[i + 1].y == sb->y) {
          selboxw = ef->selboxes[i + 1].x - sb->x;
        }
        else {
          selboxw = sb->w;
        }
      }
      else {
        selboxw = sb->w;
      }
      /* NOTE: v2_quad_corners_to_mat4 don't need the 3rd corner. */
      if (sb->rot == 0.0f) {
        box[0] = float2(sb->x, sb->y);
        box[1] = float2(sb->x + selboxw, sb->y);
        box[3] = float2(sb->x, sb->y + sb->h);
      }
      else {
        float2x2 mat = math::from_rotation<float2x2>(sb->rot);
        box[0] = float2(sb->x, sb->y);
        box[1] = mat[0] * selboxw;
        box[1] += float2(&sb->x);
        box[3] = mat[1] * sb->h;
        box[3] += float2(&sb->x);
      }
      v2_quad_corners_to_mat4(box, final_mat);
      final_mat = ob_to_world * final_mat;
      ObjectMatrices obj_mat;
      obj_mat.sync(final_mat);
      call_buffers_.text_selection_buf.append(obj_mat);
    }
  }

  void edit_text_add_cursor(const Curve &cu, const float4x4 &ob_to_world)
  {
    EditFont *edit_font = cu.editfont;
    float4x2 cursor = float4x2(&edit_font->textcurs[0][0]);
    float4x4 mat;

    v2_quad_corners_to_mat4(cursor, mat);
    ObjectMatrices ob_mat;
    ob_mat.sync(ob_to_world * mat);
    call_buffers_.text_cursor_buf.append(ob_mat);
  }
};
}  // namespace blender::draw::overlay
