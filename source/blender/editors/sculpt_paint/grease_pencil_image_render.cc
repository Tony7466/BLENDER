/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "BKE_camera.h"
#include "BKE_image.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "GPU_framebuffer.hh"
#include "GPU_immediate.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_format.hh"

namespace blender::ed::greasepencil::image_render {

struct ImageRenderData {
  GPUOffScreen *offscreen;
  int2 region_winsize;
  rcti region_winrct;
};

ImageRenderData *image_render_begin(ARegion &region, const int2 &win_size)
{
  char err_out[256] = "unknown";
  GPUOffScreen *offscreen = GPU_offscreen_create(
      win_size.x, win_size.y, true, GPU_RGBA8, GPU_TEXTURE_USAGE_HOST_READ, err_out);
  if (offscreen == nullptr) {
    return nullptr;
  }

  const int2 region_winsize = {region.winx, region.winy};
  const rcti region_winrct = region.winrct;

  /* Resize region. */
  region.winrct.xmin = 0;
  region.winrct.ymin = 0;
  region.winrct.xmax = win_size.x;
  region.winrct.ymax = win_size.y;
  region.winx = short(win_size.x);
  region.winy = short(win_size.y);

  GPU_offscreen_bind(offscreen, true);

  GPU_matrix_push_projection();
  GPU_matrix_identity_projection_set();
  GPU_matrix_push();
  GPU_matrix_identity_set();

  GPU_clear_color(0.0f, 0.0f, 0.0f, 0.0f);
  GPU_clear_depth(1.0f);

  return new ImageRenderData{offscreen, region_winsize, region_winrct};
}

Image *image_render_end(Main &bmain, ARegion &region, ImageRenderData *data)
{
  const int2 win_size = {GPU_offscreen_width(data->offscreen),
                         GPU_offscreen_height(data->offscreen)};
  /* create a image to see result of template */
  const uint imb_flag = IB_rect;
  ImBuf *ibuf = IMB_allocImBuf(win_size.x, win_size.y, 32, imb_flag);
  if (ibuf->float_buffer.data) {
    GPU_offscreen_read_color(data->offscreen, GPU_DATA_FLOAT, ibuf->float_buffer.data);
  }
  else if (ibuf->byte_buffer.data) {
    GPU_offscreen_read_color(data->offscreen, GPU_DATA_UBYTE, ibuf->byte_buffer.data);
  }
  if (ibuf->float_buffer.data && ibuf->byte_buffer.data) {
    IMB_rect_from_float(ibuf);
  }

  Image *ima = BKE_image_add_from_imbuf(&bmain, ibuf, "Grease Pencil Fill");
  ima->id.tag |= LIB_TAG_DOIT;

  BKE_image_release_ibuf(ima, ibuf, nullptr);

  /* Switch back to window-system-provided frame-buffer. */
  GPU_offscreen_unbind(data->offscreen, true);
  GPU_offscreen_free(data->offscreen);

  region.winx = data->region_winsize.x;
  region.winy = data->region_winsize.y;
  region.winrct = data->region_winrct;

  delete data;

  return ima;
}

void set_viewmat(ARegion &region,
                 View3D &view3d,
                 RegionView3D &rv3d,
                 Depsgraph &depsgraph,
                 const Scene &scene,
                 const int2 &win_size,
                 const float2 &zoom,
                 const float2 &offset)
{
  // CameraParams camera_params;
  // BKE_camera_params_init(&camera_params);
  // BKE_camera_params_from_view3d(&camera_params, &depsgraph, &view3d, &rv3d);
  // BKE_camera_params_compute_viewplane(&camera_params, win_size.x, win_size.y, 1.0f, 1.0f);
  // BKE_camera_params_compute_matrix(&camera_params);

  rctf viewplane;
  float clip_start, clip_end;
  const bool is_ortho = ED_view3d_viewplane_get(&depsgraph,
                                                &view3d,
                                                &rv3d,
                                                win_size.x,
                                                win_size.y,
                                                &viewplane,
                                                &clip_start,
                                                &clip_end,
                                                nullptr);

  /* Rescale `viewplane` to fit all strokes. */
  viewplane.xmin = viewplane.xmin * zoom.x + offset.x;
  viewplane.xmax = viewplane.xmax * zoom.x + offset.x;
  viewplane.ymin = viewplane.ymin * zoom.y + offset.y;
  viewplane.ymax = viewplane.ymax * zoom.y + offset.y;

  float4x4 winmat;
  if (is_ortho) {
    orthographic_m4(winmat.ptr(),
                    viewplane.xmin,
                    viewplane.xmax,
                    viewplane.ymin,
                    viewplane.ymax,
                    -clip_end,
                    clip_end);
  }
  else {
    perspective_m4(winmat.ptr(),
                   viewplane.xmin,
                   viewplane.xmax,
                   viewplane.ymin,
                   viewplane.ymax,
                   clip_start,
                   clip_end);
  }

  ED_view3d_update_viewmat(
      &depsgraph, &scene, &view3d, &region, nullptr, winmat.ptr(), nullptr, true);
  GPU_matrix_set(rv3d.viewmat);
  GPU_matrix_projection_set(rv3d.winmat);
}

void clear_viewmat()
{
  GPU_matrix_pop_projection();
  GPU_matrix_pop();
}

void draw_dot(const float3 &position, const float point_size, const ColorGeometry4f &color)
{
  GPUVertFormat *format = immVertexFormat();
  uint attr_pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint attr_size = GPU_vertformat_attr_add(format, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  uint attr_color = GPU_vertformat_attr_add(format, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  /* Draw mouse click position in Blue. */
  GPU_program_point_size(true);
  immBindBuiltinProgram(GPU_SHADER_3D_POINT_VARYING_SIZE_VARYING_COLOR);
  immBegin(GPU_PRIM_POINTS, 1);
  immAttr1f(attr_size, point_size * M_SQRT2);
  immAttr4fv(attr_color, color);
  immVertex3fv(attr_pos, position);
  immEnd();
  immUnbindProgram();
  GPU_program_point_size(false);
}

/* Draw a line from points. */
void draw_curve(Span<float3> positions,
                const VArray<ColorGeometry4f> &colors,
                const IndexRange indices,
                const float4x4 &layer_to_world,
                const bool cyclic,
                const float line_width)
{
  GPUVertFormat *format = immVertexFormat();
  uint attr_pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint attr_color = GPU_vertformat_attr_add(format, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_FLAT_COLOR);

  /* draw stroke curve */
  GPU_line_width(line_width);
  /* If cyclic needs one more vertex. */
  const int cyclic_add = (cyclic && indices.size() > 2) ? 1 : 0;
  immBeginAtMost(GPU_PRIM_LINE_STRIP, indices.size() + cyclic_add);

  for (const int point_i : indices) {
    immAttr4fv(attr_color, colors[point_i]);
    immVertex3fv(attr_pos, math::transform_point(layer_to_world, positions[point_i]));
  }

  if (cyclic && indices.size() > 2) {
    const int point_i = indices[0];
    immAttr4fv(attr_color, colors[point_i]);
    immVertex3fv(attr_pos, math::transform_point(layer_to_world, positions[point_i]));
  }

  immEnd();
  immUnbindProgram();
}

}  // namespace blender::ed::greasepencil::image_render
