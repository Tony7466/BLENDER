/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_extra_pass.hh"
namespace blender::draw::overlay {

static void light_sync(const ObjectRef &ob_ref,
                       const select::ID select_id,
                       Resources & /*res*/,
                       const State &state,
                       ExtraInstancePass &pass,
                       ExtraInstanceData data)
{
  /* Pack render data into object matrix. */
  float4x4 &matrix = data.matrix;
  float &area_size_x = matrix[0].w;
  float &area_size_y = matrix[1].w;
  float &spot_cosine = matrix[0].w;
  float &spot_blend = matrix[1].w;
  float &clip_start = matrix[2].w;
  float &clip_end = matrix[3].w;

  Object *ob = ob_ref.object;
  Light *la = static_cast<Light *>(ob->data);

  /* FIXME / TODO: clip_end has no meaning nowadays.
   * In EEVEE, Only clip_start is used shadow-mapping.
   * Clip end is computed automatically based on light power.
   * For now, always use the custom distance as clip_end. */
  clip_end = la->att_dist;
  clip_start = la->clipsta;

  /* Remove the alpha. */
  data.color = float4(data.color.xyz(), 1.0f);

  float4 light_color = data.color;
  if (state.overlay.flag & V3D_OVERLAY_SHOW_LIGHT_COLORS) {
    light_color = float4(la->r, la->g, la->b, 1.0f);
  }

  pass.groundline.append(float4(data.matrix.location()), select_id);

  pass.light_icon_inner.append(data.with_color(light_color), select_id);
  pass.light_icon_outer.append(data, select_id);

  if (la->type == LA_LOCAL) {
    area_size_x = area_size_y = la->radius;
    pass.light_point.append(data, select_id);
  }
  else if (la->type == LA_SUN) {
    pass.light_sun.append(data, select_id);
    pass.light_icon_sun_rays.append(data.with_color(light_color), select_id);
  }
  else if (la->type == LA_SPOT) {
    /* Previous implementation was using the clip-end distance as cone size.
     * We cannot do this anymore so we use a fixed size of 10. (see #72871) */
    matrix = math::scale(matrix, float3(10.0f));
    /* For cycles and EEVEE the spot attenuation is:
     * `y = (1/sqrt(1 + x^2) - a)/((1 - a) b)`
     * x being the tangent of the angle between the light direction and the
     * generatrix of the cone. We solve the case where spot attenuation y = 1
     * and y = 0 root for y = 1 is sqrt(1/c^2 - 1) root for y = 0 is
     * sqrt(1/a^2 - 1) and use that to position the blend circle. */
    float a = cosf(la->spotsize * 0.5f);
    float b = la->spotblend;
    float c = a * b - a - b;
    float a2 = a * a;
    float c2 = c * c;
    /* Optimized version or root1 / root0 */
    spot_blend = sqrtf((a2 - a2 * c2) / (c2 - a2 * c2));
    spot_cosine = a;
    /* HACK: We pack the area size in alpha color. This is decoded by the shader. */
    data.color.w = -max_ff(la->radius, FLT_MIN);
    pass.light_spot.append(data, select_id);
    if ((la->mode & LA_SHOW_CONE) && !DRW_state_is_select()) {
      pass.light_spot_cone_front.append(data.with_color({0.0f, 0.0f, 0.0f, 0.5f}), select_id);
      pass.light_spot_cone_back.append(data.with_color({1.0f, 1.0f, 1.0f, 0.3f}), select_id);
    }
  }
  else if (la->type == LA_AREA) {
    ExtraInstanceBuf &buf = ELEM(la->area_shape, LA_AREA_SQUARE, LA_AREA_RECT) ?
                                pass.light_area_square :
                                pass.light_area_disk;
    bool uniform_scale = !ELEM(la->area_shape, LA_AREA_RECT, LA_AREA_ELLIPSE);
    area_size_x = la->area_size;
    area_size_y = uniform_scale ? la->area_size : la->area_sizey;
    buf.append(data, select_id);
  }
}

}  // namespace blender::draw::overlay
