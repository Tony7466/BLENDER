/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Screen Space Ray Tracing
 *
 */

#pragma once

#include "eevee_raytracing.hh"
#include "eevee_instance.hh"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name RayTracing
 * \{ */

void RayTracing::init()
{
  /* Compute pixel size. Size is multiplied by 2 because it is applied in NDC [-1..1] range. */
  data_.pixel_size = float2(2.0f) / float2(inst_.film.render_extent_get());
  data_.border_fade = inst_.scene->eevee.ssr_border_fade;

  data_.ssr_quality = 1.0f - 0.95f * inst_.scene->eevee.ssr_quality;
  data_.ssr_thickness = inst_.scene->eevee.ssr_thickness;
  data_.ssr_firefly_factor = inst_.scene->eevee.ssr_firefly_fac;
  data_.ssr_max_roughness = inst_.scene->eevee.ssr_max_roughness;
  data_.ssr_brdf_bias = 0.1f + data_.ssr_quality * 0.6f; /* Range [0.1, 0.7]. */

  data_.push_update();
}

/** \} */

}  // namespace blender::eevee
