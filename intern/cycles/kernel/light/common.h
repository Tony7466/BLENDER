/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/sample/mapping.h"

CCL_NAMESPACE_BEGIN

/* Light Sample Result */

/* TODO(weizhen): make sure `prim` is PRIM_NONE when it's lamp, and `lamp` is LAMP_NONE when it's
 * a primitive. Could also write `~lamp` to `prim` for lamps. Anyway make sure things are
 * consistent. Can write utility functions if needed. */
struct LightSample {
  float3 P;            /* position on light, or direction for distant light */
  packed_float3 Ng;    /* normal on light */
  float t;             /* distance to light (FLT_MAX for distant light) */
  float3 D;            /* direction from shading point to light */
  float u, v;          /* parametric coordinate on primitive */
  float pdf;           /* pdf for selecting light and point on light */
  float pdf_selection; /* pdf for selecting light */
  float eval_fac;      /* intensity multiplier */
  int object;          /* object id for triangle/curve lights */
  int prim;            /* primitive id for triangle/curve lights */
  int shader;          /* shader id */
  int lamp;            /* lamp id */
  int group;           /* lightgroup */
  LightType type;      /* type of light */
  int emitter_id;      /* index in the emitter array */

  float jacobian_area_to_solid_angle() const
  {
    float cos_pi = dot(Ng, -D);

    if (cos_pi <= 0.0f) {
      return 0.0f;
    }

    return sqr(t) / cos_pi;
  }

  float jacobian_solid_angle_to_area() const
  {
    float cos_pi = dot(Ng, -D);

    /* TODO(weizhen): how to handle when t is smaller than zero? */
    if (cos_pi <= 0.0f || t <= 0.0f) {
      return 0.0f;
    }

    return cos_pi / sqr(t);
  }
};

/* Utilities */

ccl_device_inline float3 ellipse_sample(float3 ru, float3 rv, float2 rand)
{
  const float2 uv = sample_uniform_disk(rand);
  return ru * uv.x + rv * uv.y;
}

ccl_device_inline float3 rectangle_sample(float3 ru, float3 rv, float2 rand)
{
  return ru * (2.0f * rand.x - 1.0f) + rv * (2.0f * rand.y - 1.0f);
}

ccl_device float3 disk_light_sample(float3 n, float2 rand)
{
  float3 ru, rv;

  make_orthonormals(n, &ru, &rv);

  return ellipse_sample(ru, rv, rand);
}

ccl_device float light_pdf_area_to_solid_angle(const float3 Ng, const float3 I, float t)
{
  float cos_pi = dot(Ng, I);

  if (cos_pi <= 0.0f)
    return 0.0f;

  return t * t / cos_pi;
}

/* Visibility flag om the light shader. */
ccl_device_inline bool is_light_shader_visible_to_path(const int shader, const uint32_t path_flag)
{
  if ((shader & SHADER_EXCLUDE_ANY) == 0) {
    return true;
  }

  if (((shader & SHADER_EXCLUDE_DIFFUSE) && (path_flag & PATH_RAY_DIFFUSE)) ||
      ((shader & SHADER_EXCLUDE_GLOSSY) && ((path_flag & (PATH_RAY_GLOSSY | PATH_RAY_REFLECT)) ==
                                            (PATH_RAY_GLOSSY | PATH_RAY_REFLECT))) ||
      ((shader & SHADER_EXCLUDE_TRANSMIT) && (path_flag & PATH_RAY_TRANSMIT)) ||
      ((shader & SHADER_EXCLUDE_CAMERA) && (path_flag & PATH_RAY_CAMERA)) ||
      ((shader & SHADER_EXCLUDE_SCATTER) && (path_flag & PATH_RAY_VOLUME_SCATTER)))
  {
    return false;
  }

  return true;
}

CCL_NAMESPACE_END
