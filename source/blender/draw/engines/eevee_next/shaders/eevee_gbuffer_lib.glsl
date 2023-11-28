/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * G-buffer: Packing and unpacking of G-buffer data.
 *
 * See #GBuffer for a breakdown of the G-buffer layout.
 */

#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)

vec2 gbuffer_normal_pack(vec3 N)
{
  N /= length_manhattan(N);
  vec2 _sign = sign(N.xy);
  _sign.x = _sign.x == 0.0 ? 1.0 : _sign.x;
  _sign.y = _sign.y == 0.0 ? 1.0 : _sign.y;
  N.xy = (N.z >= 0.0) ? N.xy : ((1.0 - abs(N.yx)) * _sign);
  N.xy = N.xy * 0.5 + 0.5;
  return N.xy;
}

vec3 gbuffer_normal_unpack(vec2 N_packed)
{
  N_packed = N_packed * 2.0 - 1.0;
  vec3 N = vec3(N_packed.x, N_packed.y, 1.0 - abs(N_packed.x) - abs(N_packed.y));
  float t = clamp(-N.z, 0.0, 1.0);
  N.x += (N.x >= 0.0) ? -t : t;
  N.y += (N.y >= 0.0) ? -t : t;
  return normalize(N);
}

float gbuffer_ior_pack(float ior)
{
  return (ior > 1.0) ? (1.0 - 0.5 / ior) : (0.5 * ior);
}

float gbuffer_ior_unpack(float ior_packed)
{
  return (ior_packed > 0.5) ? (0.5 / (1.0 - ior_packed)) : (2.0 * ior_packed);
}

float gbuffer_thickness_pack(float thickness)
{
  /* TODO(fclem): Something better. */
  return gbuffer_ior_pack(thickness);
}

float gbuffer_thickness_unpack(float thickness_packed)
{
  /* TODO(fclem): Something better. */
  return gbuffer_ior_unpack(thickness_packed);
}

vec3 gbuffer_sss_radii_pack(vec3 sss_radii)
{
  /* TODO(fclem): Something better. */
  return vec3(
      gbuffer_ior_pack(sss_radii.x), gbuffer_ior_pack(sss_radii.y), gbuffer_ior_pack(sss_radii.z));
}

vec3 gbuffer_sss_radii_unpack(vec3 sss_radii_packed)
{
  /* TODO(fclem): Something better. */
  return vec3(gbuffer_ior_unpack(sss_radii_packed.x),
              gbuffer_ior_unpack(sss_radii_packed.y),
              gbuffer_ior_unpack(sss_radii_packed.z));
}

vec4 gbuffer_color_pack(vec3 color)
{
  float max_comp = max(color.x, max(color.y, color.z));
  /* Store 2bit exponent inside Alpha. Allows values up to 8 with some color degradation.
   * Above 8, the result will be clamped when writing the data to the output buffer. */
  float exponent = (max_comp > 1) ? ((max_comp > 2) ? ((max_comp > 4) ? 3.0 : 2.0) : 1.0) : 0.0;
  /* TODO(fclem): Could try dithering to avoid banding artifacts on higher exponents. */
  return vec4(color / exp2(exponent), exponent / 3.0);
}

vec3 gbuffer_color_unpack(vec4 color_packed)
{
  float exponent = color_packed.a * 3.0;
  return color_packed.rgb * exp2(exponent);
}

float gbuffer_object_id_unorm16_pack(uint object_id)
{
  return float(object_id & 0xFFFFu) / float(0xFFFF);
}

uint gbuffer_object_id_unorm16_unpack(float object_id_packed)
{
  return uint(object_id_packed * float(0xFFFF));
}

float gbuffer_object_id_f16_pack(uint object_id)
{
  /* TODO(fclem): Make use of all the 16 bits in a half float.
   * This here only correctly represent values up to 1024. */
  return float(object_id);
}

uint gbuffer_object_id_f16_unpack(float object_id_packed)
{
  return uint(object_id_packed);
}

bool gbuffer_is_refraction(vec4 gbuffer)
{
  return gbuffer.w < 1.0;
}

uint gbuffer_header_pack(GBufferMode mode, uint layer)
{
  return (mode << (4u * layer));
}

GBufferMode gbuffer_header_unpack(uint data, uint layer)
{
  return GBufferMode((data >> (4u * layer)) & 15u);
}

/* Return true if any layer of the gbuffer match the given closure. */
bool gbuffer_has_closure(uint data, eClosureBits closure)
{
  int layer = 0;

  /* Since closure order in the gbuffer is static, we check them in order. */

  bool has_refraction = (gbuffer_header_unpack(data, layer) == GBUF_REFRACTION);
  layer += int(has_refraction);

  if (closure == eClosureBits(CLOSURE_REFRACTION)) {
    return has_refraction;
  }

  bool has_reflection = (gbuffer_header_unpack(data, layer) == GBUF_REFLECTION);
  layer += int(has_reflection);

  if (closure == eClosureBits(CLOSURE_REFLECTION)) {
    return has_reflection;
  }

  bool has_diffuse = (gbuffer_header_unpack(data, layer) == GBUF_DIFFUSE);
  layer += int(has_diffuse);

  if (closure == eClosureBits(CLOSURE_DIFFUSE)) {
    return has_diffuse;
  }

  return false;
}

struct GBufferData {
  ClosureDiffuse diffuse;
  ClosureReflection reflection;
  ClosureRefraction refraction;
  float thickness;
  bool has_diffuse;
  bool has_reflection;
  bool has_refraction;
};

struct GBufferDataPacked {
  uint header;
  /* TODO(fclem): Resize arrays based on used closures. */
  vec4 closure[4];
  vec4 color[3];
};

GBufferDataPacked gbuffer_pack(ClosureDiffuse diffuse,
                               ClosureReflection reflection,
                               ClosureRefraction refraction,
                               float thickness)
{
  GBufferDataPacked gbuf;
  gbuf.header = 0u;

  /* Check special configurations first. */

  if (refraction.weight < 1e-5) {
    /* TODO(fclem): Compute this only if needed. */
    bool shared_normal = all(equal(diffuse.N, reflection.N));
    bool is_reflection_colorless = all(equal(reflection.color.rgb, reflection.color.gbr));
    if (shared_normal && is_reflection_colorless) {
      /* Opaque Dielectric. */
      /* TODO */
      // return gbuf;
    }
  }

  int layer = 0;

  if (refraction.weight > 1e-5) {
    gbuf.color[layer] = gbuffer_color_pack(refraction.color);
    gbuf.closure[layer].xy = gbuffer_normal_pack(refraction.N);
    gbuf.closure[layer].z = refraction.roughness;
    gbuf.closure[layer].w = gbuffer_ior_pack(refraction.ior);
    gbuf.header |= gbuffer_header_pack(GBUF_REFRACTION, layer);
    layer += 1;
  }

  if (reflection.weight > 1e-5) {
    gbuf.color[layer] = gbuffer_color_pack(reflection.color);
    gbuf.closure[layer].xy = gbuffer_normal_pack(reflection.N);
    gbuf.closure[layer].z = reflection.roughness;
    gbuf.closure[layer].w = 0.0; /* Unused. */
    gbuf.header |= gbuffer_header_pack(GBUF_REFLECTION, layer);
    layer += 1;
  }

  if (diffuse.weight > 1e-5) {
    gbuf.color[layer] = gbuffer_color_pack(diffuse.color);
    gbuf.closure[layer].xy = gbuffer_normal_pack(diffuse.N);
    gbuf.closure[layer].z = 0.0; /* Unused. */
    gbuf.closure[layer].w = gbuffer_thickness_pack(thickness);
    gbuf.header |= gbuffer_header_pack(GBUF_DIFFUSE, layer);
    layer += 1;
  }

  if (diffuse.sss_id > 0) {
    gbuf.closure[layer].xyz = gbuffer_sss_radii_pack(diffuse.sss_radius);
    gbuf.closure[layer].w = gbuffer_object_id_unorm16_pack(diffuse.sss_id);
    gbuf.header |= gbuffer_header_pack(GBUF_SSS, layer);
    layer += 1;
  }

  return gbuf;
}

GBufferData gbuffer_read(usampler2D header_tx,
                         sampler2DArray closure_tx,
                         sampler2DArray color_tx,
                         ivec2 texel)
{
  GBufferData gbuf;

  uint header = texelFetch(header_tx, texel, 0).r;

  gbuf.thickness = 0.0;

  int layer = 0;

  /* Since closure order in the gbuffer is static, we check them in order. */

  gbuf.has_refraction = (gbuffer_header_unpack(header, layer) == GBUF_REFRACTION);

  if (gbuf.has_refraction) {
    vec4 closure_packed = texelFetch(closure_tx, ivec3(texel, layer), 0);
    vec4 color_packed = texelFetch(color_tx, ivec3(texel, layer), 0);

    gbuf.refraction.color = gbuffer_color_unpack(color_packed);
    gbuf.refraction.N = gbuffer_normal_unpack(closure_packed.xy);
    gbuf.refraction.roughness = closure_packed.z;
    gbuf.refraction.ior = gbuffer_ior_unpack(closure_packed.w);
    layer += 1;
  }
  else {
    /* Default values. */
    gbuf.refraction.color = vec3(0.0);
    gbuf.refraction.N = vec3(0.0, 0.0, 1.0);
    gbuf.refraction.roughness = 0.0;
    gbuf.refraction.ior = 1.1;
  }

  gbuf.has_reflection = (gbuffer_header_unpack(header, layer) == GBUF_REFLECTION);

  if (gbuf.has_reflection) {
    vec4 closure_packed = texelFetch(closure_tx, ivec3(texel, layer), 0);
    vec4 color_packed = texelFetch(color_tx, ivec3(texel, layer), 0);

    gbuf.reflection.color = gbuffer_color_unpack(color_packed);
    gbuf.reflection.N = gbuffer_normal_unpack(closure_packed.xy);
    gbuf.reflection.roughness = closure_packed.z;
    layer += 1;
  }
  else {
    /* Default values. */
    gbuf.reflection.color = vec3(0.0);
    gbuf.reflection.N = vec3(0.0, 0.0, 1.0);
    gbuf.reflection.roughness = 0.0;
  }

  gbuf.has_diffuse = (gbuffer_header_unpack(header, layer) == GBUF_DIFFUSE);

  if (gbuf.has_diffuse) {
    vec4 closure_packed = texelFetch(closure_tx, ivec3(texel, layer), 0);
    vec4 color_packed = texelFetch(color_tx, ivec3(texel, layer), 0);

    gbuf.diffuse.color = gbuffer_color_unpack(color_packed);
    gbuf.diffuse.N = gbuffer_normal_unpack(closure_packed.xy);
    gbuf.thickness = gbuffer_thickness_unpack(closure_packed.w);
    layer += 1;
  }
  else {
    /* Default values. */
    gbuf.diffuse.color = vec3(0.0);
    gbuf.diffuse.N = vec3(0.0, 0.0, 1.0);
    gbuf.thickness = 0.0;
  }

  bool has_sss = (gbuffer_header_unpack(header, layer) == GBUF_SSS);

  if (has_sss) {
    vec4 closure_packed = texelFetch(closure_tx, ivec3(texel, layer), 0);

    gbuf.diffuse.sss_radius = gbuffer_sss_radii_unpack(closure_packed.xyz);
    gbuf.diffuse.sss_id = gbuffer_object_id_unorm16_unpack(closure_packed.w);
    layer += 1;
  }
  else {
    gbuf.diffuse.sss_radius = vec3(0.0, 0.0, 0.0);
    gbuf.diffuse.sss_id = 0u;
  }

  return gbuf;
}
