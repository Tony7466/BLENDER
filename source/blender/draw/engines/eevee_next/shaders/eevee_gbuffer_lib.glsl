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

/* -------------------------------------------------------------------- */
/** \name Types
 *
 * \{ */

#define GBUFFER_LAYER_MAX 8
#define GBUFFER_NORMAL_MAX 4

/* Structure used when packing the closures into the GBuffer textures & attachments. */
struct GBufferDataPacked {
  uint header;
  /* TODO(fclem): Better packing. */
  vec4 data[GBUFFER_LAYER_MAX];
  vec2 N[GBUFFER_NORMAL_MAX];
  /* Only used for book-keeping. Not actually written. Can be derived from header. */
  int layer;
  int layer_data;
  int layer_normal;
};

/* Structure used when unpacking the closures from the GBuffer textures & attachments. */
struct GBufferData {
  /* Only valid (or null) if `has_diffuse`, `has_reflection` or `has_refraction` is true. */
  /* TODO(fclem): This should eventually become ClosureUndetermined. */
  ClosureDiffuse diffuse;
  ClosureTranslucent translucent;
  ClosureReflection reflection;
  ClosureRefraction refraction;
  /* First world normal stored in the gbuffer. Only valid if `has_any_surface` is true. */
  vec3 surface_N;
  /* Additional object information if any closure needs it. */
  float thickness;
  uint object_id;

  bool has_diffuse;
  bool has_translucent;
  bool has_reflection;
  bool has_refraction;
  bool has_sss;
  bool has_any_surface;
  uint header;
  uint closure_count;
  /* Only used for book-keeping. Not actually written. Can be derived from header. */
  int layer_data;
  int layer_normal;
  /* Texel of the gbuffer being read. */
  ivec2 texel;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Pack / Unpack Utils
 *
 * \{ */

bool color_is_grayscale(vec3 color)
{
  /* This tests is R == G == B. */
  return all(equal(color.rgb, color.gbr));
}

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

vec4 gbuffer_sss_radii_pack(vec3 sss_radii)
{
  /* TODO(fclem): Something better. */
  return gbuffer_closure_color_pack(vec3(gbuffer_ior_pack(sss_radii.x),
                                         gbuffer_ior_pack(sss_radii.y),
                                         gbuffer_ior_pack(sss_radii.z)));
}

vec3 gbuffer_sss_radii_unpack(vec4 sss_radii_packed)
{
  /* TODO(fclem): Something better. */
  vec3 radii_packed = gbuffer_closure_color_unpack(sss_radii_packed);
  return vec3(gbuffer_ior_unpack(radii_packed.x),
              gbuffer_ior_unpack(radii_packed.y),
              gbuffer_ior_unpack(radii_packed.z));
}

/**
 * Pack color with values in the range of [0..8] using a 2 bit shared exponent.
 * This allows values up to 8 with some color degradation.
 * Above 8, the result will be clamped when writing the data to the output buffer.
 * This is supposed to be stored in a 10_10_10_2_unorm format with exponent in alpha.
 */
vec4 gbuffer_closure_color_pack(vec3 color)
{
  float max_comp = max(color.x, max(color.y, color.z));
  float exponent = (max_comp > 1) ? ((max_comp > 2) ? ((max_comp > 4) ? 3.0 : 2.0) : 1.0) : 0.0;
  /* TODO(fclem): Could try dithering to avoid banding artifacts on higher exponents. */
  return vec4(color / exp2(exponent), exponent / 3.0);
}
vec3 gbuffer_closure_color_unpack(vec4 color_packed)
{
  float exponent = color_packed.a * 3.0;
  return color_packed.rgb * exp2(exponent);
}

/**
 * Pack value in the range of [0..8] using a 2 bit exponent.
 * This allows values up to 8 with some color degradation.
 * Above 8, the result will be clamped when writing the data to the output buffer.
 * This is supposed to be stored in a 10_10_10_2_unorm format with exponent in alpha.
 */
vec2 gbuffer_closure_intensity_pack(float value)
{
  float exponent = (value > 1) ? ((value > 2) ? ((value > 4) ? 3.0 : 2.0) : 1.0) : 0.0;
  /* TODO(fclem): Could try dithering to avoid banding artifacts on higher exponents. */
  return vec2(value / exp2(exponent), exponent / 3.0);
}
float gbuffer_closure_intensity_unpack(vec2 value_packed)
{
  float exponent = value_packed.g * 3.0;
  return value_packed.r * exp2(exponent);
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

void gbuffer_append_closure(inout GBufferDataPacked gbuf, GBufferMode closure_type)
{
  gbuf.header |= gbuffer_header_pack(closure_type, gbuf.layer);
  gbuf.layer++;
}
void gbuffer_register_closure(inout GBufferData gbuf, ClosureUndetermined cl)
{
  switch (cl.type) {
    case CLOSURE_NONE_ID:
      /* TODO(fclem): Assert. */
      break;
    case CLOSURE_BSSRDF_BURLEY_ID:
      /* TODO(fclem): BSSSRDF closure. */
      gbuf.diffuse.N = cl.N;
      gbuf.diffuse.color = cl.color;
      gbuf.diffuse.sss_radius = cl.data.xyz;
      gbuf.has_sss = true;
      break;
    case CLOSURE_BSDF_DIFFUSE_ID:
      gbuf.diffuse.N = cl.N;
      gbuf.diffuse.color = cl.color;
      gbuf.has_diffuse = true;
      break;
    case CLOSURE_BSDF_TRANSLUCENT_ID:
      gbuf.translucent.N = cl.N;
      gbuf.translucent.color = cl.color;
      gbuf.has_translucent = true;
      break;
    case CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID:
      gbuf.reflection.N = cl.N;
      gbuf.reflection.color = cl.color;
      gbuf.reflection.roughness = cl.data.x;
      gbuf.has_reflection = true;
      break;
    case CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID:
      gbuf.refraction.N = cl.N;
      gbuf.refraction.color = cl.color;
      gbuf.refraction.roughness = cl.data.x;
      gbuf.refraction.ior = cl.data.y;
      gbuf.has_refraction = true;
      break;
  }
  gbuf.closure_count++;
}

void gbuffer_append_data(inout GBufferDataPacked gbuf, vec4 data)
{
  gbuf.data[gbuf.layer_data] = data;
  gbuf.layer_data++;
}
vec4 gbuffer_pop_first_data(inout GBufferData gbuf, sampler2DArray closure_tx)
{
  vec4 data = texelFetch(closure_tx, ivec3(gbuf.texel, gbuf.layer_data), 0);
  gbuf.layer_data++;
  return data;
}

void gbuffer_append_normal(inout GBufferDataPacked gbuf, vec3 normal)
{
  gbuf.N[gbuf.layer_normal] = gbuffer_normal_pack(normal);
  gbuf.layer_normal++;
}
vec3 gbuffer_pop_first_normal(inout GBufferData gbuf, sampler2DArray normal_tx)
{
  vec2 normal_packed = texelFetch(normal_tx, ivec3(gbuf.texel, gbuf.layer_normal), 0).rg;
  gbuf.layer_normal++;
  return gbuffer_normal_unpack(normal_packed);
}

/* Pack geometry additional infos onto the normal stack. Needs to be run last. */
void gbuffer_additional_info_pack(inout GBufferDataPacked gbuf, float thickness, uint object_id)
{
  gbuf.N[gbuf.layer_normal] = vec2(gbuffer_thickness_pack(thickness),
                                   gbuffer_object_id_unorm16_pack(object_id));
  gbuf.layer_normal++;
}
void gbuffer_additional_info_load(inout GBufferData gbuf, sampler2DArray normal_tx)
{
  vec2 data_packed = texelFetch(normal_tx, ivec3(gbuf.texel, gbuf.layer_normal), 0).rg;
  gbuf.layer_normal++;
  gbuf.thickness = gbuffer_thickness_unpack(data_packed.x);
  gbuf.object_id = gbuffer_object_id_unorm16_unpack(data_packed.y);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Pack / Unpack Closures
 *
 * \{ */

void gbuffer_closure_diffuse_pack(inout GBufferDataPacked gbuf, ClosureUndetermined cl)
{
  gbuffer_append_closure(gbuf, GBUF_DIFFUSE);
  gbuffer_append_data(gbuf, gbuffer_closure_color_pack(cl.color));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_diffuse_load(inout GBufferData gbuf,
                                  sampler2DArray closure_tx,
                                  sampler2DArray normal_tx)
{
  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);

  ClosureUndetermined cl = closure_new(CLOSURE_BSDF_DIFFUSE_ID);
  cl.color = gbuffer_closure_color_unpack(data0);
  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

void gbuffer_closure_translucent_pack(inout GBufferDataPacked gbuf, ClosureUndetermined cl)
{
  gbuffer_append_closure(gbuf, GBUF_TRANSLUCENT);
  gbuffer_append_data(gbuf, gbuffer_closure_color_pack(cl.color));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_translucent_load(inout GBufferData gbuf,
                                      sampler2DArray closure_tx,
                                      sampler2DArray normal_tx)
{
  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);

  ClosureUndetermined cl = closure_new(CLOSURE_BSDF_TRANSLUCENT_ID);
  cl.color = gbuffer_closure_color_unpack(data0);
  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

void gbuffer_closure_subsurface_pack(inout GBufferDataPacked gbuf, ClosureUndetermined cl)
{
  gbuffer_append_closure(gbuf, GBUF_SUBSURFACE);
  gbuffer_append_data(gbuf, gbuffer_closure_color_pack(cl.color));
  gbuffer_append_data(gbuf, gbuffer_sss_radii_pack(cl.data.xyz));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_subsurface_load(inout GBufferData gbuf,
                                     sampler2DArray closure_tx,
                                     sampler2DArray normal_tx)
{
  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);
  vec4 data1 = gbuffer_pop_first_data(gbuf, closure_tx);

  ClosureUndetermined cl = closure_new(CLOSURE_BSSRDF_BURLEY_ID);
  cl.color = gbuffer_closure_color_unpack(data0);
  cl.data.rgb = gbuffer_sss_radii_unpack(data1);
  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

void gbuffer_closure_reflection_pack(inout GBufferDataPacked gbuf, ClosureUndetermined cl)
{
  gbuffer_append_closure(gbuf, GBUF_REFLECTION);
  gbuffer_append_data(gbuf, gbuffer_closure_color_pack(cl.color));
  gbuffer_append_data(gbuf, vec4(cl.data.x, 0.0, 0.0, 0.0));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_reflection_load(inout GBufferData gbuf,
                                     sampler2DArray closure_tx,
                                     sampler2DArray normal_tx)
{
  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);
  vec4 data1 = gbuffer_pop_first_data(gbuf, closure_tx);

  ClosureUndetermined cl = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
  cl.color = gbuffer_closure_color_unpack(data0);
  cl.data.x = data1.x;
  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

void gbuffer_closure_refraction_pack(inout GBufferDataPacked gbuf, ClosureUndetermined cl)
{
  gbuffer_append_closure(gbuf, GBUF_REFRACTION);
  gbuffer_append_data(gbuf, gbuffer_closure_color_pack(cl.color));
  gbuffer_append_data(gbuf, vec4(cl.data.x, gbuffer_ior_pack(cl.data.y), 0.0, 0.0));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_refraction_load(inout GBufferData gbuf,
                                     sampler2DArray closure_tx,
                                     sampler2DArray normal_tx)
{
  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);
  vec4 data1 = gbuffer_pop_first_data(gbuf, closure_tx);

  ClosureUndetermined cl = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);
  cl.color = gbuffer_closure_color_unpack(data0);
  cl.data.x = data1.x;
  cl.data.y = gbuffer_ior_unpack(data1.y);
  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Pack / Unpack Parameter Optimized
 *
 * Special cases where we can save some data layers per closure.
 * \{ */

void gbuffer_closure_reflection_colorless_pack(inout GBufferDataPacked gbuf,
                                               ClosureUndetermined cl)
{
  vec2 intensity_packed = gbuffer_closure_intensity_pack(cl.color.r);
  gbuffer_append_closure(gbuf, GBUF_REFLECTION_COLORLESS);
  gbuffer_append_data(gbuf, vec4(cl.data.x, 0.0, intensity_packed));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_reflection_colorless_load(inout GBufferData gbuf,
                                               sampler2DArray closure_tx,
                                               sampler2DArray normal_tx)
{
  ClosureUndetermined cl = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);

  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);
  cl.data.x = data0.x;
  cl.color = vec3(gbuffer_closure_intensity_unpack(data0.zw));

  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

void gbuffer_closure_refraction_colorless_pack(inout GBufferDataPacked gbuf,
                                               ClosureUndetermined cl)
{
  vec2 intensity_packed = gbuffer_closure_intensity_pack(cl.color.r);
  gbuffer_append_closure(gbuf, GBUF_REFRACTION_COLORLESS);
  gbuffer_append_data(gbuf, vec4(cl.data.x, gbuffer_ior_pack(cl.data.y), intensity_packed));
  gbuffer_append_normal(gbuf, cl.N);
}

void gbuffer_closure_refraction_colorless_load(inout GBufferData gbuf,
                                               sampler2DArray closure_tx,
                                               sampler2DArray normal_tx)
{
  ClosureUndetermined cl = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID);

  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);
  cl.data.x = data0.x;
  cl.data.y = gbuffer_ior_unpack(data0.y);
  cl.color = vec3(gbuffer_closure_intensity_unpack(data0.zw));

  cl.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, cl);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Pack / Unpack Special Common Optimized
 *
 * Special cases where we can save some space by packing multiple closures data together.
 * \{ */

void gbuffer_closure_metal_clear_coat_pack(inout GBufferDataPacked gbuf,
                                           ClosureUndetermined cl_bottom,
                                           ClosureUndetermined cl_coat)
{
  vec2 intensity_packed = gbuffer_closure_intensity_pack(cl_coat.color.r);
  gbuffer_append_closure(gbuf, GBUF_METAL_CLEARCOAT);
  gbuffer_append_data(gbuf, gbuffer_closure_color_pack(cl_bottom.color));
  gbuffer_append_data(gbuf, vec4(cl_bottom.data.x, cl_coat.data.y, intensity_packed));
  gbuffer_append_normal(gbuf, cl_bottom.N);
}

void gbuffer_closure_metal_clear_coat_load(inout GBufferData gbuf,
                                           sampler2DArray closure_tx,
                                           sampler2DArray normal_tx)
{
  vec4 data0 = gbuffer_pop_first_data(gbuf, closure_tx);
  vec4 data1 = gbuffer_pop_first_data(gbuf, closure_tx);

  ClosureUndetermined bottom = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
  bottom.color = gbuffer_closure_color_unpack(data0);
  bottom.data.x = data1.x;

  ClosureUndetermined coat = closure_new(CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID);
  coat.color = vec3(gbuffer_closure_intensity_unpack(data1.zw));
  coat.data.x = data1.y;

  coat.N = bottom.N = gbuffer_pop_first_normal(gbuf, normal_tx);

  gbuffer_register_closure(gbuf, bottom);
  gbuffer_register_closure(gbuf, coat);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Gbuffer Read / Write
 *
 * \{ */

GBufferDataPacked gbuffer_pack(ClosureUndetermined diffuse,
                               ClosureUndetermined translucent,
                               ClosureUndetermined reflection,
                               ClosureUndetermined refraction,
                               vec3 default_N,
                               float thickness,
                               uint resource_id)
{
  GBufferDataPacked gbuf;
  gbuf.header = 0u;
  gbuf.layer = 0;
  gbuf.layer_data = 0;
  gbuf.layer_normal = 0;

  bool has_refraction = refraction.weight > 1e-5;
  bool has_reflection = reflection.weight > 1e-5;
  bool has_diffuse = diffuse.weight > 1e-5;
  bool has_translucent = translucent.weight > 1e-5;
  bool has_sss = (diffuse.type == CLOSURE_BSSRDF_BURLEY_ID);

  /* Check special configurations first. */

  if (has_refraction) {
    if (color_is_grayscale(refraction.color)) {
      gbuffer_closure_refraction_colorless_pack(gbuf, refraction);
    }
    else {
      gbuffer_closure_refraction_pack(gbuf, refraction);
    }
  }

  if (has_reflection) {
    if (color_is_grayscale(reflection.color)) {
      gbuffer_closure_reflection_colorless_pack(gbuf, reflection);
    }
    else {
      gbuffer_closure_reflection_pack(gbuf, reflection);
    }
  }

  if (has_diffuse) {
    if (has_sss) {
      gbuffer_closure_subsurface_pack(gbuf, diffuse);
    }
    else {
      gbuffer_closure_diffuse_pack(gbuf, diffuse);
    }
  }

  if (has_translucent) {
    gbuffer_closure_diffuse_pack(gbuf, translucent);
  }

  if (gbuf.layer_normal == 0) {
    /* If no lit BDSF is outputted, still output the surface normal in the first layer.
     * This is needed by some algorithms. */
    gbuffer_append_normal(gbuf, default_N);
  }

  if (has_sss || has_translucent) {
    gbuffer_additional_info_pack(gbuf, thickness, resource_id);
  }

  return gbuf;
}

/* Populate the GBufferData only based on the header. The rest of the data is undefined. */
GBufferData gbuffer_read_header(uint header)
{
  GBufferData gbuf;
  gbuf.header = header;
  gbuf.has_any_surface = (header != 0u);
  gbuf.has_diffuse = false;
  gbuf.has_reflection = false;
  gbuf.has_refraction = false;
  gbuf.has_translucent = false;
  gbuf.thickness = 0.0;
  gbuf.closure_count = 0u;

  for (int layer = 0; layer < 4; layer++) {
    GBufferMode mode = gbuffer_header_unpack(gbuf.header, layer);
    switch (mode) {
      case GBUF_NONE:
        break;
      case GBUF_DIFFUSE:
        gbuf.has_diffuse = true;
        break;
      case GBUF_TRANSLUCENT:
        gbuf.has_translucent = true;
        break;
      case GBUF_SUBSURFACE:
        gbuf.has_diffuse = true;
        gbuf.has_sss = true;
        break;
      case GBUF_METAL_CLEARCOAT:
      case GBUF_REFLECTION_COLORLESS:
      case GBUF_REFLECTION:
        gbuf.has_reflection = true;
        break;
      case GBUF_REFRACTION_COLORLESS:
      case GBUF_REFRACTION:
        gbuf.has_refraction = true;
        break;
    }
  }

  return gbuf;
}

GBufferData gbuffer_read(usampler2D header_tx,
                         sampler2DArray closure_tx,
                         sampler2DArray normal_tx,
                         ivec2 texel)
{
  GBufferData gbuf;
  gbuf.texel = texel;
  gbuf.header = texelFetch(header_tx, texel, 0).r;
  gbuf.has_any_surface = (gbuf.header != 0u);
  gbuf.has_diffuse = false;
  gbuf.has_reflection = false;
  gbuf.has_refraction = false;
  gbuf.has_translucent = false;
  gbuf.thickness = 0.0;
  gbuf.closure_count = 0u;
  gbuf.layer_data = 0;
  gbuf.layer_normal = 0;

  if (!gbuf.has_any_surface) {
    return gbuf;
  }

  /* First closure is always written. */
  gbuf.surface_N = gbuffer_normal_unpack(texelFetch(normal_tx, ivec3(texel, 0), 0).xy);

  /* Default values. */
  gbuf.refraction.color = vec3(0.0);
  gbuf.refraction.N = vec3(0.0, 0.0, 1.0);
  gbuf.refraction.roughness = 0.0;
  gbuf.refraction.ior = 1.1; /* Avoid NaN in some places. */

  gbuf.reflection.color = vec3(0.0);
  gbuf.reflection.N = vec3(0.0, 0.0, 1.0);
  gbuf.reflection.roughness = 0.0;

  gbuf.diffuse.color = vec3(0.0);
  gbuf.diffuse.N = vec3(0.0, 0.0, 1.0);
  gbuf.diffuse.sss_radius = vec3(0.0, 0.0, 0.0);
  gbuf.diffuse.sss_id = 0u;

  gbuf.translucent.color = vec3(0.0);
  gbuf.translucent.N = vec3(0.0, 0.0, 1.0);

  gbuf.thickness = 0.0;

  for (int layer = 0; layer < 4; layer++) {
    GBufferMode mode = gbuffer_header_unpack(gbuf.header, layer);
    switch (mode) {
      case GBUF_NONE:
        break;
      case GBUF_DIFFUSE:
        gbuffer_closure_diffuse_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_TRANSLUCENT:
        gbuffer_closure_translucent_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_SUBSURFACE:
        gbuffer_closure_subsurface_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_REFLECTION:
        gbuffer_closure_reflection_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_REFRACTION:
        gbuffer_closure_refraction_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_REFLECTION_COLORLESS:
        gbuffer_closure_reflection_colorless_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_REFRACTION_COLORLESS:
        gbuffer_closure_refraction_colorless_load(gbuf, closure_tx, normal_tx);
        break;
      case GBUF_METAL_CLEARCOAT:
        gbuffer_closure_metal_clear_coat_load(gbuf, closure_tx, normal_tx);
        break;
    }
  }

  if (gbuf.has_sss || gbuf.has_translucent) {
    gbuffer_additional_info_load(gbuf, normal_tx);
  }

  return gbuf;
}

/** \} */