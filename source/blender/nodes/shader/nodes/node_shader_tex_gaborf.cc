/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Gabor Noise
 *
 * Based on: Blender patch D287 & D3495
 * With additional controls for kernel variance.
 *
 * Adapted from Open Shading Language implementation.
 * Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
 * All Rights Reserved.
 *
 * OSL Gabor noise originally based on:
 * Lagae, Lefebvre, Drettakis and Dutré, 2009. Procedural noise using sparse Gabor convolution
 * Lagae, A. and Drettakis, G. 2011. Filtering Solid Gabor Noise
 *
 * Phasor noise option is based on:
 * Tricard et al. 2019. Procedural Phasor Noise
 */

/* See GLSL implementation for code comments. */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_texture.h"

#include "BLI_hash.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.hh"
#include "BLI_noise.hh"

#include "NOD_multi_function.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_shader_util.hh"
#include "node_util.hh"

namespace blender::nodes::node_shader_tex_gaborf_cc {

NODE_STORAGE_FUNCS(NodeTexGaborF)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();
  b.add_output<decl::Float>("Value").no_muted_links();
  b.add_input<decl::Vector>("Vector").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Float>("Scale").min(-1000.0f).max(1000.0f).default_value(5.0f);
  b.add_input<decl::Float>("Gain").min(-10.0f).max(10.0f).default_value(1.0f);
  PanelDeclarationBuilder &fractal = b.add_panel("Fractal").default_closed(true).draw_buttons(
      [](uiLayout *layout, bContext * /*C*/, PointerRNA *ptr) {
        uiItemR(layout, ptr, "use_origin_offset", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
      });
  fractal.add_input<decl::Float>("Detail").min(0.0f).max(15.0f).default_value(0.0f).description(
      "Number of noise octaves, high values are slower to compute");
  fractal.add_input<decl::Float>("Roughness")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR);
  fractal.add_input<decl::Float>("Scale Lacunarity")
      .min(-10.0f)
      .max(10.0f)
      .default_value(2.0f)
      .description("The difference between the scale of each consecutive octave");
  fractal.add_input<decl::Float>("Frequency Lacunarity")
      .min(-10.0f)
      .max(10.0f)
      .default_value(2.0f)
      .description("The difference between the kernel frequency of each consecutive octave");
  fractal.add_input<decl::Float>("Rotation Lacunarity")
      .subtype(PROP_ANGLE)
      .description(
          "The difference between the kernel rotation of each consecutive octave, does not work "
          "when Anisotropic is set to 1");
  PanelDeclarationBuilder &kernel = b.add_panel("Kernel").default_closed(true);
  kernel.add_input<decl::Float>("Frequency")
      .min(-100.0f)
      .max(100.0f)
      .default_value(4.0f)
      .description("Frequency for the kernel shape, higher values provides more detail");
  kernel.add_input<decl::Float>("Radius")
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR)
      .description("Controls the radius of the kernel, values over 1 may produce artefacts");
  kernel.add_input<decl::Float>("Impulses")
      .min(0.0f)
      .max(16.0f)
      .default_value(2.0f)
      .description("Controls the amount of kernel impulses, high values are slower to compute");
  kernel.add_input<decl::Float>("Phase Offset")
      .subtype(PROP_ANGLE)
      .description("Kernel shape phase offset");
  kernel.add_input<decl::Float>("Phase Variance")
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR);
  kernel.add_input<decl::Float>("Cell Randomness")
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR);
  PanelDeclarationBuilder &aniso = b.add_panel("Anisotropy").default_closed(false);
  aniso.add_input<decl::Float>("Rotation")
      .subtype(PROP_ANGLE)
      .description("Kernel shape rotation");
  aniso.add_input<decl::Float>("Rotation Variance")
      .subtype(PROP_ANGLE)
      .description("Rotation randomness");
  aniso.add_input<decl::Float>("Tilt Randomness")
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR);
  aniso.add_input<decl::Float>("Anisotropic Factor")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .description("Mix between Isotropic and fixed Anisotropic control");
  aniso.add_input<decl::Vector>("Direction")
      .default_value({math::numbers::sqrt2, math::numbers::sqrt2, 0.0f})
      .subtype(PROP_DIRECTION)
      .description("The direction of the anisotropic Gabor noise");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayout *col;
  uiLayout *split;
  col = uiLayoutColumn(layout, true);
  split = uiLayoutSplit(col, 0.33f, true);
  uiItemR(split, ptr, "gabor_dimensions", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  uiItemR(split, ptr, "mode", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);

  col = uiLayoutColumn(layout, true);
  uiItemR(col, ptr, "periodic", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
  uiItemR(col, ptr, "normalize", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  NodeTexGaborF *tex = MEM_cnew<NodeTexGaborF>(__func__);
  BKE_texture_mapping_default(&tex->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&tex->base.color_mapping);

  tex->mode = SHD_GABOR_MODE_GABOR;
  tex->periodic = 0;
  tex->normalize = 1;
  tex->dimensions = 2;
  tex->use_origin_offset = 1;
  node->storage = tex;
}

static int node_shader_gpu_tex_gaborf(GPUMaterial *mat,
                                      bNode *node,
                                      bNodeExecData * /*execdata*/,
                                      GPUNodeStack *in,
                                      GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(mat, node, &in[0].link);
  node_shader_gpu_tex_mapping(mat, node, in, out);

  const NodeTexGaborF &storage = node_storage(*node);
  const float dimensions = storage.dimensions;
  const float mode = storage.mode;
  const float periodic = storage.periodic;
  const float use_normalize = storage.normalize;
  const float use_origin_offset = storage.use_origin_offset;

  return GPU_stack_link(mat,
                        node,
                        "node_tex_gaborf",
                        in,
                        out,
                        GPU_constant(&dimensions),
                        GPU_constant(&mode),
                        GPU_constant(&use_normalize),
                        GPU_constant(&periodic),
                        GPU_constant(&use_origin_offset));
}

#define GABOR_SEED 1259

typedef struct GaborParams {
  float frequency;
  float radius;
  float impulses;
  float phase;
  float phase_variance;
  float rotation;
  float init_rotation;
  float rot_variance;
  float tilt_randomness;
  float cell_randomness;
  float anisotropy;
  int mode;
  float3 direction;
} GaborParams;

typedef struct FractalParams {
  float octaves;
  float roughness;
  float scl_lacunarity;
  float fre_lacunarity;
  float rot_lacunarity;
} FractalParams;

int impulses_per_cell(const float3 cell, const float impulses, const int seed)
{
  const int n = int(impulses);
  const float rmd = impulses - math::floor(impulses);
  if (rmd > 0.0f) {
    const float t = noise::hash_float_to_float(float4(cell, float(seed - GABOR_SEED)));
    return (t <= rmd) ? n + 1 : n;
  }
  return n;
}

static float3 gabor_kernel(const GaborParams gp,
                           const float3 omega,
                           const float phi,
                           const float3 position,
                           const float dv)
{
  const float g = math::cos(float(M_PI) * math::sqrt(dv)) * 0.5f + 0.5f;
  float3 r = float3(0.0f);
  float h;

  if (gp.mode == SHD_GABOR_MODE_GABOR) {
    h = gp.frequency * math::dot(omega, position) + phi;
    r = float3(math::cos(h), 0.0f, 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR) {
    h = gp.frequency * math::dot(omega, position) + phi;
    r = float3(math::cos(h), math::sin(h), 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_CROSS) {
    h = gp.frequency * math::length(omega * position) + phi;
    r = float3(math::cos(h), 0.0f, 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_CROSS) {
    h = gp.frequency * math::length(omega * position) + phi;
    r = float3(math::cos(h), math::sin(h), 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_RING) {
    h = math::cos(gp.frequency * math::dot(omega, position) + phi) +
        math::cos(gp.frequency * math::length(position) + phi);
    r = float3(h, 0.0f, 0.0f) * 0.5f;
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_RING) {
    h = math::cos(gp.frequency * math::dot(omega, position) + phi) +
        math::cos(gp.frequency * math::length(position) + phi);
    const float h2 = math::sin(gp.frequency * math::dot(omega, position) + phi) +
                     math::sin(gp.frequency * math::length(position) + phi);
    r = float3(h, h2, 0.0f) * 0.5f;
  }
  else if (gp.mode == SHD_GABOR_MODE_SQUARE) {
    const float3 positionyxz = float3(position.y, position.x, position.z);
    h = math::cos(gp.frequency * math::dot(omega, position) + phi) +
        math::cos(gp.frequency * math::dot(omega, positionyxz) + phi);
    r = float3(h, 0.0f, 0.0f) * 0.5f;
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_SQUARE) {
    const float3 positionyxz = float3(position.y, position.x, position.z);
    h = math::cos(gp.frequency * math::dot(omega, position) + phi) +
        math::cos(gp.frequency * math::dot(omega, positionyxz) + phi);
    const float h2 = math::sin(gp.frequency * math::dot(omega, position) + phi) +
                     math::sin(gp.frequency * math::dot(omega, positionyxz) + phi);
    r = float3(h, h2, 0.0f) * 0.5f;
  }

  return r * g;
}

static float3 rotate_z(const float3 &vector, float angle)
{
  float mat[3][3];
  float3 result = vector;
  eul_to_mat3(mat, float3(0.0f, 0.0f, angle));
  mul_m3_v3(mat, result);
  return result;
}

static float3 gabor_sample(const GaborParams gp, const float3 cell, const int seed, float &phi)
{
  const float3 rand_values = noise::hash_float_to_float3(float4(cell, float(seed))) * 2.0f - 1.0f;
  const float pvar = math::interpolate(0.0f, rand_values.z, gp.phase_variance);
  phi = 2.0f * float(M_PI) * pvar + gp.phase;

  const float omega_t = float(M_PI) * (rand_values.x) * gp.rot_variance - gp.rotation -
                        gp.init_rotation;
  const float cos_omega_p = math::clamp(rand_values.y * gp.tilt_randomness, -1.0f, 1.0f);
  const float sin_omega_p = math::sqrt(1.0f - cos_omega_p * cos_omega_p);
  const float sin_omega_t = math::sin(omega_t);
  const float cos_omega_t = math::cos(omega_t);

  return math::interpolate(
      math::normalize(float3(cos_omega_t * sin_omega_p, sin_omega_t * sin_omega_p, cos_omega_p)),
      math::normalize(rotate_z(gp.direction, -gp.rotation)),
      gp.anisotropy);
}

/* Generate noise based on the cell position. */
static float3 gabor_cell_3d(GaborParams &gp, float3 cell, float3 cell_position, int seed)
{
  int num_impulses = impulses_per_cell(cell, gp.impulses, seed);
  float3 sum = float3(0.0f);
  for (int i = 0; i < num_impulses; ++i) {
    float3 rand_position = math::interpolate(
        float3(0.0f),
        noise::hash_float_to_float3(float4(cell, float(seed + i * GABOR_SEED))),
        gp.cell_randomness);

    const float3 kernel_position = (cell_position - rand_position);

    const float dv = math::dot(kernel_position, kernel_position) / gp.radius;

    if (dv <= 1.0f) {
      float phi;
      float3 omega = gabor_sample(gp, cell, seed + (num_impulses + i) * GABOR_SEED, phi);
      sum += gabor_kernel(gp, omega, phi, kernel_position, dv);
    }
  }
  return sum;
}

static float3 gabor_cell_2d(GaborParams &gp, float3 cell, float3 cell_position, int seed)
{
  int num_impulses = impulses_per_cell(cell, gp.impulses, seed);
  float3 sum = float3(0.0f);
  for (int i = 0; i < num_impulses; ++i) {
    float3 rand_position = math::interpolate(
        float3(0.0f),
        noise::hash_float_to_float3(float4(cell, float(seed + i * GABOR_SEED))),
        gp.cell_randomness);
    rand_position.z = 0.0f;

    float3 kernel_position = (cell_position - rand_position);

    const float dv = math::dot(kernel_position, kernel_position) / gp.radius;

    if (dv <= 1.0f) {
      float phi;
      float3 omega = gabor_sample(gp, cell, seed + (num_impulses + i) * GABOR_SEED, phi);
      sum += gabor_kernel(gp, omega, phi, kernel_position, dv);
    }
  }
  return sum;
}

/* Wrap value. */
static float gabor_coord_wrap(const float a, const float b)
{
  return (b != 0.0f) ? a - b * math::floor(a / b) : 0.0f;
}

/* 3*3 cell grid */
static float3 gabor_grid_3d(
    GaborParams &gp, const float3 p, const float scale, const int periodic, const int seed)
{
  const float3 coords = p * scale;
  const float3 position = math::floor(coords);
  const float3 local_position = coords - position;

  float3 sum = float3(0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const float3 cell_offset = float3(i, j, k);

        /* Skip this cell if it's too far away to contribute - Bruemmer.osl */
        const float3 Pr = (float3(i > 0, j > 0, k > 0) - local_position) * cell_offset;
        if (math::dot(Pr, Pr) >= 1.0f) {
          continue;
        }

        float3 cell = position + cell_offset;
        const float3 cell_position = local_position - cell_offset;

        if (periodic) {
          cell[0] = gabor_coord_wrap(cell[0], scale);
          cell[1] = gabor_coord_wrap(cell[1], scale);
          cell[2] = gabor_coord_wrap(cell[2], scale);
        }

        sum += gabor_cell_3d(gp, cell, cell_position, seed);
      }
    }
  }
  return sum;
}

/* 2*2 cell grid */
static float3 gabor_grid_2d(
    GaborParams &gp, const float3 p, const float scale, const int periodic, const int seed)
{
  const float3 coords = float3(p.x, p.y, 0.0f) * scale;
  const float3 position = math::floor(coords);
  const float3 local_position = coords - position;

  float3 sum = float3(0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const float3 cell_offset = float3(i, j, 0.0f);

      /* Skip this cell if it's too far away to contribute - Bruemmer.osl */
      const float3 Pr = (float3(i > 0, j > 0, 0) - local_position) * cell_offset;
      if (math::dot(Pr, Pr) >= 1.0f) {
        continue;
      }

      float3 cell = position + cell_offset;
      const float3 cell_position = local_position - cell_offset;

      if (periodic) {
        cell[0] = gabor_coord_wrap(cell[0], scale);
        cell[1] = gabor_coord_wrap(cell[1], scale);
      }

      sum += gabor_cell_2d(gp, cell, cell_position, seed);
    }
  }
  return sum;
}

/* Fractal gabor noise. Layered noise with octaves, lacunarity, frequency and roughness control. */
static float gabor_fractal_noise(FractalParams fp,
                                 GaborParams &gp,
                                 const float3 p,
                                 const float scale,
                                 const int dimensions,
                                 const int periodic,
                                 const int use_origin_offset)
{
  float fscale = 1.0f;
  float amp = 1.0f;
  float maxamp = 0.0f;
  float3 sum = float3(0.0f);
  float octaves = math::clamp(fp.octaves, 0.0f, 15.0f);
  if (fp.roughness == 0.0f) {
    octaves = 0.0f;
  }
  const int n = int(octaves);
  for (int i = 0; i <= n; i++) {
    const int seed = use_origin_offset * i * GABOR_SEED;
    const float3 t = (dimensions == 3) ? gabor_grid_3d(gp, fscale * p, scale, periodic, seed) :
                                         gabor_grid_2d(gp, fscale * p, scale, periodic, seed);
    gp.frequency *= fp.fre_lacunarity;
    gp.rotation -= fp.rot_lacunarity;
    sum += t * amp;
    maxamp += amp;
    amp *= fp.roughness;
    fscale *= fp.scl_lacunarity;
  }
  float rmd = octaves - math::floor(octaves);
  if (rmd != 0.0f) {
    const int seed = use_origin_offset * (n + 1) * GABOR_SEED;
    const float3 t = (dimensions == 3) ? gabor_grid_3d(gp, fscale * p, scale, periodic, seed) :
                                         gabor_grid_2d(gp, fscale * p, scale, periodic, seed);
    const float3 sum2 = sum + t * amp;
    sum = ((1.0f - rmd) * sum + rmd * sum2);
  }
  sum /= maxamp;

  if (gp.mode == SHD_GABOR_MODE_PHASOR || gp.mode == SHD_GABOR_MODE_PHASOR_RING ||
      gp.mode == SHD_GABOR_MODE_PHASOR_CROSS || gp.mode == SHD_GABOR_MODE_PHASOR_SQUARE)
  {
    const float pn = math::atan2(sum.y, sum.x) / float(M_PI);
    return pn;
  }
  else {
    return sum.x;
  }
}

/* Set parameters used by Gabor noise. */
static GaborParams gabor_parameters(float3 direction,
                                    float frequency,
                                    float radius,
                                    float impulses,
                                    float phase,
                                    float phase_variance,
                                    float rotation,
                                    float rot_variance,
                                    float tilt_randomness,
                                    float cell_randomness,
                                    float anisotropy,
                                    int mode)
{
  GaborParams gp;
  gp.impulses = math::clamp(impulses, 0.0001f, 32.0f);
  gp.rot_variance = rot_variance;
  gp.anisotropy = anisotropy;
  gp.mode = mode;
  gp.direction = direction;
  gp.phase = phase;
  gp.rotation = 0.0f;
  gp.init_rotation = rotation;
  gp.phase_variance = phase_variance;
  gp.tilt_randomness = tilt_randomness;
  gp.cell_randomness = cell_randomness;
  gp.radius = radius;
  gp.frequency = frequency * float(M_PI);
  return gp;
}

static float gabor_noise(const float3 p,
                         const float3 direction,
                         const float scale,
                         const float frequency,
                         const float detail,
                         const float roughness,
                         const float scl_lacunarity,
                         const float fre_lacunarity,
                         const float rot_lacunarity,
                         const float gain,
                         const float radius,
                         const float impulses,
                         const float phase,
                         const float phase_variance,
                         const float rotation,
                         const float rot_variance,
                         const float tilt_randomness,
                         const float cell_randomness,
                         const float anisotropy,
                         const int dimensions,
                         const int mode,
                         const int use_normalize,
                         const int periodic,
                         const int use_origin_offset)
{
  if (impulses == 0.0f || gain == 0.0f || radius <= 0.0f || scale == 0.0f) {
    return (use_normalize) ? 0.5f : 0.0f;
  }

  FractalParams fp;
  fp.roughness = roughness;
  fp.octaves = detail;
  fp.scl_lacunarity = scl_lacunarity;
  fp.fre_lacunarity = fre_lacunarity;
  fp.rot_lacunarity = rot_lacunarity;

  GaborParams gp = gabor_parameters(direction,
                                    frequency,
                                    radius,
                                    impulses,
                                    phase,
                                    phase_variance,
                                    rotation,
                                    rot_variance,
                                    tilt_randomness,
                                    cell_randomness,
                                    anisotropy,
                                    mode);

  float g = gabor_fractal_noise(fp, gp, p, scale, dimensions, periodic, use_origin_offset) * gain;

  if (gp.mode == SHD_GABOR_MODE_GABOR || gp.mode == SHD_GABOR_MODE_RING ||
      gp.mode == SHD_GABOR_MODE_CROSS || gp.mode == SHD_GABOR_MODE_SQUARE)
  {
    const float impulse_scale = impulses > 1.0 ? 1.2613446229f * math::sqrt(gp.impulses) :
                                                 1.2613446229f;
    g = g / impulse_scale;
  }

  if (use_normalize) {
    return math::clamp(0.5f * g + 0.5f, 0.0f, 1.0f);
  }
  return g;
}

static mf::Signature gabor_signature(const char *name)
{
  mf::Signature signature;
  mf::SignatureBuilder builder{name, signature};
  builder.single_input<float3>("Vector");
  builder.single_input<float>("Scale");
  builder.single_input<float>("Gain");
  builder.single_input<float>("Detail");
  builder.single_input<float>("Roughness");
  builder.single_input<float>("Scale Lacunarity");
  builder.single_input<float>("Frequency Lacunarity");
  builder.single_input<float>("Rotation Lacunarity");
  builder.single_input<float>("Frequency");
  builder.single_input<float>("Radius");
  builder.single_input<float>("Impulses");
  builder.single_input<float>("Phase Offset");
  builder.single_input<float>("Phase Variance");
  builder.single_input<float>("Cell Randomness");
  builder.single_input<float>("Rotation");
  builder.single_input<float>("Rotation Variance");
  builder.single_input<float3>("Direction");
  builder.single_input<float>("Tilt Randomness");
  builder.single_input<float>("Anisotropic Factor");
  builder.single_output<float>("Value");
  return signature;
}

class GaborNoiseFunction : public mf::MultiFunction {
 private:
  int dimensions_;
  int mode_;
  int periodic_;
  int normalize_;
  int use_origin_offset_;

 public:
  GaborNoiseFunction(
      int dimensions, int kernel, int periodic, int normalize, int use_origin_offset)
      : dimensions_(dimensions),
        mode_(kernel),
        periodic_(periodic),
        normalize_(normalize),
        use_origin_offset_(use_origin_offset)

  {
    static mf::Signature signature = gabor_signature("GaborNoise");
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float3> &vector = params.readonly_single_input<float3>(param++, "Vector");
    const VArray<float> &scale = params.readonly_single_input<float>(param++, "Scale");
    const VArray<float> &gain = params.readonly_single_input<float>(param++, "Gain");
    const VArray<float> &detail = params.readonly_single_input<float>(param++, "Detail");
    const VArray<float> &roughness = params.readonly_single_input<float>(param++, "Roughness");
    const VArray<float> &scale_lacunarity = params.readonly_single_input<float>(
        param++, "Scale Lacunarity");
    const VArray<float> &freq_lacunarity = params.readonly_single_input<float>(
        param++, "Frequency Lacunarity");
    const VArray<float> &rot_lacunarity = params.readonly_single_input<float>(
        param++, "Rotation Lacunarity");
    const VArray<float> &frequency = params.readonly_single_input<float>(param++, "Frequency");
    const VArray<float> &radius = params.readonly_single_input<float>(param++, "Radius");
    const VArray<float> &impulses = params.readonly_single_input<float>(param++, "Impulses");

    const VArray<float> &phase = params.readonly_single_input<float>(param++, "Phase Offset");
    const VArray<float> &phase_variance = params.readonly_single_input<float>(param++,
                                                                              "Phase Variance");
    const VArray<float> &cell_randomness = params.readonly_single_input<float>(param++,
                                                                               "Cell Randomness");
    const VArray<float> &rotation = params.readonly_single_input<float>(param++, "Rotation");
    const VArray<float> &rot_variance = params.readonly_single_input<float>(param++,
                                                                            "Rotation Variance");
    const VArray<float> &tilt_randomness = params.readonly_single_input<float>(param++,
                                                                               "Tilt Randomness");
    const VArray<float> &anisotropy = params.readonly_single_input<float>(param++,
                                                                          "Anisotropic Factor");
    const VArray<float3> &direction = params.readonly_single_input<float3>(param++, "Direction");

    MutableSpan<float> r_value = params.uninitialized_single_output_if_required<float>(param++,
                                                                                       "Value");

    mask.foreach_index([&](const int64_t i) {
      r_value[i] = gabor_noise(vector[i],
                               direction[i],
                               scale[i],
                               frequency[i],
                               detail[i],
                               roughness[i],
                               scale_lacunarity[i],
                               freq_lacunarity[i],
                               rot_lacunarity[i],
                               gain[i],
                               radius[i],
                               impulses[i],
                               phase[i],
                               phase_variance[i],
                               rotation[i],
                               rot_variance[i],
                               tilt_randomness[i],
                               cell_randomness[i],
                               anisotropy[i],
                               dimensions_,
                               mode_,
                               normalize_,
                               periodic_,
                               use_origin_offset_);
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.allocates_array = false;
    hints.min_grain_size = 100;
    return hints;
  }
};

static void build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeTexGaborF &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<GaborNoiseFunction>(storage.dimensions,
                                                            storage.mode,
                                                            storage.periodic,
                                                            storage.normalize,
                                                            storage.use_origin_offset);
}

}  // namespace blender::nodes::node_shader_tex_gaborf_cc

void register_node_type_sh_tex_gaborf()
{
  namespace file_ns = blender::nodes::node_shader_tex_gaborf_cc;

  static blender::bke::bNodeType ntype;

  sh_fn_node_type_base(&ntype, SH_NODE_TEX_GABORF, "GaborF Texture", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_gaborf;
  node_type_storage(
      &ntype, "NodeTexGaborF", node_free_standard_storage, node_copy_standard_storage);
  ntype.build_multi_function = file_ns::build_multi_function;

  nodeRegisterType(&ntype);
}
