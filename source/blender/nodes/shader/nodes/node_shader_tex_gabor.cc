/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Implements Gabor noise based on the paper:
 *
 *   Lagae, Ares, et al. "Procedural noise using sparse Gabor convolution." ACM Transactions on
 *   Graphics (TOG) 28.3 (2009): 1-10.
 *
 * But with the improvements from the paper:
 *
 *   Tavernier, Vincent, et al. "Making gabor noise fast and normalized." Eurographics 2019-40th
 *   Annual Conference of the European Association for Computer Graphics. 2019.
 *
 * And compute the Phase and Intensity of the Gabor based on the paper:
 *
 *   Tricard, Thibault, et al. "Procedural phasor noise." ACM Transactions on Graphics (TOG) 38.4
 *   (2019): 1-13.
 */

#include "BLI_hash.hh"
#include "BLI_math_numbers.hh"
#include "BLI_math_vector.hh"
#include "BLI_noise.hh"

#include "BKE_texture.h"

#include "node_shader_util.hh"
#include "node_util.hh"

#include "NOD_multi_function.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_tex_gabor_cc {

NODE_STORAGE_FUNCS(NodeTexGabor)

static void sh_node_tex_gabor_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>("Vector")
      .implicit_field(implicit_field_inputs::position)
      .description(
          "The coordinates at which Gabor noise will be evaluated. The Z component is ignored in "
          "the 2D case");
  b.add_input<decl::Float>("Scale").default_value(5.0f).description(
      "The scale of the Gabor noise");
  b.add_input<decl::Float>("Frequency")
      .default_value(2.0f)
      .min(0.0f)
      .description(
          "The rate at which the Gabor noise changes across space. This is different from the "
          "Scale input in that it only scales perpendicular to the Gabor noise direction");
  b.add_input<decl::Float>("Anisotropy")
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description(
          "The directionality of Gabor noise. 1 means the noise is completely directional, while "
          "0 means the noise is omnidirectional");
  b.add_input<decl::Float>("Orientation", "Orientation 2D")
      .default_value(math::numbers::pi / 4)
      .subtype(PROP_ANGLE)
      .description("The direction of the anisotropic Gabor noise");
  b.add_input<decl::Vector>("Orientation", "Orientation 3D")
      .default_value({math::numbers::sqrt2, math::numbers::sqrt2, 0.0f})
      .subtype(PROP_DIRECTION)
      .description("The direction of the anisotropic Gabor noise");
  b.add_output<decl::Float>("Value").description(
      "The Gabor noise value with both random intensity and phase. This is equal to sine the "
      "phase multiplied by the intensity");
  b.add_output<decl::Float>("Phase").description(
      "The phase of the Gabor noise, which has no random intensity");
  b.add_output<decl::Float>("Intensity")
      .description("The intensity of the Gabor noise, which has no random phase");
}

static void node_shader_buts_tex_gabor(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "gabor_type", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_shader_init_tex_gabor(bNodeTree * /*ntree*/, bNode *node)
{
  NodeTexGabor *storage = MEM_cnew<NodeTexGabor>(__func__);
  BKE_texture_mapping_default(&storage->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&storage->base.color_mapping);

  storage->type = SHD_GABOR_TYPE_2D;

  node->storage = storage;
}

static void node_shader_update_tex_gabor(bNodeTree *ntree, bNode *node)
{
  const NodeTexGabor &storage = node_storage(*node);

  bNodeSocket *orientation_2d_socket = bke::nodeFindSocket(node, SOCK_IN, "Orientation 2D");
  bke::nodeSetSocketAvailability(ntree, orientation_2d_socket, storage.type == SHD_GABOR_TYPE_2D);

  bNodeSocket *orientation_3d_socket = bke::nodeFindSocket(node, SOCK_IN, "Orientation 3D");
  bke::nodeSetSocketAvailability(ntree, orientation_3d_socket, storage.type == SHD_GABOR_TYPE_3D);
}

static int node_shader_gpu_tex_gabor(GPUMaterial *material,
                                     bNode *node,
                                     bNodeExecData * /* execdata */,
                                     GPUNodeStack *in,
                                     GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(material, node, &in[0].link);
  node_shader_gpu_tex_mapping(material, node, in, out);

  const float type = float(node_storage(*node).type);
  return GPU_stack_link(material, node, "node_tex_gabor", in, out, GPU_constant(&type));
}

/* The original Gabor noise paper specifies that the impulses count for each cell should be
 * computed by sampling a Poisson distribution whose mean is the impulse density. However,
 * Tavernier's paper showed that stratified Poisson point sampling is better assuming the weights
 * are sampled using a Bernoulli distribution, as shown in Figure (3). By stratified sampling, they
 * mean a constant number of impulses per cell, so the stratification is the grid itself in that
 * sense, as described in the supplementary material of the paper. */
static constexpr int IMPULSES_COUNT = 8;

/* Computes a 2D Gabor kernel based on Equation (6) in the original Gabor noise paper. Where the
 * frequency argument is the F_0 parameter and the orientation argument is the w_0 parameter. We
 * assume the Gaussian envelope has a unit magnitude, that is, K = 1. That is because we will
 * eventually normalize the final noise value to the unit range, so the multiplication by the
 * magnitude will be canceled by the normalization. Further, we also assume a unit Gaussian width,
 * that is, a = 1. That is because it does not provide much artistic control. It follows that the
 * Gaussian will be truncated at pi.
 *
 * To avoid the discontinuities caused by the aforementioned truncation, the Gaussian is windowed
 * using a Hann window, that is because contrary to the claim made in the original Gabor paper,
 * truncating the Gaussian produces significant artifacts especially when differentiated for bump
 * mapping. The Hann window is C1 continuous and has limited effect on the shape of the Gaussian,
 * so it felt like an appropriate choice.
 *
 * Finally, instead of computing the Gabor value directly, we instead use the complex phasor
 * formulation described in section 3.1.1 in Tricard's paper. That's done to be able to compute the
 * phase and intensity of the Gabor noise after summation based on equations (8) and (9). The
 * return value of the Gabor kernel function is then a complex number whose real value is the
 * value computed in the original Gabor noise paper, and whose imaginary part is the sine
 * counterpart of the real part, which is the only extra computation in the new formulation.
 *
 * Note that while the original Gabor noise paper uses the cosine part of the phasor, that is, the
 * real part of the phasor, we use the sine part instead, that is, the imaginary part of the
 * phasor, as suggested by Tavernier's paper in "Section 3.3. Instance stationarity and
 * normalization", to ensure a zero mean, which should help with normalization. */
static float2 compute_2d_gabor_kernel(const float2 position,
                                      const float frequency,
                                      const float orientation)
{
  /* The kernel is windowed beyond the unit distance, so early exist with a zero for points that
   * are further than a unit radius. */
  const float distance_squared = math::dot(position, position);
  if (distance_squared >= 1.0f) {
    return float2(0.0f);
  }

  const float hann_window = 0.5f + 0.5f * math::cos(math::numbers::pi * distance_squared);
  const float gaussian_envelop = math::exp(-math::numbers::pi * distance_squared);
  const float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  const float2 frequency_vector = frequency * float2(cos(orientation), sin(orientation));
  const float angle = 2.0f * math::numbers::pi * math::dot(position, frequency_vector);
  const float2 phasor = float2(math::cos(angle), math::sin(angle));

  return windowed_gaussian_envelope * phasor;
}

/* Computes the approximate standard deviation of the zero mean normal distribution representing
 * the amplitude distribution of the noise based on Equation (9) in the original Gabor noise paper.
 * For simplicity, the Hann window is ignored and the orientation is fixed since the variance is
 * orientation invariant. We start integrating the squared Gabor kernel with respect to x:
 *
 *   \int_{-\infty}^{-\infty} (e^{- \pi (x^2 + y^2)} cos(2 \pi f_0 x))^2 dx
 *
 * Which gives:
 *
 *  \frac{(e^{2 \pi f_0^2}-1) e^{-2 \pi y^2 - 2 pi f_0^2}}{2^\frac{3}{2}}
 *
 * Then we similarly integrate with respect to y to get:
 *
 *  \frac{1 - e^{-2 \pi f_0^2}}{4}
 *
 * Secondly, we note that the second moment of the weights distribution is 0.5 since it is a
 * fair Bernoulli distribution. So the final standard deviation expression is square root the
 * integral multiplied by the impulse density multiplied by the second moment. */
static float compute_2d_gabor_standard_deviation(float frequency)
{
  const float integral_of_gabor_squared =
      (1.0f - math::exp(-2.0f * math::numbers::pi * frequency * frequency)) / 4.0f;
  const float second_moment = 0.5f;
  return math::sqrt(IMPULSES_COUNT * second_moment * integral_of_gabor_squared);
}

/* Computes the Gabor noise value at the given position for the given cell. This is essentially the
 * sum in Equation (8) in the original Gabor noise paper, where we sum Gabor kernels sampled at a
 * random position with a random weight. The orientation of the kernel is constant for anisotropic
 * noise while it is random for isotropic noise. The original Gabor noise paper mentions that the
 * weights should be uniformly distributed in the [-1, 1] range, however, Tavernier's paper showed
 * that using a Bernoulli distribution yields better results, so that is what we do. */
float2 compute_2d_gabor_noise_cell(const float2 cell,
                                   const float2 position,
                                   const float frequency,
                                   const float isotropy,
                                   const float base_orientation)

{
  float2 noise(0.0f);
  for (const int cell_i : IndexRange(IMPULSES_COUNT)) {
    /* Compute unique seeds for each of the needed random variables. */
    const float3 seed_for_orientation(cell.x, cell.y, cell_i * 3);
    const float3 seed_for_kernel_center(cell.x, cell.y, cell_i * 3 + 1);
    const float3 seed_for_weight(cell.x, cell.y, cell_i * 3 + 2);

    /* For isotropic noise, add a random orientation amount, while for anisotropic noise, use the
     * base orientation. Linearly interpolate between the two cases using the isotropy factor. Note
     * that the random orientation range is to pi as opposed to two pi, that's because the Gabor
     * kernel is symmetric around pi. */
    const float random_orientation = noise::hash_float_to_float(seed_for_orientation) *
                                     math::numbers::pi;
    const float orientation = base_orientation + random_orientation * isotropy;

    const float2 kernel_center = noise::hash_float_to_float2(seed_for_kernel_center);
    const float2 position_in_kernel_space = position - kernel_center;

    /* We either add or subtract the Gabor kernel based on a Bernoulli distribution of equal
     * probability. */
    const float weight = noise::hash_float_to_float(seed_for_weight) < 0.5f ? -1.0f : 1.0f;

    noise += weight * compute_2d_gabor_kernel(position_in_kernel_space, frequency, orientation);
  }
  return noise;
}

/* Computes the Gabor noise value by dividing the space into a grid and evaluating the Gabor noise
 * in the space of each cell of the 3x3 cell neighborhood. */
static float2 compute_2d_gabor_noise(const float2 coordinates,
                                     const float frequency,
                                     const float isotropy,
                                     const float base_orientation)
{
  const float2 cell_position = math::floor(coordinates);
  const float2 local_position = coordinates - cell_position;

  float2 sum(0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const float2 cell_offset = float2(i, j);
      const float2 current_cell_position = cell_position + cell_offset;
      const float2 position_in_cell_space = local_position - cell_offset;
      sum += compute_2d_gabor_noise_cell(
          current_cell_position, position_in_cell_space, frequency, isotropy, base_orientation);
    }
  }

  return sum;
}

/* Identical to compute_2d_gabor_kernel, except it is evaluated in 3D space. Notice that Equation
 * (6) in the original Gabor noise paper computes the frequency vector using (cos(w_0), sin(w_0)),
 * which we also do in the 2D variant, however, for 3D, the orientation is already a unit frequency
 * vector, so we just need to scale it by the frequency value. */
static float2 compute_3d_gabor_kernel(const float3 position,
                                      const float frequency,
                                      const float3 orientation)
{
  /* The kernel is windowed beyond the unit distance, so early exist with a zero for points that
   * are further than a unit radius. */
  const float distance_squared = math::dot(position, position);
  if (distance_squared >= 1.0f) {
    return float2(0.0f);
  }

  const float hann_window = 0.5f + 0.5f * math::cos(math::numbers::pi * distance_squared);
  const float gaussian_envelop = math::exp(-math::numbers::pi * distance_squared);
  const float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  const float3 frequency_vector = frequency * orientation;
  const float angle = 2.0f * math::numbers::pi * math::dot(position, frequency_vector);
  const float2 phasor = float2(math::cos(angle), math::sin(angle));

  return windowed_gaussian_envelope * phasor;
}

/* Identical to compute_2d_gabor_standard_deviation except we do triple integration in 3D. The only
 * difference is the denominator in the integral expression, which is 2^{5 / 2} for the 3D case
 * instead of 4 for the 2D case.  */
static float compute_3d_gabor_standard_deviation(float frequency)
{
  const float integral_of_gabor_squared = (1.0f - math::exp(-2.0f * math::numbers::pi * frequency *
                                                            frequency)) /
                                          math::pow(2.0f, 5.0f / 2.0f);
  const float second_moment = 0.5f;
  return math::sqrt(IMPULSES_COUNT * second_moment * integral_of_gabor_squared);
}

/* Computes the orientation of the Gabor kernel such that it is constant for anisotropic
 * noise while it is random for isotropic noise. We randomize in spherical coordinates for a
 * uniform distribution. */
float3 compute_3d_orientation(float3 orientation, float isotropy, float4 seed)
{
  /* Return the base orientation in case we are completely anisotropic. */
  if (isotropy == 0.0) {
    return orientation;
  }

  /* Compute the orientation in spherical coordinates. */
  float inclination = math::acos(orientation.z);
  float azimuth = math::sign(orientation.y) *
                  math::acos(orientation.x / math::length(float2(orientation.x, orientation.y)));

  /* For isotropic noise, add a random orientation amount, while for anisotropic noise, use the
   * base orientation. Linearly interpolate between the two cases using the isotropy factor. Note
   * that the random orientation range is to pi as opposed to two pi, that's because the Gabor
   * kernel is symmetric around pi. */
  const float2 random_angles = noise::hash_float_to_float2(seed) * math::numbers::pi;
  inclination += random_angles.x * isotropy;
  azimuth += random_angles.y * isotropy;

  /* Convert back to Cartesian coordinates, */
  return float3(math::sin(inclination) * math::cos(azimuth),
                math::sin(inclination) * math::sin(azimuth),
                math::cos(inclination));
}

static float2 compute_3d_gabor_noise_cell(const float3 cell,
                                          const float3 position,
                                          const float frequency,
                                          const float isotropy,
                                          const float3 base_orientation)

{
  float2 noise(0.0f);
  for (const int cell_i : IndexRange(IMPULSES_COUNT)) {
    /* Compute unique seeds for each of the needed random variables. */
    const float4 seed_for_orientation(cell.x, cell.y, cell.z, cell_i * 3);
    const float4 seed_for_kernel_center(cell.x, cell.y, cell.z, cell_i * 3 + 1);
    const float4 seed_for_weight(cell.x, cell.y, cell.z, cell_i * 3 + 2);

    const float3 orientation = compute_3d_orientation(
        base_orientation, isotropy, seed_for_orientation);

    const float3 kernel_center = noise::hash_float_to_float3(seed_for_kernel_center);
    const float3 position_in_kernel_space = position - kernel_center;

    /* We either add or subtract the Gabor kernel based on a Bernoulli distribution of equal
     * probability. */
    const float weight = noise::hash_float_to_float(seed_for_weight) < 0.5f ? -1.0f : 1.0f;

    noise += weight * compute_3d_gabor_kernel(position_in_kernel_space, frequency, orientation);
  }
  return noise;
}

/* Identical to compute_2d_gabor_noise but works in the 3D neighborhood of the noise. */
static float2 compute_3d_gabor_noise(const float3 coordinates,
                                     const float frequency,
                                     const float isotropy,
                                     const float3 base_orientation)
{
  const float3 cell_position = math::floor(coordinates);
  const float3 local_position = coordinates - cell_position;

  float2 sum(0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const float3 cell_offset = float3(i, j, k);
        const float3 current_cell_position = cell_position + cell_offset;
        const float3 position_in_cell_space = local_position - cell_offset;
        sum += compute_3d_gabor_noise_cell(
            current_cell_position, position_in_cell_space, frequency, isotropy, base_orientation);
      }
    }
  }

  return sum;
}

class GaborNoiseFunction : public mf::MultiFunction {
 private:
  int type_;

 public:
  GaborNoiseFunction(const int type) : type_(type)
  {
    BLI_assert(type >= 0 && type <= 1);
    static std::array<mf::Signature, 2> signatures{
        create_signature(SHD_GABOR_TYPE_2D),
        create_signature(SHD_GABOR_TYPE_3D),
    };
    this->set_signature(&signatures[type]);
  }

  static mf::Signature create_signature(const int type)
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"GaborNoise", signature};

    builder.single_input<float3>("Vector");
    builder.single_input<float>("Scale");
    builder.single_input<float>("Frequency");
    builder.single_input<float>("Anistropy");

    if (ELEM(type, SHD_GABOR_TYPE_2D)) {
      builder.single_input<float>("Orientation");
    }
    else {
      builder.single_input<float3>("Orientation");
    }

    builder.single_output<float>("Value", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float>("Phase", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float>("Intensity", mf::ParamFlag::SupportsUnusedOutput);

    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float3> &vector = params.readonly_single_input<float3>(param++, "Vector");
    const VArray<float> &scale = params.readonly_single_input<float>(param++, "Scale");
    const VArray<float> &freq = params.readonly_single_input<float>(param++, "Frequency");
    const VArray<float> &anistropy = params.readonly_single_input<float>(param++, "Anistropy");
    const int orientation_param = param++;
    MutableSpan<float> r_value = params.uninitialized_single_output_if_required<float>(param++,
                                                                                       "Value");
    MutableSpan<float> r_phase = params.uninitialized_single_output_if_required<float>(param++,
                                                                                       "Phase");
    MutableSpan<float> r_intesity = params.uninitialized_single_output_if_required<float>(
        param++, "Intensity");

    const bool compute_value = !r_value.is_empty();
    const bool compute_phase = !r_phase.is_empty();
    const bool compute_intesity = !r_intesity.is_empty();

    if (type_ == SHD_GABOR_TYPE_2D) {
      const VArray<float> &orientation = params.readonly_single_input<float>(orientation_param,
                                                                             "Orientation");
      mask.foreach_index([&](const int64_t i) {
        const float3 scaled_coordinates = vector[i] * scale[i];

        const float isotropy = 1.0f - math::clamp(anistropy[i], 0.0f, 1.0f);
        const float frequency = math::max(0.001f, freq[i]);

        float2 phasor(0.0f);
        float standard_deviation = 1.0f;

        phasor = compute_2d_gabor_noise(float2(scaled_coordinates.x, scaled_coordinates.y),
                                        frequency,
                                        isotropy,
                                        orientation[i]);
        standard_deviation = compute_2d_gabor_standard_deviation(frequency);

        /* Normalize the noise by dividing by six times the standard deviation, which was
         * determined empirically. */
        const float normalization_factor = 6.0f * standard_deviation;

        /* As discussed in compute_2d_gabor_kernel, we use the imaginary part of the phasor as the
         * Gabor value. But remap to [0, 1] from [-1, 1]. */
        if (compute_value) {
          r_value[i] = (phasor.y / normalization_factor) * 0.5f + 0.5f;
        }
        /* Compute the phase based on equation (9) in Tricard's paper. But remap the phase into the
         * [0, 1] range. */
        if (compute_phase) {
          r_phase[i] = (math::atan2(phasor.y, phasor.x) + math::numbers::pi) /
                       (2.0f * math::numbers::pi);
        }

        /* Compute the intensity based on equation (8) in Tricard's paper. */
        if (compute_intesity) {
          r_intesity[i] = math::length(phasor) / normalization_factor;
        }
      });
    }
    else if (type_ == SHD_GABOR_TYPE_3D) {
      const VArray<float3> &orientation = params.readonly_single_input<float3>(orientation_param,
                                                                               "Orientation");
      mask.foreach_index([&](const int64_t i) {
        const float3 scaled_coordinates = vector[i] * scale[i];

        const float isotropy = 1.0f - math::clamp(anistropy[i], 0.0f, 1.0f);
        const float frequency = math::max(0.001f, freq[i]);

        float2 phasor(0.0f);
        float standard_deviation = 1.0f;

        phasor = compute_3d_gabor_noise(
            scaled_coordinates, frequency, isotropy, math::normalize(orientation[i]));
        standard_deviation = compute_3d_gabor_standard_deviation(frequency);

        /* Normalize the noise by dividing by six times the standard deviation, which was
         * determined empirically. */
        const float normalization_factor = 6.0f * standard_deviation;

        /* As discussed in compute_2d_gabor_kernel, we use the imaginary part of the phasor as the
         * Gabor value. But remap to [0, 1] from [-1, 1]. */
        if (compute_value) {
          r_value[i] = (phasor.y / normalization_factor) * 0.5f + 0.5f;
        }
        /* Compute the phase based on equation (9) in Tricard's paper. But remap the phase into the
         * [0, 1] range. */
        if (compute_phase) {
          r_phase[i] = (math::atan2(phasor.y, phasor.x) + math::numbers::pi) /
                       (2.0f * math::numbers::pi);
        }

        /* Compute the intensity based on equation (8) in Tricard's paper. */
        if (compute_intesity) {
          r_intesity[i] = math::length(phasor) / normalization_factor;
        }
      });
    }
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
  const NodeTexGabor &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<GaborNoiseFunction>(storage.type);
}

}  // namespace blender::nodes::node_shader_tex_gabor_cc

void register_node_type_sh_tex_gabor()
{
  namespace file_ns = blender::nodes::node_shader_tex_gabor_cc;

  static blender::bke::bNodeType ntype;

  sh_fn_node_type_base(&ntype, SH_NODE_TEX_GABOR, "Gabor Texture", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::sh_node_tex_gabor_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_tex_gabor;
  ntype.initfunc = file_ns::node_shader_init_tex_gabor;
  node_type_storage(
      &ntype, "NodeTexGabor", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_gabor;
  ntype.updatefunc = file_ns::node_shader_update_tex_gabor;
  ntype.build_multi_function = file_ns::build_multi_function;

  nodeRegisterType(&ntype);
}
