/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_KuwaharaAnisotropicOperation.h"

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"
#include "IMB_colormanagement.h"

namespace blender::compositor {

/* Compute x to the given power, in a safe manner which does not produce non-finite values.
 * For non-positive values of x zero is returned. */
static inline float safe_pow_positive(const float &x, const float &power)
{
  if (x <= 0.0f) {
    return 0.0f;
  }

  return math::pow(x, power);
}
static inline float3 safe_pow_positive(const float3 &x, const float power)
{
  float3 result;
  result.x = safe_pow_positive(x.x, power);
  result.y = safe_pow_positive(x.y, power);
  result.z = safe_pow_positive(x.z, power);
  return result;
}

KuwaharaAnisotropicOperation::KuwaharaAnisotropicOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);

  this->add_output_socket(DataType::Color);

  this->n_div_ = 8;
  this->set_kernel_size(5);

  this->flags_.is_fullframe_operation = true;
}

void KuwaharaAnisotropicOperation::init_execution()
{
  image_reader_ = this->get_input_socket_reader(0);
  s_xx_reader_ = this->get_input_socket_reader(1);
  s_yy_reader_ = this->get_input_socket_reader(2);
  s_xy_reader_ = this->get_input_socket_reader(3);
}

void KuwaharaAnisotropicOperation::deinit_execution()
{
  image_reader_ = nullptr;
}

void KuwaharaAnisotropicOperation::execute_pixel_sampled(float output[4],
                                                         const float x,
                                                         const float y,
                                                         const PixelSampler sampler)
{
  const int x_i = x;
  const int y_i = y;
  const int width = this->get_width();
  const int height = this->get_height();

  BLI_assert(width == s_xx_reader_->get_width());
  BLI_assert(height == s_xx_reader_->get_height());
  BLI_assert(width == s_yy_reader_->get_width());
  BLI_assert(height == s_yy_reader_->get_height());
  BLI_assert(width == s_xy_reader_->get_width());
  BLI_assert(height == s_xy_reader_->get_height());

  /* Values recommended by authors in original paper. */
  const float angle = 2.0f * float(M_PI) / n_div_;
  const float angle_inv = 1.0f / angle;
  const float q = 3.0f;

  /* For now use green channel to compute orientation. */
  /* TODO: convert to HSV and compute orientation and strength on luminance channel */
  float tmp[4];
  s_xx_reader_->read(tmp, x, y, nullptr);
  const float a = tmp[1];
  s_xy_reader_->read(tmp, x, y, nullptr);
  const float b = tmp[1];
  s_yy_reader_->read(tmp, x, y, nullptr);
  const float c = tmp[1];

  /* Compute egenvalues of structure tensor. */
  const float tr = a + c;
  const float discr = math::sqrt((a - b) * (a - b) + 4 * b * c);
  const float lambda1 = (tr + discr) / 2;
  const float lambda2 = (tr - discr) / 2;

  /* Compute orientation and its strength based on structure tensor. */
  const float orientation = 0.5 * math::atan2(2 * b, a - c);
  const float strength = (lambda1 == 0 && lambda2 == 0) ?
                             0 :
                             (lambda1 - lambda2) / (lambda1 + lambda2);

  const float sx = 1.0f / (strength + 1.0f);
  const float sy = (1.0f + strength) / 1.0f;
  const float theta = -orientation;

  const float cos_theta = math::cos(theta);
  const float sin_theta = math::sin(theta);

  Vector<float3> mean(n_div_, float3(0.0f));
  Vector<float3> sum(n_div_, float3(0.0f));
  Vector<float3> var(n_div_, float3(0.0f));
  Vector<float3> weight(n_div_, float3(0.0f));

  for (int dy = -kernel_size_; dy <= kernel_size_; dy++) {
    for (int dx = -kernel_size_; dx <= kernel_size_; dx++) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      /* Rotate and scale the kernel. This is the "anisotropic" part. */
      const int dx2 = int(sx * (cos_theta * dx - sin_theta * dy));
      const int dy2 = int(sy * (sin_theta * dx + cos_theta * dy));

      /* Clamp image to avoid artifacts at borders. */
      const int xx = math::clamp(x_i + dx2, 0, width - 1);
      const int yy = math::clamp(y_i + dy2, 0, height - 1);

      const float ddx2 = float(dx2);
      const float ddy2 = float(dy2);
      const float theta = math::atan2(ddy2, ddx2) + M_PI;
      const int t = int(math::floor(theta * angle_inv)) % n_div_;
      const float d2 = dx2 * dx2 + dy2 * dy2;
      const float g = math::exp(-d2 / (2.0f * kernel_size_));

      float4 color;
      image_reader_->read(&color.x, xx, yy, nullptr);

      const float lum = IMB_colormanagement_get_luminance(color);
      /* TODO(@zazizizou): only compute mean for the selected region. */
      mean[t] += g * color.xyz();
      sum[t] += g * lum;
      var[t] += g * lum * lum;
      weight[t] += g;
    }
  }

  /* Calculate weighted average. */
  float3 de{0.0f};
  float3 nu{0.0f};
  for (int i = 0; i < n_div_; i++) {
    const float3 weight_inv = math::safe_rcp(weight[i]);
    mean[i] = mean[i] * weight_inv;
    sum[i] = sum[i] * weight_inv;
    var[i] = var[i] * weight_inv;
    var[i] = var[i] - sum[i] * sum[i];
    var[i] = math::max(var[i], float3(FLT_EPSILON * FLT_EPSILON));
    var[i] = math::sqrt(var[i]);

    const float3 w = safe_pow_positive(var[i], -q);
    de += mean[i] * w;
    nu += w;
  }

  const float3 val = de * math::safe_rcp(nu);
  output[0] = val.x;
  output[1] = val.y;
  output[2] = val.z;

  /* No changes for alpha channel. */
  image_reader_->read_sampled(tmp, x, y, sampler);
  output[3] = tmp[3];
}

void KuwaharaAnisotropicOperation::set_kernel_size(int kernel_size)
{
  /* Filter will be split into n_div.
   * Add n_div / 2 to avoid artifacts such as random black pixels in image. */
  kernel_size_ = kernel_size + n_div_ / 2;
}

int KuwaharaAnisotropicOperation::get_kernel_size()
{
  return kernel_size_;
}

int KuwaharaAnisotropicOperation::get_n_div()
{
  return n_div_;
}

void KuwaharaAnisotropicOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                                const rcti &area,
                                                                Span<MemoryBuffer *> inputs)
{
  /* Implementation based on Kyprianidis, Jan & Kang, Henry & Döllner, Jürgen. (2009).
   * "Image and Video Abstraction by Anisotropic Kuwahara Filtering".
   * Comput. Graph. Forum. 28. 1955-1963. 10.1111/j.1467-8659.2009.01574.x.
   * Used reference implementation from lime image processing library (MIT license). */

  MemoryBuffer *image = inputs[0];
  MemoryBuffer *s_xx = inputs[1];
  MemoryBuffer *s_yy = inputs[2];
  MemoryBuffer *s_xy = inputs[3];

  const int width = image->get_width();
  const int height = image->get_height();

  BLI_assert(width == s_xx->get_width());
  BLI_assert(height == s_xx->get_height());
  BLI_assert(width == s_yy->get_width());
  BLI_assert(height == s_yy->get_height());
  BLI_assert(width == s_xy->get_width());
  BLI_assert(height == s_xy->get_height());

  /* Values recommended by authors in original paper. */
  const float angle = 2.0f * float(M_PI) / n_div_;
  const float angle_inv = 1.0f / angle;
  const float q = 3.0f;

  Vector<float3> mean(n_div_);
  Vector<float3> sum(n_div_);
  Vector<float3> var(n_div_);
  Vector<float3> weight(n_div_);

  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;

    /* For now use green channel to compute orientation. */
    /* TODO: convert to HSV and compute orientation and strength on luminance channel. */
    const float a = s_xx->get_value(x, y, 1);
    const float b = s_xy->get_value(x, y, 1);
    const float c = s_yy->get_value(x, y, 1);

    /* Compute egenvalues of structure tensor */
    const float tr = a + c;
    const float discr = math::sqrt((a - b) * (a - b) + 4 * b * c);
    const float lambda1 = (tr + discr) / 2;
    const float lambda2 = (tr - discr) / 2;

    /* Compute orientation and its strength based on structure tensor. */
    const float orientation = 0.5 * math::atan2(2 * b, a - c);
    const float strength = (lambda1 == 0 && lambda2 == 0) ?
                               0 :
                               (lambda1 - lambda2) / (lambda1 + lambda2);

    const float sx = 1.0f / (strength + 1.0f);
    const float sy = (1.0f + strength) / 1.0f;
    const float theta = -orientation;

    const float cos_theta = math::cos(theta);
    const float sin_theta = math::sin(theta);

    mean.fill(float3(0, 0, 0));
    sum.fill(float3(0, 0, 0));
    var.fill(float3(0, 0, 0));
    weight.fill(float3(0, 0, 0));

    for (int dy = -kernel_size_; dy <= kernel_size_; dy++) {
      for (int dx = -kernel_size_; dx <= kernel_size_; dx++) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        /* Rotate and scale the kernel. This is the "anisotropic" part. */
        const int dx2 = int(sx * (cos_theta * dx - sin_theta * dy));
        const int dy2 = int(sy * (sin_theta * dx + cos_theta * dy));

        /* Clamp image to avoid artifacts at borders. */
        const int xx = math::clamp(x + dx2, 0, width - 1);
        const int yy = math::clamp(y + dy2, 0, height - 1);

        const float ddx2 = float(dx2);
        const float ddy2 = float(dy2);
        const float theta = math::atan2(ddy2, ddx2) + M_PI;
        const int t = int(math::floor(theta * angle_inv)) % n_div_;
        const float d2 = dx2 * dx2 + dy2 * dy2;
        const float g = math::exp(-d2 / (2.0f * kernel_size_));

        float4 color;
        image->read_elem(xx, yy, &color.x);

        const float lum = IMB_colormanagement_get_luminance(color);
        /* TODO(@zazizizou): only compute mean for the selected region. */
        mean[t] += g * color.xyz();
        sum[t] += g * lum;
        var[t] += g * lum * lum;
        weight[t] += g;
      }
    }

    /* Calculate weighted average. */
    float3 de{0.0f};
    float3 nu{0.0f};
    for (int i = 0; i < n_div_; i++) {
      const float3 weight_inv = math::safe_rcp(weight[i]);
      mean[i] = mean[i] * weight_inv;
      sum[i] = sum[i] * weight_inv;
      var[i] = var[i] * weight_inv;
      var[i] = var[i] - sum[i] * sum[i];
      var[i] = math::max(var[i], float3(FLT_EPSILON * FLT_EPSILON));
      var[i] = math::sqrt(var[i]);

      const float3 w = safe_pow_positive(var[i], -q);
      de += mean[i] * w;
      nu += w;
    }

    const float3 val = de * math::safe_rcp(nu);
    it.out[0] = val.x;
    it.out[1] = val.y;
    it.out[2] = val.z;

    /* No changes for alpha channel. */
    it.out[3] = image->get_value(x, y, 3);
  }
}

}  // namespace blender::compositor
