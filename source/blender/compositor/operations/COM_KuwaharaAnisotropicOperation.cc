/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "COM_KuwaharaAnisotropicOperation.h"
#include "BLI_vector.hh"

namespace blender::compositor {

KuwaharaAnisotropicOperation::KuwaharaAnisotropicOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color); 
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);

  this->add_output_socket(DataType::Color);
  // output for debug
  // todo: remove
  this->add_output_socket(DataType::Color);
  this->add_output_socket(DataType::Color);
  this->add_output_socket(DataType::Color);

  this->set_kernel_size(8);

  this->flags_.is_fullframe_operation = true;
}

void KuwaharaAnisotropicOperation::init_execution()
{
  image_reader_ = this->get_input_socket_reader(0);
}

void KuwaharaAnisotropicOperation::deinit_execution()
{
  image_reader_ = nullptr;
}

void KuwaharaAnisotropicOperation::execute_pixel_sampled(float output[4],
                                                         float x,
                                                         float y,
                                                         PixelSampler sampler)
{
 /* Not implemented */
}

void KuwaharaAnisotropicOperation::set_kernel_size(int kernel_size)
{
  kernel_size_ = kernel_size;
}

int KuwaharaAnisotropicOperation::get_kernel_size()
{
  return kernel_size_;
}

void KuwaharaAnisotropicOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                                const rcti &area,
                                                                Span<MemoryBuffer *> inputs)
{
  /*
   Implementation based on Kyprianidis, Jan & Kang, Henry & Döllner, Jürgen. (2009).
   "Image and Video Abstraction by Anisotropic Kuwahara Filtering".
   Comput. Graph. Forum. 28. 1955-1963. 10.1111/j.1467-8659.2009.01574.x.
   Used reference implementation from lime image processing library (MIT license). 
   */

  MemoryBuffer *image = inputs[0];
  MemoryBuffer *s_xx = inputs[1];
  MemoryBuffer *s_yy = inputs[2];
  MemoryBuffer *s_xy = inputs[3];

  // BLI_assert all inputs have same size

  int n_div = 8; // recommended by authors in original paper
  double angle = 2.0 * M_PI / n_div;
  double q = 3.0;
  const float EPS = 1.0e-10;


  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;

    /* Compute orientation */
    // todo: make orientation separate operation

    // for now use green channel to compute orientation
    // todo: convert to HSV and compute orientation and strength on luminance channel
    const float a = s_xx->get_value(x, y, 1);
    const float b = s_xy->get_value(x, y, 1);
    const float c = s_yy->get_value(x, y, 1);


    /* Compute egenvalues of structure tensor */
    const double tr = a + c;
    const double discr = sqrt((a - b)*(a-b) + 4 * b * c);
    const double lambda1 = (tr + discr) / 2;
    const double lambda2 = (tr - discr) / 2;

    /* Compute orientation and its strength based on structure tensor */
    const double orientation = 0.5 * atan2(2 * b, a - c);
    const double strength = (lambda1 == 0 && lambda2 == 0) ? 0 : (lambda1 - lambda2) / (lambda1 + lambda2);

    for(int ch = 0; ch < 3; ch++) {
      // todo: compute anisotropy and weights on luminance channel to avoid color artifacts

      Vector<float> sum(n_div, 0.0f);
      Vector<float> var(n_div, 0.0f);
      Vector<float> weight(n_div, 0.0f);

      float sx = 1.0f / (strength + 1.0f);
      float sy = (1.0f + strength) / 1.0f;
      float theta = -orientation;

      for (int dy = -kernel_size_; dy <= kernel_size_; dy++) {
        for (int dx = -kernel_size_; dx <= kernel_size_; dx++) {
          if (dx == 0 && dy == 0) continue;

          // rotate and scale the kernel. This is the "anisotropic" part.
          int dx2 = static_cast<int>(sx * (cos(theta) * dx - sin(theta) * dy));
          int dy2 = static_cast<int>(sy * (sin(theta) * dx + cos(theta) * dy));
          int xx = x + dx2;
          int yy = y + dy2;

          if (xx >= 0 && yy >= 0 && xx < image->get_width() && yy < image->get_height()) {
            float ddx2 = (float)dx2;
            float ddy2 = (float)dy2;
            float theta = atan2(ddy2, ddx2) + M_PI;
            int t = static_cast<int>(floor(theta / angle)) % n_div;

            float d2 = dx2 * dx2 + dy2 * dy2;
            float g = exp(-d2 / (2.0 * kernel_size_));
            float v = image->get_value(xx, yy, ch);
            sum[t] += g * v;
            var[t] += g * v * v;
            weight[t] += g;
          }
        }
      }

      // Calculate weighted average
      float de = 0.0;
      float nu = 0.0;
      for (int i = 0; i < n_div; i++) {
        sum[i] = weight[i] != 0 ? sum[i] / weight[i] : 0.0;
        var[i] = weight[i] != 0 ? var[i] / weight[i] : 0.0;
        var[i] = var[i] - sum[i] * sum[i];
        var[i] = var[i] > EPS ? sqrt(var[i]) : EPS;
        float w = powf(var[i], -q);

        de += sum[i] * w;
        nu += w;
      }

      float val = nu > EPS ? de / nu : 0.0;
      CLAMP_MAX(val, 1.0f);
      it.out[ch] = val;
    }

    /* No changes for alpha channel */
    it.out[3] = image->get_value(x, y, 3);
  }
}


}  // namespace blender::compositor
