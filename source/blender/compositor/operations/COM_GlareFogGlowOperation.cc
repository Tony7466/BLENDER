/* SPDX-FileCopyrightText: 2011 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <complex>
#include <cstring>
#include <memory>

#if defined(WITH_FFTW3)
#  include <fftw3.h>
#endif

#include "BLI_assert.h"
#include "BLI_cache_mutex.hh"
#include "BLI_fftw.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_base.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"
#include "BLI_threads.h"
#include "BLI_timeit.hh"

#include "COM_GlareFogGlowOperation.h"

namespace blender::compositor {

#if defined(WITH_FFTW3)

/* Given the x and y location in the range from 0 to kernel_size - 1, where kernel_size is odd,
 * compute the fog glow kernel value. The equations are arbitrary and were chosen using visual
 * judgement. The kernel is not normalized and need normalization. */
static float compute_fog_glow_kernel_value(int x, int y, int kernel_size)
{
  const int half_kernel_size = kernel_size / 2;
  const float scale = 0.25f * math::sqrt(math::square(kernel_size));
  const float v = ((y - half_kernel_size) / float(half_kernel_size));
  const float u = ((x - half_kernel_size) / float(half_kernel_size));
  const float r = (math::square(u) + math::square(v)) * scale;
  const float d = -math::sqrt(math::sqrt(math::sqrt(r))) * 9.0f;
  const float kernel_value = math::exp(d);

  const float window = (0.5f + 0.5f * math::cos(u * math::numbers::pi)) *
                       (0.5f + 0.5f * math::cos(v * math::numbers::pi));
  const float windowed_kernel_value = window * kernel_value;

  return windowed_kernel_value;
}

/* A class that stores cached kernels for Fog Glow in the frequency domain. */
class CachedFogGlowKernel {
 private:
  /* Global cache to store cached kernels. There is no mechanism for freeing the cache for the
   * moment, but there are only 4 possible kernel sizes, the smallest of which is 32x64 and the
   * largest of which is 256x512, so it will not occupy a lot of memory. */
  inline static CacheMutex cached_kernels_mutex_;
  inline static Map<int, std::unique_ptr<CachedFogGlowKernel>> cached_kernels_;

  /* The computed kernel in frequency domain, see the implementation for more information. */
  std::complex<float> *kernel_frequency_domain_ = nullptr;

 public:
  /* Computes a Fog Glow kernel of the given size in the frequency domain, zero padding it to the
   * given spatial size. */
  CachedFogGlowKernel(int kernel_size, int2 spatial_size)
  {
    fftw::initialize_float();

    /* The FFTW real to complex transforms utilizes the hermitian symmetry of real transforms and
     * stores only half the output since the other half is redundant, so we only allocate half of
     * the first dimension. See Section 4.3.4 Real-data DFT Array Format in the FFTW manual for
     * more information. */
    const int2 frequency_size = int2(spatial_size.x / 2 + 1, spatial_size.y);

    float *kernel_spatial_domain = fftwf_alloc_real(spatial_size.x * spatial_size.y);
    kernel_frequency_domain_ = reinterpret_cast<std::complex<float> *>(
        fftwf_alloc_complex(frequency_size.x * frequency_size.y));

    /* Create a real to complex plan to transform the kernel to the frequency domain. */
    fftwf_plan kernel_forward_plan = fftwf_plan_dft_r2c_2d(
        spatial_size.y,
        spatial_size.x,
        kernel_spatial_domain,
        reinterpret_cast<fftwf_complex *>(kernel_frequency_domain_),
        FFTW_ESTIMATE);

    /* Compute the kernel while zero padding to match the padded image size. */
    memset(kernel_spatial_domain, 0, sizeof(float) * spatial_size.x * spatial_size.y);
    threading::parallel_for(IndexRange(kernel_size), 1, [&](const IndexRange sub_y_range) {
      for (const int64_t y : sub_y_range) {
        for (const int64_t x : IndexRange(kernel_size)) {
          /* We offset the computed kernel with wrap around such that it is centered at the zero
           * point, which is the expected format for doing circular convolutions in the frequency
           * domain. */
          const int half_kernel_size = kernel_size / 2;
          int64_t output_x = mod_i(x - half_kernel_size, spatial_size.x);
          int64_t output_y = mod_i(y - half_kernel_size, spatial_size.y);

          const float kernel_value = compute_fog_glow_kernel_value(x, y, kernel_size);
          kernel_spatial_domain[output_x + output_y * spatial_size.x] = kernel_value;
        }
      }
    });

    fftwf_execute(kernel_forward_plan);

    fftwf_destroy_plan(kernel_forward_plan);
    fftwf_free(kernel_spatial_domain);
  }

  ~CachedFogGlowKernel()
  {
    fftwf_free(kernel_frequency_domain_);
  }

  static std::complex<float> *get_in_frequency_domain(int kernel_size, int2 spatial_size)
  {
    cached_kernels_mutex_.ensure([&]() {
      cached_kernels_.lookup_or_add_cb(kernel_size, [&]() {
        return std::make_unique<CachedFogGlowKernel>(kernel_size, spatial_size);
      });
    });
    return cached_kernels_.lookup(kernel_size)->kernel_frequency_domain_;
  }
};
#endif

void GlareFogGlowOperation::generate_glare(float *output,
                                           MemoryBuffer *image,
                                           const NodeGlare *settings)
{
#if defined(WITH_FFTW3)
  fftw::initialize_float();

  /* We use an odd sized kernel since an even one will typically introduce a tiny offset as it has
   * no exact center value. */
  const int kernel_size = (1 << settings->size) + 1;

  /* Since we will be doing a circular convolution, we need to zero pad our input image by half the
   * kernel size to avoid the kernel affecting the pixels at the other side of image. Therefore,
   * zero boundary is assumed. */
  const int needed_padding_amount = kernel_size / 2;
  const int2 image_size = int2(image->get_width(), image->get_height());
  const int2 needed_spatial_size = image_size + needed_padding_amount;
  const int2 spatial_size = fftw::optimal_size_for_real_transform(needed_spatial_size);

  /* The FFTW real to complex transforms utilizes the hermitian symmetry of real transforms and
   * stores only half the output since the other half is redundant, so we only allocate half of the
   * first dimension. See Section 4.3.4 Real-data DFT Array Format in the FFTW manual for more
   * information. */
  const int2 frequency_size = int2(spatial_size.x / 2 + 1, spatial_size.y);

  std::complex<float> *kernel_frequency_domain = CachedFogGlowKernel::get_in_frequency_domain(
      kernel_size, spatial_size);

  const int64_t spatial_memory_size = int64_t(spatial_size.x) * spatial_size.y;
  float *image_spatial_domain = fftwf_alloc_real(spatial_memory_size);
  std::complex<float> *image_frequency_domain = reinterpret_cast<std::complex<float> *>(
      fftwf_alloc_complex(frequency_size.x * frequency_size.y));

  fftwf_plan image_forward_plan = fftwf_plan_dft_r2c_2d(
      spatial_size.y,
      spatial_size.x,
      image_spatial_domain,
      reinterpret_cast<fftwf_complex *>(image_frequency_domain),
      FFTW_ESTIMATE);
  fftwf_plan image_backward_plan = fftwf_plan_dft_c2r_2d(
      spatial_size.y,
      spatial_size.x,
      reinterpret_cast<fftwf_complex *>(image_frequency_domain),
      image_spatial_domain,
      FFTW_ESTIMATE);

  /* Convolve each channel of the input. We only process the color channels, the alpha channel is
   * written to the output as is. */
  const int channels_count = 3;
  BLI_assert(image->get_num_channels() == COM_DATA_TYPE_COLOR_CHANNELS);
  for (const int64_t channel : IndexRange(channels_count)) {
    /* Zero pad the image to the required spatial domain size. */
    memset(image_spatial_domain, 0, sizeof(float) * spatial_memory_size);
    threading::parallel_for(IndexRange(image_size.y), 1, [&](const IndexRange sub_y_range) {
      for (const int64_t y : sub_y_range) {
        for (const int64_t x : IndexRange(image_size.x)) {
          const int64_t output_index = x + y * spatial_size.x;
          image_spatial_domain[output_index] = image->get_elem(x, y)[channel];
        }
      }
    });

    fftwf_execute(image_forward_plan);

    /* Multiply the kernel and the image in the frequency domain to perform the convolution. The
     * FFT is not normalized, meaning the result of the FFT followed by an inverse FFT will result
     * in an image that is scaled by a factor of the product of the width and height, so we take
     * that into account by dividing by that scale. See the FFTW manual for more information. */
    const float normalization_scale = float(spatial_size.x) * spatial_size.y;
    threading::parallel_for(IndexRange(frequency_size.y), 1, [&](const IndexRange sub_y_range) {
      for (const int64_t y : sub_y_range) {
        for (const int64_t x : IndexRange(frequency_size.x)) {
          const int64_t index = x + y * frequency_size.x;
          const std::complex<float> image_value = image_frequency_domain[index];
          const std::complex<float> kernel_value = kernel_frequency_domain[index];
          image_frequency_domain[index] = (image_value * kernel_value) / normalization_scale;
        }
      }
    });

    fftwf_execute(image_backward_plan);

    /* Copy the result of the convolution to the output channel. */
    threading::parallel_for(IndexRange(image_size.y), 1, [&](const IndexRange sub_y_range) {
      for (const int64_t y : sub_y_range) {
        for (const int64_t x : IndexRange(image_size.x)) {
          const int64_t output_index = (x + y * image_size.x) * image->get_num_channels();
          const int64_t input_index = x + y * spatial_size.x;
          output[output_index + channel] = image_spatial_domain[input_index];
        }
      }
    });
  }

  fftwf_free(image_spatial_domain);
  fftwf_destroy_plan(image_forward_plan);
  fftwf_destroy_plan(image_backward_plan);
  fftwf_free(image_frequency_domain);

  /* Copy the alpha channel. */
  threading::parallel_for(IndexRange(image_size.y), 1, [&](const IndexRange sub_y_range) {
    for (const int64_t y : sub_y_range) {
      for (const int64_t x : IndexRange(image_size.x)) {
        const int64_t index = (x + y * image_size.x) * image->get_num_channels();
        output[index + 3] = image->get_buffer()[index + 3];
      }
    }
  });
#else
  UNUSED_VARS(output, image, settings);
#endif
}

}  // namespace blender::compositor
