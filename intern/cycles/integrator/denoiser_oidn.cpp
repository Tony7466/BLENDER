/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "integrator/denoiser_oidn.h"

#include <array>

#include "device/device.h"
#include "device/queue.h"
#include "integrator/pass_accessor_cpu.h"
#include "session/buffers.h"
#include "util/array.h"
#include "util/log.h"
#include "util/openimagedenoise.h"

#include "kernel/device/cpu/compat.h"
#include "kernel/device/cpu/kernel.h"

#if OIDN_VERSION_MAJOR < 2
#  define oidnExecuteFilterAsync oidnExecuteFilter
#  define oidnSetFilterInt oidnSetFilter1i
#  define oidnSetFilterBool oidnSetFilter1b
#endif

CCL_NAMESPACE_BEGIN

thread_mutex OIDNDenoiser::mutex_;
bool OIDNDenoiser::is_device_supported(const DeviceInfo &device)
{

  switch (device.type) {
    case DEVICE_CPU:
      return openimagedenoise_supported();
#if OIDN_VERSION_MAJOR >= 2
#  ifdef OIDN_DEVICE_SYCL
    case DEVICE_ONEAPI:
      return true;
#  endif
#endif
    default:
      return false;
  }
}

OIDNDenoiser::OIDNDenoiser(Device *path_trace_device, const DenoiseParams &params)
    : DenoiserGPU(path_trace_device, params)
{
  DCHECK_EQ(params.type, DENOISER_OPENIMAGEDENOISE);
}

OIDNDenoiser::~OIDNDenoiser()
{
  if (albedo_memory_) {
    delete albedo_memory_;
  }
  if (normal_memory_) {
    delete normal_memory_;
  }
#ifdef WITH_OPENIMAGEDENOISE
  if (albedo_filter_) {
    oidnReleaseFilter(albedo_filter_);
  }
  if (normal_filter_) {
    oidnReleaseFilter(normal_filter_);
  }
  if (oidn_filter_) {
    oidnReleaseFilter(oidn_filter_);
  }
  if (oidn_device_) {
    oidnReleaseDevice(oidn_device_);
  }
#endif
}

#ifdef WITH_OPENIMAGEDENOISE
static bool oidn_progress_monitor_function(void *user_ptr, double /*n*/)
{
  OIDNDenoiser *oidn_denoiser = reinterpret_cast<OIDNDenoiser *>(user_ptr);
  return !oidn_denoiser->is_cancelled();
}

class OIDNPass {
 public:
  OIDNPass() = default;

  OIDNPass(const BufferParams &buffer_params,
           const char *name,
           PassType type,
           PassMode mode = PassMode::NOISY)
      : name(name), type(type), mode(mode)
  {
    offset = buffer_params.get_pass_offset(type, mode);
    need_scale = (type == PASS_DENOISING_ALBEDO || type == PASS_DENOISING_NORMAL);

    const PassInfo pass_info = Pass::get_info(type);
    num_components = pass_info.num_components;
    use_compositing = pass_info.use_compositing;
    use_denoising_albedo = pass_info.use_denoising_albedo;
  }

  inline operator bool() const
  {
    return name[0] != '\0';
  }

  /* Name of an image which will be passed to the OIDN library.
   * Should be one of the following: color, albedo, normal, output.
   * The albedo and normal images are optional. */
  const char *name = "";

  PassType type = PASS_NONE;
  PassMode mode = PassMode::NOISY;
  int num_components = -1;
  bool use_compositing = false;
  bool use_denoising_albedo = true;

  /* Offset of beginning of this pass in the render buffers. */
  int offset = -1;

  /* Denotes whether the data is to be scaled down with the number of passes.
   * Is required for albedo and normal passes. The color pass OIDN will perform auto-exposure, so
   * scaling is not needed for the color pass unless adaptive sampling is used.
   *
   * NOTE: Do not scale the output pass, as that requires to be a pointer in the original buffer.
   * All the scaling on the output needed for integration with adaptive sampling will happen
   * outside of generic pass handling. */
  bool need_scale = false;

  /* The content of the pass has been pre-filtered. */
  bool is_filtered = false;

  /* For the scaled passes, the data which holds values of scaled pixels. */
  array<float> scaled_buffer;
};

class OIDNDenoiseContext {
 public:
  OIDNDenoiseContext(OIDNDenoiser *denoiser,
                     const DenoiseParams &denoise_params,
                     const BufferParams &buffer_params,
                     RenderBuffers *render_buffers,
                     const int num_samples,
                     const bool allow_inplace_modification)
      : denoiser_(denoiser),
        denoise_params_(denoise_params),
        buffer_params_(buffer_params),
        render_buffers_(render_buffers),
        num_samples_(num_samples),
        allow_inplace_modification_(allow_inplace_modification),
        pass_sample_count_(buffer_params_.get_pass_offset(PASS_SAMPLE_COUNT))
  {
    if (denoise_params_.use_pass_albedo) {
      oidn_albedo_pass_ = OIDNPass(buffer_params_, "albedo", PASS_DENOISING_ALBEDO);
    }

    if (denoise_params_.use_pass_normal) {
      oidn_normal_pass_ = OIDNPass(buffer_params_, "normal", PASS_DENOISING_NORMAL);
    }
  }

  bool need_denoising() const
  {
    if (buffer_params_.width == 0 && buffer_params_.height == 0) {
      return false;
    }

    return true;
  }

  /* Make the guiding passes available by a sequential denoising of various passes. */
  void read_guiding_passes()
  {
    read_guiding_pass(oidn_albedo_pass_);
    read_guiding_pass(oidn_normal_pass_);
  }

  void denoise_pass(const PassType pass_type)
  {
    OIDNPass oidn_color_pass(buffer_params_, "color", pass_type);
    if (oidn_color_pass.offset == PASS_UNUSED) {
      return;
    }

    if (oidn_color_pass.use_denoising_albedo) {
      if (albedo_replaced_with_fake_) {
        LOG(ERROR) << "Pass which requires albedo is denoised after fake albedo has been set.";
        return;
      }
    }

    OIDNPass oidn_output_pass(buffer_params_, "output", pass_type, PassMode::DENOISED);
    if (oidn_output_pass.offset == PASS_UNUSED) {
      LOG(DFATAL) << "Missing denoised pass " << pass_type_as_string(pass_type);
      return;
    }

    OIDNPass oidn_color_access_pass = read_input_pass(oidn_color_pass, oidn_output_pass);

    oidn::DeviceRef oidn_device = oidn::newDevice(oidn::DeviceType::CPU);
    oidn_device.set("setAffinity", false);
    oidn_device.commit();

    /* Create a filter for denoising a beauty (color) image using prefiltered auxiliary images too.
     */
    oidn::FilterRef oidn_filter = oidn_device.newFilter("RT");
    set_input_pass(oidn_filter, oidn_color_access_pass);
    set_guiding_passes(oidn_filter, oidn_color_pass);
    set_output_pass(oidn_filter, oidn_output_pass);
    oidn_filter.setProgressMonitorFunction(oidn_progress_monitor_function, denoiser_);
    oidn_filter.set("hdr", true);
    oidn_filter.set("srgb", false);
    if (denoise_params_.prefilter == DENOISER_PREFILTER_NONE ||
        denoise_params_.prefilter == DENOISER_PREFILTER_ACCURATE)
    {
      oidn_filter.set("cleanAux", true);
    }
    oidn_filter.commit();

    filter_guiding_pass_if_needed(oidn_device, oidn_albedo_pass_);
    filter_guiding_pass_if_needed(oidn_device, oidn_normal_pass_);

    /* Filter the beauty image. */
    oidn_filter.execute();

    /* Check for errors. */
    const char *error_message;
    const oidn::Error error = oidn_device.getError(error_message);
    if (error != oidn::Error::None && error != oidn::Error::Cancelled) {
      LOG(ERROR) << "OpenImageDenoise error: " << error_message;
    }

    postprocess_output(oidn_color_pass, oidn_output_pass);
  }

 protected:
  void filter_guiding_pass_if_needed(oidn::DeviceRef &oidn_device, OIDNPass &oidn_pass)
  {
    if (denoise_params_.prefilter != DENOISER_PREFILTER_ACCURATE || !oidn_pass ||
        oidn_pass.is_filtered)
    {
      return;
    }

    oidn::FilterRef oidn_filter = oidn_device.newFilter("RT");
    set_pass(oidn_filter, oidn_pass);
    set_output_pass(oidn_filter, oidn_pass);
    oidn_filter.commit();
    oidn_filter.execute();

    oidn_pass.is_filtered = true;
  }

  /* Make pixels of a guiding pass available by the denoiser. */
  void read_guiding_pass(OIDNPass &oidn_pass)
  {
    if (!oidn_pass) {
      return;
    }

    DCHECK(!oidn_pass.use_compositing);

    if (denoise_params_.prefilter != DENOISER_PREFILTER_ACCURATE &&
        !is_pass_scale_needed(oidn_pass)) {
      /* Pass data is available as-is from the render buffers. */
      return;
    }

    if (allow_inplace_modification_) {
      scale_pass_in_render_buffers(oidn_pass);
      return;
    }

    read_pass_pixels_into_buffer(oidn_pass);
  }

  /* Special reader of the input pass.
   * To save memory it will read pixels into the output, and let the denoiser to perform an
   * in-place operation. */
  OIDNPass read_input_pass(OIDNPass &oidn_input_pass, const OIDNPass &oidn_output_pass)
  {
    const bool use_compositing = oidn_input_pass.use_compositing;

    /* Simple case: no compositing is involved, no scaling is needed.
     * The pass pixels will be referenced as-is, without extra processing. */
    if (!use_compositing && !is_pass_scale_needed(oidn_input_pass)) {
      return oidn_input_pass;
    }

    float *buffer_data = render_buffers_->buffer.data();
    float *pass_data = buffer_data + oidn_output_pass.offset;

    PassAccessor::Destination destination(pass_data, 3);
    destination.pixel_stride = buffer_params_.pass_stride;

    read_pass_pixels(oidn_input_pass, destination);

    OIDNPass oidn_input_pass_at_output = oidn_input_pass;
    oidn_input_pass_at_output.offset = oidn_output_pass.offset;

    return oidn_input_pass_at_output;
  }

  /* Read pass pixels using PassAccessor into the given destination. */
  void read_pass_pixels(const OIDNPass &oidn_pass, const PassAccessor::Destination &destination)
  {
    PassAccessor::PassAccessInfo pass_access_info;
    pass_access_info.type = oidn_pass.type;
    pass_access_info.mode = oidn_pass.mode;
    pass_access_info.offset = oidn_pass.offset;

    /* Denoiser operates on passes which are used to calculate the approximation, and is never used
     * on the approximation. The latter is not even possible because OIDN does not support
     * denoising of semi-transparent pixels. */
    pass_access_info.use_approximate_shadow_catcher = false;
    pass_access_info.use_approximate_shadow_catcher_background = false;
    pass_access_info.show_active_pixels = false;

    /* OIDN will perform an auto-exposure, so it is not required to know exact exposure configured
     * by users. What is important is to use same exposure for read and write access of the pass
     * pixels. */
    const PassAccessorCPU pass_accessor(pass_access_info, 1.0f, num_samples_);

    BufferParams buffer_params = buffer_params_;
    buffer_params.window_x = 0;
    buffer_params.window_y = 0;
    buffer_params.window_width = buffer_params.width;
    buffer_params.window_height = buffer_params.height;

    pass_accessor.get_render_tile_pixels(render_buffers_, buffer_params, destination);
  }

  /* Read pass pixels using PassAccessor into a temporary buffer which is owned by the pass.. */
  void read_pass_pixels_into_buffer(OIDNPass &oidn_pass)
  {
    VLOG_WORK << "Allocating temporary buffer for pass " << oidn_pass.name << " ("
              << pass_type_as_string(oidn_pass.type) << ")";

    const int64_t width = buffer_params_.width;
    const int64_t height = buffer_params_.height;

    array<float> &scaled_buffer = oidn_pass.scaled_buffer;
    scaled_buffer.resize(width * height * 3);

    const PassAccessor::Destination destination(scaled_buffer.data(), 3);

    read_pass_pixels(oidn_pass, destination);
  }

  /* Set OIDN image to reference pixels from the given render buffer pass.
   * No transform to the pixels is done, no additional memory is used. */
  void set_pass_referenced(oidn::FilterRef &oidn_filter,
                           const char *name,
                           const OIDNPass &oidn_pass)
  {
    const int64_t x = buffer_params_.full_x;
    const int64_t y = buffer_params_.full_y;
    const int64_t width = buffer_params_.width;
    const int64_t height = buffer_params_.height;
    const int64_t offset = buffer_params_.offset;
    const int64_t stride = buffer_params_.stride;
    const int64_t pass_stride = buffer_params_.pass_stride;

    const int64_t pixel_index = offset + x + y * stride;
    const int64_t buffer_offset = pixel_index * pass_stride;

    float *buffer_data = render_buffers_->buffer.data();

    oidn_filter.setImage(name,
                         buffer_data + buffer_offset + oidn_pass.offset,
                         oidn::Format::Float3,
                         width,
                         height,
                         0,
                         pass_stride * sizeof(float),
                         stride * pass_stride * sizeof(float));
  }

  void set_pass_from_buffer(oidn::FilterRef &oidn_filter, const char *name, OIDNPass &oidn_pass)
  {
    const int64_t width = buffer_params_.width;
    const int64_t height = buffer_params_.height;

    oidn_filter.setImage(
        name, oidn_pass.scaled_buffer.data(), oidn::Format::Float3, width, height, 0, 0, 0);
  }

  void set_pass(oidn::FilterRef &oidn_filter, OIDNPass &oidn_pass)
  {
    set_pass(oidn_filter, oidn_pass.name, oidn_pass);
  }
  void set_pass(oidn::FilterRef &oidn_filter, const char *name, OIDNPass &oidn_pass)
  {
    if (oidn_pass.scaled_buffer.empty()) {
      set_pass_referenced(oidn_filter, name, oidn_pass);
    }
    else {
      set_pass_from_buffer(oidn_filter, name, oidn_pass);
    }
  }

  void set_input_pass(oidn::FilterRef &oidn_filter, OIDNPass &oidn_pass)
  {
    set_pass_referenced(oidn_filter, oidn_pass.name, oidn_pass);
  }

  void set_guiding_passes(oidn::FilterRef &oidn_filter, OIDNPass &oidn_pass)
  {
    if (oidn_albedo_pass_) {
      if (oidn_pass.use_denoising_albedo) {
        set_pass(oidn_filter, oidn_albedo_pass_);
      }
      else {
        /* NOTE: OpenImageDenoise library implicitly expects albedo pass when normal pass has been
         * provided. */
        set_fake_albedo_pass(oidn_filter);
      }
    }

    if (oidn_normal_pass_) {
      set_pass(oidn_filter, oidn_normal_pass_);
    }
  }

  void set_fake_albedo_pass(oidn::FilterRef &oidn_filter)
  {
    const int64_t width = buffer_params_.width;
    const int64_t height = buffer_params_.height;

    if (!albedo_replaced_with_fake_) {
      const int64_t num_pixel_components = width * height * 3;
      oidn_albedo_pass_.scaled_buffer.resize(num_pixel_components);

      for (int i = 0; i < num_pixel_components; ++i) {
        oidn_albedo_pass_.scaled_buffer[i] = 0.5f;
      }

      albedo_replaced_with_fake_ = true;
    }

    set_pass(oidn_filter, oidn_albedo_pass_);
  }

  void set_output_pass(oidn::FilterRef &oidn_filter, OIDNPass &oidn_pass)
  {
    set_pass(oidn_filter, "output", oidn_pass);
  }

  /* Scale output pass to match adaptive sampling per-pixel scale, as well as bring alpha channel
   * back. */
  void postprocess_output(const OIDNPass &oidn_input_pass, const OIDNPass &oidn_output_pass)
  {
    kernel_assert(oidn_input_pass.num_components == oidn_output_pass.num_components);

    const int64_t x = buffer_params_.full_x;
    const int64_t y = buffer_params_.full_y;
    const int64_t width = buffer_params_.width;
    const int64_t height = buffer_params_.height;
    const int64_t offset = buffer_params_.offset;
    const int64_t stride = buffer_params_.stride;
    const int64_t pass_stride = buffer_params_.pass_stride;
    const int64_t row_stride = stride * pass_stride;

    const int64_t pixel_offset = offset + x + y * stride;
    const int64_t buffer_offset = (pixel_offset * pass_stride);

    float *buffer_data = render_buffers_->buffer.data();

    const bool has_pass_sample_count = (pass_sample_count_ != PASS_UNUSED);
    const bool need_scale = has_pass_sample_count || oidn_input_pass.use_compositing;

    for (int y = 0; y < height; ++y) {
      float *buffer_row = buffer_data + buffer_offset + y * row_stride;
      for (int x = 0; x < width; ++x) {
        float *buffer_pixel = buffer_row + x * pass_stride;
        float *denoised_pixel = buffer_pixel + oidn_output_pass.offset;

        if (need_scale) {
          const float pixel_scale = has_pass_sample_count ?
                                        __float_as_uint(buffer_pixel[pass_sample_count_]) :
                                        num_samples_;

          denoised_pixel[0] = denoised_pixel[0] * pixel_scale;
          denoised_pixel[1] = denoised_pixel[1] * pixel_scale;
          denoised_pixel[2] = denoised_pixel[2] * pixel_scale;
        }

        if (oidn_output_pass.num_components == 3) {
          /* Pass without alpha channel. */
        }
        else if (!oidn_input_pass.use_compositing) {
          /* Currently compositing passes are either 3-component (derived by dividing light passes)
           * or do not have transparency (shadow catcher). Implicitly rely on this logic, as it
           * simplifies logic and avoids extra memory allocation. */
          const float *noisy_pixel = buffer_pixel + oidn_input_pass.offset;
          denoised_pixel[3] = noisy_pixel[3];
        }
        else {
          /* Assigning to zero since this is a default alpha value for 3-component passes, and it
           * is an opaque pixel for 4 component passes. */
          denoised_pixel[3] = 0;
        }
      }
    }
  }

  bool is_pass_scale_needed(OIDNPass &oidn_pass) const
  {
    if (pass_sample_count_ != PASS_UNUSED) {
      /* With adaptive sampling pixels will have different number of samples in them, so need to
       * always scale the pass to make pixels uniformly sampled. */
      return true;
    }

    if (!oidn_pass.need_scale) {
      return false;
    }

    if (num_samples_ == 1) {
      /* If the avoid scaling if there is only one sample, to save up time (so we don't divide
       * buffer by 1). */
      return false;
    }

    return true;
  }

  void scale_pass_in_render_buffers(OIDNPass &oidn_pass)
  {
    const int64_t x = buffer_params_.full_x;
    const int64_t y = buffer_params_.full_y;
    const int64_t width = buffer_params_.width;
    const int64_t height = buffer_params_.height;
    const int64_t offset = buffer_params_.offset;
    const int64_t stride = buffer_params_.stride;
    const int64_t pass_stride = buffer_params_.pass_stride;
    const int64_t row_stride = stride * pass_stride;

    const int64_t pixel_offset = offset + x + y * stride;
    const int64_t buffer_offset = (pixel_offset * pass_stride);

    float *buffer_data = render_buffers_->buffer.data();

    const bool has_pass_sample_count = (pass_sample_count_ != PASS_UNUSED);

    for (int y = 0; y < height; ++y) {
      float *buffer_row = buffer_data + buffer_offset + y * row_stride;
      for (int x = 0; x < width; ++x) {
        float *buffer_pixel = buffer_row + x * pass_stride;
        float *pass_pixel = buffer_pixel + oidn_pass.offset;

        const float pixel_scale = 1.0f / (has_pass_sample_count ?
                                              __float_as_uint(buffer_pixel[pass_sample_count_]) :
                                              num_samples_);

        pass_pixel[0] = pass_pixel[0] * pixel_scale;
        pass_pixel[1] = pass_pixel[1] * pixel_scale;
        pass_pixel[2] = pass_pixel[2] * pixel_scale;
      }
    }
  }

  OIDNDenoiser *denoiser_ = nullptr;

  const DenoiseParams &denoise_params_;
  const BufferParams &buffer_params_;
  RenderBuffers *render_buffers_ = nullptr;
  int num_samples_ = 0;
  bool allow_inplace_modification_ = false;
  int pass_sample_count_ = PASS_UNUSED;

  /* Optional albedo and normal passes, reused by denoising of different pass types. */
  OIDNPass oidn_albedo_pass_;
  OIDNPass oidn_normal_pass_;

  /* For passes which don't need albedo channel for denoising we replace the actual albedo with
   * the (0.5, 0.5, 0.5). This flag indicates that the real albedo pass has been replaced with
   * the fake values and denoising of passes which do need albedo can no longer happen. */
  bool albedo_replaced_with_fake_ = false;
};

static unique_ptr<DeviceQueue> create_device_queue(const RenderBuffers *render_buffers)
{
  Device *device = render_buffers->buffer.device;
  if (device->info.has_gpu_queue) {
    return device->gpu_queue_create();
  }
  return nullptr;
}

static void copy_render_buffers_from_device(unique_ptr<DeviceQueue> &queue,
                                            RenderBuffers *render_buffers)
{
  if (queue) {
    queue->copy_from_device(render_buffers->buffer);
    queue->synchronize();
  }
  else {
    render_buffers->copy_from_device();
  }
}

static void copy_render_buffers_to_device(unique_ptr<DeviceQueue> &queue,
                                          RenderBuffers *render_buffers)
{
  if (queue) {
    queue->copy_to_device(render_buffers->buffer);
    queue->synchronize();
  }
  else {
    render_buffers->copy_to_device();
  }
}

#endif

bool OIDNDenoiser::denoise_buffer(const BufferParams &buffer_params,
                                  RenderBuffers *render_buffers,
                                  const int num_samples,
                                  bool allow_inplace_modification)
{
  if (denoiser_device_->info.type != DEVICE_CPU && !cpu_fallback_) {
    return DenoiserGPU::denoise_buffer(
        buffer_params, render_buffers, num_samples, allow_inplace_modification);
  }

  DCHECK(openimagedenoise_supported())
      << "OpenImageDenoiser is not supported on this platform or build.";

#ifdef WITH_OPENIMAGEDENOISE
  thread_scoped_lock lock(mutex_);
  /* Make sure the host-side data is available for denoising. */
  unique_ptr<DeviceQueue> queue = create_device_queue(render_buffers);
  copy_render_buffers_from_device(queue, render_buffers);

  OIDNDenoiseContext context(
      this, params_, buffer_params, render_buffers, num_samples, allow_inplace_modification);

  if (context.need_denoising()) {
    context.read_guiding_passes();

    const std::array<PassType, 3> passes = {
        {/* Passes which will use real albedo when it is available. */
         PASS_COMBINED,
         PASS_SHADOW_CATCHER_MATTE,

         /* Passes which do not need albedo and hence if real is present it needs to become fake.
          */
         PASS_SHADOW_CATCHER}};

    for (const PassType pass_type : passes) {
      context.denoise_pass(pass_type);
      if (is_cancelled()) {
        return false;
      }
    }

    /* TODO: It may be possible to avoid this copy, but we have to ensure that when other code
     * copies data from the device it doesn't overwrite the denoiser buffers. */
    copy_render_buffers_to_device(queue, render_buffers);
  }
#else
  (void)buffer_params;
  (void)render_buffers;
  (void)num_samples;
  (void)allow_inplace_modification;
#endif

  /* This code is not supposed to run when compiled without OIDN support, so can assume if we made
   * it up here all passes are properly denoised. */
  return true;
}

uint OIDNDenoiser::get_device_type_mask() const
{
  uint device_mask = 0;
#ifdef WITH_OPENIMAGEDENOISE
#  if OIDN_VERSION_MAJOR < 2
  device_mask = DEVICE_MASK_CPU;
#  else
#    ifdef OIDN_DEVICE_CPU
  device_mask |= DEVICE_MASK_CPU;
#    endif
#    ifdef OIDN_DEVICE_SYCL
  device_mask |= DEVICE_MASK_ONEAPI;
#    endif
#  endif
#endif
  return device_mask;
}

Device *OIDNDenoiser::ensure_denoiser_device(Progress *progress)
{
#ifndef WITH_OPENIMAGEDENOISE
  (void)progress;
  path_trace_device_->set_error("Build without OpenImageDenoiser");
  return nullptr;
#else
#  if OIDN_VERSION_MAJOR < 2
  (void)progress;
  if (!openimagedenoise_supported()) {
    path_trace_device_->set_error(
        "OpenImageDenoiser is not supported on this CPU: missing SSE 4.1 support");
    return nullptr;
  }
  return Denoiser::ensure_denoiser_device(progress);
#  else
  Device *denoiser_device = Denoiser::ensure_denoiser_device(progress);
  if (!denoiser_device) {
    return nullptr;
  }

  if (denoiser_device->info.type != DEVICE_CPU) {
    return DenoiserGPU::ensure_denoiser_device(progress);
  }
  return denoiser_device;
#  endif
#endif
}

bool OIDNDenoiser::denoise_create_if_needed(DenoiseContext &context)
{
#ifdef WITH_OPENIMAGEDENOISE
  const bool recreate_denoiser = (oidn_device_ == nullptr) || (oidn_filter_ == nullptr) ||
                                 (use_pass_albedo_ != context.use_pass_albedo) ||
                                 (use_pass_normal_ != context.use_pass_normal) ||
                                 (use_pass_motion_ != context.use_pass_motion);
  if (!recreate_denoiser) {
    return true;
  }

  /* Destroy existing handle before creating new one. */
  if (oidn_filter_) {
    oidnReleaseFilter(oidn_filter_);
  }

  if (oidn_device_) {
    oidnReleaseDevice(oidn_device_);
  }

  cpu_fallback_ = false;

#  if OIDN_VERSION_MAJOR < 2
  oidn_device_ = oidnNewDevice(OIDN_DEVICE_TYPE_CPU);
#  else
  switch (denoiser_device_->info.type) {
#    if defined(OIDN_DEVICE_SYCL)
    case DEVICE_ONEAPI:
      oidn_device_ = oidnNewDevice(OIDN_DEVICE_TYPE_SYCL);
      denoiser_queue_->init_execution();
      break;
#    endif
    /* Devices without explicit support fall through to CPU backend. */
    default:
      oidn_device_ = oidnNewDevice(OIDN_DEVICE_TYPE_CPU);
      break;
  }
  if (!oidn_device_) {
    denoiser_device_->set_error("Failed to create OIDN device");
    return false;
  }

#  endif
  oidnCommitDevice(oidn_device_);

  oidn_filter_ = oidnNewFilter(oidn_device_, "RT");
  if (oidn_filter_ == nullptr) {
    denoiser_device_->set_error("Failed to create OIDN filter");
    return false;
  }

  oidnSetFilterBool(oidn_filter_, "hdr", true);
  oidnSetFilterBool(oidn_filter_, "srgb", false);
  oidnSetFilterInt(oidn_filter_, "maxMemoryMB", 1024);
  if (params_.prefilter == DENOISER_PREFILTER_NONE ||
      params_.prefilter == DENOISER_PREFILTER_ACCURATE) {
    oidnSetFilterInt(oidn_filter_, "cleanAux", true);
  }

  albedo_filter_ = oidnNewFilter(oidn_device_, "RT");
  oidnSetFilterInt(albedo_filter_, "maxMemoryMB", 1024);
  if (albedo_filter_ == nullptr) {
    denoiser_device_->set_error("Failed to create OIDN filter");
    return false;
  }

  normal_filter_ = oidnNewFilter(oidn_device_, "RT");
  oidnSetFilterInt(normal_filter_, "maxMemoryMB", 1024);
  if (normal_filter_ == nullptr) {
    denoiser_device_->set_error("Failed to create OIDN filter");
    return false;
  }

  /* OIDN denoiser handle was created with the requested number of input passes. */
  use_pass_albedo_ = context.use_pass_albedo;
  use_pass_normal_ = context.use_pass_normal;
  use_pass_motion_ = context.use_pass_motion;

  /* OIDN denoiser has been created, but it needs configuration. */
  is_configured_ = false;
#endif
  return true;
}

bool OIDNDenoiser::denoise_configure_if_needed(DenoiseContext &context)
{
  /* Limit maximum tile size denoiser can be invoked with. */
  const int2 size = make_int2(context.buffer_params.width, context.buffer_params.height);

  if (is_configured_ && (configured_size_.x == size.x && configured_size_.y == size.y)) {
    return true;
  }

  if (params_.prefilter != DENOISER_PREFILTER_NONE) {
    if (albedo_memory_) {
      delete albedo_memory_;
    }
    if (normal_memory_) {
      delete normal_memory_;
    }
    size_t buffer_size = context.buffer_params.width * context.buffer_params.height *
                         sizeof(float) * 3;

    albedo_memory_ = new device_only_memory<char>(denoiser_device_, "__oidn_albedo");
    albedo_memory_->alloc_to_device(buffer_size);
    normal_memory_ = new device_only_memory<char>(denoiser_device_, "__oidn_normal");
    normal_memory_->alloc_to_device(buffer_size);
  }

  is_configured_ = true;
  configured_size_ = size;

  return true;
}

bool OIDNDenoiser::denoise_run(const DenoiseContext &context, const DenoisePass &pass)
{
#ifdef WITH_OPENIMAGEDENOISE
  /* Color pass. */
  const int64_t pass_stride_in_bytes = context.buffer_params.pass_stride * sizeof(float);

  oidnSetSharedFilterImage(oidn_filter_,
                           "color",
                           (void *)context.render_buffers->buffer.device_pointer,
                           OIDN_FORMAT_FLOAT3,
                           context.buffer_params.width,
                           context.buffer_params.height,
                           pass.denoised_offset * sizeof(float),
                           pass_stride_in_bytes,
                           pass_stride_in_bytes * context.buffer_params.stride);
  oidnSetSharedFilterImage(oidn_filter_,
                           "output",
                           (void *)context.render_buffers->buffer.device_pointer,
                           OIDN_FORMAT_FLOAT3,
                           context.buffer_params.width,
                           context.buffer_params.height,
                           pass.denoised_offset * sizeof(float),
                           pass_stride_in_bytes,
                           pass_stride_in_bytes * context.buffer_params.stride);

  /* Optional albedo and color passes. */
  if (context.num_input_passes > 1) {
    const device_ptr d_guiding_buffer = context.guiding_params.device_pointer;
    const int64_t pixel_stride_in_bytes = context.guiding_params.pass_stride * sizeof(float);
    const int64_t row_stride_in_bytes = context.guiding_params.stride * pixel_stride_in_bytes;

    if (context.use_pass_albedo) {
      if (params_.prefilter == DENOISER_PREFILTER_NONE) {
        oidnSetSharedFilterImage(oidn_filter_,
                                 "albedo",
                                 (void *)d_guiding_buffer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 context.guiding_params.pass_albedo * sizeof(float),
                                 pixel_stride_in_bytes,
                                 row_stride_in_bytes);
      }
      else {
        oidnSetSharedFilterImage(albedo_filter_,
                                 "color",
                                 (void *)d_guiding_buffer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 context.guiding_params.pass_albedo * sizeof(float),
                                 pixel_stride_in_bytes,
                                 row_stride_in_bytes);
        oidnSetSharedFilterImage(albedo_filter_,
                                 "output",
                                 (void *)albedo_memory_->device_pointer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 0,
                                 3 * sizeof(float),
                                 context.buffer_params.width * 3 * sizeof(float));
        oidnCommitFilter(albedo_filter_);
        oidnExecuteFilterAsync(albedo_filter_);

        oidnSetSharedFilterImage(oidn_filter_,
                                 "albedo",
                                 (void *)albedo_memory_->device_pointer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 0,
                                 3 * sizeof(float),
                                 context.buffer_params.width * 3 * sizeof(float));
      }
    }

    if (context.use_pass_normal) {
      if (params_.prefilter == DENOISER_PREFILTER_NONE) {
        oidnSetSharedFilterImage(oidn_filter_,
                                 "normal",
                                 (void *)d_guiding_buffer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 context.guiding_params.pass_normal * sizeof(float),
                                 pixel_stride_in_bytes,
                                 row_stride_in_bytes);
      }
      else {
        oidnSetSharedFilterImage(normal_filter_,
                                 "color",
                                 (void *)d_guiding_buffer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 context.guiding_params.pass_normal * sizeof(float),
                                 pixel_stride_in_bytes,
                                 row_stride_in_bytes);

        oidnSetSharedFilterImage(normal_filter_,
                                 "output",
                                 (void *)normal_memory_->device_pointer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 0,
                                 3 * sizeof(float),
                                 context.buffer_params.width * 3 * sizeof(float));

        oidnCommitFilter(normal_filter_);
        oidnExecuteFilterAsync(normal_filter_);

        oidnSetSharedFilterImage(oidn_filter_,
                                 "normal",
                                 (void *)normal_memory_->device_pointer,
                                 OIDN_FORMAT_FLOAT3,
                                 context.buffer_params.width,
                                 context.buffer_params.height,
                                 0,
                                 3 * sizeof(float),
                                 context.buffer_params.width * 3 * sizeof(float));
      }
    }
  }

  oidnCommitFilter(oidn_filter_);
  oidnExecuteFilter(oidn_filter_);

  const char *out_message = nullptr;
  OIDNError err = oidnGetDeviceError(oidn_device_, (const char **)&out_message);
  if (OIDN_ERROR_NONE != err) {
    /* If OIDN runs out of memory, reduce mem limit and retry */
    while (err == OIDN_ERROR_OUT_OF_MEMORY && max_mem_ > 200) {
      max_mem_ = max_mem_ / 2;
      oidnSetFilterInt(oidn_filter_, "maxMemoryMB", max_mem_);
      oidnCommitFilter(oidn_filter_);
      oidnExecuteFilter(oidn_filter_);
      err = oidnGetDeviceError(oidn_device_, &out_message);
    }
    if (out_message) {
      LOG(ERROR) << "OIDN error: " << out_message;
      denoiser_device_->set_error(out_message);
    }
    else {
      LOG(ERROR) << "OIDN error: unspecified";
      denoiser_device_->set_error("Unspecified OIDN error");
    }
    return false;
  }
#endif
  return true;
}

CCL_NAMESPACE_END
