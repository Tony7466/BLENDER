/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "device/device.h"

#include "integrator/path_trace_display.h"
#include "integrator/path_trace_work.h"
#include "integrator/path_trace_work_cpu.h"
#include "integrator/path_trace_work_gpu.h"
#include "scene/film.h"
#include "scene/scene.h"
#include "session/buffers.h"

#include "kernel/types.h"

CCL_NAMESPACE_BEGIN

unique_ptr<PathTraceWork> PathTraceWork::create(Device *device,
                                                Film *film,
                                                DeviceScene *device_scene,
                                                bool *cancel_requested_flag)
{
  if (device->info.type == DEVICE_CPU) {
    return make_unique<PathTraceWorkCPU>(device, film, device_scene, cancel_requested_flag);
  }
  if (device->info.type == DEVICE_DUMMY) {
    /* Dummy devices can't perform any work. */
    return nullptr;
  }

  return make_unique<PathTraceWorkGPU>(device, film, device_scene, cancel_requested_flag);
}

PathTraceWork::PathTraceWork(Device *device,
                             Film *film,
                             DeviceScene *device_scene,
                             bool *cancel_requested_flag)
    : device_(device),
      film_(film),
      device_scene_(device_scene),
      buffers_(make_unique<RenderBuffers>(device)),
      effective_buffer_params_(buffers_->params),
      cancel_requested_flag_(cancel_requested_flag),
      device_scale_factor_(-1)
{
}

PathTraceWork::~PathTraceWork() {}

RenderBuffers *PathTraceWork::get_render_buffers()
{
  return buffers_.get();
}

void PathTraceWork::set_effective_buffer_params(const BufferParams &effective_full_params,
                                                const BufferParams &effective_big_tile_params,
                                                const BufferParams &effective_buffer_params)
{
  effective_full_params_ = effective_full_params;
  effective_big_tile_params_ = effective_big_tile_params;
  effective_buffer_params_ = effective_buffer_params;
}

bool PathTraceWork::has_multiple_works() const
{
  /* Assume if there are multiple works working on the same big tile none of the works gets the
   * entire big tile to work on. */
  return !(effective_big_tile_params_.width == effective_buffer_params_.width &&
           effective_big_tile_params_.height == effective_buffer_params_.height &&
           effective_big_tile_params_.full_x == effective_buffer_params_.full_x &&
           effective_big_tile_params_.full_y == effective_buffer_params_.full_y);
}

void PathTraceWork::copy_to_render_buffers(RenderBuffers *render_buffers)
{
  VLOG_INFO << "Copy to render buffers";
  SCOPED_MARKER(device_, "copy_to_render_buffers");
  copy_render_buffers_from_device();
  const int y_stride = effective_buffer_params_.slice_stride;
  const int slice_height = effective_buffer_params_.slice_height;
  const int total_height = effective_buffer_params_.height;
  const int64_t width = effective_buffer_params_.width;
  const int64_t pass_stride = effective_buffer_params_.pass_stride;
  const int64_t row_stride = width * pass_stride;
  const int64_t data_size = row_stride * sizeof(float);

  int y_slice = 0;
  int y_render = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  for (int i = 0; i < device_scale_factor_; i++) {
    SCOPED_MARKER(device_, "copy_to_render_buffers_work_set");
    int height = std::min(total_height - i*slice_height, slice_height);
    //VLOG_INFO << "\t(" << i << ") Buffer height " << height << " y slice:" << y_slice << " y render:" << y_render;
    if(height > 0) {
      const float *src = buffers_->buffer.data() + y_slice*row_stride;
      float *dst = render_buffers->buffer.data() + y_render*row_stride;
      
      memcpy(dst, src, data_size*height);
      y_slice += slice_height;
      y_render += y_stride;
    }
  }
}

void PathTraceWork::copy_from_render_buffers(const RenderBuffers *render_buffers)
{
  SCOPED_MARKER(device_, "copy_from_render_buffers");
  const int y_stride = effective_buffer_params_.slice_stride;  
  const int slice_height = effective_buffer_params_.slice_height;
  const int total_height = effective_buffer_params_.height;
  const int64_t width = effective_buffer_params_.width;
  const int64_t pass_stride = effective_buffer_params_.pass_stride;
  const int64_t row_stride = width * pass_stride;
  const int64_t data_size = row_stride * sizeof(float);

  int y_slice = 0;
  int y_render = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  for (int i = 0; i < device_scale_factor_; i++) {
    SCOPED_MARKER(device_, "copy_from_render_buffers_work_set");
    int height = std::min(total_height - i*slice_height, slice_height);
    if(height > 0) {      
      const float *src = render_buffers->buffer.data() + y_render*row_stride;
      float *dst = buffers_->buffer.data() + y_slice*row_stride;
      
      memcpy(dst, src, data_size*height);
      y_slice += slice_height;
      y_render += y_stride;
    }
  }
  copy_render_buffers_to_device();
}

void PathTraceWork::copy_from_denoised_render_buffers(const RenderBuffers *render_buffers)
{
  const int64_t width = effective_buffer_params_.width;
  const int y_stride = effective_buffer_params_.slice_stride;  
  const int slice_height = effective_buffer_params_.slice_height;
  
  int y_slice = 0;
  int64_t y_render = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  for (int i = 0; i < device_scale_factor_; i++) {
    const int64_t dst_offset = y_render * width;
    const int64_t src_offset = y_slice * width;

    render_buffers_host_copy_denoised(
				      buffers_.get(), effective_buffer_params_, src_offset, slice_height, render_buffers, effective_buffer_params_, dst_offset);

    y_slice += slice_height;
    y_render += y_stride;
  }
  copy_render_buffers_to_device();
}

bool PathTraceWork::get_render_tile_pixels(const PassAccessor &pass_accessor,
                                           const PassAccessor::Destination &destination)
{
  if(effective_buffer_params_.height > 0) {
  const int offset_y = (effective_buffer_params_.full_y + effective_buffer_params_.window_y) -
                       (effective_big_tile_params_.full_y + effective_big_tile_params_.window_y);
  const int width = effective_buffer_params_.width;

  PassAccessor::Destination slice_destination = destination;
  slice_destination.offset += offset_y * width;

  return pass_accessor.get_render_tile_pixels(buffers_.get(), slice_destination);
  }
  return false;
}

bool PathTraceWork::set_render_tile_pixels(PassAccessor &pass_accessor,
                                           const PassAccessor::Source &source)
{
  if(effective_buffer_params_.height > 0) {
  const int offset_y = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  const int width = effective_buffer_params_.width;

  PassAccessor::Source slice_source = source;
  slice_source.offset += offset_y * width;

  return pass_accessor.set_render_tile_pixels(buffers_.get(), slice_source);
  }
  return false;
}

PassAccessor::PassAccessInfo PathTraceWork::get_display_pass_access_info(PassMode pass_mode) const
{
  const KernelFilm &kfilm = device_scene_->data.film;
  const KernelBackground &kbackground = device_scene_->data.background;

  const BufferParams &params = buffers_->params;

  const BufferPass *display_pass = params.get_actual_display_pass(film_->get_display_pass());

  PassAccessor::PassAccessInfo pass_access_info;
  pass_access_info.type = display_pass->type;
  pass_access_info.offset = PASS_UNUSED;

  if (pass_mode == PassMode::DENOISED) {
    pass_access_info.mode = PassMode::DENOISED;
    pass_access_info.offset = params.get_pass_offset(pass_access_info.type, PassMode::DENOISED);
  }

  if (pass_access_info.offset == PASS_UNUSED) {
    pass_access_info.mode = PassMode::NOISY;
    pass_access_info.offset = params.get_pass_offset(pass_access_info.type);
  }

  pass_access_info.use_approximate_shadow_catcher = kfilm.use_approximate_shadow_catcher;
  pass_access_info.use_approximate_shadow_catcher_background =
      kfilm.use_approximate_shadow_catcher && !kbackground.transparent;

  pass_access_info.show_active_pixels = film_->get_show_active_pixels();

  return pass_access_info;
}

PassAccessor::Destination PathTraceWork::get_display_destination_template(
    const PathTraceDisplay *display) const
{
  PassAccessor::Destination destination(film_->get_display_pass());

  const int2 display_texture_size = display->get_texture_size();
  const int texture_x = effective_buffer_params_.full_x - effective_big_tile_params_.full_x +
                        effective_buffer_params_.window_x - effective_big_tile_params_.window_x;
  const int texture_y = effective_buffer_params_.full_y - effective_big_tile_params_.full_y +
                        effective_buffer_params_.window_y - effective_big_tile_params_.window_y;

  destination.offset = texture_y * display_texture_size.x + texture_x;
  destination.stride = display_texture_size.x;

  return destination;
}
//===========================REMOVE BELOW THIS POINT===================
// /*
//    Deteremines a full buffer and parameters from the work_set_
//  */
// void PathTraceWork::set_slices_effective_params() {
//   // int height = 0;
//   // slices_buffer_params_ = work_set_.effective_buffer_params_set_[0];
//   // for (int i = 0; i < work_set_.size(); i++) {
//   //   height += work_set_.effective_buffer_params_set_[i].height;
//   // }
//   // slices_buffer_params_.height = height;
//   // slices_buffer_params_.window_height = height;
//   // slices_buffer_params_.update_offset_stride();
//   // VLOG_INFO << "Slices buffer params y:" << (slices_buffer_params_.full_y + slices_buffer_params_.window_y) << " height:" << slices_buffer_params_.height << " slice height:" <<  slices_buffer_params_.slice_height << " slice stride:" << slices_buffer_params_.slice_stride;
//   // master_buffers_->params = slices_buffer_params_;
//   //buffers_ = master_buffers_.get();
//   effective_buffer_params_ = slices_buffer_params_;
// }

// void PathTraceWork::update_slices_buffer_params() {
//   int height = 0;
//   slices_buffer_params_ = work_set_.effective_buffer_params_set_[0]
//   for (int i = 0; i < work_set_.size(); i++) {
//     height += work_set_.effective_buffer_params_set_[i].height;
//   }
//   slices_buffer_params_.height = height;
//   slices_buffer_params_.window_height = height;
//   slices_buffer_params_.update_offset_stride();
//   VLOG_INFO << "Slices buffer params y:" << (slices_buffer_params_.full_y + slices_buffer_params_.window_y) << " height:" << slices_buffer_params_.height << " slice height:" <<  slices_buffer_params_.slice_height << " slice stride:" << slices_buffer_params_.slice_stride;
//   //master_buffers_->params = slices_buffer_params_;
//   //buffers_ = master_buffers_.get();
// }

// void PathTraceWork::render_samples(RenderStatistics &statistics,
//                               int start_sample,
//                               int samples_num,
//                               int sample_offset)
// {
//   SCOPED_MARKER(device_, "render_samples");
//   set_slices_effective_params();
//   //for (int i = 0; i < work_set_.size(); i++) {
//   //   SCOPED_MARKER(device_, "render_samples_work_set");
//   // set_current_work_set(i);
//   if(effective_buffer_params_.height > 0) {
//   render_samples(statistics, start_sample, samples_num, sample_offset);
//   }
//   //}
// }

// void PathTraceWork::copy_to_display(PathTraceDisplay *display, PassMode pass_mode, int num_samples)
// {
//   SCOPED_MARKER(device_, "copy_to_display");
//   set_slices_effective_params();
//   //for (int i = 0; i < work_set_.size(); i++) {
//   //  SCOPED_MARKER(device_, "copy_to_display_work_set");
//   //set_current_work_set(i);
//   if(effective_buffer_params_.height > 0)
//     copy_to_display(display, pass_mode, num_samples);
//   //}
// }

// bool PathTraceWork::copy_render_buffers_from_device()
// {
//   // for (int i = 0; i < work_set_.size(); i++) {
//   // set_current_work_set(i);
//   // if (!copy_render_buffers_from_device_impl()) return false;
//   // }
//   // return true;

//   SCOPED_MARKER(device_, "copy_render_buffers_from_device");
//   return copy_render_buffers_from_device();
// }

// bool PathTraceWork::copy_render_buffers_to_device()
// {
//   // for (int i = 0; i < work_set_.size(); i++) {
//  //    set_current_work_set(i);
//  //    if (!copy_render_buffers_to_device_impl()) return false;
//  // }
//  // return true;

//   SCOPED_MARKER(device_, "copy_render_buffers_to_device");
//   return copy_render_buffers_to_device();
// }

// bool PathTraceWork::zero_render_buffers()
// {
//   SCOPED_MARKER(device_, "zerop_render_buffers");
//   return zero_render_buffers();
// }

CCL_NAMESPACE_END
