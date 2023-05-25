/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

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
      buffers_(NULL),
      master_buffers_(make_unique<RenderBuffers>(device)),
      effective_buffer_params_(effective_full_params_),
      cancel_requested_flag_(cancel_requested_flag)
{
}

PathTraceWork::~PathTraceWork() {}

WorkSet::~WorkSet()
{
      for (int i = 0; i < size(); i++) {
        if (render_buffers_set_[i])  {
          delete render_buffers_set_[i];
          render_buffers_set_[i] = NULL;
        }
      }
}

void PathTraceWork::clear_or_create_work_set(int device_scale_factor)
{
  if (work_set_.size() < device_scale_factor) {
    if (work_set_.size() > 0) {
      for (int i = 0; i < work_set_.size(); i++) {
        if (work_set_.render_buffers_set_[i])  {
          delete work_set_.render_buffers_set_[i];
          work_set_.render_buffers_set_[i] = NULL;
        }
      }
    }
    work_set_.resize(device_scale_factor);
    for (int i = 0; i < device_scale_factor; i++) {
      work_set_.render_buffers_set_[i] = new RenderBuffers(device_);
    }
  }
}

void PathTraceWork::clear_work_set_buffers(const BufferParams& empty_params) 
{
 for (int i = 0; i < work_set_.size(); i++) {
   if (work_set_.render_buffers_set_[i])  {
     work_set_.render_buffers_set_[i]->reset(empty_params); 
   }
 }
}

void PathTraceWork::set_render_buffers_in_work_set(const BufferParams& p, int i, size_t offset) 
{
  work_set_.render_buffers_set_[i]->reset(p, offset, master_buffers_.get());
}

void PathTraceWork::set_effective_buffer_params_in_work_set(const BufferParams& p, int i, size_t offset) 
{
  work_set_.effective_buffer_params_set_[i] = p;
}

void PathTraceWork::set_effective_full_buffer_params(const BufferParams &effective_full_params,
                                                const BufferParams &effective_big_tile_params)
{
  effective_full_params_ = effective_full_params;
  effective_big_tile_params_ = effective_big_tile_params;
}

void PathTraceWork::set_current_work_set(int i) 
{
  
  buffers_ = work_set_.render_buffers_set_[i];
  effective_buffer_params_ = work_set_.effective_buffer_params_set_[i];
}

void PathTraceWork::render_samples(RenderStatistics &statistics,
                              int start_sample,
                              int samples_num,
                              int sample_offset)
{
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0) {
  render_samples_impl(statistics, start_sample, samples_num, sample_offset);
  }
  }
}

void PathTraceWork::copy_to_display(PathTraceDisplay *display, PassMode pass_mode, int num_samples)
{
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0)
    copy_to_display_impl(display, pass_mode, num_samples);
  }
}

bool PathTraceWork::copy_render_buffers_from_device()
{
  copy_master_render_buffers_from_device_impl();
  // for (int i = 0; i < work_set_.size(); i++) {
  // set_current_work_set(i);
  // if (!copy_render_buffers_from_device_impl()) return false;
  // }
  
  return synchronize();
}

bool PathTraceWork::copy_render_buffers_to_device()
{
  copy_master_render_buffers_to_device_impl();
  // for (int i = 0; i < work_set_.size(); i++) {
  // set_current_work_set(i);
  // if (!copy_render_buffers_to_device_impl()) return false;
  // }
  return true;
}

bool PathTraceWork::zero_render_buffers()
{
  zero_master_render_buffers_impl();
  // for (int i = 0; i < work_set_.size(); i++) {
  // set_current_work_set(i);
  // if (!zero_render_buffers_impl()) return false;
  // }
  return true;
}

void PathTraceWork::copy_to_render_buffers(RenderBuffers *render_buffers)
{
  copy_render_buffers_from_device();
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0) {
  const int64_t width = effective_buffer_params_.width;
  const int64_t height = effective_buffer_params_.height;
  const int64_t pass_stride = effective_buffer_params_.pass_stride;
  const int64_t row_stride = width * pass_stride;
  const int64_t data_size = row_stride * height * sizeof(float);

  const int64_t offset_y = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  const int64_t offset_in_floats = offset_y * row_stride;

  const float *src = buffers_->buffer.data();
  float *dst = render_buffers->buffer.data() + offset_in_floats;

  memcpy(dst, src, data_size);
  }
  }
}

void PathTraceWork::copy_from_render_buffers(const RenderBuffers *render_buffers)
{
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0) {
  const int64_t width = effective_buffer_params_.width;
  const int64_t height = effective_buffer_params_.height;
  const int64_t pass_stride = effective_buffer_params_.pass_stride;
  const int64_t row_stride = width * pass_stride;
  const int64_t data_size = row_stride * height * sizeof(float);

  const int64_t offset_y = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  const int64_t offset_in_floats = offset_y * row_stride;

  const float *src = render_buffers->buffer.data() + offset_in_floats;
  float *dst = buffers_->buffer.data();

  memcpy(dst, src, data_size);
  }
  }
  copy_render_buffers_to_device();
}

void PathTraceWork::copy_from_denoised_render_buffers(const RenderBuffers *render_buffers)
{
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0) {
  const int64_t width = effective_buffer_params_.width;
  const int64_t offset_y = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  const int64_t offset = offset_y * width;

  render_buffers_host_copy_denoised(
      buffers_, effective_buffer_params_, render_buffers, effective_buffer_params_, offset);
  }
  }
  copy_render_buffers_to_device();
}

bool PathTraceWork::get_render_tile_pixels(const PassAccessor &pass_accessor,
                                           const PassAccessor::Destination &destination)
{
  
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0) {
  const int offset_y = (effective_buffer_params_.full_y + effective_buffer_params_.window_y) -
                       (effective_big_tile_params_.full_y + effective_big_tile_params_.window_y);
  const int width = effective_buffer_params_.width;

  PassAccessor::Destination slice_destination = destination;
  slice_destination.offset += offset_y * width;

  if (!pass_accessor.get_render_tile_pixels(buffers_, slice_destination)) return false;
  }
  }
  return true;
}

bool PathTraceWork::set_render_tile_pixels(PassAccessor &pass_accessor,
                                           const PassAccessor::Source &source)
{
  for (int i = 0; i < work_set_.size(); i++) {
  set_current_work_set(i);
  if(effective_buffer_params_.height > 0) {
  const int offset_y = effective_buffer_params_.full_y - effective_big_tile_params_.full_y;
  const int width = effective_buffer_params_.width;

  PassAccessor::Source slice_source = source;
  slice_source.offset += offset_y * width;

  if (!pass_accessor.set_render_tile_pixels(buffers_, slice_source)) return false;
  }
  }
  return true;
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

CCL_NAMESPACE_END
