/* SPDX-License-Identifier: GPL-2.0-or-later
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation */

#include "render_task_delegate.h"

#include <epoxy/gl.h>

#include "GPU_context.h"

#include <pxr/imaging/hd/renderBuffer.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hdx/renderTask.h>

#include "MEM_guardedalloc.h"

#include "engine.h"
#include "render_task_delegate.h"

namespace blender::render::hydra {

RenderTaskDelegate::RenderTaskDelegate(pxr::HdRenderIndex *parent_index,
                                       pxr::SdfPath const &delegate_id)
    : pxr::HdSceneDelegate(parent_index, delegate_id)
{
  task_id_ = GetDelegateID().AppendElementString("task");
  GetRenderIndex().InsertTask<pxr::HdxRenderTask>(this, task_id_);

  task_params_.enableLighting = true;
  task_params_.alphaThreshold = 0.1f;

  CLOG_INFO(LOG_RENDER_HYDRA, 1, "%s", task_id_.GetText());
}

pxr::VtValue RenderTaskDelegate::Get(pxr::SdfPath const &id, pxr::TfToken const &key)
{
  CLOG_INFO(LOG_RENDER_HYDRA, 3, "%s, %s", id.GetText(), key.GetText());

  if (key == pxr::HdTokens->params) {
    return pxr::VtValue(task_params_);
  }
  if (key == pxr::HdTokens->collection) {
    return pxr::VtValue(pxr::HdRprimCollection(
        pxr::HdTokens->geometry, pxr::HdReprSelector(pxr::HdReprTokens->smoothHull)));
  }
  return pxr::VtValue();
}

pxr::TfTokenVector RenderTaskDelegate::GetTaskRenderTags(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA, 3, "%s", id.GetText());

  return {pxr::HdRenderTagTokens->geometry};
}

pxr::HdRenderBufferDescriptor RenderTaskDelegate::GetRenderBufferDescriptor(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA, 3, "%s", id.GetText());

  return buffer_descriptors_[id];
}

pxr::HdTaskSharedPtr RenderTaskDelegate::task()
{
  return GetRenderIndex().GetTask(task_id_);
}

void RenderTaskDelegate::set_camera(pxr::SdfPath const &camera_id)
{
  if (task_params_.camera == camera_id) {
    return;
  }
  task_params_.camera = camera_id;
  GetRenderIndex().GetChangeTracker().MarkTaskDirty(task_id_, pxr::HdChangeTracker::DirtyParams);
}

bool RenderTaskDelegate::is_converged()
{
  return static_cast<pxr::HdxRenderTask *>(task().get())->IsConverged();
}

void RenderTaskDelegate::set_viewport(pxr::GfVec4d const &viewport)
{
  if (task_params_.viewport == viewport) {
    return;
  }
  auto &render_index = GetRenderIndex();
  task_params_.viewport = viewport;
  render_index.GetChangeTracker().MarkTaskDirty(task_id_, pxr::HdChangeTracker::DirtyParams);

  int w = viewport[2] - viewport[0];
  int h = viewport[3] - viewport[1];
  for (auto &it : buffer_descriptors_) {
    it.second.dimensions = pxr::GfVec3i(w, h, 1);
    render_index.GetChangeTracker().MarkBprimDirty(it.first,
                                                   pxr::HdRenderBuffer::DirtyDescription);
  }
}

void RenderTaskDelegate::add_aov(pxr::TfToken const &aov_key)
{
  pxr::SdfPath buf_id = buffer_id(aov_key);
  if (buffer_descriptors_.find(buf_id) != buffer_descriptors_.end()) {
    return;
  }
  auto &render_index = GetRenderIndex();
  pxr::HdAovDescriptor aov_desc = render_index.GetRenderDelegate()->GetDefaultAovDescriptor(
      aov_key);

  if (aov_desc.format == pxr::HdFormatInvalid) {
    return;
  }

  int w = task_params_.viewport[2] - task_params_.viewport[0];
  int h = task_params_.viewport[3] - task_params_.viewport[1];
  render_index.InsertBprim(pxr::HdPrimTypeTokens->renderBuffer, this, buf_id);
  buffer_descriptors_[buf_id] = pxr::HdRenderBufferDescriptor(
      pxr::GfVec3i(w, h, 1), aov_desc.format, aov_desc.multiSampled);

  pxr::HdRenderPassAovBinding binding;
  binding.aovName = aov_key;
  binding.renderBufferId = buf_id;
  binding.aovSettings = aov_desc.aovSettings;
  binding.clearValue = pxr::VtValue(pxr::GfVec4f(0));
  task_params_.aovBindings.push_back(binding);
  render_index.GetChangeTracker().MarkTaskDirty(task_id_, pxr::HdChangeTracker::DirtyParams);

  CLOG_INFO(LOG_RENDER_HYDRA, 1, "%s", aov_key.GetText());
}

void RenderTaskDelegate::read_aov(pxr::TfToken const &aov_key, void *data)
{
  pxr::HdRenderBuffer *buffer = static_cast<pxr::HdRenderBuffer *>(
      GetRenderIndex().GetBprim(pxr::HdPrimTypeTokens->renderBuffer, buffer_id(aov_key)));
  if (!buffer) {
    return;
  }
  void *buf_data = buffer->Map();
  memcpy(data,
         buf_data,
         buffer->GetWidth() * buffer->GetHeight() * pxr::HdDataSizeOfFormat(buffer->GetFormat()));
  buffer->Unmap();
}

void RenderTaskDelegate::read_aov(pxr::TfToken const &aov_key, GPUTexture *texture)
{
  pxr::HdRenderBuffer *buffer = (pxr::HdRenderBuffer *)GetRenderIndex().GetBprim(
      pxr::HdPrimTypeTokens->renderBuffer, buffer_id(aov_key));
  if (!buffer) {
    return;
  }
  eGPUDataFormat format = buffer->GetFormat() == pxr::HdFormat::HdFormatFloat16Vec4 ?
                              GPU_DATA_HALF_FLOAT :
                              GPU_DATA_FLOAT;
  void *buf_data = buffer->Map();
  GPU_texture_update(texture, format, buf_data);
  buffer->Unmap();
}

void RenderTaskDelegate::bind() {}

void RenderTaskDelegate::unbind() {}

pxr::SdfPath RenderTaskDelegate::buffer_id(pxr::TfToken const &aov_key) const
{
  return GetDelegateID().AppendElementString("aov_" + aov_key.GetString());
}

GPURenderTaskDelegate::~GPURenderTaskDelegate()
{
  unbind();
  if (tex_color_) {
    GPU_texture_free(tex_color_);
  }
  if (tex_depth_) {
    GPU_texture_free(tex_depth_);
  }
}

void GPURenderTaskDelegate::set_viewport(pxr::GfVec4d const &viewport)
{
  if (task_params_.viewport == viewport) {
    return;
  }
  auto &render_index = GetRenderIndex();
  task_params_.viewport = viewport;
  render_index.GetChangeTracker().MarkTaskDirty(task_id_, pxr::HdChangeTracker::DirtyParams);

  if (tex_color_) {
    GPU_texture_free(tex_color_);
    tex_color_ = nullptr;
    add_aov(pxr::HdAovTokens->color);
  }
  if (tex_depth_) {
    GPU_texture_free(tex_depth_);
    tex_depth_ = nullptr;
    add_aov(pxr::HdAovTokens->depth);
  }
}

void GPURenderTaskDelegate::add_aov(pxr::TfToken const &aov_key)
{
  eGPUTextureFormat format;
  GPUTexture **tex;
  if (aov_key == pxr::HdAovTokens->color) {
    format = GPU_RGBA32F;
    tex = &tex_color_;
  }
  else if (aov_key == pxr::HdAovTokens->depth) {
    format = GPU_DEPTH_COMPONENT32F;
    tex = &tex_depth_;
  }
  else {
    return;
  }

  if (*tex) {
    return;
  }

  *tex = GPU_texture_create_2d(("tex_render_hydra_" + aov_key.GetString()).c_str(),
                               task_params_.viewport[2] - task_params_.viewport[0],
                               task_params_.viewport[3] - task_params_.viewport[1],
                               1,
                               format,
                               GPU_TEXTURE_USAGE_GENERAL,
                               nullptr);

  CLOG_INFO(LOG_RENDER_HYDRA, 1, "%s", aov_key.GetText());
}

void GPURenderTaskDelegate::read_aov(pxr::TfToken const &aov_key, void *data)
{
  GPUTexture *tex = nullptr;
  int c;
  if (aov_key == pxr::HdAovTokens->color) {
    tex = tex_color_;
    c = 4;
  }
  else if (aov_key == pxr::HdAovTokens->depth) {
    tex = tex_depth_;
    c = 1;
  }
  if (!tex) {
    return;
  }

  int w = GPU_texture_width(tex), h = GPU_texture_height(tex);
  void *tex_data = GPU_texture_read(tex, GPU_DATA_FLOAT, 0);
  memcpy(data, tex_data, sizeof(float) * w * h * c);
  MEM_freeN(tex_data);
}

void GPURenderTaskDelegate::read_aov(pxr::TfToken const &aov_key, GPUTexture *texture)
{
  GPUTexture *tex = nullptr;
  int c;
  if (aov_key == pxr::HdAovTokens->color) {
    tex = tex_color_;
    c = 4;
  }
  else if (aov_key == pxr::HdAovTokens->depth) {
    tex = tex_depth_;
    c = 1;
  }
  if (!tex) {
    return;
  }

  void *tex_data = GPU_texture_read(tex, GPU_DATA_FLOAT, 0);
  GPU_texture_update(texture, GPU_DATA_FLOAT, tex_data);
  MEM_freeN(tex_data);
}

void GPURenderTaskDelegate::bind()
{
  if (!framebuffer_) {
    framebuffer_ = GPU_framebuffer_create("fb_render_hydra");
  }
  GPU_framebuffer_ensure_config(
      &framebuffer_, {GPU_ATTACHMENT_TEXTURE(tex_depth_), GPU_ATTACHMENT_TEXTURE(tex_color_)});
  GPU_framebuffer_bind(framebuffer_);

  float clear_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  GPU_framebuffer_clear_color_depth(framebuffer_, clear_color, 1.0f);

  /* Workaround missing/buggy VAOs in hgiGL and hdSt. For OpenGL compatibility
   * profile this is not a problem, but for core profile it is. */
  if (VAO_ == 0 && GPU_backend_get_type() == GPU_BACKEND_OPENGL) {
    glGenVertexArrays(1, &VAO_);
    glBindVertexArray(VAO_);
  }
}

void GPURenderTaskDelegate::unbind()
{
  if (VAO_) {
    glDeleteVertexArrays(1, &VAO_);
    VAO_ = 0;
  }
  if (framebuffer_) {
    GPU_framebuffer_free(framebuffer_);
    framebuffer_ = nullptr;
  }
}

GPUTexture *GPURenderTaskDelegate::aov_texture(pxr::TfToken const &aov_key)
{
  if (aov_key == pxr::HdAovTokens->color) {
    return tex_color_;
  }
  if (aov_key == pxr::HdAovTokens->depth) {
    return tex_depth_;
  }
  return nullptr;
}

}  // namespace blender::render::hydra
