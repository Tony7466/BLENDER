/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_staging_buffer.hh"
#include "vk_backend.hh"
#include "vk_context.hh"
#include "vk_device.hh"

namespace blender::gpu {

VKStagingBuffer::VKStagingBuffer(const VKBuffer &device_buffer, Direction /*direction*/)
    : device_buffer_(device_buffer)
{
  VKDevice &device = VKBackend::get().device;
  VKResourcePool &resource_pool = device.current_thread_data().resource_pool_get();
  host_buffer_ = &resource_pool.staging_buffers.acquire(device_buffer.size_in_bytes());
}

VKStagingBuffer::~VKStagingBuffer()
{
  VKDevice &device = VKBackend::get().device;
  VKResourcePool &resource_pool = device.current_thread_data().resource_pool_get();
  resource_pool.staging_buffers.release(*host_buffer_);
  host_buffer_ = nullptr;
}

void VKStagingBuffer::copy_to_device(VKContext &context)
{
  BLI_assert(host_buffer_ && host_buffer_->is_allocated() && host_buffer_->is_mapped());
  render_graph::VKCopyBufferNode::CreateInfo copy_buffer = {};
  copy_buffer.src_buffer = host_buffer_->vk_handle();
  copy_buffer.dst_buffer = device_buffer_.vk_handle();
  copy_buffer.region.size = device_buffer_.size_in_bytes();

  context.render_graph.add_node(copy_buffer);
}

void VKStagingBuffer::copy_from_device(VKContext &context)
{
  BLI_assert(host_buffer_ && host_buffer_->is_allocated() && host_buffer_->is_mapped());
  render_graph::VKCopyBufferNode::CreateInfo copy_buffer = {};
  copy_buffer.src_buffer = device_buffer_.vk_handle();
  copy_buffer.dst_buffer = host_buffer_->vk_handle();
  copy_buffer.region.size = device_buffer_.size_in_bytes();

  context.render_graph.add_node(copy_buffer);
}

void VKStagingBuffer::free()
{
  VKDevice &device = VKBackend::get().device;
  VKResourcePool &resource_pool = device.current_thread_data().resource_pool_get();
  resource_pool.staging_buffers.release(*host_buffer_);
  host_buffer_ = nullptr;
}

}  // namespace blender::gpu
