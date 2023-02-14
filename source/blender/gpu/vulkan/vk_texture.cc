/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_texture.hh"

#include "vk_buffer.hh"
#include "vk_context.hh"
#include "vk_memory.hh"
#include "vk_shader.hh"

namespace blender::gpu {

static VkImageAspectFlagBits to_vk_image_aspect_flag_bits(const eGPUTextureFormat format)
{
  switch (format) {
    case GPU_RGBA32F:
    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_R8UI:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R32UI:
    case GPU_R32I:
    case GPU_R32F:
    case GPU_R16UI:
    case GPU_R16I:
    case GPU_R16F:
    case GPU_R16:
    case GPU_RGB10_A2:
    case GPU_R11F_G11F_B10F:
    case GPU_SRGB8_A8:
    case GPU_RGB16F:
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:
      return VK_IMAGE_ASPECT_COLOR_BIT;

    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
      return static_cast<VkImageAspectFlagBits>(VK_IMAGE_ASPECT_DEPTH_BIT |
                                                VK_IMAGE_ASPECT_STENCIL_BIT);

    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      return VK_IMAGE_ASPECT_DEPTH_BIT;

    case GPU_DEPTH_COMPONENT32F:
      /* Not supported by Vulkan*/
      BLI_assert_unreachable();
  }
  return static_cast<VkImageAspectFlagBits>(0);
}

VKTexture::~VKTexture()
{
  VK_ALLOCATION_CALLBACKS

  VKContext &context = *VKContext::get();
  vmaDestroyImage(context.mem_allocator_get(), vk_image_, allocation_);
  vkDestroyImageView(context.device_get(), vk_image_view_, vk_allocation_callbacks);
}

void VKTexture::generate_mipmap()
{
}

void VKTexture::copy_to(Texture * /*tex*/)
{
}

void VKTexture::clear(eGPUDataFormat /*format*/, const void * /*data*/)
{
}

void VKTexture::swizzle_set(const char /*swizzle_mask*/[4])
{
}

void VKTexture::stencil_texture_mode_set(bool /*use_stencil*/)
{
}

void VKTexture::mip_range_set(int /*min*/, int /*max*/)
{
}

void *VKTexture::read(int mip, eGPUDataFormat format)
{
  /* Vulkan images cannot be directly mapped to host memory and requires a staging buffer.*/
  VKContext &context = *VKContext::get();
  VKBuffer staging_buffer;

  /* NOTE: mip_size_get() won't override any dimension that is equal to 0. */
  int extent[3] = {1, 1, 1};
  mip_size_get(mip, extent);
  size_t sample_len = extent[0] * extent[1] * extent[2];
  /* NOTE: to_bytesize returns number of bits. */
  size_t device_memory_size = sample_len * to_component_len(format_) * to_bytesize(format_) / 8;
  /* NOTE: to_bytesize returns number of bytes here. */
  size_t host_memory_size = sample_len * to_bytesize(format_, format);

  staging_buffer.create(
      context, device_memory_size, GPU_USAGE_DEVICE_ONLY, VK_BUFFER_USAGE_TRANSFER_DST_BIT);

  VkBufferImageCopy region = {};
  region.imageExtent.width = extent[0];
  region.imageExtent.height = extent[1];
  region.imageExtent.depth = extent[2];
  region.imageSubresource.aspectMask = to_vk_image_aspect_flag_bits(format_);
  region.imageSubresource.mipLevel = mip;
  region.imageSubresource.layerCount = 1;

  VKCommandBuffer &command_buffer = context.command_buffer_get();
  command_buffer.copy(staging_buffer, *this, Span<VkBufferImageCopy>(&region, 1));
  command_buffer.submit();

  void *mapped_data;
  staging_buffer.map(context, &mapped_data);

  void *data = MEM_mallocN(host_memory_size, __func__);

  /* TODO: add conversion when data format is different.*/
  BLI_assert_msg(device_memory_size == host_memory_size,
                 "Memory data conversions not implemented yet");

  memcpy(data, mapped_data, host_memory_size);
  staging_buffer.unmap(context);

  return data;
}

void VKTexture::update_sub(int /*mip*/,
                           int /*offset*/[3],
                           int /*extent*/[3],
                           eGPUDataFormat /*format*/,
                           const void * /*data*/)
{
}

void VKTexture::update_sub(int /*offset*/[3],
                           int /*extent*/[3],
                           eGPUDataFormat /*format*/,
                           GPUPixelBuffer * /*pixbuf*/)
{
}

/* TODO(fclem): Legacy. Should be removed at some point. */
uint VKTexture::gl_bindcode_get() const
{
  return 0;
}

static VkFormat to_vk_format(const eGPUTextureFormat format)
{
  switch (format) {
    case GPU_RGBA32F:
      return VK_FORMAT_R32G32B32A32_SFLOAT;
    case GPU_RGBA8UI:
    case GPU_RGBA8I:
    case GPU_RGBA8:
    case GPU_RGBA32UI:
    case GPU_RGBA32I:
    case GPU_RGBA16UI:
    case GPU_RGBA16I:
    case GPU_RGBA16F:
    case GPU_RGBA16:
    case GPU_RG8UI:
    case GPU_RG8I:
    case GPU_RG8:
    case GPU_RG32UI:
    case GPU_RG32I:
    case GPU_RG32F:
    case GPU_RG16UI:
    case GPU_RG16I:
    case GPU_RG16F:
    case GPU_RG16:
    case GPU_R8UI:
    case GPU_R8I:
    case GPU_R8:
    case GPU_R32UI:
    case GPU_R32I:
    case GPU_R32F:
    case GPU_R16UI:
    case GPU_R16I:
    case GPU_R16F:
    case GPU_R16:

    case GPU_RGB10_A2:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:

    /* Texture only format */
    case GPU_RGB16F:

    /* Special formats texture only */
    case GPU_SRGB8_A8_DXT1:
    case GPU_SRGB8_A8_DXT3:
    case GPU_SRGB8_A8_DXT5:
    case GPU_RGBA8_DXT1:
    case GPU_RGBA8_DXT3:
    case GPU_RGBA8_DXT5:

      /* Depth Formats */
    case GPU_DEPTH_COMPONENT32F:
    case GPU_DEPTH_COMPONENT24:
    case GPU_DEPTH_COMPONENT16:
      BLI_assert_unreachable();
  }
  return VK_FORMAT_UNDEFINED;
}

bool VKTexture::init_internal()
{
  /* TODO: add some pre-initialization to reduce some work later on.*/
  return true;
}

bool VKTexture::init_internal(GPUVertBuf * /*vbo*/)
{
  return false;
}

bool VKTexture::init_internal(const GPUTexture * /*src*/, int /*mip_offset*/, int /*layer_offset*/)
{
  return false;
}

bool VKTexture::is_allocated()
{
  return vk_image_ != VK_NULL_HANDLE && allocation_ != VK_NULL_HANDLE;
}

static VkImageType to_vk_image_type(const eGPUTextureType type)
{
  switch (type) {
    case GPU_TEXTURE_1D:
    case GPU_TEXTURE_BUFFER:
    case GPU_TEXTURE_1D_ARRAY:
      return VK_IMAGE_TYPE_1D;
    case GPU_TEXTURE_2D:
    case GPU_TEXTURE_2D_ARRAY:
      return VK_IMAGE_TYPE_2D;
    case GPU_TEXTURE_3D:
    case GPU_TEXTURE_CUBE:
    case GPU_TEXTURE_CUBE_ARRAY:
      return VK_IMAGE_TYPE_3D;

    case GPU_TEXTURE_ARRAY:
      /* GPU_TEXTURE_ARRAY should always be used together with 1D, 2D, or CUBE*/
      BLI_assert_unreachable();
      break;
  }

  return VK_IMAGE_TYPE_1D;
}

static VkImageViewType to_vk_image_view_type(const eGPUTextureType type)
{
  switch (type) {
    case GPU_TEXTURE_1D:
    case GPU_TEXTURE_BUFFER:
      return VK_IMAGE_VIEW_TYPE_1D;
    case GPU_TEXTURE_2D:
      return VK_IMAGE_VIEW_TYPE_2D;
    case GPU_TEXTURE_3D:
      return VK_IMAGE_VIEW_TYPE_3D;
    case GPU_TEXTURE_CUBE:
      return VK_IMAGE_VIEW_TYPE_CUBE;
    case GPU_TEXTURE_1D_ARRAY:
      return VK_IMAGE_VIEW_TYPE_1D_ARRAY;
    case GPU_TEXTURE_2D_ARRAY:
      return VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    case GPU_TEXTURE_CUBE_ARRAY:
      return VK_IMAGE_VIEW_TYPE_CUBE_ARRAY;

    case GPU_TEXTURE_ARRAY:
      /* GPU_TEXTURE_ARRAY should always be used together with 1D, 2D, or CUBE*/
      BLI_assert_unreachable();
      break;
  }

  return VK_IMAGE_VIEW_TYPE_1D;
}

static VkComponentMapping to_vk_component_mapping(const eGPUTextureFormat /*format*/)
{
  /* TODO: this should map to OpenGL defaults based on the eGPUTextureFormat*/
  VkComponentMapping component_mapping;
  component_mapping.r = VK_COMPONENT_SWIZZLE_R;
  component_mapping.g = VK_COMPONENT_SWIZZLE_G;
  component_mapping.b = VK_COMPONENT_SWIZZLE_B;
  component_mapping.a = VK_COMPONENT_SWIZZLE_A;
  return component_mapping;
}

bool VKTexture::allocate()
{
  BLI_assert(!is_allocated());

  int extent[3] = {1, 1, 1};
  mip_size_get(0, extent);

  VKContext &context = *VKContext::get();
  VkImageCreateInfo image_info = {};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = to_vk_image_type(type_);
  image_info.extent.width = extent[0];
  image_info.extent.height = extent[1];
  image_info.extent.depth = extent[2];
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = to_vk_format(format_);
  image_info.tiling = VK_IMAGE_TILING_LINEAR;
  /* TODO: PREINITIALIZED should be for device only textures. Others should use
   * VK_IMAGE_LAYOUT_UNDEFINED and upload its content.*/
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                     VK_IMAGE_USAGE_STORAGE_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;

  VmaAllocationCreateInfo allocCreateInfo = {};
  allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
  allocCreateInfo.flags = static_cast<VmaAllocationCreateFlagBits>(
      VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT);
  allocCreateInfo.priority = 1.0f;

  VkResult result = vmaCreateImage(context.mem_allocator_get(),
                                   &image_info,
                                   &allocCreateInfo,
                                   &vk_image_,
                                   &allocation_,
                                   nullptr);
  if (result != VK_SUCCESS) {
    return false;
  }

  /* Promote image to the correct layout.*/
  VkImageMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
  barrier.image = vk_image_;
  barrier.subresourceRange.aspectMask = to_vk_image_aspect_flag_bits(format_);
  barrier.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
  barrier.subresourceRange.layerCount = VK_REMAINING_ARRAY_LAYERS;
  context.command_buffer_get().pipeline_barrier(Span<VkImageMemoryBarrier>(&barrier, 1));

  VK_ALLOCATION_CALLBACKS
  VkImageViewCreateInfo image_view_info = {};
  image_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  image_view_info.image = vk_image_;
  image_view_info.viewType = to_vk_image_view_type(type_);
  image_view_info.format = to_vk_format(format_);
  image_view_info.components = to_vk_component_mapping(format_);
  image_view_info.subresourceRange.aspectMask = to_vk_image_aspect_flag_bits(format_);
  image_view_info.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
  image_view_info.subresourceRange.layerCount = VK_REMAINING_ARRAY_LAYERS;

  result = vkCreateImageView(
      context.device_get(), &image_view_info, vk_allocation_callbacks, &vk_image_view_);
  return result == VK_SUCCESS;
}

void VKTexture::image_bind(int location)
{
  if (!is_allocated()) {
    allocate();
  }
  VKContext &context = *VKContext::get();
  VKShader *shader = static_cast<VKShader *>(context.shader);
  shader->pipeline_get().descriptor_set_get().image_bind(*this, location);
}

}  // namespace blender::gpu
