/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_texture.hh"

#include "vk_context.hh"
#include "vk_shader.hh"

namespace blender::gpu {

VKTexture::~VKTexture()
{
  VKContext &context = *VKContext::get();
  vmaDestroyImage(context.mem_allocator_get(), vk_image_, allocation_);
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

void *VKTexture::read(int /*mip*/, eGPUDataFormat /*format*/)
{
  return nullptr;
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

    /*
    case GPU_RGB10_A2:
    case GPU_R11F_G11F_B10F:
    case GPU_DEPTH32F_STENCIL8:
    case GPU_DEPTH24_STENCIL8:
    case GPU_SRGB8_A8:*/

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
    default:
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

bool VKTexture::allocate()
{
  BLI_assert(!is_allocated());

  VKContext &context = *VKContext::get();
  VkImageCreateInfo imgCreateInfo = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  imgCreateInfo.imageType = VK_IMAGE_TYPE_1D;
  imgCreateInfo.extent.width = width_get();
  imgCreateInfo.extent.height = 1;
  imgCreateInfo.extent.depth = 1;
  imgCreateInfo.mipLevels = 1;
  imgCreateInfo.arrayLayers = 1;
  imgCreateInfo.format = to_vk_format(format_);
  imgCreateInfo.tiling = VK_IMAGE_TILING_LINEAR;
  imgCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  imgCreateInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
  imgCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;

  VmaAllocationCreateInfo allocCreateInfo = {};
  allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
  allocCreateInfo.flags = static_cast<VmaAllocationCreateFlagBits>(
      VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT);
  allocCreateInfo.priority = 1.0f;

  VkResult result = vmaCreateImage(context.mem_allocator_get(),
                                   &imgCreateInfo,
                                   &allocCreateInfo,
                                   &vk_image_,
                                   &allocation_,
                                   nullptr);
  if (result != VK_SUCCESS) {
    return false;
  }
  return true;
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
