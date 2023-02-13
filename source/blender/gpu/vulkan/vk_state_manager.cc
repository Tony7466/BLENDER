/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_state_manager.hh"
#include "vk_texture.hh"

namespace blender::gpu {
void VKStateManager::apply_state()
{
}

void VKStateManager::force_state()
{
}

void VKStateManager::issue_barrier(eGPUBarrier /*barrier_bits*/)
{
}

void VKStateManager::texture_bind(Texture * /*tex*/, eGPUSamplerState /*sampler*/, int /*unit*/)
{
}

void VKStateManager::texture_unbind(Texture * /*tex*/)
{
}

void VKStateManager::texture_unbind_all()
{
}

void VKStateManager::image_bind(Texture *tex, int binding)
{
  VKTexture *texture = unwrap(tex);
  texture->image_bind(binding);
}

void VKStateManager::image_unbind(Texture * /*tex*/)
{
}

void VKStateManager::image_unbind_all()
{
}

void VKStateManager::texture_unpack_row_length_set(uint /*len*/)
{
}

}  // namespace blender::gpu