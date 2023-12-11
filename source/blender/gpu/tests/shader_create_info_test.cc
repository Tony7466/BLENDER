/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "gpu_shader_create_info_private.hh"
#include "gpu_testing.hh"

namespace blender::gpu::tests {

/**
 * Test if all static shaders can be compiled.
 *
 * \NOTE: OpenGL is not tested as this test is enabled on the buildbot that isn't capable to run
 *        OpenGL tests. Therefore only Metal and Vulkan are tested.
 */
static void test_static_shaders()
{
  EXPECT_TRUE(gpu_shader_create_info_compile(nullptr));
}
GPU_METAL_TEST(static_shaders)
GPU_VULKAN_TEST(static_shaders)

}  // namespace blender::gpu::tests
