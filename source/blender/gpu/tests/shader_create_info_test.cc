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
 * OpenGL tests.
 *
 * \NOTE: Vulkan is not tested as it requires GHOST system creation without Wayland or X11 on
 * Linux. This should be possible as we support headless rendering. On Windows it requires a
 * software vulkan driver to be installed but that requires some software packages to be installed
 * on the Windows build-bot machines.
 */
static void test_static_shaders()
{
  EXPECT_TRUE(gpu_shader_create_info_compile(nullptr));
}
GPU_METAL_TEST(static_shaders)

}  // namespace blender::gpu::tests
