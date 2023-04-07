#include "testing/testing.h"

#include "gpu_testing.hh"
#include "vulkan/vk_context.hh"

namespace blender::gpu::tests {

static void test_get_memory_statistics() 
{
  VKContext *ctx = VKContext::get();
  EXPECT_TRUE(ctx != nullptr);
  
  int total_mem, free_mem;
  ctx->memory_statistics_get(&total_mem, &free_mem);
  EXPECT_GT(free_mem, 0);
  EXPECT_GT(total_mem, free_mem);
}

GPU_TEST(get_memory_statistics)

} 

