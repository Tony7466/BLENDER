#include "testing/testing.h"

#include "GPU_shader.h"
#include "GPU_vertex_buffer.h"

#include "gpu_testing.hh"

namespace blender::gpu::tests {

template<GPUVertCompType comp_type, GPUVertFetchMode fetch_mode> static void test_comp_fetch()
{
  GPUShader *shader = GPU_shader_create_from_info_name("gpu_vertex_buffer_fetch_mode_test");
  EXPECT_NE(shader, nullptr);
  GPU_shader_bind(shader);

  GPUVertFormat format = {0};
  GPU_vertformat_attr_add(&format, "pos", comp_type, 2, fetch_mode);
  GPUVertBuf *vbo = GPU_vertbuf_create_with_format(&format);
  GPU_vertbuf_data_alloc(vbo, 4);
}

static void test_vertex_buffer_fetch_mode__GPU_COMP_I32__GPU_FETCH_INT_TO_FLOAT()
{
  test_comp_fetch<GPU_COMP_I32, GPU_FETCH_INT_TO_FLOAT>();
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_I32__GPU_FETCH_INT_TO_FLOAT);

}  // namespace blender::gpu::tests
