/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "GPU_context.h"
#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_shader.h"
#include "GPU_vertex_buffer.h"
#include "GPU_vertex_format.h"

#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"

#include "gpu_testing.hh"
#include "gpu_shader_create_info.hh"

namespace blender::gpu::tests {

static constexpr int Size = 256;

template<GPUVertCompType comp_type, GPUVertFetchMode fetch_mode, typename ColorType>
static void vertex_buffer_fetch_mode(ColorType color)
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size,
                                                 Size,
                                                 false,
                                                 GPU_RGBA16F,
                                                 GPU_TEXTURE_USAGE_ATTACHMENT |
                                                     GPU_TEXTURE_USAGE_HOST_READ,
                                                 nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);
  GPUTexture *color_texture = GPU_offscreen_color_texture(offscreen);
  GPU_texture_clear(color_texture, GPU_DATA_FLOAT, float4(0.0f));

  GPUVertFormat format = {0};
  GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  GPU_vertformat_attr_add(&format, "color", comp_type, 4, fetch_mode);

  GPUVertBuf *vbo = GPU_vertbuf_create_with_format(&format);
  GPU_vertbuf_data_alloc(vbo, 4);

  struct Vert {
    float2 pos;
    ColorType color;
  };
  Vert data[4] = {
      {float2(-1.0, -1.0), color},
      {float2(1.0, -1.0), color},
      {float2(1.0, 1.0), color},
      {float2(-1.0, 1.0), color},
  };
  for (int i : IndexRange(4)) {
    GPU_vertbuf_vert_set(vbo, i, &data[i]);
  }

  GPUBatch *batch = GPU_batch_create(GPU_PRIM_TRI_FAN, vbo, nullptr);
  GPU_batch_program_set_builtin(batch, GPU_SHADER_3D_FLAT_COLOR);
  GPU_batch_draw(batch);

  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  /* Read back data and perform some basic tests. */
  float read_data[4 * Size * Size];
  GPU_offscreen_read_color(offscreen, GPU_DATA_FLOAT, &read_data);
  for (int pixel_index = 0; pixel_index < Size * Size; pixel_index++) {
    float4 read_color = float4(&read_data[pixel_index * 4]);
    EXPECT_EQ(read_color, float4(color));
  }

  GPU_batch_discard(batch);
  GPU_vertbuf_discard(vbo);
  GPU_offscreen_free(offscreen);
}
static inline const char *to_string(const blender::gpu::shader::Type type)
{
  using namespace blender::gpu::shader;
  switch (type) {
    case Type::FLOAT:
      return "float";
    case Type::VEC2:
      return "vec2";
    case Type::VEC3:
      return "vec3";
    case Type::VEC4:
      return "vec4";
    case Type::MAT3:
      return "mat3";
    case Type::MAT4:
      return "mat4";
    case Type::VEC3_101010I2:
      return "vec3";
    case Type::UCHAR:
      return "uchar";
    case Type::UCHAR2:
      return "uchar2";
    case Type::UCHAR3:
      return "uchar3";
    case Type::UCHAR4:
      return "uchar4";
    case Type::CHAR:
      return "char";
    case Type::CHAR2:
      return "char2";
    case Type::CHAR3:
      return "char3";
    case Type::CHAR4:
      return "char4";
    case Type::INT:
      return "int";
    case Type::IVEC2:
      return "ivec2";
    case Type::IVEC3:
      return "ivec3";
    case Type::IVEC4:
      return "ivec4";
    case Type::UINT:
      return "uint";
    case Type::UVEC2:
      return "uvec2";
    case Type::UVEC3:
      return "uvec3";
    case Type::UVEC4:
      return "uvec4";
    case Type::USHORT:
      return "ushort";
    case Type::USHORT2:
      return "ushort2";
    case Type::USHORT3:
      return "ushort3";
    case Type::USHORT4:
      return "ushort4";
    case Type::SHORT:
      return "short";
    case Type::SHORT2:
      return "short2";
    case Type::SHORT3:
      return "short3";
    case Type::SHORT4:
      return "short4";
    default:
      BLI_assert(0);
      return "";
  }
}

template<typename CompType,blender::gpu::shader::Type Type,
         size_t item_size>
static void vertex_buffer_default_value(Vector<CompType> data)
{
  using namespace blender::gpu::shader;
  
  size_t out_size = sizeof(CompType) * item_size;
  GPU_render_begin();
  out_size = (out_size % 16 != 0) ? out_size + (16 - (out_size % 16)) : out_size;
  GPUStorageBuf *out_value = GPU_storagebuf_create_ex(out_size,
                                                           nullptr,
                                                           GPU_USAGE_STATIC,
                                                           "out_value");


  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d(
      __func__, 1,1, 1, GPU_RGBA32F, usage, nullptr);

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create(__func__);
  GPU_framebuffer_ensure_config(&framebuffer,
                                {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(texture)});

  GPU_framebuffer_bind(framebuffer);
  ShaderCreateInfo create_info("");
  create_info.builtins(BuiltinBits::POINT_SIZE);
  create_info.vertex_source("gpu_vertex_input_default_test.glsl");
  create_info.vertex_in(0, Type, "in_value");
  create_info.storage_buf(0, Qualifier::WRITE,to_string(Type), "out_value");
  create_info.fragment_source("gpu_vertex_input_default_test.glsl");


  GPUShader *shader = GPU_shader_create_from_info(
      reinterpret_cast<GPUShaderCreateInfo *>(&create_info));

  GPUBatch *batch = GPU_batch_create_ex(GPU_PRIM_POINTS, nullptr, nullptr, GPU_BATCH_OWNS_VBO);

  GPU_batch_set_shader(batch, shader);

  const int positions_ssbo_location = GPU_shader_get_ssbo_binding(shader, "out_value");
  GPU_storagebuf_bind(out_value, positions_ssbo_location);

  GPU_batch_draw_advanced(batch, 0, 1, 0, 1);

  GPU_batch_discard(batch);

  GPU_finish();

  /* Read back data from SSBO. */
  Vector<CompType> read_data;
  read_data.resize(item_size, 0);
  GPU_storagebuf_read(out_value, read_data.data());
  /* Check if data is the same. */
  for (int i : IndexRange(item_size)) {
    EXPECT_EQ(data[i], read_data[i]);
  }

  GPU_framebuffer_free(framebuffer);
  GPU_storagebuf_free(out_value);
  GPU_shader_free(shader);
  GPU_texture_free(texture);
  GPU_render_end();
  }

  static void test_vertex_buffer_fetch_mode__GPU_COMP_I8__GPU_FETCH_INT_TO_FLOAT()
{
  vertex_buffer_fetch_mode<GPU_COMP_I8, GPU_FETCH_INT_TO_FLOAT, char4>(char4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_I8__GPU_FETCH_INT_TO_FLOAT);

static void test_vertex_buffer_fetch_mode__GPU_COMP_U8__GPU_FETCH_INT_TO_FLOAT()
{
  vertex_buffer_fetch_mode<GPU_COMP_U8, GPU_FETCH_INT_TO_FLOAT, uchar4>(uchar4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_U8__GPU_FETCH_INT_TO_FLOAT);

static void test_vertex_buffer_fetch_mode__GPU_COMP_I16__GPU_FETCH_INT_TO_FLOAT()
{
  vertex_buffer_fetch_mode<GPU_COMP_I16, GPU_FETCH_INT_TO_FLOAT, short4>(short4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_I16__GPU_FETCH_INT_TO_FLOAT);

static void test_vertex_buffer_fetch_mode__GPU_COMP_U16__GPU_FETCH_INT_TO_FLOAT()
{
  vertex_buffer_fetch_mode<GPU_COMP_U16, GPU_FETCH_INT_TO_FLOAT, ushort4>(ushort4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_U16__GPU_FETCH_INT_TO_FLOAT);

static void test_vertex_buffer_fetch_mode__GPU_COMP_I32__GPU_FETCH_INT_TO_FLOAT()
{
  if (GPU_backend_type_selection_get() == GPU_BACKEND_VULKAN) {
    GTEST_SKIP() << "interleaved format not supported yet";
  }
  vertex_buffer_fetch_mode<GPU_COMP_I32, GPU_FETCH_INT_TO_FLOAT, int4>(int4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_I32__GPU_FETCH_INT_TO_FLOAT);

static void test_vertex_buffer_fetch_mode__GPU_COMP_U32__GPU_FETCH_INT_TO_FLOAT()
{
  if (GPU_backend_type_selection_get() == GPU_BACKEND_VULKAN) {
    GTEST_SKIP() << "interleaved format not supported yet";
  }
  vertex_buffer_fetch_mode<GPU_COMP_U32, GPU_FETCH_INT_TO_FLOAT, uint4>(uint4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_U32__GPU_FETCH_INT_TO_FLOAT);

static void test_vertex_buffer_fetch_mode__GPU_COMP_F32__GPU_FETCH_FLOAT()
{
  vertex_buffer_fetch_mode<GPU_COMP_F32, GPU_FETCH_FLOAT, float4>(float4(4, 5, 6, 1));
}
GPU_TEST(vertex_buffer_fetch_mode__GPU_COMP_F32__GPU_FETCH_FLOAT);

static void test_vertex_buffer_default_vec4()
{
  vertex_buffer_default_value<float, blender::gpu::shader::Type::VEC4, 4>({0., 0., 0., 1.f});
}
GPU_TEST(vertex_buffer_default_vec4);

static void test_vertex_buffer_default_vec3()
{
  vertex_buffer_default_value<float, blender::gpu::shader::Type::VEC3, 3>({0., 0., 0.f});
}
GPU_TEST(vertex_buffer_default_vec3);

static void test_vertex_buffer_default_vec2()
{
  vertex_buffer_default_value<float, blender::gpu::shader::Type::VEC2, 2>({0., 0.});
}
GPU_TEST(vertex_buffer_default_vec2);

static void test_vertex_buffer_default_mat4()
{
  vertex_buffer_default_value<float, blender::gpu::shader::Type::MAT4, 16>({
      0.,
      0.,
      0.,
      1.,
      0.,
      0.,
      0.,
      1.,
      0.,
      0.,
      0.,
      1.,
      0.,
      0.,
      0.,
      1.,
  });
}
GPU_TEST(vertex_buffer_default_mat4);

static void test_vertex_buffer_default_mat3()
{
  vertex_buffer_default_value<float, blender::gpu::shader::Type::MAT3, 12>(
      {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.});
}
GPU_TEST(vertex_buffer_default_mat3);

static void test_vertex_buffer_default_ivec4()
{
  vertex_buffer_default_value<int, blender::gpu::shader::Type::IVEC4, 4>({0, 0, 0, 1});
}
GPU_TEST(vertex_buffer_default_ivec4);

static void test_vertex_buffer_default_ivec3()
{
  vertex_buffer_default_value<int, blender::gpu::shader::Type::IVEC3, 3>({0, 0, 0});
}
GPU_TEST(vertex_buffer_default_ivec3);

static void test_vertex_buffer_default_uvec4()
{
  vertex_buffer_default_value<uint, blender::gpu::shader::Type::UVEC4, 4>({0, 0, 0, 1});
}
GPU_TEST(vertex_buffer_default_uvec4);

static void test_vertex_buffer_default_uvec3()
{
  vertex_buffer_default_value<uint, blender::gpu::shader::Type::UVEC3, 3>({0, 0, 0});
}
GPU_TEST(vertex_buffer_default_uvec3);

}  // namespace blender::gpu::tests
