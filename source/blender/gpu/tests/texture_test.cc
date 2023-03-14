#include "gpu_testing.hh"

#include "MEM_guardedalloc.h"

#include "BLI_math_vector.hh"

#include "GPU_context.h"
#include "GPU_texture.h"

namespace blender::gpu::tests {

static void test_texture_read()
{
  GPU_render_begin();

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *rgba32u = GPU_texture_create_2d("rgba32u", 1, 1, 1, GPU_RGBA32UI, usage, nullptr);
  GPUTexture *rgba16u = GPU_texture_create_2d("rgba16u", 1, 1, 1, GPU_RGBA16UI, usage, nullptr);
  GPUTexture *rgba32f = GPU_texture_create_2d("rgba32f", 1, 1, 1, GPU_RGBA32F, usage, nullptr);

  const float4 fcol = {0.0f, 1.3f, -231.0f, 1000.0f};
  const uint4 ucol = {0, 1, 2, 12223};
  GPU_texture_clear(rgba32u, GPU_DATA_UINT, ucol);
  GPU_texture_clear(rgba16u, GPU_DATA_UINT, ucol);
  GPU_texture_clear(rgba32f, GPU_DATA_FLOAT, fcol);

  GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);

  uint4 *rgba32u_data = (uint4 *)GPU_texture_read(rgba32u, GPU_DATA_UINT, 0);
  uint4 *rgba16u_data = (uint4 *)GPU_texture_read(rgba16u, GPU_DATA_UINT, 0);
  float4 *rgba32f_data = (float4 *)GPU_texture_read(rgba32f, GPU_DATA_FLOAT, 0);

  EXPECT_EQ(ucol, *rgba32u_data);
  EXPECT_EQ(ucol, *rgba16u_data);
  EXPECT_EQ(fcol, *rgba32f_data);

  MEM_freeN(rgba32u_data);
  MEM_freeN(rgba16u_data);
  MEM_freeN(rgba32f_data);

  GPU_texture_free(rgba32u);
  GPU_texture_free(rgba16u);
  GPU_texture_free(rgba32f);

  GPU_render_end();
}
GPU_TEST(texture_read)

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing 32F
 * \{ */

static float *generate_test_data_float(size_t data_len)
{
  float *data = static_cast<float *>(MEM_mallocN(data_len * sizeof(float), __func__));
  for (int i : IndexRange(data_len)) {
    data[i] = 8.0 / max_ff(i % 8, 0.5f);
  }
  return data;
}

template<eGPUTextureFormat DeviceFormat, int ComponentLen, int Size = 256>
static void texture_create_upload_read_float()
{
  size_t data_len = Size * Size * ComponentLen;
  float *data = generate_test_data_float(data_len);

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d("texture", Size, Size, 1, DeviceFormat, usage, data);
  EXPECT_NE(texture, nullptr);

  float *read_data = (float *)GPU_texture_read(texture, GPU_DATA_FLOAT, 0);
  for (int i : IndexRange(data_len)) {
    EXPECT_EQ(read_data[i], data[i]);
  }
  MEM_freeN(read_data);

  GPU_texture_free(texture);
  MEM_freeN(data);
}

static void test_texture_roundtrip_FLOAT_RGBA32F()
{
  texture_create_upload_read_float<GPU_RGBA32F, 4>();
}
GPU_TEST(texture_roundtrip_FLOAT_RGBA32F)

#if 0
/* Isn't supported natively on NVidia/Vulkan. */
static void test_texture_roundtrip_FLOAT_RGBA32F()
{
  texture_create_upload_read_float<GPU_RGBA32F, 4>();
}
GPU_TEST(texture_roundtrip_FLOAT_RGBA32F)
#endif

static void test_texture_roundtrip_FLOAT_RG32F()
{
  texture_create_upload_read_float<GPU_RG32F, 2>();
}
GPU_TEST(texture_roundtrip_FLOAT_RG32F)

static void test_texture_roundtrip_FLOAT_R32F()
{
  texture_create_upload_read_float<GPU_R32F, 1>();
}
GPU_TEST(texture_roundtrip_FLOAT_R32F)

/** \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing 32UI
 * \{ */

static uint32_t *generate_test_data_uint(size_t data_len)
{
  uint32_t *data = static_cast<uint32_t *>(MEM_mallocN(data_len * sizeof(uint32_t), __func__));
  for (int i : IndexRange(data_len)) {
    data[i] = 8 / max_ii(i % 8, 1);
  }
  return data;
}

template<eGPUTextureFormat DeviceFormat, int ComponentLen, int Size = 256>
static void texture_create_upload_read_uint()
{

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d(
      "texture", Size, Size, 1, DeviceFormat, usage, nullptr);
  EXPECT_NE(texture, nullptr);

  size_t data_len = Size * Size * ComponentLen;
  uint32_t *data = generate_test_data_uint(data_len);
  GPU_texture_update(texture, GPU_DATA_UINT, data);

  uint32_t *read_data = (uint32_t *)GPU_texture_read(texture, GPU_DATA_UINT, 0);
  for (int i : IndexRange(data_len)) {
    EXPECT_EQ(read_data[i], data[i]);
  }
  MEM_freeN(read_data);

  GPU_texture_free(texture);
  MEM_freeN(data);
}

static void test_texture_roundtrip_UINT_RGBA32UI()
{
  texture_create_upload_read_uint<GPU_RGBA32UI, 4>();
}
GPU_TEST(texture_roundtrip_UINT_RGBA32UI)

#if 0
/* Isn't supported natively on NVidia/Vulkan. */
static void test_texture_roundtrip_UINT_RGB32UI()
{
  texture_create_upload_read_uint<GPU_RGB32UI, 3>();
}
GPU_TEST(texture_roundtrip_UINT_RGB32UI)
#endif

static void test_texture_roundtrip_UINT_RG32UI()
{
  texture_create_upload_read_uint<GPU_RG32UI, 2>();
}
GPU_TEST(texture_roundtrip_UINT_RG32UI)

static void test_texture_roundtrip_UINT_R32UI()
{
  texture_create_upload_read_uint<GPU_R32UI, 1>();
}
GPU_TEST(texture_roundtrip_UINT_R32UI)

/** \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing 32I
 * \{ */

static int32_t *generate_test_data_int(size_t data_len)
{
  int32_t *data = static_cast<int32_t *>(MEM_mallocN(data_len * sizeof(int32_t), __func__));
  for (int i : IndexRange(data_len)) {
    data[i] = 8 / max_ii(i % 8, 1);
  }
  return data;
}

template<eGPUTextureFormat DeviceFormat, int ComponentLen, int Size = 256>
static void texture_create_upload_read_int()
{

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d(
      "texture", Size, Size, 1, DeviceFormat, usage, nullptr);
  EXPECT_NE(texture, nullptr);

  size_t data_len = Size * Size * ComponentLen;
  int32_t *data = generate_test_data_int(data_len);
  GPU_texture_update(texture, GPU_DATA_INT, data);

  uint32_t *read_data = (uint32_t *)GPU_texture_read(texture, GPU_DATA_INT, 0);
  for (int i : IndexRange(data_len)) {
    EXPECT_EQ(read_data[i], data[i]);
  }
  MEM_freeN(read_data);

  GPU_texture_free(texture);
  MEM_freeN(data);
}

static void test_texture_roundtrip_INT_RGBA32I()
{
  texture_create_upload_read_int<GPU_RGBA32I, 4>();
}
GPU_TEST(texture_roundtrip_INT_RGBA32I)

#if 0
/* Isn't supported natively on NVidia/Vulkan. */
static void test_texture_roundtrip_INT_RGB32I()
{
  texture_create_upload_read_int<GPU_RGB32I, 3>();
}
GPU_TEST(texture_roundtrip_INT_RGB32I)
#endif

static void test_texture_roundtrip_INT_RG32I()
{
  texture_create_upload_read_int<GPU_RG32I, 2>();
}
GPU_TEST(texture_roundtrip_INT_RG32I)

static void test_texture_roundtrip_INT_R32I()
{
  texture_create_upload_read_int<GPU_R32I, 1>();
}
GPU_TEST(texture_roundtrip_INT_R32I)

/** \} */

}  // namespace blender::gpu::tests