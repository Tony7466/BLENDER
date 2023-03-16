#include "gpu_testing.hh"

#include "MEM_guardedalloc.h"

#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "GPU_context.h"
#include "GPU_texture.h"

#include "gpu_texture_private.hh"

/* Not all texture types are supported by all platforms. This define safe guards them until we have
 * a working workaround or decided to remove support for those texture types. */
#define RUN_UNSUPPORTED false
/* Skip tests that haven't been developed yet due to non standard data types. */
#define RUN_16F_UNIMPLEMENTED false
#define RUN_SRGB_UNIMPLEMENTED false
#define RUN_NON_STANDARD_UNIMPLEMENTED false
#define RUN_COMPONENT_UNIMPLEMENTED false

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

template<typename DataType> static DataType *generate_test_data(size_t data_len)
{
  DataType *data = static_cast<DataType *>(MEM_mallocN(data_len * sizeof(DataType), __func__));
  for (int i : IndexRange(data_len)) {
    if (std::is_same<DataType, float>()) {
      data[i] = (DataType)(i % 8) / 8.0f;
    }
    else {
      data[i] = (DataType)(i % 8);
    }
  }
  return data;
}

template<eGPUTextureFormat DeviceFormat,
         eGPUDataFormat HostFormat,
         typename DataType,
         int Size = 16>
static void texture_create_upload_read()
{
  static_assert(validate_data_format(DeviceFormat, HostFormat));
  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d(
      "texture", Size, Size, 1, DeviceFormat, usage, nullptr);
  EXPECT_NE(texture, nullptr);

  size_t data_len = Size * Size * to_component_len(DeviceFormat);
  DataType *data = static_cast<DataType *>(generate_test_data<DataType>(data_len));
  GPU_texture_update(texture, HostFormat, data);

  DataType *read_data = static_cast<DataType *>(GPU_texture_read(texture, HostFormat, 0));
  bool failed = false;
  for (int i : IndexRange(data_len)) {
    bool ok = abs(read_data[i] - data[i]) < 0.01;
    failed |= !ok;
    //EXPECT_EQ(read_data[i], data[i]);
  }
  EXPECT_FALSE(failed);

  MEM_freeN(read_data);
  MEM_freeN(data);

  GPU_texture_free(texture);
}

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_FLOAT
 * \{ */
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8()
{
  texture_create_upload_read<GPU_RGBA8, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8);

#if RUN_16F_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA16F()
{
  texture_create_upload_read<GPU_RGBA16F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA16F);
#endif

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA16()
{
  texture_create_upload_read<GPU_RGBA16, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA16);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA32F()
{
  texture_create_upload_read<GPU_RGBA32F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA32F);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RG8()
{
  texture_create_upload_read<GPU_RG8, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RG8);

#if RUN_16F_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RG16F()
{
  texture_create_upload_read<GPU_RG16F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RG16F);
#endif

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RG16()
{
  texture_create_upload_read<GPU_RG16, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RG16);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RG32F()
{
  texture_create_upload_read<GPU_RG32F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RG32F);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R8()
{
  texture_create_upload_read<GPU_R8, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R8);

#if RUN_16F_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R16F()
{
  texture_create_upload_read<GPU_R16F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R16F);
#endif

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R16()
{
  texture_create_upload_read<GPU_R16, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R16);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R32F()
{
  texture_create_upload_read<GPU_R32F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R32F);

#if RUN_NON_STANDARD_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB10_A2()
{
  texture_create_upload_read<GPU_RGB10_A2, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB10_A2);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB10_A2UI()
{
  texture_create_upload_read<GPU_RGB10_A2UI, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB10_A2UI);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R11F_G11F_B10F()
{
  texture_create_upload_read<GPU_R11F_G11F_B10F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R11F_G11F_B10F);
#endif

#if RUN_SRGB_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8()
{
  texture_create_upload_read<GPU_SRGB8_A8, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8);
#endif

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_SNORM()
{
  texture_create_upload_read<GPU_RGBA8_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_SNORM);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA16_SNORM()
{
  texture_create_upload_read<GPU_RGBA16_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA16_SNORM);

#if RUN_UNSUPPORTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB8()
{
  texture_create_upload_read<GPU_RGB8, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB8);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB8_SNORM()
{
  texture_create_upload_read<GPU_RGB8_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB8_SNORM);
#endif

#if RUN_16F_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB16F()
{
  texture_create_upload_read<GPU_RGB16F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB16F);
#endif
#if RUN_UNSUPPORTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB16()
{
  texture_create_upload_read<GPU_RGB16, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB16);
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB16_SNORM()
{
  texture_create_upload_read<GPU_RGB16_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB16_SNORM);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB32F()
{
  texture_create_upload_read<GPU_RGB32F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB32F);
#endif

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RG8_SNORM()
{
  texture_create_upload_read<GPU_RG8_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RG8_SNORM);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RG16_SNORM()
{
  texture_create_upload_read<GPU_RG16_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RG16_SNORM);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R8_SNORM()
{
  texture_create_upload_read<GPU_R8_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R8_SNORM);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_R16_SNORM()
{
  texture_create_upload_read<GPU_R16_SNORM, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_R16_SNORM);

#if RUN_NON_STANDARD_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8_DXT1()
{
  texture_create_upload_read<GPU_SRGB8_A8_DXT1, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8_DXT1);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8_DXT3()
{
  texture_create_upload_read<GPU_SRGB8_A8_DXT3, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8_DXT3);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8_DXT5()
{
  texture_create_upload_read<GPU_SRGB8_A8_DXT5, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8_A8_DXT5);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_DXT1()
{
  texture_create_upload_read<GPU_RGBA8_DXT1, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_DXT1);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_DXT3()
{
  texture_create_upload_read<GPU_RGBA8_DXT3, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_DXT3);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_DXT5()
{
  texture_create_upload_read<GPU_RGBA8_DXT5, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGBA8_DXT5);
#endif

#if RUN_SRGB_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8()
{
  texture_create_upload_read<GPU_SRGB8, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_SRGB8);
#endif

#if RUN_NON_STANDARD_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB9_E5()
{
  texture_create_upload_read<GPU_RGB9_E5, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_RGB9_E5);
#endif

#if RUN_UNSUPPORTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_DEPTH_COMPONENT32F()
{
  texture_create_upload_read<GPU_DEPTH_COMPONENT32F, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_DEPTH_COMPONENT32F);
#endif

#if RUN_COMPONENT_UNIMPLEMENTED
static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_DEPTH_COMPONENT24()
{
  texture_create_upload_read<GPU_DEPTH_COMPONENT24, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_DEPTH_COMPONENT24);

static void test_texture_roundtrip__GPU_DATA_FLOAT__GPU_DEPTH_COMPONENT16()
{
  texture_create_upload_read<GPU_DEPTH_COMPONENT16, GPU_DATA_FLOAT, float>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_FLOAT__GPU_DEPTH_COMPONENT16);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_HALF_FLOAT
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_RGBA16F()
{
  texture_create_upload_read<GPU_RGBA16F, GPU_DATA_HALF_FLOAT, half>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_RGBA16F);

static void test_texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_RG16F()
{
  texture_create_upload_read<GPU_RG16F, GPU_DATA_HALF_FLOAT, half>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_RG16F);

static void test_texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_R16F()
{
  texture_create_upload_read<GPU_R16F, GPU_DATA_HALF_FLOAT, half>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_R16F);

static void test_texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_RGB16F()
{
  texture_create_upload_read<GPU_RGB16F, GPU_DATA_HALF_FLOAT, half>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_HALF_FLOAT__GPU_RGB16F);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_INT
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_INT__GPU_RGBA8I()
{
  texture_create_upload_read<GPU_RGBA8I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RGBA8I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RGBA16I()
{
  texture_create_upload_read<GPU_RGBA16I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RGBA16I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RGBA32I()
{
  texture_create_upload_read<GPU_RGBA32I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RGBA32I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RG8I()
{
  texture_create_upload_read<GPU_RG8I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RG8I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RG16I()
{
  texture_create_upload_read<GPU_RG16I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RG16I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RG32I()
{
  texture_create_upload_read<GPU_RG32I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RG32I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_R8I()
{
  texture_create_upload_read<GPU_R8I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_R8I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_R16I()
{
  texture_create_upload_read<GPU_R16I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_R16I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_R32I()
{
  texture_create_upload_read<GPU_R32I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_R32I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RGB8I()
{
  texture_create_upload_read<GPU_RGB8I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RGB8I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RGB16I()
{
  texture_create_upload_read<GPU_RGB16I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RGB16I);

static void test_texture_roundtrip__GPU_DATA_INT__GPU_RGB32I()
{
  texture_create_upload_read<GPU_RGB32I, GPU_DATA_INT, int32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_INT__GPU_RGB32I);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_UINT
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RGBA8UI()
{
  texture_create_upload_read<GPU_RGBA8UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RGBA8UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RGBA16UI()
{
  texture_create_upload_read<GPU_RGBA16UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RGBA16UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RGBA32UI()
{
  texture_create_upload_read<GPU_RGBA32UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RGBA32UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RG8UI()
{
  texture_create_upload_read<GPU_RG8UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RG8UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RG16UI()
{
  texture_create_upload_read<GPU_RG16UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RG16UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RG32UI()
{
  texture_create_upload_read<GPU_RG32UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RG32UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_R8UI()
{
  texture_create_upload_read<GPU_R8UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_R8UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_R16UI()
{
  texture_create_upload_read<GPU_R16UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_R16UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_R32UI()
{
  texture_create_upload_read<GPU_R32UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_R32UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH32F_STENCIL8()
{
  texture_create_upload_read<GPU_DEPTH32F_STENCIL8, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH32F_STENCIL8);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH24_STENCIL8()
{
  texture_create_upload_read<GPU_DEPTH24_STENCIL8, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH24_STENCIL8);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RGB8UI()
{
  texture_create_upload_read<GPU_RGB8UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RGB8UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RGB16UI()
{
  texture_create_upload_read<GPU_RGB16UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RGB16UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_RGB32UI()
{
  texture_create_upload_read<GPU_RGB32UI, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_RGB32UI);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH_COMPONENT32F()
{
  texture_create_upload_read<GPU_DEPTH_COMPONENT32F, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH_COMPONENT32F);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH_COMPONENT24()
{
  texture_create_upload_read<GPU_DEPTH_COMPONENT24, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH_COMPONENT24);

static void test_texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH_COMPONENT16()
{
  texture_create_upload_read<GPU_DEPTH_COMPONENT16, GPU_DATA_UINT, uint32_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT__GPU_DEPTH_COMPONENT16);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_UBYTE
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_RGBA8UI()
{
  texture_create_upload_read<GPU_RGBA8UI, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_RGBA8UI);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_RGBA8()
{
  texture_create_upload_read<GPU_RGBA8, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_RGBA8);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_RG8UI()
{
  texture_create_upload_read<GPU_RG8UI, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_RG8UI);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_RG8()
{
  texture_create_upload_read<GPU_RG8, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_RG8);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_R8UI()
{
  texture_create_upload_read<GPU_R8UI, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_R8UI);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_R8()
{
  texture_create_upload_read<GPU_R8, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_R8);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_SRGB8_A8()
{
  texture_create_upload_read<GPU_SRGB8_A8, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_SRGB8_A8);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_RGB8I()
{
  texture_create_upload_read<GPU_RGB8I, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_RGB8I);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_RGB8()
{
  texture_create_upload_read<GPU_RGB8, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_RGB8);

static void test_texture_roundtrip__GPU_DATA_UBYTE__GPU_SRGB8()
{
  texture_create_upload_read<GPU_SRGB8, GPU_DATA_UBYTE, uint8_t>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UBYTE__GPU_SRGB8);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_UINT_24_8
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_UINT_24_8__GPU_DEPTH32F_STENCIL8()
{
  texture_create_upload_read<GPU_DEPTH32F_STENCIL8, GPU_DATA_UINT_24_8, void>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT_24_8__GPU_DEPTH32F_STENCIL8);

static void test_texture_roundtrip__GPU_DATA_UINT_24_8__GPU_DEPTH24_STENCIL8()
{
  texture_create_upload_read<GPU_DEPTH24_STENCIL8, GPU_DATA_UINT_24_8, void>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_UINT_24_8__GPU_DEPTH24_STENCIL8);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_10_11_11_REV
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_10_11_11_REV__GPU_R11F_G11F_B10F()
{
  texture_create_upload_read<GPU_R11F_G11F_B10F, GPU_DATA_10_11_11_REV, void>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_10_11_11_REV__GPU_R11F_G11F_B10F);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Roundtrip testing GPU_DATA_2_10_10_10_REV
 * \{ */
#if 0
static void test_texture_roundtrip__GPU_DATA_2_10_10_10_REV__GPU_RGB10_A2()
{
  texture_create_upload_read<GPU_RGB10_A2, GPU_DATA_2_10_10_10_REV, void>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_2_10_10_10_REV__GPU_RGB10_A2);
static void test_texture_roundtrip__GPU_DATA_2_10_10_10_REV__GPU_RGB10_A2UI()
{
  texture_create_upload_read<GPU_RGB10_A2UI, GPU_DATA_2_10_10_10_REV, void>();
}
GPU_TEST(texture_roundtrip__GPU_DATA_2_10_10_10_REV__GPU_RGB10_A2UI);
#endif
/* \} */

/* -------------------------------------------------------------------- */
/** \name Generate test cases.
 *
 * Next section is kept for convenience to regenerate test cases.
 *
 * \{ */
#if 0
  static std::string to_prim_type_string(eGPUDataFormat host_format)
  {
    switch (host_format) {
      case GPU_DATA_FLOAT:
        return std::string("float");

      case GPU_DATA_HALF_FLOAT:
        return std::string("half");
      case GPU_DATA_INT:
        return std::string("int32_t");
      case GPU_DATA_UINT:
        return std::string("uint32_t");
      case GPU_DATA_UBYTE:
        return std::string("uint8_t");
      case GPU_DATA_UINT_24_8:
      case GPU_DATA_10_11_11_REV:
      case GPU_DATA_2_10_10_10_REV:
        return std::string("void");
    }
    return std::string("UNKNOWN");
  }
  static std::string to_string(eGPUDataFormat host_format)
  {
    switch (host_format) {
      case GPU_DATA_FLOAT:
        return std::string("GPU_DATA_FLOAT");

      case GPU_DATA_HALF_FLOAT:
        return std::string("GPU_DATA_HALF_FLOAT");
      case GPU_DATA_INT:
        return std::string("GPU_DATA_INT");
      case GPU_DATA_UINT:
        return std::string("GPU_DATA_UINT");
      case GPU_DATA_UBYTE:
        return std::string("GPU_DATA_UBYTE");
      case GPU_DATA_UINT_24_8:
        return std::string("GPU_DATA_UINT_24_8");
      case GPU_DATA_10_11_11_REV:
        return std::string("GPU_DATA_10_11_11_REV");
      case GPU_DATA_2_10_10_10_REV:
        return std::string("GPU_DATA_2_10_10_10_REV");
    }
    return std::string("UNKNOWN");
  }

  static std::string to_string(eGPUTextureFormat texture_format)
  {
    return std::string("GPU_") + std::string(GPU_texture_format_name(texture_format));
  }

  TEST(gpu_util, generate_test_cases)
  {
    Vector<eGPUDataFormat> host_formats;
    host_formats.append(GPU_DATA_FLOAT);
    host_formats.append(GPU_DATA_HALF_FLOAT);
    host_formats.append(GPU_DATA_INT);
    host_formats.append(GPU_DATA_UINT);
    host_formats.append(GPU_DATA_UBYTE);
    host_formats.append(GPU_DATA_UINT_24_8);
    host_formats.append(GPU_DATA_10_11_11_REV);
    host_formats.append(GPU_DATA_2_10_10_10_REV);

    Vector<eGPUTextureFormat> texture_formats;
    texture_formats.append(GPU_RGBA8UI);
    texture_formats.append(GPU_RGBA8I);
    texture_formats.append(GPU_RGBA8);
    texture_formats.append(GPU_RGBA16UI);
    texture_formats.append(GPU_RGBA16I);
    texture_formats.append(GPU_RGBA16F);
    texture_formats.append(GPU_RGBA16);
    texture_formats.append(GPU_RGBA32UI);
    texture_formats.append(GPU_RGBA32I);
    texture_formats.append(GPU_RGBA32F);
    texture_formats.append(GPU_RG8UI);
    texture_formats.append(GPU_RG8I);
    texture_formats.append(GPU_RG8);
    texture_formats.append(GPU_RG16UI);
    texture_formats.append(GPU_RG16I);
    texture_formats.append(GPU_RG16F);
    texture_formats.append(GPU_RG16);
    texture_formats.append(GPU_RG32UI);
    texture_formats.append(GPU_RG32I);
    texture_formats.append(GPU_RG32F);
    texture_formats.append(GPU_R8UI);
    texture_formats.append(GPU_R8I);
    texture_formats.append(GPU_R8);
    texture_formats.append(GPU_R16UI);
    texture_formats.append(GPU_R16I);
    texture_formats.append(GPU_R16F);
    texture_formats.append(GPU_R16);
    texture_formats.append(GPU_R32UI);
    texture_formats.append(GPU_R32I);
    texture_formats.append(GPU_R32F);
    texture_formats.append(GPU_RGB10_A2);
    texture_formats.append(GPU_RGB10_A2UI);
    texture_formats.append(GPU_R11F_G11F_B10F);
    texture_formats.append(GPU_DEPTH32F_STENCIL8);
    texture_formats.append(GPU_DEPTH24_STENCIL8);
    texture_formats.append(GPU_SRGB8_A8);
    texture_formats.append(GPU_RGBA8_SNORM);
    texture_formats.append(GPU_RGBA16_SNORM);
    texture_formats.append(GPU_RGB8UI);
    texture_formats.append(GPU_RGB8I);
    texture_formats.append(GPU_RGB8);
    texture_formats.append(GPU_RGB8_SNORM);
    texture_formats.append(GPU_RGB16UI);
    texture_formats.append(GPU_RGB16I);
    texture_formats.append(GPU_RGB16F);
    texture_formats.append(GPU_RGB16);
    texture_formats.append(GPU_RGB16_SNORM);
    texture_formats.append(GPU_RGB32UI);
    texture_formats.append(GPU_RGB32I);
    texture_formats.append(GPU_RGB32F);
    texture_formats.append(GPU_RG8_SNORM);
    texture_formats.append(GPU_RG16_SNORM);
    texture_formats.append(GPU_R8_SNORM);
    texture_formats.append(GPU_R16_SNORM);
    texture_formats.append(GPU_SRGB8_A8_DXT1);
    texture_formats.append(GPU_SRGB8_A8_DXT3);
    texture_formats.append(GPU_SRGB8_A8_DXT5);
    texture_formats.append(GPU_RGBA8_DXT1);
    texture_formats.append(GPU_RGBA8_DXT3);
    texture_formats.append(GPU_RGBA8_DXT5);
    texture_formats.append(GPU_SRGB8);
    texture_formats.append(GPU_RGB9_E5);
    texture_formats.append(GPU_DEPTH_COMPONENT32F);
    texture_formats.append(GPU_DEPTH_COMPONENT24);
    texture_formats.append(GPU_DEPTH_COMPONENT16);

    for (eGPUDataFormat host_format : host_formats) {
      std::cout << "/* -------------------------------------------------------------------- */\n";
      std::cout << "/** \\name Roundtrip testing " << to_string(host_format) << "\n";
      std::cout << " * \\{ */\n\n";

      for (eGPUTextureFormat texture_format : texture_formats) {
        if (!validate_data_format(texture_format, host_format)) {
          continue;
        }

        std::cout << "static void test_texture_roundtrip__" << to_string(host_format) << "__"
                  << to_string(texture_format) << "()\n";
        std::cout << "{\n";

        std::cout << "  texture_create_upload_read<" << to_string(texture_format) << ", "
                  << to_string(host_format) << ", " << to_prim_type_string(host_format)
                  << ">();\n";

        std::cout << "}\n";
        std::cout << "GPU_TEST(texture_roundtrip__" << to_string(host_format) << "__"
                  << to_string(texture_format) << ");\n\n";
      }
      std::cout << "/* \\} */\n\n";
    }
  }
#endif
/** \} */

}  // namespace blender::gpu::tests