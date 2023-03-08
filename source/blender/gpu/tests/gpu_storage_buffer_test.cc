/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "GPU_storage_buffer.h"

#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "gpu_testing.hh"

namespace blender::gpu::tests {

constexpr size_t SIZE = 128;
constexpr size_t SIZE_IN_BYTES = SIZE * sizeof(int);

static Vector<int32_t> test_data()
{
  Vector<int32_t> data;
  for (int i : IndexRange(SIZE)) {
    data.append(i);
  }
  return data;
}

static void test_storage_buffer_create_update_read()
{
  GPUStorageBuf *ssbo = GPU_storagebuf_create_ex(
      SIZE_IN_BYTES, nullptr, GPU_USAGE_STATIC, __func__);
  EXPECT_NE(ssbo, nullptr);

  /* Upload some dummy data. */
  const Vector<int32_t> data = test_data();
  GPU_storagebuf_update(ssbo, data.data());

  /* Read back data from SSBO. */
  Vector<int32_t> read_data;
  read_data.resize(SIZE, 0);
  GPU_storagebuf_read(ssbo, read_data.data());

  /* Check if data is the same. */
  for (int i : IndexRange(SIZE)) {
    EXPECT_EQ(data[i], read_data[i]);
  }

  GPU_storagebuf_free(ssbo);
}

GPU_TEST(storage_buffer_create_update_read);

static void test_storage_buffer_clear_zero()
{
  GPUStorageBuf *ssbo = GPU_storagebuf_create_ex(
      SIZE_IN_BYTES, nullptr, GPU_USAGE_STATIC, __func__);
  EXPECT_NE(ssbo, nullptr);

  /* Upload some dummy data. */
  const Vector<int32_t> data = test_data();
  GPU_storagebuf_update(ssbo, data.data());
  GPU_storagebuf_clear_to_zero(ssbo);

  /* Read back data from SSBO. */
  Vector<int32_t> read_data;
  read_data.resize(SIZE, 0);
  GPU_storagebuf_read(ssbo, read_data.data());

  /* Check if data is the same. */
  for (int i : IndexRange(SIZE)) {
    EXPECT_EQ(0, read_data[i]);
  }

  GPU_storagebuf_free(ssbo);
}

GPU_TEST(storage_buffer_clear_zero);

template<int DataLen> static void storage_buffer_clear_int_uniform()
{
  GPUStorageBuf *ssbo = GPU_storagebuf_create_ex(
      SIZE_IN_BYTES, nullptr, GPU_USAGE_STATIC, __func__);
  EXPECT_NE(ssbo, nullptr);

  /* Read back data from SSBO. */
  int4 clear_data = {-1, -1, -1, -1};
  GPU_storagebuf_clear_int(ssbo, clear_data, DataLen);

  /* Check if data is the same. */
  Vector<int32_t> read_data;
  read_data.resize(SIZE, 0);
  GPU_storagebuf_read(ssbo, read_data.data());
  for (int i : IndexRange(SIZE)) {
    EXPECT_EQ(clear_data[i % DataLen], read_data[i]);
  }

  GPU_storagebuf_free(ssbo);
}

static void test_storage_buffer_clear_int_uniform()
{
  storage_buffer_clear_int_uniform<1>();
  storage_buffer_clear_int_uniform<2>();
  storage_buffer_clear_int_uniform<3>();
  storage_buffer_clear_int_uniform<4>();
}
GPU_TEST(storage_buffer_clear_int_uniform);

static void test_storage_buffer_clear_non_uniform()
{
  GPUStorageBuf *ssbo = GPU_storagebuf_create_ex(
      SIZE_IN_BYTES, nullptr, GPU_USAGE_STATIC, __func__);
  EXPECT_NE(ssbo, nullptr);

  /* Read back data from SSBO. */
  int4 clear_data = {-1, -2, -3, -4};
  GPU_storagebuf_clear_int(ssbo, clear_data, 4);

  /* Check if data is the same. */
  Vector<int32_t> read_data;
  read_data.resize(SIZE, 0);
  GPU_storagebuf_read(ssbo, read_data.data());
  for (int i : IndexRange(SIZE)) {
    EXPECT_EQ(clear_data[i % 4], read_data[i]);
  }

  GPU_storagebuf_free(ssbo);
}
GPU_TEST(storage_buffer_clear_non_uniform);

}  // namespace blender::gpu::tests
