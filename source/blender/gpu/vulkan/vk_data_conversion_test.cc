#include "testing/testing.h"

#include "vk_data_conversion.hh"

namespace blender::gpu::tests {
TEST(VulkanDataConversion, ConvertF32F16)
{
  uint32_t f32_2 = 0b01000000000000000000000000000000;
  uint32_t f16_2_expected = 0b0100000000000000;
  uint32_t f16_2 = convert_float_formats<Format16F, Format32F>(f32_2);
  EXPECT_EQ(f16_2, f16_2_expected);

  uint32_t f32_3 = 0b01000000010000000000000000000000;
  uint32_t f16_3_expected = 0b0100001000000000;
  uint32_t f16_3 = convert_float_formats<Format16F, Format32F>(f32_2);
  EXPECT_EQ(f16_3, f16_3_expected);

  uint32_t f32_4 = 0b01000000100000000000000000000000;
  uint32_t f16_4_expected = 0b0100010000000000;
  uint32_t f16_4 = convert_float_formats<Format16F, Format32F>(f32_2);
  EXPECT_EQ(f16_4, f16_4_expected);
}
}  // namespace blender::gpu::tests