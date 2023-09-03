/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "COM_SummedAreaTableOperation.h"

namespace blender::compositor::tests {

static void print_area(MemoryBuffer *input, const rcti &area, std::string description = "")
{
  std::cout << description << ":" << std::endl;
  for (BuffersIterator<float> it = input->iterate_with({}, area); !it.is_end(); ++it) {
    std::setprecision(2);
    std::cout << it.out[0] << "\t";
    if (it.x == area.xmax - 1) {
      std::cout << std::endl;
    }
  }
  std::cout << std::endl;
}

TEST(SummedTableArea, FullFrame_5x2)
{
  SummedAreaTableOperation sat;

  sat.set_execution_model(eExecutionModel::FullFrame);
  sat.set_mode(SummedAreaTableOperation::eMode::Identity);

  const rcti area{0, 5, 0, 2};
  MemoryBuffer output(DataType::Color, area);

  const float val[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  MemoryBuffer input(DataType::Color, area);
  input.fill(area, val);
  Span<MemoryBuffer *> inputs{&input};

  print_area(&input, area, "input");

  /* sat.render() doesn't work because of a dependency of Operations on nodetree,
   * so call sat.update_memory_buffer() directly instead. */

  sat.update_memory_buffer(&output, area, inputs);

  /* First row. */
  EXPECT_FLOAT_EQ(output.get_elem(0, 0)[0], 1);
  EXPECT_FLOAT_EQ(output.get_elem(1, 0)[1], 2);
  EXPECT_FLOAT_EQ(output.get_elem(2, 0)[2], 3);
  EXPECT_FLOAT_EQ(output.get_elem(3, 0)[3], 4);
  EXPECT_FLOAT_EQ(output.get_elem(4, 0)[0], 5);

  /* Second row. */
  EXPECT_FLOAT_EQ(output.get_elem(0, 1)[3], 2);
  EXPECT_FLOAT_EQ(output.get_elem(1, 1)[0], 4);
  EXPECT_FLOAT_EQ(output.get_elem(2, 1)[1], 6);
  EXPECT_FLOAT_EQ(output.get_elem(3, 1)[2], 8);
  EXPECT_FLOAT_EQ(output.get_elem(4, 1)[0], 10);
}

TEST(SummedTableArea, FullFrame_3x2_squared)
{
  SummedAreaTableOperation sat;

  sat.set_execution_model(eExecutionModel::FullFrame);
  sat.set_mode(SummedAreaTableOperation::eMode::Squared);
  const rcti area{0, 3, 0, 2};
  MemoryBuffer output(DataType::Color, area);

  const float val[4] = {2.0f, 2.0f, 1.5f, 0.1f};
  MemoryBuffer input(DataType::Color, area);
  input.fill(area, val);
  Span<MemoryBuffer *> inputs{&input};

  print_area(&input, area, "input");

  sat.update_memory_buffer(&output, area, inputs);

  print_area(&output, area, "output");

  /* First row. */
  EXPECT_FLOAT_EQ(output.get_elem(0, 0)[0], 4);
  EXPECT_FLOAT_EQ(output.get_elem(1, 0)[1], 8);
  EXPECT_FLOAT_EQ(output.get_elem(2, 0)[2], 6.75);

  /* Second row. */
  EXPECT_FLOAT_EQ(output.get_elem(0, 1)[3], 0.02);
  EXPECT_FLOAT_EQ(output.get_elem(1, 1)[0], 16);
  EXPECT_FLOAT_EQ(output.get_elem(2, 1)[1], 24);
}

/* Can't test tiled operation in an isolated way. */
TEST(DISABLED_SummedTableArea, Fullfram_vs_tiled)
{
  SummedAreaTableOperation sat_tiled, sat_fullframe;

  sat_tiled.set_execution_model(eExecutionModel::Tiled);
  sat_fullframe.set_execution_model(eExecutionModel::FullFrame);
  rcti area{0, 2, 0, 3};

  MemoryBuffer input(DataType::Color, area);
  auto output_fullframe = std::make_shared<MemoryBuffer>(DataType::Color, area);
  auto output_tiled = std::make_shared<MemoryBuffer>(DataType::Color, area);

  /* Fill input with random numbers. */
  for (BuffersIterator<float> it = input.iterate_with({}, area); !it.is_end(); ++it) {
    it.out[0] = (float)rand();
    it.out[1] = (float)rand();
    it.out[2] = (float)rand();
    it.out[3] = (float)rand();
  }

  Span<MemoryBuffer *> inputs{&input};

  print_area(&input, area, "input");

  output_tiled.reset(sat_tiled.create_memory_buffer(&area));
  sat_fullframe.update_memory_buffer(output_fullframe.get(), area, inputs);

  print_area(output_tiled.get(), area, "output_tiled");
  print_area(output_fullframe.get(), area, "output_fullframe");

  /* Expect full-frame and tiled implementation to give same results. */
  bool loop_run = false;
  for (BuffersIterator<float> it = output_tiled->iterate_with({}, area); !it.is_end(); ++it) {
    loop_run = true;
    float color_tiled[4], color_fullframe[4];

    output_tiled->read_elem(it.x, it.y, color_tiled);
    output_fullframe->read_elem(it.x, it.y, color_fullframe);

    EXPECT_V4_NEAR(color_tiled, color_fullframe, FLT_EPSILON);
  }
  ASSERT_TRUE(loop_run);
}

class SummedTableAreaSumTest : public ::testing::Test {
 public:
  SummedTableAreaSumTest()
  {
    operation_ = std::make_shared<SummedAreaTableOperation>();
  }

 protected:
  void SetUp() override
  {
    operation_->set_execution_model(eExecutionModel::FullFrame);
    operation_->set_mode(SummedAreaTableOperation::eMode::Squared);

    area_ = rcti{0, 5, 0, 4};
    sat_ = std::make_shared<MemoryBuffer>(DataType::Color, area_);

    const float val[4] = {1.0f, 2.0f, 1.5f, 0.1f};
    MemoryBuffer input(DataType::Color, area_);
    input.fill(area_, val);
    Span<MemoryBuffer *> inputs{&input};

    operation_->update_memory_buffer(sat_.get(), area_, inputs);
  }

  std::shared_ptr<SummedAreaTableOperation> operation_;
  std::shared_ptr<MemoryBuffer> sat_;
  rcti area_;
};

TEST_F(SummedTableAreaSumTest, FullyInside)
{
  rcti area{.xmin = 1, .xmax = 3, .ymin = 1, .ymax = 3};
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 9);
}

TEST_F(SummedTableAreaSumTest, LeftEdge)
{
  rcti area{.xmin = 0, .xmax = 2, .ymin = 0, .ymax = 2};
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 9);
}

TEST_F(SummedTableAreaSumTest, RightEdge)
{
  rcti area{.xmin = area_.xmax - 2, .xmax = area_.xmax, .ymin = 0, .ymax = 2};
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 6);
}

TEST_F(SummedTableAreaSumTest, LowerRightCorner)
{
  rcti area{
      .xmin = area_.xmax - 1, .xmax = area_.xmax, .ymin = area_.ymax - 1, .ymax = area_.ymax};
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 1);
}

TEST_F(SummedTableAreaSumTest, TopLine)
{
  rcti area{.xmin = 0, .xmax = 1, .ymin = 0, .ymax = 0};
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 2);
}

TEST_F(SummedTableAreaSumTest, ButtomLine)
{
  rcti area{.xmin = 0, .xmax = 4, .ymin = 3, .ymax = 3};
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 5);
}

}  // namespace blender::compositor::tests
