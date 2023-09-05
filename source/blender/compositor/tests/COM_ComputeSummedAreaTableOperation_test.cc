/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "COM_SummedAreaTableOperation.h"

namespace blender::compositor::tests {

TEST(SummedTableArea, FullFrame_5x2)
{
  SummedAreaTableOperation sat;

  sat.set_execution_model(eExecutionModel::FullFrame);
  sat.set_mode(SummedAreaTableOperation::eMode::Identity);

  const rcti area{0, 5, 0, 2};
  MemoryBuffer output(DataType::Color, area);

  const float val[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  std::shared_ptr<MemoryBuffer> input = std::make_shared<MemoryBuffer>(DataType::Color, area);
  input->fill(area, val);

  /* sat.render() doesn't work because of a dependency of Operations on nodetree,
   * so call sat.update_memory_buffer() directly instead. */

  sat.update_memory_buffer(&output, area, Span<MemoryBuffer *>{input.get()});

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
  std::shared_ptr<MemoryBuffer> input = std::make_shared<MemoryBuffer>(DataType::Color, area);
  input->fill(area, val);
  Span<MemoryBuffer *> inputs{input.get()};

  sat.update_memory_buffer(&output, area, Span<MemoryBuffer *>{input.get()});

  /* First row. */
  EXPECT_FLOAT_EQ(output.get_elem(0, 0)[0], 4);
  EXPECT_FLOAT_EQ(output.get_elem(1, 0)[1], 8);
  EXPECT_FLOAT_EQ(output.get_elem(2, 0)[2], 6.75);

  /* Second row. */
  EXPECT_FLOAT_EQ(output.get_elem(0, 1)[3], 0.02);
  EXPECT_FLOAT_EQ(output.get_elem(1, 1)[0], 16);
  EXPECT_FLOAT_EQ(output.get_elem(2, 1)[1], 24);
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
    std::shared_ptr<MemoryBuffer> input = std::make_shared<MemoryBuffer>(DataType::Color, area_);
    input->fill(area_, val);

    operation_->update_memory_buffer(sat_.get(), area_, Span<MemoryBuffer *>{input.get()});
  }

  std::shared_ptr<SummedAreaTableOperation> operation_;
  std::shared_ptr<MemoryBuffer> sat_;
  rcti area_;
};

TEST_F(SummedTableAreaSumTest, FullyInside)
{
  rcti area;
  area.xmin = 1; area.xmax = 3;
  area.ymin = 1; area.ymax = 3;
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 9);
}

TEST_F(SummedTableAreaSumTest, LeftEdge)
{
  rcti area;
  area.xmin = 0; area.xmax = 2;
  area.ymin = 0; area.ymax = 2;
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 9);
}

TEST_F(SummedTableAreaSumTest, RightEdge)
{
  rcti area;
  area.xmin = area_.xmax - 2;
  area.xmax = area_.xmax;
  area.ymin = 0;
  area.ymax = 2;
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 6);
}

TEST_F(SummedTableAreaSumTest, LowerRightCorner)
{
  rcti area;
  area.xmin = area_.xmax - 1;
  area.xmax = area_.xmax;
  area.ymin = area_.ymax - 1;
  area.ymax = area_.ymax;
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 1);
}

TEST_F(SummedTableAreaSumTest, TopLine)
{
  rcti area;
  area.xmin = 0; area.xmax = 1;
  area.ymin = 0; area.ymax = 0;
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 2);
}

TEST_F(SummedTableAreaSumTest, ButtomLine)
{
  rcti area;
  area.xmin = 0; area.xmax = 4;
  area.ymin = 3; area.ymax = 3;
  float4 sum = summed_area_table_sum(sat_.get(), area);
  ASSERT_EQ(sum[0], 5);
}

}  // namespace blender::compositor::tests
