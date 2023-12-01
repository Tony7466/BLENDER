/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_rect.hh"

#include "testing/testing.h"

namespace blender::tests {

TEST(rect, DefaultConstructor)
{
  rect2f rect;
  EXPECT_TRUE(rect.is_valid());
  EXPECT_TRUE(rect.is_empty());
}

TEST(rect, Sanitize)
{
  rect2f rect({1, 1}, {-1, -1});
  EXPECT_FALSE(rect.is_valid());
  rect.sanitize();
  EXPECT_TRUE(rect.is_valid());
  EXPECT_EQ(rect.min.x, -1);
  EXPECT_EQ(rect.min.y, -1);
  EXPECT_EQ(rect.max.x, 1);
  EXPECT_EQ(rect.max.y, 1);
}

TEST(rect, Size)
{
  rect2f rect({-1, -1}, {1, 1});
  EXPECT_EQ(rect.size().x, 2);
  EXPECT_EQ(rect.size().y, 2);
}

TEST(rect, Intersects)
{
  rect2f rect({-1, -1}, {1, 1});
  EXPECT_FALSE(rect.intersects({-2, -2}));
  EXPECT_FALSE(rect.intersects({-2, 0}));
  EXPECT_FALSE(rect.intersects({-2, 2}));
  EXPECT_FALSE(rect.intersects({0, -2}));
  EXPECT_TRUE(rect.intersects({0, 0}));
  EXPECT_FALSE(rect.intersects({0, 2}));
  EXPECT_FALSE(rect.intersects({2, -2}));
  EXPECT_FALSE(rect.intersects({2, 0}));
  EXPECT_FALSE(rect.intersects({2, 2}));
}

}  // namespace blender::tests