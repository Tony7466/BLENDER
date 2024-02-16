/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include <array>

#include "testing/testing.h"

#include "BLI_random_access_iterator_mixin.hh"
#include "BLI_vector.hh"

namespace blender::iterator::tests {

struct DoublingIterator : public RandomAccessIteratorMixin<DoublingIterator> {
 private:
  const int *data_;

 public:
  DoublingIterator(const int *data) : data_(data) {}

  int operator*() const
  {
    return *data_ * 2;
  }

  const int *&iter_prop()
  {
    return data_;
  }

  const int *const &iter_prop() const
  {
    return data_;
  }
};

TEST(random_access_iterator_mixin, Test)
{
  std::array<int, 4> my_array = {3, 6, 1, 2};

  DoublingIterator begin{my_array.begin()};
  DoublingIterator end{my_array.end()};

  Vector<int> values;
  for (DoublingIterator it = begin; it != end; ++it) {
    values.append(*it);
  }

  EXPECT_EQ(values.size(), 4);
  EXPECT_EQ(values[0], 6);
  EXPECT_EQ(values[1], 12);
  EXPECT_EQ(values[2], 2);
  EXPECT_EQ(values[3], 4);
}

}  // namespace blender::iterator::tests
