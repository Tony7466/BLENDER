/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_inline_or_shared_polymorphic_value.hh"
#include "testing/testing.h"

#include "BLI_strict_flags.h" /* Keep last. */

namespace blender::tests {

class BaseIntGetter {
 public:
  virtual int get() const = 0;
};

class Get1 : public BaseIntGetter {
  int get() const override
  {
    return 1;
  }
};

class Get2 : public BaseIntGetter {
  int get() const override
  {
    return 2;
  }
};

class GetDynamic : public BaseIntGetter {
 public:
  int value = 0;

  int get() const override
  {
    return this->value;
  }
};

using IntGetter = InlineOrSharedPolymorphicValue<BaseIntGetter>;

TEST(polymorphic_value, Test)
{
  {
    IntGetter a;
    a.emplace<Get1>();
    EXPECT_EQ(a->get(), 1);
  }
  {
    GetDynamic getter;
    getter.value = 5;
    IntGetter a{&getter};
    EXPECT_EQ(a->get(), 5);
    getter.value = 10;
    EXPECT_EQ(a->get(), 10);
    IntGetter b = a;
    EXPECT_EQ(b->get(), 10);
    getter.value = 20;
    EXPECT_EQ(a->get(), 20);
    EXPECT_EQ(b->get(), 20);
  }
}

}  // namespace blender::tests
