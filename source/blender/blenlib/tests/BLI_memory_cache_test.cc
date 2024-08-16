/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_memory_cache.hh"

#include "testing/testing.h"

#include "BLI_strict_flags.h" /* Keep last. */

namespace blender::memory_cache::tests {

class GenericIntKey : public GenericKey {
 private:
  int value_ = 0;

 public:
  GenericIntKey(const int value) : value_(value) {}

  uint64_t hash() const override
  {
    return get_default_hash(value_);
  }

  bool equal_to(const GenericKey &other) const override
  {
    if (const auto *other_typed = dynamic_cast<const GenericIntKey *>(&other)) {
      return value_ == other_typed->value_;
    }
    return false;
  }

  std::unique_ptr<GenericKey> to_storable() const
  {
    return std::make_unique<GenericIntKey>(*this);
  }
};

TEST(memory_cache, Simple)
{
  MemoryCache cache(2);

  bool value_1_freed = false;
  cache.add(
      std::make_unique<GenericIntKey>(50),
      [&](MemoryCounter &memory) { memory.add(1); },
      [&]() { value_1_freed = true; });
  EXPECT_FALSE(value_1_freed);

  bool value_2_freed = false;
  cache.add(
      std::make_unique<GenericIntKey>(60),
      [&](MemoryCounter &memory) { memory.add(1); },
      [&]() { value_2_freed = true; });
  EXPECT_FALSE(value_1_freed);
  EXPECT_FALSE(value_2_freed);

  bool value_3_freed = false;
  cache.add(
      std::make_unique<GenericIntKey>(70),
      [&](MemoryCounter &memory) { memory.add(1); },
      [&]() { value_3_freed = true; });
  EXPECT_TRUE(value_1_freed);
  EXPECT_FALSE(value_2_freed);
  EXPECT_FALSE(value_3_freed);
}

}  // namespace blender::memory_cache::tests
