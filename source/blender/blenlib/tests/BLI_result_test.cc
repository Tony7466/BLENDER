/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_result.hh"
#include "BLI_string_ref.hh"

#include "testing/testing.h"

namespace blender::tests {

TEST(result, construct_basic)
{
  Result<std::string, int> r1 = "hello";
  EXPECT_TRUE(r1.is_value());
  EXPECT_EQ("hello", r1.value());

  Result<std::string, int> r2 = Err{4};
  EXPECT_TRUE(r2.is_error());
  EXPECT_EQ(4, r2.error());
}

TEST(result, construct_same_type)
{
  Result<std::string, std::string> r1 = "hello";
  EXPECT_TRUE(r1.is_value());
  EXPECT_EQ("hello", r1.value());

  Result<std::string, std::string> r2 = Err{"bye"};
  EXPECT_TRUE(r2.is_error());
  EXPECT_EQ("bye", r2.error());
}

Result<blender::StringRef, int> foo(bool do_success)
{
  if (do_success) {
    return "success";
  }

  return Err{5};
}

TEST(result, value_or)
{
  EXPECT_EQ("success", foo(true).value_or("hello"));
  EXPECT_EQ("hello", foo(false).value_or("hello"));
}

TEST(result, error_or)
{
  EXPECT_EQ(10, foo(true).error_or(10));
  EXPECT_EQ(5, foo(false).error_or(10));
}

}  // namespace blender::tests
