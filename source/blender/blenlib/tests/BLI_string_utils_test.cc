/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_string_utils.hh"

#include <string>

#include "testing/testing.h"

#include "BLI_vector.hh"

namespace blender {

TEST(BLI_string_utils, BLI_uniquename_cb)
{
  const Vector<std::string> current_names{"Foo", "Bar", "Baz.001"};

  const auto unique_check = [&](const blender::StringRef name) -> bool {
    return current_names.contains(name);
  };

  EXPECT_EQ(BLI_uniquename_cb(unique_check, '.', ""), "");
  EXPECT_EQ(BLI_uniquename_cb(unique_check, '.', "Baz"), "Baz");
  EXPECT_EQ(BLI_uniquename_cb(unique_check, '.', "Foo"), "Foo.001");
  EXPECT_EQ(BLI_uniquename_cb(unique_check, '.', "Baz.001"), "Baz.002");
}

}  // namespace blender