/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "dna_utils.hh"

namespace blender::tests {

static std::vector<blender::dna::DefineConstValue> defines;

TEST(util, gather_defines)
{
  const char *str =
      "#define FILEPATH_MAX 1024\n"
      "# define DIR_MAX 512\n"
      "#define PROP_NAME_MAX 1*024\n"
      "#           define PROP_DESCRIPTION_MAX 256\n"
      "#           define PROP_5_MAX 5\n"
      "#           define PROP_TOOLTIP\n"
      "#define PROP_DEFAULT 0\n";
  dna::gather_defines(str, defines);
  std::vector<dna::DefineConstValue> expected_defines{
      {"FILEPATH_MAX", 1024},
      {"DIR_MAX", 512},
      {"PROP_DESCRIPTION_MAX", 256},
      {"PROP_5_MAX", 5},
      {"PROP_DEFAULT", 0},
  };
  EXPECT_EQ(expected_defines, defines);
}

TEST(util, array_size)
{
  EXPECT_EQ(dna::array_size("filepath[FILEPATH_MAX]", defines), 1024);
  EXPECT_EQ(dna::array_size("filepaths[FILEPATH_MAX][2]", defines), 1024 * 2);
  EXPECT_EQ(dna::array_size("ICON_PATH[FILEPATH", defines), 0);
  EXPECT_EQ(dna::array_size("filepath[FILEPAX]", defines), 0);
  EXPECT_EQ(dna::array_size("dim[256][256]", defines), 256 * 256);
  EXPECT_EQ(dna::array_size("extensions[]", defines), 0);
  EXPECT_EQ(dna::array_size("extensions[12]", defines), 12);
}
}  // namespace blender::tests
