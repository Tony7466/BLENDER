/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "dna_parser.hh"

namespace blender::dna::parser::tests {

TEST(parser, parse_file)
{
  using namespace ast;
#if 0
  std::string_view anim_types_path = "dna_anim_types_src.hh";
  std::string_view anim_types_parsed = "dna_anim_types_parsed.hh";
  const std::optional<blender::dna::parser::CppFile> cpp_file = blender::dna::parser::parse_file(
      anim_types_path);
  ASSERT_TRUE(cpp_file.has_value());
  ASSERT_TRUE(to_string(cpp_file.value()) == read_file(anim_types_parsed));
#endif
}

}  // namespace blender::dna::parser::tests
