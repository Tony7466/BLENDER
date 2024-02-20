/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "dna_parser.hh"
#include <fstream>

namespace blender::dna::parser::tests {

TEST(parser, parse_file)
{
  const char *filepath = R"x(D:\blender-git\blender\source\blender\makesdna\DNA_vec_types.h)x";
  std::ifstream file;
  file.open(filepath);
  std::string text;
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      text += "\n" + line;
    }
  }

  using namespace ast;
  Vector<CppType> parsed;
  bool parse_result = parse_include(text, filepath, parsed);
  Vector<CppType> expected{
      {Struct{"vec2s", {{Variable{false, "short", {{{}, "x", {}}, {{}, "y", {}}}}}}}},
      {Struct{"vec2f", {{Variable{false, "float", {{{}, "x", {}}, {{}, "y", {}}}}}}}},
      {Struct{"vec2i", {{Variable{false, "int", {{{}, "x", {}}, {{}, "y", {}}}}}}}},
      {Struct{"vec3i", {{Variable{false, "int", {{{}, "x", {}}, {{}, "y", {}}, {{}, "z", {}}}}}}}},
      {Struct{"vec3f",
              {{Variable{false, "float", {{{}, "x", {}}, {{}, "y", {}}, {{}, "z", {}}}}}}}},
      {Struct{
          "vec4f",
          {{Variable{
              false, "float", {{{}, "x", {}}, {{}, "y", {}}, {{}, "z", {}}, {{}, "w", {}}}}}}}},
      {Struct{"mat4x4f", {{Variable{false, "float", {{{}, "value", {{4, 4}}}}}}}}},
      {Struct{"rcti",
              {{Variable{false, "int", {{{}, "xmin", {}}, {{}, "xmax", {}}}}},
               {Variable{false, "int", {{{}, "ymin", {}}, {{}, "ymax", {}}}}}}}},
      {Struct{"rctf",
              {{Variable{false, "float", {{{}, "xmin", {}}, {{}, "xmax", {}}}}},
               {Variable{false, "float", {{{}, "ymin", {}}, {{}, "ymax", {}}}}}}}},
      {Struct{"DualQuat",
              {
                  {Variable{false, "float", {{{}, "quat", {{4}}}}}},
                  {Variable{false, "float", {{{}, "trans", {{4}}}}}},
                  {Variable{false, "float", {{{}, "scale", {{4, 4}}}}}},
                  {Variable{false, "float", {{{}, "scale_weight", {}}}}},
              }}},
  };
  ASSERT_TRUE(parse_result);
  ASSERT_EQ(expected, parsed);
}

}  // namespace blender::dna::parser::tests
