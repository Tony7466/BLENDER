/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "dna_parser.hh"
#include <fstream>

namespace blender::dna::parser::tests {

TEST(parser, parse_file)
{
#if 0
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
#endif
  std::string_view text = R"xxx(
/*----------------------------------------------------------------*/
/** #pragma once and includes */

#pragma once

#include "...."
#include "DNA_ID.h"
#include "DNA_armature_types.h"
#include "DNA_listBase.h"
#include "DNA_session_uid_types.h"
#include "DNA_userdef_types.h"
#include "DNA_vec_types.h"

// Forward declarations
struct Collection;
struct GHash;
struct Object, SpaceLink;

#if 0
typedef enum class UnusedEnum: uint8_t {
  /* vert is selected */
  UNUSED_1 = (1 + 0*FILE_MAX),
  UNUSED_2 = (1 << 1),
} UnusedEnum;
ENUM_OPERATORS(UnusedEnum, UNUSED_2);
#endif


/* Const int define */
#define FILE_MAX 1024

/* define non const int*/
#define DNA_DEFINE_CXX_METHODS() ....

/** bMotionPathVert, taken from DNA_action_Types.h
 * modified with a child struct. */
typedef struct bMotionPathVert {
  DNA_DEFINE_CXX_METHODS(bMotionPathVert);
  /* Child struct */
  struct bMotionPathVertItem {
    DNA_DEFINE_CXX_METHODS(bMotionPathVert);
    float co[3], pre[235];
    struct Link *next, *pre;
    char path[FILE_MAX];
    bool (*poll)(bContext *, ARegion *);
    float (*data_src)[256];
  };
  /** Coordinates of point in 3D-space. */
  float co[3];
  /** Quick settings. */
  int flag;
} bMotionPathVert;

/** eMotionPathVert_Flag, taken from DNA_action_Types.h
 * modified with a fixed size. */
typedef enum class eMotionPathVert_Flag : uint8_t {
  /* vert is selected */
  VERT_SEL = (1 << 0),
  VERT_KEY = (1 << 1),
} eMotionPathVert_Flag;
ENUM_OPERATORS(eMotionPathVert_Flag, VERT_KEY);
)xxx";
  using namespace ast;

  blender::dna::lex::TokenIterator iterator;
  iterator.process_text("", text);

  blender::Vector<blender::dna::parser::ast::CppType> cpp_defines;
  const bool parse_result = blender::dna::parser::parse_include("", text, iterator, cpp_defines);
  Vector<CppType> expected{
      {DefineInt{"FILE_MAX", 1024}},
      {
          Struct{"bMotionPathVert",
                 {Struct{"bMotionPathVertItem",
                         {
                             {Variable{false, "float", {{{}, "co", {{3}}}, {{}, "pre", {{235}}}}}},
                             {Variable{false, "Link", {{"*", "next", {}}, {"*", "pre", {}}}}},
                             {Variable{false, "char", {{{}, "path", {{"FILE_MAX"}}}}}},
                             {FunctionPtr{false, "bool", "poll"}},
                             {PointerToArray{"float", "data_src", 256}},
                         }},
                  {Variable{false, "float", {{{}, "co", {{3}}}}}},
                  {Variable{false, "int", {{{}, "flag", {}}}}}}},
      },
      Enum{"eMotionPathVert_Flag", "uint8_t"}};
  ASSERT_TRUE(parse_result);
  ASSERT_EQ(expected, cpp_defines);
}

}  // namespace blender::dna::parser::tests
