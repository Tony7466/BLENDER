/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <string>
#include <vector>

namespace blender::dna {

struct DefineConstValue {
  std::string identifier;
  int value;
  bool operator==(const DefineConstValue &other) const
  {
    return this->identifier == other.identifier && this->value == other.value;
  }
};
/**
 * Finds all const int defines directives in `str` and puts all in `defines`.
 * Only support expressions like:
 * `# define FILEPATH_MAX 1024`
 * `#define FILEPATH_MAX 1024`
 * This don't support math expressions.
 */
void gather_defines(std::string str, std::vector<DefineConstValue> &defines);
/**
 * For the `str` array member, parses `[123][FILEPATH_MAX]...` array size expresion,
 * return the number of array elements `123*FILEPATH_MAX*...`.
 */
int array_size(std::string str, std::vector<DefineConstValue> &defines);
}  // namespace blender::dna
