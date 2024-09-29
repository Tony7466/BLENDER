/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup glsl_preprocess
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
  if (argc != 3) {
    std::cerr << "Usage: glsl_preprocess <data_file_from> <data_file_to>" << std::endl;
    exit(1);
  }

  const char *input_file_name = argv[1];
  const char *output_file_name = argv[2];

  /* Open the input file for reading */
  std::ifstream input_file(input_file_name);
  if (!input_file) {
    std::cerr << "Error: Could not open input file " << input_file_name << std::endl;
    exit(1);
  }

  /* Open the output file for writing */
  std::ofstream output_file(output_file_name);
  if (!output_file) {
    std::cerr << "Error: Could not open output file " << output_file_name << std::endl;
    input_file.close();
    exit(1);
  }

  bool first_comment = true;
  bool inside_comment = false;

  int error = 0;

  std::string line;
  size_t line_index = 0;
  while (std::getline(input_file, line)) {
    line_index++;

    /* Remove licence headers (first comment). */
    if (line.rfind("/*", 0) == 0 && first_comment) {
      first_comment = false;
      inside_comment = true;
    }

    const bool skip_line = inside_comment;

    if (inside_comment && (line.find("*/") != std::string::npos)) {
      inside_comment = false;
    }

    if (skip_line) {
      line = "";
    }
    else if (line.rfind("#include ", 0) == 0 || line.rfind("#pragma once", 0) == 0) {
      line[0] = line[1] = '/';
    }
    else {
      {
        /* Argument decorator macro injection. */
        std::regex inout("(out|inout|in)\\s+(\\w+)\\s+(\\w+)");
        line = std::regex_replace(line, inout, "$1 $2 _$1_sta $3 _$1_end");
      }
      {
        /* Invalid matrix constructors (linting). */
        std::regex matrix_cast(" (mat(\\d|\\dx\\d)|float\\dx\\d)\\([^,\\s\\d]+\\)");
        std::smatch match;
        if (std::regex_search(line, match, matrix_cast)) {
          /* This only catches some invalid usage. For the rest, the CI will catch them. */
          std::cerr << input_file_name << ':' << std::to_string(line_index) << ':'
                    << std::to_string(line_index) << ':';
          std::cerr << " error: Matrix cast is not cross API compatible. Use to_floatNxM to "
                       "reshape the matrix or use other constructors instead.\n"
                    << std::endl;
          std::cerr << line << std::endl;
          std::cerr << std::string(match.position(), ' ') << '^' << std::endl;
          error = 1;
        }
      }
    }

    output_file << line << "\n";
  }

  input_file.close();
  output_file.close();

  return error;
}
