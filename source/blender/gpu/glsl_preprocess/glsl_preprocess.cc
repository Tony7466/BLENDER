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
#include <sstream>
#include <string>
#include <vector>

struct SharedVar {
  std::string type;
  std::string name;
  std::string array;
};

void report_error(std::string filename,
                  std::string src_line,
                  size_t err_line,
                  size_t err_char,
                  std::string err_msg)
{
  std::cerr << filename << ':' << std::to_string(err_line) << ':' << std::to_string(err_char);
  std::cerr << ": error: " << err_msg << std::endl;
  std::cerr << src_line << std::endl;
  std::cerr << std::string(err_char, ' ') << '^' << std::endl;
}

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

  std::vector<SharedVar> shared_vars;

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
        /* Shared variable parsing. */
        std::regex shared_variable("shared\\s+(\\w+)\\s+(\\w+)([^;]*);");
        std::smatch match;
        if (std::regex_search(line, match, shared_variable)) {
          shared_vars.push_back({match[1].str(), match[2].str(), match[3].str()});
        }
      }
      {
        /* Argument decorator macro injection. */
        std::regex inout("(out|inout|in|shared)\\s+(\\w+)\\s+(\\w+)");
        line = std::regex_replace(line, inout, "$1 $2 _$1_sta $3 _$1_end");
      }
      {
        /* Invalid matrix constructors (linting). */
        std::regex matrix_cast(" (mat(\\d|\\dx\\d)|float\\dx\\d)\\([^,\\s\\d]+\\)");
        std::smatch match;
        if (std::regex_search(line, match, matrix_cast)) {
          /* This only catches some invalid usage. For the rest, the CI will catch them. */
          report_error(input_file_name,
                       line,
                       line_index,
                       match.position(),
                       "Matrix cast is not cross API compatible. "
                       "Use to_floatNxM to reshape the matrix or use other constructors instead.");
          error = 1;
        }
      }
      {
        /* Invalid array constructor (linting). */
        std::regex matrix_cast(" (i?u?vec\\d?|float|u?int)\\s*\\[[\\s\\*\\w]*\\]\\s*\\(");
        std::smatch match;
        if (std::regex_search(line, match, matrix_cast)) {
          /* This only catches some invalid usage. For the rest, the CI will catch them. */
          report_error(input_file_name,
                       line,
                       line_index,
                       match.position(),
                       "Array constructor is not cross API compatible. "
                       "Use type_array instead of type[].");
          error = 1;
        }
      }
    }

    output_file << line << "\n";
  }

  if (!shared_vars.empty()) {
    /**
     * For Metal shaders to compile, shared (threadgroup) variable cannot be declared globally.
     * They must reside within a function scope. Hence, we need to extract these uses and generate
     * shared memory blocks within the entry point function. These shared memory blocks can
     * then be passed as references to the remaining shader via the class function scope.
     *
     * The shared variable definitions from the source file are replaced with references to
     * threadgroup memory blocks (using _shared_sta and _shared_end macros), but kept in-line in
     * case external macros are used to declare the dimensions.
     *
     * Each part of the codegen is stored inside macros so that we don't have to do string
     * replacement at runtime.
     */
    /* Arguments of the wrapper class constructor. */
    output_file << "#undef MSL_SHARED_VARS_ARGS\n";
    /* References assignment inside wrapper class constructor. */
    output_file << "#undef MSL_SHARED_VARS_ASSIGN\n";
    /* Declaration of threadgroup variables in entry point function. */
    output_file << "#undef MSL_SHARED_VARS_DECLARE\n";
    /* Arguments for wrapper class constructor call. */
    output_file << "#undef MSL_SHARED_VARS_PASS\n";

    /**
     * Example replacement:
     *
     * `
     * // Source
     * shared float bar[10];                                    // Source declaration.
     * shared float foo;                                        // Source declaration.
     * // Rest of the source ...
     * // End of Source
     *
     * // Backend Output
     * class Wrapper {                                          // Added at runtime by backend.
     *
     * threadgroup float (&foo);                                // Replaced by regex and macros.
     * threadgroup float (&bar)[10];                            // Replaced by regex and macros.
     * // Rest of the source ...
     *
     * Wrapper (                                                // Added at runtime by backend.
     * threadgroup float (&_foo), threadgroup float (&_bar)[10] // MSL_SHARED_VARS_ARGS
     * )                                                        // Added at runtime by backend.
     * : foo(_foo), bar(_bar)                                   // MSL_SHARED_VARS_ASSIGN
     * {}                                                       // Added at runtime by backend.
     *
     * }; // End of Wrapper                                     // Added at runtime by backend.
     *
     * kernel entry_point() {                                   // Added at runtime by backend.
     *
     * threadgroup float foo;                                   // MSL_SHARED_VARS_DECLARE
     * threadgroup float bar[10]                                // MSL_SHARED_VARS_DECLARE
     *
     * Wrapper wrapper                                          // Added at runtime by backend.
     * (foo, bar)                                               // MSL_SHARED_VARS_PASS
     * ;                                                        // Added at runtime by backend.
     *
     * }                                                        // Added at runtime by backend.
     * // End of Backend Output
     * `
     */
    std::stringstream args, assign, declare, pass;

    bool first = true;
    for (SharedVar &var : shared_vars) {
      char sep = first ? ' ' : ',';
      /*  */
      args << sep << "threadgroup " << var.type << "(&_" << var.name << ")" << var.array;
      assign << (first ? ':' : ',') << var.name << "(_" << var.name << ")";
      declare << "threadgroup " << var.type << ' ' << var.name << var.array << ";";
      pass << sep << var.name;
      first = false;
    }

    output_file << "#define MSL_SHARED_VARS_ARGS " << args.str() << "\n";
    output_file << "#define MSL_SHARED_VARS_ASSIGN " << assign.str() << "\n";
    output_file << "#define MSL_SHARED_VARS_DECLARE " << declare.str() << "\n";
    output_file << "#define MSL_SHARED_VARS_PASS (" << pass.str() << ")\n";
  }

  input_file.close();
  output_file.close();

  return error;
}
