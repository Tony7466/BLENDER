/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup glsl_preprocess
 */

#pragma once

#include <algorithm>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace blender::gpu::shader {

namespace detail {
// FNV-1a 32bit hashing algorithm.
constexpr std::uint32_t fnv1a_32(char const *s, std::size_t count)
{
  return ((count ? fnv1a_32(s, count - 1) : 2166136261u) ^ s[count]) * 16777619u;
}
}  // namespace detail

constexpr std::uint32_t operator"" _hash(char const *s, std::size_t count)
{
  return detail::fnv1a_32(s, count);
}

/**
 * Shader source preprocessor that allow to mutate GLSL into cross API source that can be
 * interpreted by the different GPU backends. Some syntax are mutated or reported as incompatible.
 *
 * Implementation speed is not a huge concern as we only apply this at compile time or on python
 * shaders source.
 */
class Preprocessor {
  using uint = unsigned int;

  struct SharedVar {
    std::string type;
    std::string name;
    std::string array;
  };
  std::vector<SharedVar> shared_vars_;

  std::unordered_set<std::string> static_strings_;

  std::stringstream gpu_functions_;

 public:
  /* Takes a whole source file and output processed source. */
  template<typename ReportErrorF>
  std::string process(std::string str,
                      std::string filename,
                      bool do_linting,
                      bool do_string_mutation,
                      bool do_include_mutation,
                      bool do_small_type_linting,
                      const ReportErrorF &report_error)
  {
    str = remove_comments(str, report_error);
    threadgroup_variables_parsing(str);
    if (true) {
      parse_library_functions(str);
    }
    if (do_include_mutation) {
      str = preprocessor_directive_mutation(str);
    }
    if (do_string_mutation) {
      static_strings_parsing(str);
      str = static_strings_mutation(str);
      str = printf_processing(str, report_error);
      str = remove_quotes(str);
    }
    if (do_linting) {
      matrix_constructor_linting(str, report_error);
      array_constructor_linting(str, report_error);
    }
    if (do_small_type_linting) {
      small_type_linting(str, report_error);
    }
    str = enum_macro_injection(str);
    str = argument_decorator_macro_injection(str);
    str = array_constructor_macro_injection(str);
    return line_directive_prefix(filename) + str + static_strings_suffix() +
           threadgroup_variables_suffix() + gpu_functions_.str();
  }

  /* Variant use for python shaders. */
  std::string process(const std::string &str)
  {
    auto no_err_report = [](std::string, std::smatch, const char *) {};
    return process(str, "", false, false, false, false, no_err_report);
  }

 private:
  template<typename ReportErrorF>
  std::string remove_comments(const std::string &str, const ReportErrorF &report_error)
  {
    std::string out_str = str;
    {
      /* Multi-line comments. */
      size_t start, end = 0;
      while ((start = out_str.find("/*", end)) != std::string::npos) {
        end = out_str.find("*/", start + 2);
        if (end == std::string::npos) {
          break;
        }
        for (size_t i = start; i < end + 2; ++i) {
          if (out_str[i] != '\n') {
            out_str[i] = ' ';
          }
        }
      }

      if (end == std::string::npos) {
        /* TODO(fclem): Add line / char position to report. */
        report_error(str, std::smatch(), "Malformed multi-line comment.");
        return out_str;
      }
    }
    {
      /* Single-line comments. */
      size_t start, end = 0;
      while ((start = out_str.find("//", end)) != std::string::npos) {
        end = out_str.find('\n', start + 2);
        if (end == std::string::npos) {
          break;
        }
        for (size_t i = start; i < end; ++i) {
          out_str[i] = ' ';
        }
      }

      if (end == std::string::npos) {
        /* TODO(fclem): Add line / char position to report. */
        report_error(str, std::smatch(), "Malformed single line comment, missing newline.");
        return out_str;
      }
    }
    /* Remove trailing whitespaces as they make the subsequent regex much slower. */
    std::regex regex("(\\ )*?\\n");
    return std::regex_replace(out_str, regex, "\n");
  }

  std::string remove_quotes(std::string str)
  {
    return std::regex_replace(str, std::regex(R"(["'])"), " ");
  }

  std::string preprocessor_directive_mutation(const std::string &str)
  {
    std::string out_str = str;
    {
      /* Example: `#include "deps.glsl"` > `//__gpu_include "deps.glsl"` */
      std::regex regex("#\\s*(include|pragma once)");
      out_str = std::regex_replace(out_str, regex, "//__gpu_$1");
    }
    {
      /* Example: `__gpu_include "deps.glsl"` > `__gpu_include(deps.glsl)` */
      std::regex regex("__gpu_include\\s+\"([\\w\\.]+)\"");
      out_str = std::regex_replace(out_str, regex, "__gpu_include($1)");
    }
    return out_str;
  }

  void threadgroup_variables_parsing(std::string str)
  {
    std::regex regex("shared\\s+(\\w+)\\s+(\\w+)([^;]*);");
    for (std::smatch match; std::regex_search(str, match, regex); str = match.suffix()) {
      shared_vars_.push_back({match[1].str(), match[2].str(), match[3].str()});
    }
  }

  void parse_library_functions(std::string str)
  {
    gpu_functions_ << "\n";
    std::regex regex_func(R"(void\s+(\w+)\s*\(([^)]+\))\s*\{)");
    for (std::smatch match; std::regex_search(str, match, regex_func); str = match.suffix()) {
      std::string name = match[1].str();
      std::string args = match[2].str();
      gpu_functions_ << "//__gpu_function(" << name << ")";

      std::regex regex_arg(R"((?:(const|in|out|inout)\s)?(\w+)\s([\w\[\]]+)(?:,|\)))");
      for (std::smatch arg; std::regex_search(args, arg, regex_arg); args = arg.suffix()) {
        std::string qualifier = arg[1].str();
        std::string type = arg[2].str();
        if (qualifier.empty()) {
          qualifier = "in";
        }
        gpu_functions_ << detail::fnv1a_32(qualifier.c_str(), qualifier.size()) << ','
                       << detail::fnv1a_32(type.c_str(), type.size()) << ";";
      }

      gpu_functions_ << "\n";
    }
    gpu_functions_ << "\n";
  }

  template<typename ReportErrorF>
  std::string printf_processing(const std::string &str, const ReportErrorF &report_error)
  {
    std::string out_str = str;
    {
      /* Example: `printf(2, b, f(c, d));` > `printf(2@ b@ f(c@ d))$` */
      size_t start, end = 0;
      while ((start = out_str.find("printf(", end)) != std::string::npos) {
        end = out_str.find(';', start);
        if (end == std::string::npos) {
          break;
        }
        out_str[end] = '$';
        int bracket_depth = 0;
        int arg_len = 0;
        for (size_t i = start; i < end; ++i) {
          if (out_str[i] == '(') {
            bracket_depth++;
          }
          else if (out_str[i] == ')') {
            bracket_depth--;
          }
          else if (bracket_depth == 1 && out_str[i] == ',') {
            out_str[i] = '@';
            arg_len++;
          }
        }
        if (arg_len > 99) {
          report_error(str, std::smatch(), "Too many parameters in printf. Max is 99.");
          break;
        }
        /* Encode number of arg in the `ntf` of `printf`. */
        out_str[start + sizeof("printf") - 4] = '$';
        out_str[start + sizeof("printf") - 3] = ((arg_len / 10) > 0) ? ('0' + arg_len / 10) : '$';
        out_str[start + sizeof("printf") - 2] = '0' + arg_len % 10;
      }
      if (end == 0) {
        /* No printf in source. */
        return str;
      }
    }
    /* Example: `pri$$1(2@ b)$` > `{int c_ = print_header(1, 2); c_ = print_data(c_, b); }` */
    {
      std::regex regex(R"(pri\$\$?(\d{1,2})\()");
      out_str = std::regex_replace(out_str, regex, "{int c_ = print_header($1, ");
    }
    {
      std::regex regex("\\@");
      out_str = std::regex_replace(out_str, regex, "); c_ = print_data(c_,");
    }
    {
      std::regex regex("\\$");
      out_str = std::regex_replace(out_str, regex, "; }");
    }
    return out_str;
  }

  void static_strings_parsing(std::string str)
  {
    /* Matches any character inside a pair of unescaped quote. */
    std::regex regex(R"("(?:[^"])*")");
    for (std::smatch match; std::regex_search(str, match, regex); str = match.suffix()) {
      static_strings_.insert(match[0].str());
    }
  }

  uint string_hash(const std::string &str)
  {
    std::hash<std::string> string_hasher;
    size_t hash_64 = string_hasher(str);
    return uint((hash_64 >> 32) ^ hash_64);
  }

  std::string static_strings_mutation(std::string str)
  {
    /* Replaces all matches by the respective string hash. */
    for (const std::string &str_var : static_strings_) {
      std::regex escape_regex(R"([\\\.\^\$\+\(\)\[\]\{\}\|\?\*])");
      std::string str_regex = std::regex_replace(str_var, escape_regex, "\\$&");

      std::regex regex(str_regex);
      str = std::regex_replace(str, regex, std::to_string(string_hash(str_var)) + 'u');
    }
    return str;
  }

  std::string static_strings_suffix()
  {
    if (static_strings_.empty()) {
      return "";
    }

    std::stringstream suffix;
    suffix << "\n";
    for (const std::string &str_var : static_strings_) {
      uint hash = string_hash(str_var);
      suffix << "//__gpu_string(" << std::to_string(hash) << ")" << str_var << "\n";
    }
    suffix << "\n";
    return suffix.str();
  }

  std::string enum_macro_injection(std::string str)
  {
    /**
     * Transform C,C++ enum declaration into GLSL compatible defines and constants:
     *
     * \code{.cpp}
     * enum eMyEnum : uint32_t {
     *   ENUM_1 = 0u,
     *   ENUM_2 = 1u,
     *   ENUM_3 = 2u,
     * };
     * \endcode
     *
     * or
     *
     * \code{.c}
     * enum eMyEnum {
     *   ENUM_1 = 0u,
     *   ENUM_2 = 1u,
     *   ENUM_3 = 2u,
     * };
     * \endcode
     *
     * becomes
     *
     * \code{.glsl}
     * ENUM_DECL(_eMyEnum)
     *   ENUM_1 = 0u,
     *   ENUM_2 = 1u,
     *   ENUM_3 = 2u, ENUM_END
     * #define eMyEnum ENUM_TYPE(_eMyEnum)
     * \endcode
     *
     * IMPORTANT: This has some requirements:
     * - Enums needs to have underlying types set to uint32_t to make them usable in UBO and SSBO.
     * - All values needs to be specified using constant literals to avoid compiler differences.
     * - All values needs to have the 'u' suffix to avoid GLSL compiler errors.
     */
    {
      /* Replaces all matches by the respective string hash. */
      std::regex regex(R"(enum\s+((\w+)\s*(?:\:\s*\w+\s*)?)\{(\n[^}]+)\n\};)");
      str = std::regex_replace(str, regex, "ENUM_DECL(_$1)$3 ENUM_END\n#define $2 ENUM_TYPE(_$2)");
    }
    {
      /* Remove trailing comma if any. */
      std::regex regex(R"(,(\s*ENUM_END))");
      str = std::regex_replace(str, regex, "$1");
    }
    return str;
  }

  std::string argument_decorator_macro_injection(const std::string &str)
  {
    /* Example: `out float var[2]` > `out float _out_sta var _out_end[2]` */
    std::regex regex("(out|inout|in|shared)\\s+(\\w+)\\s+(\\w+)");
    return std::regex_replace(str, regex, "$1 $2 _$1_sta $3 _$1_end");
  }

  std::string array_constructor_macro_injection(const std::string &str)
  {
    /* Example: `= float[2](0.0, 0.0)` > `= ARRAY_T(float) ARRAY_V(0.0, 0.0)` */
    std::regex regex("=\\s*(\\w+)\\s*\\[[^\\]]*\\]\\s*\\(");
    return std::regex_replace(str, regex, "= ARRAY_T($1) ARRAY_V(");
  }

  /* TODO(fclem): Too many false positive and false negative to be applied to python shaders. */
  template<typename ReportErrorF>
  void matrix_constructor_linting(std::string str, const ReportErrorF &report_error)
  {
    /* Example: `mat4(other_mat)`. */
    std::regex regex("\\s+(mat(\\d|\\dx\\d)|float\\dx\\d)\\([^,\\s\\d]+\\)");
    for (std::smatch match; std::regex_search(str, match, regex); str = match.suffix()) {
      /* This only catches some invalid usage. For the rest, the CI will catch them. */
      const char *msg =
          "Matrix constructor is not cross API compatible. "
          "Use to_floatNxM to reshape the matrix or use other constructors instead.";
      report_error(str, match, msg);
    }
  }

  template<typename ReportErrorF>
  void array_constructor_linting(std::string str, const ReportErrorF &report_error)
  {
    std::regex regex("=\\s*(\\w+)\\s*\\[[^\\]]*\\]\\s*\\(");
    for (std::smatch match; std::regex_search(str, match, regex); str = match.suffix()) {
      /* This only catches some invalid usage. For the rest, the CI will catch them. */
      const char *msg =
          "Array constructor is not cross API compatible. Use type_array instead of type[].";
      report_error(str, match, msg);
    }
  }

  template<typename ReportErrorF>
  void small_type_linting(std::string str, const ReportErrorF &report_error)
  {
    std::regex regex(R"(\su?(char|short|half)(2|3|4)?\s)");
    for (std::smatch match; std::regex_search(str, match, regex); str = match.suffix()) {
      report_error(str, match, "Small types are forbidden in shader interfaces.");
    }
  }

  std::string threadgroup_variables_suffix()
  {
    if (shared_vars_.empty()) {
      return "";
    }

    std::stringstream suffix;
    /**
     * For Metal shaders to compile, shared (threadgroup) variable cannot be declared globally.
     * They must reside within a function scope. Hence, we need to extract these declarations and
     * generate shared memory blocks within the entry point function. These shared memory blocks
     * can then be passed as references to the remaining shader via the class function scope.
     *
     * The shared variable definitions from the source file are replaced with references to
     * threadgroup memory blocks (using _shared_sta and _shared_end macros), but kept in-line in
     * case external macros are used to declare the dimensions.
     *
     * Each part of the codegen is stored inside macros so that we don't have to do string
     * replacement at runtime.
     */
    suffix << "\n";
    /* Arguments of the wrapper class constructor. */
    suffix << "#undef MSL_SHARED_VARS_ARGS\n";
    /* References assignment inside wrapper class constructor. */
    suffix << "#undef MSL_SHARED_VARS_ASSIGN\n";
    /* Declaration of threadgroup variables in entry point function. */
    suffix << "#undef MSL_SHARED_VARS_DECLARE\n";
    /* Arguments for wrapper class constructor call. */
    suffix << "#undef MSL_SHARED_VARS_PASS\n";

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
     * kernel entry_point() {                                   // Added at runtime by backend.
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
    for (SharedVar &var : shared_vars_) {
      char sep = first ? ' ' : ',';
      /*  */
      args << sep << "threadgroup " << var.type << "(&_" << var.name << ")" << var.array;
      assign << (first ? ':' : ',') << var.name << "(_" << var.name << ")";
      declare << "threadgroup " << var.type << ' ' << var.name << var.array << ";";
      pass << sep << var.name;
      first = false;
    }

    suffix << "#define MSL_SHARED_VARS_ARGS " << args.str() << "\n";
    suffix << "#define MSL_SHARED_VARS_ASSIGN " << assign.str() << "\n";
    suffix << "#define MSL_SHARED_VARS_DECLARE " << declare.str() << "\n";
    suffix << "#define MSL_SHARED_VARS_PASS (" << pass.str() << ")\n";
    suffix << "\n";

    return suffix.str();
  }

  std::string line_directive_prefix(const std::string &filepath)
  {
#ifndef __APPLE__
    /* For now, only Metal supports filname in line directive. */
    /* TODO(fclem): Set file source-string-number to the filename hash. */
    return std::string("");
#else
    std::string filename = std::regex_replace(filepath, std::regex(R"((?:.*)\/(.*))"), "$1");

    std::stringstream suffix;
    suffix << "#line 1 ";
    if (!filename.empty()) {
      suffix << "\"" << filename << "\"";
    }
    suffix << "\n";
    return suffix.str();
#endif
  }
};

}  // namespace blender::gpu::shader
