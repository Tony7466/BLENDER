/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#include "csv_string_utils.hh"

/* NOTE: we could use C++17 <charconv> from_chars to parse
 * floats, but even if some compilers claim full support,
 * their standard libraries are not quite there yet.
 * LLVM/libc++ only has a float parser since LLVM 14,
 * and gcc/libstdc++ since 11.1. So until at least these are
 * the minimum spec, use an external library. */
#include "fast_float.h"
#include <charconv>

namespace blender::io::csv {

StringRef read_next_line(StringRef &buffer)
{
  const char *start = buffer.begin();
  const char *end = buffer.end();
  size_t len = 0;
  const char *ptr = start;
  while (ptr < end) {
    char c = *ptr++;
    if (c == '\n') {
      break;
    }
    ++len;
  }

  buffer = StringRef(ptr, end);
  return StringRef(start, len - 1);
}

static bool is_whitespace(char c)
{
  return c <= ' ';
}

void fixup_line_continuations(char *p, char *end)
{
  while (true) {
    /* Find next backslash, if any. */
    char *backslash = std::find(p, end, '\\');
    if (backslash == end) {
      break;
    }
    /* Skip over possible whitespace right after it. */
    p = backslash + 1;
    while (p < end && is_whitespace(*p) && *p != '\n') {
      ++p;
    }
    /* If then we have a newline, turn both backslash
     * and the newline into regular spaces. */
    if (p < end && *p == '\n') {
      *backslash = ' ';
      *p = ' ';
    }
  }
}

const char *drop_whitespace(const char *p, const char *end)
{
  while (p < end && is_whitespace(*p)) {
    ++p;
  }
  return p;
}

const char *drop_non_whitespace(const char *p, const char *end)
{
  while (p < end && !is_whitespace(*p)) {
    ++p;
  }
  return p;
}

static const char *drop_sign(const char *p, const char *end, int &sign)
{
  sign = 1;
  if (p < end) {
    if (*p == '+') {
      ++p;
    }
    if (*p == '-') {
      sign = -1;
      ++p;
    }
  }
  return p;
}

const char *parse_float(const char *p, const char *end, bool &success, float &dst, bool skip_space)
{
  if (skip_space) {
    p = drop_whitespace(p, end);
  }
  int sign = 0;
  p = drop_sign(p, end, sign);
  fast_float::from_chars_result res = fast_float::from_chars(p, end, dst);
  if (ELEM(res.ec, std::errc::invalid_argument, std::errc::result_out_of_range) || res.ptr < end) {
    success = false;
  }
  else {
    dst *= sign;
    success = true;
  }
  return res.ptr;
}

const char *parse_int(const char *p, const char *end, bool &success, int &dst, bool skip_space)
{
  if (skip_space) {
    p = drop_whitespace(p, end);
  }
  int sign = 0;
  p = drop_sign(p, end, sign);
  std::from_chars_result res = std::from_chars(p, end, dst);
  if (ELEM(res.ec, std::errc::invalid_argument, std::errc::result_out_of_range) || res.ptr < end) {
    success = false;
  }
  else {
    dst *= sign;
    success = true;
  }
  return res.ptr;
}

}  // namespace blender::io::csv
