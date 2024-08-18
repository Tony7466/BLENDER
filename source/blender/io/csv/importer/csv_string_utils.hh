/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup csv
 */

#pragma once

#include "BLI_string_ref.hh"

/*
 * Various text parsing utilities used by CSV importer.
 *
 * Many of these functions take two pointers (p, end) indicating
 * which part of a string to operate on, and return a possibly
 * changed new start of the string. They could be taking a StringRef
 * as input and returning a new StringRef, but this is a hot path
 * in CSV parsing, and the StringRef approach does lose performance
 * (mostly due to return of StringRef being two register-size values
 * instead of just one pointer).
 */

namespace blender::io::csv {

/**
 * Fetches next line from an input string buffer.
 *
 * The returned line will not have '\n' characters at the end;
 * the `buffer` is modified to contain remaining text without
 * the input line.
 */
StringRef read_next_line(StringRef &buffer);

/**
 * Fix up OBJ line continuations by replacing backslash (\) and the
 * following newline with spaces.
 */
void fixup_line_continuations(char *p, char *end);

/**
 * Drop leading white-space from a string part.
 */
const char *drop_whitespace(const char *p, const char *end);

/**
 * Drop leading non-white-space from a string part.
 */
const char *drop_non_whitespace(const char *p, const char *end);

/**
 * Parse an integer from an input string.
 * The parsed result is stored in `dst`. The function skips
 * leading white-space unless `skip_space=false`. If the
 * number can't be parsed (invalid syntax, out of range),
 * `fallback` value is stored instead.
 *
 * Returns the start of remainder of the input string after parsing.
 */
const char *parse_int(
    const char *p, const char *end, bool &success, int &dst, bool skip_space = true);

/**
 * Parse a float from an input string.
 * The parsed result is stored in `dst`. The function skips
 * leading white-space unless `skip_space=false`. If the
 * number can't be parsed (invalid syntax, out of range),
 * `fallback` value is stored instead. If `require_trailing_space`
 * is true, the character after the number has to be whitespace.
 *
 * Returns the start of remainder of the input string after parsing.
 */
const char *parse_float(
    const char *p, const char *end, bool &success, float &dst, bool skip_space = true);

}  // namespace blender::io::csv
