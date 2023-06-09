/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sptext
 */

#include <cstring>

#include "BLI_blenlib.h"

#include "DNA_space_types.h"
#include "DNA_text_types.h"

#include "BKE_text.h"

#include "text_format.hh"

/* *** Lua Keywords (for format_line) *** */

/**
 * Checks the specified source string for a Lua keyword (minus boolean & 'nil').
 * This name must start at the beginning of the source string and must be
 * followed by a non-identifier (see #text_check_identifier(char)) or null char.
 *
 * If a keyword is found, the length of the matching word is returned.
 * Otherwise, -1 is returned.
 *
 * See:
 * http://www.lua.org/manual/5.1/manual.html#2.1
 */
static int txtfmt_lua_find_keyword(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  constexpr char *keywords[]{
      "and",
      "break",
      "do",
      "else",
      "elseif",
      "end",
      "for",
      "function",
      "if",
      "in",
      "local",
      "not",
      "or",
      "repeat",
      "return",
      "then",
      "until",
      "while",
  };

  // i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  if (i == 0 || text_check_identifier(string[i])) {
    return -1;
  }
  return i;
}

/**
 * Checks the specified source string for a Lua special name/function. This
 * name must start at the beginning of the source string and must be followed
 * by a non-identifier (see *text_check_identifier(char)) or null character.
 *
 * If a special name is found, the length of the matching name is returned.
 * Otherwise, -1 is returned.
 *
 * See:
 * http://www.lua.org/manual/5.1/manual.html#5.1
 */
static int txtfmt_lua_find_specialvar(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  constexpr char* keywords[]{
      "assert",       "collectgarbage",
      "dofile",       "error",
      "_G",           "getfenv",
      "getmetatable", "__index",
      "ipairs",       "load",
      "loadfile",     "loadstring",
      "next",         "pairs",
      "pcall",        "print",
      "rawequal",     "rawget",
      "rawset",       "select",
      "setfenv",      "setmetatable",
      "tonumber",     "tostring",
      "type",         "unpack",
      "_VERSION",     "xpcall",
  };
  // i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  if (i == 0 || text_check_identifier(string[i])) {
    return -1;
  }
  return i;
}

static int txtfmt_lua_find_bool(const char *string)
{
  int i;

  /* Keep aligned args for readability. */

  constexpr char *keywords[]{
      "nil",
      "true",
      "false",
  };

  //i = find_keyword_length(keywords, string);

  /* If next source char is an identifier (eg. 'i' in "Nonetheless") no match */
  if (i == 0 || text_check_identifier(string[i])) {
    return -1;
  }
  return i;
}

static char txtfmt_lua_format_identifier(const char *str)
{
  char fmt;

  /* Keep aligned args for readability. */
  /* clang-format off */

  if        (txtfmt_lua_find_specialvar(str)  != -1) { fmt = FMT_TYPE_SPECIAL;
  } else if (txtfmt_lua_find_keyword(str)     != -1) { fmt = FMT_TYPE_KEYWORD;
  } else                                             { fmt = FMT_TYPE_DEFAULT;
  }

  /* clang-format on */

  return fmt;
}

static void txtfmt_lua_format_line(SpaceText *st, TextLine *line, const bool do_next)
{
  FlattenString fs;
  const char *str;
  char *fmt;
  char cont_orig, cont, find, prev = ' ';
  int len, i;

  /* Get continuation from previous line */
  if (line->prev && line->prev->format != nullptr) {
    fmt = line->prev->format;
    cont = fmt[strlen(fmt) + 1]; /* Just after the null-terminator */
    BLI_assert((FMT_CONT_ALL & cont) == cont);
  }
  else {
    cont = FMT_CONT_NOP;
  }

  /* Get original continuation from this line */
  if (line->format != nullptr) {
    fmt = line->format;
    cont_orig = fmt[strlen(fmt) + 1]; /* Just after the null-terminator */
    BLI_assert((FMT_CONT_ALL & cont_orig) == cont_orig);
  }
  else {
    cont_orig = 0xFF;
  }

  len = flatten_string(st, &fs, line->line);
  str = fs.buf;
  if (!text_check_format_len(line, len)) {
    flatten_string_free(&fs);
    return;
  }
  fmt = line->format;

  while (*str) {
    /* Handle escape sequences by skipping both \ and next char */
    if (*str == '\\') {
      *fmt = prev;
      fmt++;
      str++;
      if (*str == '\0') {
        break;
      }
      *fmt = prev;
      fmt++;
      str += BLI_str_utf8_size_safe(str);
      continue;
    }
    /* Handle continuations */
    if (cont) {
      /* Multi-line comments */
      if (cont & FMT_CONT_COMMENT_C) {
        if (*str == ']' && *(str + 1) == ']') {
          *fmt = FMT_TYPE_COMMENT;
          fmt++;
          str++;
          *fmt = FMT_TYPE_COMMENT;
          cont = FMT_CONT_NOP;
        }
        else {
          *fmt = FMT_TYPE_COMMENT;
        }
        /* Handle other comments */
      }
      else {
        find = (cont & FMT_CONT_QUOTEDOUBLE) ? '"' : '\'';
        if (*str == find) {
          cont = 0;
        }
        *fmt = FMT_TYPE_STRING;
      }

      str += BLI_str_utf8_size_safe(str) - 1;
    }
    /* Not in a string... */
    else {
      /* Multi-line comments */
      if (*str == '-' && *(str + 1) == '-' && *(str + 2) == '[' && *(str + 3) == '[') {
        cont = FMT_CONT_COMMENT_C;
        *fmt = FMT_TYPE_COMMENT;
        fmt++;
        str++;
        *fmt = FMT_TYPE_COMMENT;
        fmt++;
        str++;
        *fmt = FMT_TYPE_COMMENT;
        fmt++;
        str++;
        *fmt = FMT_TYPE_COMMENT;
      }
      /* Single line comment */
      else if (*str == '-' && *(str + 1) == '-') {
        text_format_fill(&str, &fmt, FMT_TYPE_COMMENT, len - int(fmt - line->format));
      }
      else if (ELEM(*str, '"', '\'')) {
        /* Strings */
        find = *str;
        cont = (*str == '"') ? FMT_CONT_QUOTEDOUBLE : FMT_CONT_QUOTESINGLE;
        *fmt = FMT_TYPE_STRING;
      }
      /* White-space (all white-space has been converted to spaces). */
      else if (*str == ' ') {
        *fmt = FMT_TYPE_WHITESPACE;
      }
      /* Numbers (digits not part of an identifier and periods followed by digits) */
      else if ((prev != FMT_TYPE_DEFAULT && text_check_digit(*str)) ||
               (*str == '.' && text_check_digit(*(str + 1))))
      {
        *fmt = FMT_TYPE_NUMERAL;
      }
      /* Booleans */
      else if (prev != FMT_TYPE_DEFAULT && (i = txtfmt_lua_find_bool(str)) != -1) {
        if (i > 0) {
          text_format_fill_ascii(&str, &fmt, FMT_TYPE_NUMERAL, i);
        }
        else {
          str += BLI_str_utf8_size_safe(str) - 1;
          *fmt = FMT_TYPE_DEFAULT;
        }
      }
      /* Punctuation */
      else if ((*str != '#') && text_check_delim(*str)) {
        *fmt = FMT_TYPE_SYMBOL;
      }
      /* Identifiers and other text (no previous white-space/delimiters so text continues). */
      else if (prev == FMT_TYPE_DEFAULT) {
        str += BLI_str_utf8_size_safe(str) - 1;
        *fmt = FMT_TYPE_DEFAULT;
      }
      /* Not white-space, a digit, punctuation, or continuing text.
       * Must be new, check for special words. */
      else {
        /* Keep aligned arguments for readability. */
        /* clang-format off */

        /* Special `vars(v)` or built-in `keywords(b)` */
        /* keep in sync with `txtfmt_osl_format_identifier()`. */
        if        ((i = txtfmt_lua_find_specialvar(str))   != -1) { prev = FMT_TYPE_SPECIAL;
        } else if ((i = txtfmt_lua_find_keyword(str))      != -1) { prev = FMT_TYPE_KEYWORD;
        }

        /* clang-format on */

        if (i > 0) {
          text_format_fill_ascii(&str, &fmt, prev, i);
        }
        else {
          str += BLI_str_utf8_size_safe(str) - 1;
          *fmt = FMT_TYPE_DEFAULT;
        }
      }
    }
    prev = *fmt;
    fmt++;
    str++;
  }

  /* Terminate and add continuation char */
  *fmt = '\0';
  fmt++;
  *fmt = cont;

  /* If continuation has changed and we're allowed, process the next line */
  if (cont != cont_orig && do_next && line->next) {
    txtfmt_lua_format_line(st, line->next, do_next);
  }

  flatten_string_free(&fs);
}

void ED_text_format_register_lua()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"lua", nullptr};

  tft.format_identifier = txtfmt_lua_format_identifier;
  tft.format_line = txtfmt_lua_format_line;
  tft.ext = ext;
  tft.comment_line = "--";

  ED_text_format_register(&tft);
}
