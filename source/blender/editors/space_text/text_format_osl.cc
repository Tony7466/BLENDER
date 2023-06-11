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

/* -------------------------------------------------------------------- */
/** \name Local Literal Definitions
 * \{ */

/**
 * OSL builtin function.
 * list is from
 * https://github.com/imageworks/OpenShadingLanguage/raw/master/src/doc/osl-languagespec.pdf
 */
static StringRef text_format_osl_literals_builtinfunc[]{
    /* clang-format off */
    StringRef("break"),
    StringRef("closure"),
    StringRef("color"),
    StringRef("continue"),
    StringRef("do"),
    StringRef("else"),
    StringRef("emit"),
    StringRef("float"),
    StringRef("for"),
    StringRef("if"),
    StringRef("illuminance"),
    StringRef("illuminate"),
    StringRef("int"),
    StringRef("matrix"),
    StringRef("normal"),
    StringRef("output"),
    StringRef("point"),
    StringRef("public"),
    StringRef("return"),
    StringRef("string"),
    StringRef("struct"),
    StringRef("vector"),
    StringRef("void"),
    StringRef("while"),
    /* clang-format on */
};

/**
 * OSL reserved keywords
 * See:
 * https://github.com/imageworks/OpenShadingLanguage/raw/master/src/doc/osl-languagespec.pdf
 */
static StringRef text_format_osl_literals_reserved[]{
    /* clang-format off */
    StringRef("bool"),
    StringRef("case"),
    StringRef("catch"),
    StringRef("char"),
    StringRef("const"),
    StringRef("default"),
    StringRef("delete"),
    StringRef("double"),
    StringRef("enum"),
    StringRef("extern"),
    StringRef("false"),
    StringRef("friend"),
    StringRef("goto"),
    StringRef("inline"),
    StringRef("long"),
    StringRef("new"),
    StringRef("operator"),
    StringRef("private"),
    StringRef("protected"),
    StringRef("short"),
    StringRef("signed"),
    StringRef("sizeof"),
    StringRef("static"),
    StringRef("switch"),
    StringRef("template"),
    StringRef("this"),
    StringRef("throw"),
    StringRef("true"),
    StringRef("try"),
    StringRef("typedef"),
    StringRef("uniform"),
    StringRef("union"),
    StringRef("unsigned"),
    StringRef("varying"),
    StringRef("virtual"),
    StringRef("volatile"),
    /* clang-format on */
};

/* OSL shader types */
static StringRef text_format_osl_literals_specialvar[]{
    /* clang-format off */
    StringRef("displacement"),
    StringRef("shader"),
    StringRef("surface"),
    StringRef("volume"),
    /* clang-format on */
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Local Functions (for #TextFormatType::format_line)
 * \{ */

static int txtfmt_osl_find_builtinfunc(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_osl_literals_builtinfunc, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  if (i == 0 || text_check_identifier(string[i])) {
    return -1;
  }
  return i;
}

static int txtfmt_osl_find_reserved(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_osl_literals_reserved, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  if (i == 0 || text_check_identifier(string[i])) {
    return -1;
  }
  return i;
}

static int txtfmt_osl_find_specialvar(const char *string)
{
  const StringRef *string_literal = text_format_string_literal_find(
      text_format_osl_literals_specialvar, string);
  if (!string_literal) {
    return -1;
  }

  const int i = string_literal->size();

  /* If next source char is an identifier (eg. 'i' in "definite") no match */
  if (i == 0 || text_check_identifier(string[i])) {
    return -1;
  }
  return i;
}

/* matches py 'txtfmt_osl_find_decorator' */
static int txtfmt_osl_find_preprocessor(const char *string)
{
  if (string[0] == '#') {
    int i = 1;
    /* White-space is ok '#  foo'. */
    while (text_check_whitespace(string[i])) {
      i++;
    }
    while (text_check_identifier(string[i])) {
      i++;
    }
    return i;
  }
  return -1;
}

static char txtfmt_osl_format_identifier(const char *str)
{
  char fmt;

  /* Keep aligned args for readability. */
  /* clang-format off */

  if        (txtfmt_osl_find_specialvar(str)   != -1) { fmt = FMT_TYPE_SPECIAL;
  } else if (txtfmt_osl_find_builtinfunc(str)  != -1) { fmt = FMT_TYPE_KEYWORD;
  } else if (txtfmt_osl_find_reserved(str)     != -1) { fmt = FMT_TYPE_RESERVED;
  } else if (txtfmt_osl_find_preprocessor(str) != -1) { fmt = FMT_TYPE_DIRECTIVE;
  } else                                              { fmt = FMT_TYPE_DEFAULT;
  }

  /* clang-format on */

  return fmt;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Format Line Implementation (#TextFormatType::format_line)
 * \{ */

static void txtfmt_osl_format_line(SpaceText *st, TextLine *line, const bool do_next)
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
      /* C-Style comments */
      if (cont & FMT_CONT_COMMENT_C) {
        if (*str == '*' && *(str + 1) == '/') {
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
      /* Deal with comments first */
      if (*str == '/' && *(str + 1) == '/') {
        /* fill the remaining line */
        text_format_fill(&str, &fmt, FMT_TYPE_COMMENT, len - int(fmt - line->format));
      }
      /* C-Style (multi-line) comments */
      else if (*str == '/' && *(str + 1) == '*') {
        cont = FMT_CONT_COMMENT_C;
        *fmt = FMT_TYPE_COMMENT;
        fmt++;
        str++;
        *fmt = FMT_TYPE_COMMENT;
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
      /* Punctuation */
      else if ((*str != '#') && text_check_delim(*str)) {
        *fmt = FMT_TYPE_SYMBOL;
      }
      /* Identifiers and other text (no previous white-space or delimiters. so text continues). */
      else if (prev == FMT_TYPE_DEFAULT) {
        str += BLI_str_utf8_size_safe(str) - 1;
        *fmt = FMT_TYPE_DEFAULT;
      }
      /* Not white-space, a digit, punctuation, or continuing text.
       * Must be new, check for special words. */
      else {
        /* Keep aligned arguments for readability. */
        /* clang-format off */

        /* Special vars(v) or built-in keywords(b) */
        /* keep in sync with `txtfmt_osl_format_identifier()`. */
        if        ((i = txtfmt_osl_find_specialvar(str))   != -1) { prev = FMT_TYPE_SPECIAL;
        } else if ((i = txtfmt_osl_find_builtinfunc(str))  != -1) { prev = FMT_TYPE_KEYWORD;
        } else if ((i = txtfmt_osl_find_reserved(str))     != -1) { prev = FMT_TYPE_RESERVED;
        } else if ((i = txtfmt_osl_find_preprocessor(str)) != -1) { prev = FMT_TYPE_DIRECTIVE;
        }
        /* clang-format on */

        if (i > 0) {
          if (prev == FMT_TYPE_DIRECTIVE) { /* can contain utf8 */
            text_format_fill(&str, &fmt, prev, i);
          }
          else {
            text_format_fill_ascii(&str, &fmt, prev, i);
          }
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
    txtfmt_osl_format_line(st, line->next, do_next);
  }

  flatten_string_free(&fs);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_text_format_register_osl()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"osl", nullptr};

  tft.format_identifier = txtfmt_osl_format_identifier;
  tft.format_line = txtfmt_osl_format_line;
  tft.ext = ext;
  tft.comment_line = "//";

  ED_text_format_register(&tft);

  text_format_string_literals_sort_for_lookup(text_format_osl_literals_builtinfunc);
  text_format_string_literals_sort_for_lookup(text_format_osl_literals_reserved);
  text_format_string_literals_sort_for_lookup(text_format_osl_literals_specialvar);
}

/** \} */
