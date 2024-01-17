/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
/** \file
 * \ingroup DNA
 *
 * Text blocks used for Python-Scripts, OpenShadingLanguage
 * and arbitrary text data to store in blend files.
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_listBase.h"
#ifdef __cplusplus
namespace blender::ed::text {
struct StringMatchContainer;
}  // namespace blender::ed::text
using StringMatchContainer = blender::ed::text::StringMatchContainer;
#else
typedef struct StringMatchContainer StringMatchContainer;
#endif
typedef struct TextLine {
  struct TextLine *next, *prev;

  char *line;
  /** May be NULL if syntax is off or not yet formatted. */
  char *format;
  int len;
  char _pad0[4];
} TextLine;

typedef struct Text {
  ID id;

  /**
   * Optional file path, when NULL text is considered internal.
   * Otherwise this path will be used when saving/reloading.
   *
   * When set this is where the file will or has been saved.
   */
  char *filepath;

  /**
   * Python code object for this text (cached result of #Py_CompileStringObject).
   */
  void *compiled;

  int flags;
  char _pad0[4];

  ListBase lines;
  TextLine *curl, *sell;
  int curc, selc;

  double mtime;
} Text;

#define TXT_TABSIZE 4

/** #Text.flags */
enum {
  /** Set if the file in run-time differs from the file on disk, or if there is no file on disk. */
  TXT_ISDIRTY = 1 << 0,
  /** When the text hasn't been written to a file. #Text.filepath may be NULL or invalid. */
  TXT_ISMEM = 1 << 2,
  /** Should always be set if the Text is not to be written into the `.blend`. */
  TXT_ISEXT = 1 << 3,
  /** Load the script as a Python module when loading the `.blend` file. */
  TXT_ISSCRIPT = 1 << 4,

  TXT_FLAG_UNUSED_8 = 1 << 8, /* cleared */
  TXT_FLAG_UNUSED_9 = 1 << 9, /* cleared */

  /** Use space instead of tabs. */
  TXT_TABSTOSPACES = 1 << 10,
};

struct StringMatch {
  /** Line where a match was found. */
  TextLine *text_line;
  /** Index of the line in the Text. */
  int line_index;
  /** Start and end position in #tex_line where the `SpaceText.findstr` string was found. */
  int start, end;
  int flags;
};

/** #StringMatch.flags */
enum {
  /** Set if a match is selected for replace. */
  TXT_SM_SELECTED = 1 << 0,
};

struct TextSearch {
  /** Text data-block. */
  Text *text;
  StringMatchContainer *matches;
  int active_string_match;
  char _pad0[4];
};
