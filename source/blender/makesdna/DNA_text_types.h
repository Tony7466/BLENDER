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
#  include "BLI_vector.hh"
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
  struct TextLine *text_line;
  /** Index of the line in the Text. */
  int line_index;
  /** Start and end position in #tex_line where the `SpaceText.findstr` string was found. */
  int start, end;

  int flags;

#ifdef __cplusplus
  bool operator==(const StringMatch &other) const
  {
    return text_line == other.text_line && line_index == other.line_index &&
           start == other.start && end == other.end;
  }
#endif
};

typedef struct TextSearch {
  /** Text data block. */
  struct Text *text;
  /* Pointer to `blender::Vector<StringMatch>` match vector. */
  void *_string_matches;
#ifdef __cplusplus
  TextSearch(Text *text) : text(text)
  {
    _string_matches = MEM_new<blender::Vector<StringMatch>>(__func__);
  };

  TextSearch(TextSearch &&other)
  {
    text = other.text;
    _string_matches = other._string_matches;
    other._string_matches = nullptr;
    other.text = nullptr;
  };

  TextSearch &operator=(TextSearch &&other)
  {
    text = other.text;
    string_matches_free();
    _string_matches = other._string_matches;
    other._string_matches = nullptr;
    other.text = nullptr;
    return *this;
  };

  void string_matches_free()
  {
    MEM_delete(static_cast<blender::Vector<StringMatch> *>(_string_matches));
    _string_matches = nullptr;
  }
  ~TextSearch()
  {
    string_matches_free();
  }

  blender::Vector<StringMatch> &string_matches() const
  {
    BLI_assert(_string_matches != nullptr);
    return *(static_cast<blender::Vector<StringMatch> *>(_string_matches));
  }
#endif
} TextMatch;

/** #StringMatch.flags */
enum {
  /** Set if the occurrence is selected for replace. */
  TXT_SM_SELECTED = 1 << 0,
};
