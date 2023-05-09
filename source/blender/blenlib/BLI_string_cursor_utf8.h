/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2011 Blender Foundation */

#pragma once

/** \file
 * \ingroup bli
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum eStrCursorDelimType {
  STRCUR_DELIM_NONE,
  STRCUR_DELIM_ALPHANUMERIC,
  STRCUR_DELIM_PUNCT,
  STRCUR_DELIM_BRACE,
  STRCUR_DELIM_OPERATOR,
  STRCUR_DELIM_QUOTE,
  STRCUR_DELIM_WHITESPACE,
  STRCUR_DELIM_OTHER,
} eStrCursorDelimType;

typedef enum eStrCursorJumpType {
  STRCUR_JUMP_NONE,
  STRCUR_JUMP_DELIM,
  STRCUR_JUMP_ALL,
} eStrCursorJumpType;

typedef enum eStrCursorJumpDirection {
  STRCUR_DIR_PREV,
  STRCUR_DIR_NEXT,
} eStrCursorJumpDirection;


eStrCursorDelimType BLI_str_cursor_delim_type_utf32(const char32_t *ch_utf32);

eStrCursorDelimType BLI_str_cursor_delim_type_utf8(const char *ch_utf8,
                                                   const size_t ch_utf8_len,
                                                   const int pos);

bool BLI_str_cursor_step_next_utf8(const char *str, size_t str_maxlen, int *pos);
bool BLI_str_cursor_step_prev_utf8(const char *str, size_t str_maxlen, int *pos);

bool BLI_str_cursor_step_next_utf32(const char32_t *str, size_t str_maxlen, int *pos);
bool BLI_str_cursor_step_prev_utf32(const char32_t *str, size_t str_maxlen, int *pos);

void BLI_str_cursor_step_utf8(const char *str,
                              size_t str_maxlen,
                              int *pos,
                              eStrCursorJumpDirection direction,
                              eStrCursorJumpType jump);

void BLI_str_cursor_step_utf32(const char32_t *str,
                               size_t str_maxlen,
                               int *pos,
                               eStrCursorJumpDirection direction,
                               eStrCursorJumpType jump);

#ifdef __cplusplus
}
#endif
