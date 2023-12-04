/* SPDX-FileCopyrightText: 2009 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BLI_vector.hh"

#include "DNA_text_types.h"
#include "DNA_vec_types.h"

struct ARegion;
struct SpaceText;
struct Text;
struct UndoStep;
struct UndoType;
struct bContext;

bool ED_text_activate_in_screen(bContext *C, Text *text);

/**
 * Moves the view to the cursor location, also used to make sure the view isn't outside the file.
 */
void ED_text_scroll_to_cursor(SpaceText *st, ARegion *region, bool center);

/**
 * Takes a cursor (row, character) and returns x,y pixel coords.
 */
bool ED_text_region_location_from_cursor(SpaceText *st,
                                         ARegion *region,
                                         const int cursor_co[2],
                                         int r_pixel_co[2]);

/* text_undo.cc */

/** Export for ED_undo_sys. */
void ED_text_undosys_type(UndoType *ut);

/** Use operator system to finish the undo step. */
UndoStep *ED_text_undo_push_init(bContext *C);

/* `text_format.cc` */

const char *ED_text_format_comment_line_prefix(Text *text);

bool ED_text_is_syntax_highlight_supported(Text *text);

/* Get the position of the active text search, of the active text in the #space_text. */
int ED_text_get_active_text_search(const SpaceText *st);

/* Get the index of the string match selected by the cursor.  */
int ED_text_get_active_string_match(const SpaceText *st);

/* Updates the search result done in the #space_text. */
void ED_text_update_search(const bContext *C, const SpaceText *st);

/* Get the #text search result int the space. */
const TextSearch *ED_text_get_text_search(const SpaceText *st, const Text *text);

struct SpaceText_Runtime {

  /** Actual line height, scaled by DPI. */
  int lheight_px;

  /** Runtime computed, character width. */
  int cwidth_px;

  /** The handle of the scroll-bar which can be clicked and dragged. */
  struct rcti scroll_region_handle;
  /** The region for selected text to show in the scrolling area. */
  struct rcti scroll_region_select;

  /** Number of digits to show in the line numbers column (when enabled). */
  int line_number_display_digits;

  /** Number of lines this window can display (even when they aren't used). */
  int viewlines;

  /** Use for drawing scroll-bar & calculating scroll operator motion scaling. */
  float scroll_px_per_line;

  /**
   * Run-time for scroll increments smaller than a line (smooth scroll).
   * Values must be between zero and the line, column width: (cwidth, TXT_LINE_HEIGHT(st)).
   */
  int scroll_ofs_px[2];

  char _pad1[4];

  /** Cache for faster drawing. */
  void *drawcache;

  /** Search result in text data-blocks. */
  blender::Vector<TextSearch> texts_search;
};
