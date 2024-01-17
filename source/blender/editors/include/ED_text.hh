/* SPDX-FileCopyrightText: 2009 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

struct ARegion;
struct SpaceText;
struct Text;
struct UndoStep;
struct UndoType;
struct bContext;
struct TextSearch;
struct StringMatch;

/* `text_draw.cc` */

bool ED_text_activate_in_screen(bContext *C, Text *text);

int ED_space_text_visible_lines_get(const SpaceText *st);
/**
 * Moves the view to the cursor location, also used to make sure the view isn't outside the file.
 */
void ED_space_text_scroll_to_cursor(SpaceText *st, ARegion *region, bool center);
/**
 * Takes a cursor (row, character) and returns x,y pixel coords.
 */
bool ED_space_text_region_location_from_cursor(const SpaceText *st,
                                               const ARegion *region,
                                               const int cursor_co[2],
                                               int r_pixel_co[2]);

/* `text_undo.cc` */

/** Export for ED_undo_sys. */
void ED_text_undosys_type(UndoType *ut);

/** Use operator system to finish the undo step. */
UndoStep *ED_text_undo_push_init(bContext *C);

/* `text_format.cc` */

const char *ED_text_format_comment_line_prefix(Text *text);

bool ED_text_is_syntax_highlight_supported(Text *text);

namespace blender::ed::text {

/* Get the index of the active Text data-block in the Space Text search list. */
int active_text_search_get(const SpaceText *st);

/* Updates the search result in a space text. */
void texts_search_update(const bContext *C, const SpaceText *st);

/**
 * Any modification to Text data-blocks can invalidate the search result when a space text is
 * inactive. To prevent a invalid searches, this removes runtime text search and tags to restore
 * text search on space text reactivation.
 */
void texts_search_tag_restore(const SpaceText *st);

/** Get the text search result in a Space Text. */
TextSearch *text_search_get(const SpaceText *st, const Text *text);

TextSearch *texts_search_begin_get(const SpaceText *st);
const int texts_search_size_get(const SpaceText *st);

StringMatch *string_matches_begin_get(const TextSearch *ts);
const int string_matches_size_get(const TextSearch *ts);

}  // namespace blender::ed::text
