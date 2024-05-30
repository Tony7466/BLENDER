/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#pragma once

#include "BLI_string_cursor_utf8.h"

struct ARegion;
struct bContext;
struct uiUndoStack_Text;
struct uiBut;

namespace blender::ui {

/**
 * Data for editing the value of the button as text.
 */
struct TextEdit {
  /** The currently displayed/edited string, use #textedit_string_set() to assign new strings. */
  char *edit_string;
  /* Maximum string size the button accepts, and as such the maximum size for #edit_string
   * (including terminator). */
  int max_string_size;
  /* Allow reallocating #edit_string and using #max_string_size to track alloc size (maxlen + 1) */
  bool is_str_dynamic;
  char *original_string;

  /* Button text selection:
   * extension direction, selextend, inside ui_do_but_TEX */
  int sel_pos_init;

  /* Text field undo. */
  uiUndoStack_Text *undo_stack_text;
};

/** Mode for #textedit_copypaste() */
enum {
  TEXTEDIT_PASTE = 1,
  TEXTEDIT_COPY,
  TEXTEDIT_CUT,
};

/**
 * \param searchbox_region: The searchbox to handle the auto-completion, if any.
 */
int textedit_autocomplete(bContext *C, uiBut *but, TextEdit &text_edit, ARegion *searchbox_region);
bool textedit_copypaste(uiBut *but, TextEdit &text_edit, int mode);

/**
 * \param x: Screen space cursor location - #wmEvent.x
 *
 * \note `but->block->aspect` is used here, so drawing button style is getting scaled too.
 */
void textedit_set_cursor_pos(uiBut *but, const ARegion *region, float x);
void textedit_set_cursor_select(uiBut *but, const ARegion *region, TextEdit &text_edit, float x);
void textedit_move(uiBut *but,
                   TextEdit &text_edit,
                   eStrCursorJumpDirection direction,
                   bool select,
                   eStrCursorJumpType jump);

bool textedit_delete(uiBut *but,
                     TextEdit &text_edit,
                     eStrCursorJumpDirection direction,
                     eStrCursorJumpType jump);
void textedit_string_set(uiBut *but, TextEdit &text_edit, const char *str);
bool textedit_delete_selection(uiBut *but, TextEdit &text_edit);
/**
 * This is used for both utf8 and ascii
 *
 * For unicode buttons, \a buf is treated as unicode.
 */
bool textedit_insert_buf(uiBut *but, TextEdit &text_edit, const char *buf, int buf_len);
#ifdef WITH_INPUT_IME
bool textedit_insert_ascii(uiBut *but, TextEdit &text_edit, const char ascii);
#endif

}  // namespace blender::ui
