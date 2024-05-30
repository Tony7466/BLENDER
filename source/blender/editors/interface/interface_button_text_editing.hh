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

/**
 * Data for editing the value of the button as text.
 */
struct uiTextEdit {
  /** The currently displayed/edited string, use 'ui_textedit_string_set' to assign new strings. */
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

/** Mode for #ui_textedit_copypaste() */
enum {
  UI_TEXTEDIT_PASTE = 1,
  UI_TEXTEDIT_COPY,
  UI_TEXTEDIT_CUT,
};

/**
 * \param searchbox_region: The searchbox to handle the auto-completion, if any.
 */
int ui_textedit_autocomplete(bContext *C,
                             uiBut *but,
                             uiTextEdit &text_edit,
                             ARegion *searchbox_region);
bool ui_textedit_copypaste(uiBut *but, uiTextEdit &text_edit, const int mode);

/**
 * \param x: Screen space cursor location - #wmEvent.x
 *
 * \note `but->block->aspect` is used here, so drawing button style is getting scaled too.
 */
void ui_textedit_set_cursor_pos(uiBut *but, const ARegion *region, const float x);
void ui_textedit_set_cursor_select(uiBut *but,
                                   const ARegion *region,
                                   uiTextEdit &text_edit,
                                   float x);
void ui_textedit_move(uiBut *but,
                      uiTextEdit &text_edit,
                      eStrCursorJumpDirection direction,
                      bool select,
                      eStrCursorJumpType jump);

bool ui_textedit_delete(uiBut *but,
                        uiTextEdit &text_edit,
                        eStrCursorJumpDirection direction,
                        eStrCursorJumpType jump);
void ui_textedit_string_set(uiBut *but, uiTextEdit &text_edit, const char *str);
bool ui_textedit_delete_selection(uiBut *but, uiTextEdit &text_edit);
/**
 * This is used for both utf8 and ascii
 *
 * For unicode buttons, \a buf is treated as unicode.
 */
bool ui_textedit_insert_buf(uiBut *but, uiTextEdit &text_edit, const char *buf, int buf_len);
#ifdef WITH_INPUT_IME
bool ui_textedit_insert_ascii(uiBut *but, uiTextEdit &text_edit, const char ascii);
#endif
