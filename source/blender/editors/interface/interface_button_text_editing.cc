/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include <cstring>

#include "BLF_api.hh"

#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "WM_api.hh"

#include "interface_button_text_editing.hh"
#include "interface_intern.hh"

namespace blender::ui {

static void ui_textedit_string_ensure_max_length(uiBut *but, TextEdit &text_edit, int str_maxncpy)
{
  BLI_assert(text_edit.is_str_dynamic);
  BLI_assert(text_edit.edit_string == but->editstr);

  if (str_maxncpy > text_edit.max_string_size) {
    text_edit.edit_string = but->editstr = static_cast<char *>(
        MEM_reallocN(text_edit.edit_string, sizeof(char) * str_maxncpy));
    text_edit.max_string_size = str_maxncpy;
  }
}

int textedit_autocomplete(bContext *C, uiBut *but, TextEdit &text_edit, ARegion *searchbox_region)
{
  char *str = text_edit.edit_string;

  int changed;
  if (searchbox_region) {
    changed = ui_searchbox_autocomplete(C, searchbox_region, but, text_edit.edit_string);
  }
  else {
    changed = but->autocomplete_func(C, str, but->autofunc_arg);
  }

  but->pos = strlen(str);
  but->selsta = but->selend = but->pos;

  return changed;
}

bool textedit_copypaste(uiBut *but, TextEdit &text_edit, const int mode)
{
  bool changed = false;

  /* paste */
  if (mode == TEXTEDIT_PASTE) {
    /* extract the first line from the clipboard */
    int buf_len;
    char *pbuf = WM_clipboard_text_get_firstline(false, UI_but_is_utf8(but), &buf_len);

    if (pbuf) {
      textedit_insert_buf(but, text_edit, pbuf, buf_len);

      changed = true;

      MEM_freeN(pbuf);
    }
  }
  /* cut & copy */
  else if (ELEM(mode, TEXTEDIT_COPY, TEXTEDIT_CUT)) {
    /* copy the contents to the copypaste buffer */
    const int sellen = but->selend - but->selsta;
    char *buf = static_cast<char *>(
        MEM_mallocN(sizeof(char) * (sellen + 1), "textedit_copypaste"));

    memcpy(buf, text_edit.edit_string + but->selsta, sellen);
    buf[sellen] = '\0';

    WM_clipboard_text_set(buf, false);
    MEM_freeN(buf);

    /* for cut only, delete the selection afterwards */
    if (mode == TEXTEDIT_CUT) {
      if ((but->selend - but->selsta) > 0) {
        changed = textedit_delete_selection(but, text_edit);
      }
    }
  }

  return changed;
}

void textedit_set_cursor_pos(uiBut *but, const ARegion *region, const float x)
{
  /* XXX pass on as arg. */
  uiFontStyle fstyle = UI_style_get()->widget;
  const float aspect = but->block->aspect;

  float startx = but->rect.xmin;
  float starty_dummy = 0.0f;
  char password_str[UI_MAX_PASSWORD_STR];
  /* treat 'str_last' as null terminator for str, no need to modify in-place */
  const char *str = but->editstr, *str_last;

  ui_block_to_window_fl(region, but->block, &startx, &starty_dummy);

  ui_fontscale(&fstyle.points, aspect);

  UI_fontstyle_set(&fstyle);

  ui_but_text_password_hide(password_str, but, false);

  if (ELEM(but->type, UI_BTYPE_TEXT, UI_BTYPE_SEARCH_MENU)) {
    if (but->flag & UI_HAS_ICON) {
      startx += UI_ICON_SIZE / aspect;
    }
  }
  startx += (UI_TEXT_MARGIN_X * U.widget_unit - U.pixelsize) / aspect;

  /* mouse dragged outside the widget to the left */
  if (x < startx) {
    int i = but->ofs;

    str_last = &str[but->ofs];

    while (i > 0) {
      if (BLI_str_cursor_step_prev_utf8(str, but->ofs, &i)) {
        /* 0.25 == scale factor for less sensitivity */
        if (BLF_width(fstyle.uifont_id, str + i, (str_last - str) - i) > (startx - x) * 0.25f) {
          break;
        }
      }
      else {
        break; /* unlikely but possible */
      }
    }
    but->ofs = i;
    but->pos = but->ofs;
  }
  /* mouse inside the widget, mouse coords mapped in widget space */
  else {
    but->pos = but->ofs +
               BLF_str_offset_from_cursor_position(
                   fstyle.uifont_id, str + but->ofs, strlen(str + but->ofs), int(x - startx));
  }

  ui_but_text_password_hide(password_str, but, true);
}

void textedit_set_cursor_select(uiBut *but,
                                const ARegion *region,
                                TextEdit &text_edit,
                                const float x)
{
  textedit_set_cursor_pos(but, region, x);

  but->selsta = but->pos;
  but->selend = text_edit.sel_pos_init;
  if (but->selend < but->selsta) {
    std::swap(but->selsta, but->selend);
  }

  ui_but_update(but);
}

void textedit_move(uiBut *but,
                   TextEdit &text_edit,
                   eStrCursorJumpDirection direction,
                   const bool select,
                   eStrCursorJumpType jump)
{
  const char *str = text_edit.edit_string;
  const int len = strlen(str);
  const int pos_prev = but->pos;
  const bool has_sel = (but->selend - but->selsta) > 0;

  ui_but_update(but);

  /* special case, quit selection and set cursor */
  if (has_sel && !select) {
    if (jump == STRCUR_JUMP_ALL) {
      but->selsta = but->selend = but->pos = direction ? len : 0;
    }
    else {
      if (direction) {
        but->selsta = but->pos = but->selend;
      }
      else {
        but->pos = but->selend = but->selsta;
      }
    }
    text_edit.sel_pos_init = but->pos;
  }
  else {
    int pos_i = but->pos;
    BLI_str_cursor_step_utf8(str, len, &pos_i, direction, jump, true);
    but->pos = pos_i;

    if (select) {
      if (has_sel == false) {
        /* Holding shift but with no previous selection. */
        but->selsta = but->pos;
        but->selend = pos_prev;
      }
      else if (but->selsta == pos_prev) {
        /* Previous selection, extending start position. */
        but->selsta = but->pos;
      }
      else {
        /* Previous selection, extending end position. */
        but->selend = but->pos;
      }
    }
    if (but->selend < but->selsta) {
      std::swap(but->selsta, but->selend);
    }
  }
}

bool textedit_delete(uiBut *but,
                     TextEdit &text_edit,
                     eStrCursorJumpDirection direction,
                     eStrCursorJumpType jump)
{
  char *str = text_edit.edit_string;
  const int len = strlen(str);

  bool changed = false;

  if (jump == STRCUR_JUMP_ALL) {
    if (len) {
      changed = true;
    }
    str[0] = '\0';
    but->pos = 0;
  }
  else if (direction) { /* delete */
    if ((but->selend - but->selsta) > 0) {
      changed = textedit_delete_selection(but, text_edit);
    }
    else if (but->pos >= 0 && but->pos < len) {
      int pos = but->pos;
      int step;
      BLI_str_cursor_step_utf8(str, len, &pos, direction, jump, true);
      step = pos - but->pos;
      memmove(&str[but->pos], &str[but->pos + step], (len + 1) - (but->pos + step));
      changed = true;
    }
  }
  else { /* backspace */
    if (len != 0) {
      if ((but->selend - but->selsta) > 0) {
        changed = textedit_delete_selection(but, text_edit);
      }
      else if (but->pos > 0) {
        int pos = but->pos;
        int step;

        BLI_str_cursor_step_utf8(str, len, &pos, direction, jump, true);
        step = but->pos - pos;
        memmove(&str[but->pos - step], &str[but->pos], (len + 1) - but->pos);
        but->pos -= step;
        changed = true;
      }
    }
  }

  return changed;
}

void textedit_string_set(uiBut *but, TextEdit &text_edit, const char *str)
{
  if (text_edit.is_str_dynamic) {
    ui_textedit_string_ensure_max_length(but, text_edit, strlen(str) + 1);
  }

  if (UI_but_is_utf8(but)) {
    BLI_strncpy_utf8(text_edit.edit_string, str, text_edit.max_string_size);
  }
  else {
    BLI_strncpy(text_edit.edit_string, str, text_edit.max_string_size);
  }
}

bool textedit_delete_selection(uiBut *but, TextEdit &text_edit)
{
  char *str = text_edit.edit_string;
  const int len = strlen(str);
  bool changed = false;
  if (but->selsta != but->selend && len) {
    memmove(str + but->selsta, str + but->selend, (len - but->selend) + 1);
    changed = true;
  }

  but->pos = but->selend = but->selsta;
  return changed;
}

bool textedit_insert_buf(uiBut *but, TextEdit &text_edit, const char *buf, int buf_len)
{
  int len = strlen(text_edit.edit_string);
  const int str_maxncpy_new = len - (but->selend - but->selsta) + 1;
  bool changed = false;

  if (text_edit.is_str_dynamic) {
    ui_textedit_string_ensure_max_length(but, text_edit, str_maxncpy_new + buf_len);
  }

  if (str_maxncpy_new <= text_edit.max_string_size) {
    char *str = text_edit.edit_string;
    size_t step = buf_len;

    /* type over the current selection */
    if ((but->selend - but->selsta) > 0) {
      changed = textedit_delete_selection(but, text_edit);
      len = strlen(str);
    }

    if ((len + step >= text_edit.max_string_size) && (text_edit.max_string_size - (len + 1) > 0)) {
      if (UI_but_is_utf8(but)) {
        /* Shorten 'step' to a utf8 aligned size that fits. */
        BLI_strnlen_utf8_ex(buf, text_edit.max_string_size - (len + 1), &step);
      }
      else {
        step = text_edit.max_string_size - (len + 1);
      }
    }

    if (step && (len + step < text_edit.max_string_size)) {
      memmove(&str[but->pos + step], &str[but->pos], (len + 1) - but->pos);
      memcpy(&str[but->pos], buf, step * sizeof(char));
      but->pos += step;
      changed = true;
    }
  }

  return changed;
}

#ifdef WITH_INPUT_IME
bool textedit_insert_ascii(uiBut *but, TextEdit &text_edit, const char ascii)
{
  BLI_assert(isascii(ascii));
  const char buf[2] = {ascii, '\0'};
  return textedit_insert_buf(but, text_edit, buf, sizeof(buf) - 1);
}
#endif

}  // namespace blender::ui
