/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 *
 * A special set of icons to represent input devices,
 * this is a mix of text (via fonts) and a handful of custom glyphs for special keys.
 *
 * Event codes are used as identifiers.
 */

#include "BLI_string.h"

#include "BLF_api.hh"

#include "UI_interface.hh"

#include "interface_intern.hh"

static void icon_draw_icon(const rctf *rect, int icon_id, bool inverted)
{
  const float aspect = float(ICON_DEFAULT_WIDTH) / (rect->ymax - rect->ymin);
  UI_icon_draw_ex(rect->xmin,
                  rect->ymin,
                  inverted ? icon_id + 1 : icon_id,
                  aspect,
                  1.0f,
                  0.0f,
                  nullptr,
                  false,
                  nullptr,
                  false);
}

static void icon_draw_rect_input_text(const rctf *rect, bool inverted, const char *str)
{
  float color[4];
  UI_GetThemeColor4fv(inverted ? TH_BACK : TH_TEXT, color);

  // BLF_batch_draw_flush();

  icon_draw_icon(rect, ICON_KEY_EMPTY1, inverted);

  const int font_id = BLF_default();
  BLF_color4fv(font_id, color);

  float available_width = rect->xmax - rect->xmin - (2.0f * UI_SCALE_FAC);

  float font_size = 13.0f * UI_SCALE_FAC;
  float width, height;
  BLF_size(font_id, font_size);
  BLF_width_and_height(font_id, str, BLF_DRAW_STR_DUMMY_MAX, &width, &height);

  if (width > available_width) {
    font_size *= available_width / width;
  }

  BLF_size(font_id, font_size);
  BLF_width_and_height(font_id, str, BLF_DRAW_STR_DUMMY_MAX, &width, &height);
  const float x = trunc(rect->xmin + (((rect->xmax - rect->xmin) - width) / 2.0f));
  const float y = rect->ymin + (((rect->ymax - rect->ymin) - height) / 2.0f);
  BLF_position(font_id, x, y, 0.0f);
  BLF_draw(font_id, str, BLF_DRAW_STR_DUMMY_MAX);
  // BLF_batch_draw_flush();
}

void icon_draw_rect_input(float x,
                          float y,
                          int w,
                          int h,
                          float /*alpha*/,
                          short event_type,
                          short /*event_value*/,
                          bool inverted)
{
  rctf rect{};
  rect.xmin = int(x);
  rect.xmax = int(x + w);
  rect.ymin = int(y);
  rect.ymax = int(y + h);

  const enum {
    UNIX,
    MACOS,
    MSWIN,
  } platform =

#if defined(__APPLE__)
      MACOS
#elif defined(_WIN32)
      MSWIN
#else
      UNIX
#endif
      ;

  if ((event_type >= EVT_AKEY) && (event_type <= EVT_ZKEY)) {
    const char str[2] = {char('A' + (event_type - EVT_AKEY)), '\0'};
    icon_draw_rect_input_text(&rect, inverted, str);
  }
  else if ((event_type >= EVT_F1KEY) && (event_type <= EVT_F24KEY)) {
    char str[4];
    SNPRINTF(str, "F%d", 1 + (event_type - EVT_F1KEY));
    icon_draw_rect_input_text(&rect, inverted, str);
  }
  else if (event_type == EVT_LEFTSHIFTKEY) { /* Right Shift has already been converted to left. */
    icon_draw_icon(&rect, ICON_KEY_SHIFT, inverted);
  }
  else if (event_type == EVT_LEFTCTRLKEY) { /* Right Ctrl has already been converted to left. */
    if (platform == MACOS) {
      icon_draw_icon(&rect, ICON_KEY_CONTROL, inverted);
    }
    else {
      icon_draw_rect_input_text(&rect, inverted, "Ctrl");
    }
  }
  else if (event_type == EVT_LEFTALTKEY) { /* Right Alt has already been converted to left. */
    if (platform == MACOS) {
      icon_draw_icon(&rect, ICON_KEY_OPTION, inverted);
    }
    else {
      icon_draw_rect_input_text(&rect, inverted, "Alt");
    }
  }
  else if (event_type == EVT_OSKEY) {
    if (platform == MACOS) {
      icon_draw_icon(&rect, ICON_KEY_COMMAND, inverted);
    }
    else if (platform == MSWIN) {
      icon_draw_icon(&rect, ICON_KEY_WINDOWS, inverted);
    }
    else {
      icon_draw_rect_input_text(&rect, inverted, "OS");
    }
  }
  else if (event_type == EVT_DELKEY) {
    icon_draw_rect_input_text(&rect, inverted, "Del");
  }
  else if (event_type == EVT_TABKEY) {
    icon_draw_icon(&rect, ICON_KEY_TAB, inverted);
  }
  else if (event_type == EVT_HOMEKEY) {
    icon_draw_rect_input_text(&rect, inverted, "Home");
  }
  else if (event_type == EVT_ENDKEY) {
    icon_draw_rect_input_text(&rect, inverted, "End");
  }
  else if (event_type == EVT_RETKEY) {
    icon_draw_icon(&rect, ICON_KEY_RETURN, inverted);
  }
  else if (event_type == EVT_ESCKEY) {
    if (platform == MACOS) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_BROKEN_CIRCLE_WITH_NORTHWEST_ARROW);
    }
    else {
      icon_draw_rect_input_text(&rect, inverted, "Esc");
    }
  }
  else if (event_type == EVT_PAGEUPKEY) {
    icon_draw_rect_input_text(&rect, inverted, "P" BLI_STR_UTF8_UPWARDS_ARROW);
  }
  else if (event_type == EVT_PAGEDOWNKEY) {
    icon_draw_rect_input_text(&rect, inverted, "P" BLI_STR_UTF8_DOWNWARDS_ARROW);
  }
  else if (event_type == EVT_LEFTARROWKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_LEFTWARDS_ARROW);
  }
  else if (event_type == EVT_UPARROWKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_UPWARDS_ARROW);
  }
  else if (event_type == EVT_RIGHTARROWKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_RIGHTWARDS_ARROW);
  }
  else if (event_type == EVT_DOWNARROWKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_DOWNWARDS_ARROW);
  }
  else if (event_type == EVT_SPACEKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_OPEN_BOX);
  }
  else if (event_type == BUTTON4MOUSE) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "4");
  }
  else if (event_type == BUTTON5MOUSE) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "5");
  }
  else if (event_type == BUTTON6MOUSE) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "6");
  }
  else if (event_type == BUTTON7MOUSE) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "7");
  }
  else if (event_type == TABLET_STYLUS) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_LOWER_RIGHT_PENCIL);
  }
  else if (event_type == TABLET_ERASER) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_UPPER_RIGHT_PENCIL);
  }
  else if ((event_type >= EVT_ZEROKEY) && (event_type <= EVT_NINEKEY)) {
    const char str[2] = {char('0' + (event_type - EVT_ZEROKEY)), '\0'};
    icon_draw_rect_input_text(&rect, inverted, str);
  }
  else if ((event_type >= EVT_PAD0) && (event_type <= EVT_PAD9)) {
    char str[5];
    SNPRINTF(str, "%s%i", BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH, event_type - EVT_PAD0);
    icon_draw_rect_input_text(&rect, inverted, str);
  }
  else if (event_type == EVT_PADASTERKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "6");
  }
  else if (event_type == EVT_PADSLASHKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "/");
  }
  else if (event_type == EVT_PADMINUS) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "-");
  }
  else if (event_type == EVT_PADENTER) {
    icon_draw_rect_input_text(
        &rect,
        inverted,
        BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH BLI_STR_UTF8_RETURN_SYMBOL);
  }
  else if (event_type == EVT_PADPLUSKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "+");
  }
  else if (event_type == EVT_PAUSEKEY) {
    icon_draw_rect_input_text(&rect, inverted, "Pause");
  }
  else if (event_type == EVT_INSERTKEY) {
    icon_draw_rect_input_text(&rect, inverted, "Insert");
  }
  else if (event_type == EVT_UNKNOWNKEY) {
    icon_draw_rect_input_text(&rect, inverted, " ");
  }
  else if (event_type == EVT_GRLESSKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_GREATER_THAN_OR_LESS_THAN);
  }
  else if (event_type == EVT_MEDIAPLAY) {
    icon_draw_rect_input_text(
        &rect, inverted, BLI_STR_UTF8_BLACK_RIGHT_POINTING_TRIANGLE_WITH_DOUBLE_VERTICAL_BAR);
  }
  else if (event_type == EVT_MEDIASTOP) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_BLACK_SQUARE_FOR_STOP);
  }
  else if (event_type == EVT_MEDIAFIRST) {
    icon_draw_rect_input_text(
        &rect, inverted, BLI_STR_UTF8_BLACK_LEFT_POINTING_DOUBLE_TRIANGLE_WITH_VERTICAL_BAR);
  }
  else if (event_type == EVT_MEDIALAST) {
    icon_draw_rect_input_text(
        &rect, inverted, BLI_STR_UTF8_BLACK_RIGHT_POINTING_DOUBLE_TRIANGLE_WITH_VERTICAL_BAR);
  }
  else if (event_type == EVT_APPKEY) {
    icon_draw_rect_input_text(&rect, inverted, "App");
  }
  else if (event_type == EVT_PADPERIOD) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH ".");
  }
  else if (event_type == EVT_CAPSLOCKKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_UPWARDS_UP_ARROW_FROM_BAR);
  }
  else if (event_type == EVT_LINEFEEDKEY) {
    icon_draw_rect_input_text(&rect, inverted, "LF");
  }
  else if (event_type == EVT_BACKSPACEKEY) {
    icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_ERASE_TO_THE_LEFT);
  }
  else if (event_type == EVT_SEMICOLONKEY) {
    icon_draw_rect_input_text(&rect, inverted, ";");
  }
  else if (event_type == EVT_PERIODKEY) {
    icon_draw_rect_input_text(&rect, inverted, ".");
  }
  else if (event_type == EVT_COMMAKEY) {
    icon_draw_rect_input_text(&rect, inverted, ",");
  }
  else if (event_type == EVT_QUOTEKEY) {
    icon_draw_rect_input_text(&rect, inverted, "'");
  }
  else if (event_type == EVT_ACCENTGRAVEKEY) {
    icon_draw_rect_input_text(&rect, inverted, "`");
  }
  else if (event_type == EVT_MINUSKEY) {
    icon_draw_rect_input_text(&rect, inverted, "-");
  }
  else if (event_type == EVT_PLUSKEY) {
    icon_draw_rect_input_text(&rect, inverted, "+");
  }
  else if (event_type == EVT_SLASHKEY) {
    icon_draw_rect_input_text(&rect, inverted, "/");
  }
  else if (event_type == EVT_BACKSLASHKEY) {
    icon_draw_rect_input_text(&rect, inverted, "\\");
  }
  else if (event_type == EVT_EQUALKEY) {
    icon_draw_rect_input_text(&rect, inverted, "=");
  }
  else if (event_type == EVT_LEFTBRACKETKEY) {
    icon_draw_rect_input_text(&rect, inverted, "[");
  }
  else if (event_type == EVT_RIGHTBRACKETKEY) {
    icon_draw_rect_input_text(&rect, inverted, "]");
  }
  else if (ISNDOF_BUTTON(event_type)) {
    if ((event_type >= NDOF_BUTTON_V1) && (event_type <= NDOF_BUTTON_V3)) {
      char str[7];
      SNPRINTF(str, "%sv%i", BLI_STR_UTF8_CIRCLED_WHITE_BULLET, (event_type + 1) - NDOF_BUTTON_V1);
      icon_draw_rect_input_text(&rect, inverted, str);
    }
    if ((event_type >= NDOF_BUTTON_SAVE_V1) && (event_type <= NDOF_BUTTON_SAVE_V3)) {
      char str[7];
      SNPRINTF(
          str, "%ss%i", BLI_STR_UTF8_CIRCLED_WHITE_BULLET, (event_type + 1) - NDOF_BUTTON_SAVE_V1);
      icon_draw_rect_input_text(&rect, inverted, str);
    }
    else if ((event_type >= NDOF_BUTTON_1) && (event_type <= NDOF_BUTTON_9)) {
      char str[6];
      SNPRINTF(str, "%s%i", BLI_STR_UTF8_CIRCLED_WHITE_BULLET, (1 + event_type) - NDOF_BUTTON_1);
      icon_draw_rect_input_text(&rect, inverted, str);
    }
    else if (event_type >= NDOF_BUTTON_10 && event_type <= NDOF_BUTTON_12) {
      char str[7];
      SNPRINTF(str, "%s1%i", BLI_STR_UTF8_CIRCLED_WHITE_BULLET, event_type - NDOF_BUTTON_10);
      icon_draw_rect_input_text(&rect, inverted, str);
    }
    else if (event_type == NDOF_BUTTON_MENU) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Me");
    }
    else if (event_type == NDOF_BUTTON_FIT) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ft");
    }
    else if (event_type == NDOF_BUTTON_TOP) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Tp");
    }
    else if (event_type == NDOF_BUTTON_BOTTOM) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Bt");
    }
    else if (event_type == NDOF_BUTTON_LEFT) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Le");
    }
    else if (event_type == NDOF_BUTTON_RIGHT) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ri");
    }
    else if (event_type == NDOF_BUTTON_FRONT) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Fr");
    }
    else if (event_type == NDOF_BUTTON_BACK) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Bk");
    }
    else if (event_type == NDOF_BUTTON_ISO1) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "I1");
    }
    else if (event_type == NDOF_BUTTON_ISO2) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "I2");
    }
    else if (event_type == NDOF_BUTTON_ROLL_CW) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Rl");
    }
    else if (event_type == NDOF_BUTTON_ROLL_CCW) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Rc");
    }
    else if (event_type == NDOF_BUTTON_SPIN_CW) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Sp");
    }
    else if (event_type == NDOF_BUTTON_SPIN_CCW) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Sc");
    }
    else if (event_type == NDOF_BUTTON_TILT_CW) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ti");
    }
    else if (event_type == NDOF_BUTTON_TILT_CCW) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Tc");
    }
    else if (event_type == NDOF_BUTTON_ROTATE) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ro");
    }
    else if (event_type == NDOF_BUTTON_PANZOOM) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "PZ");
    }
    else if (event_type == NDOF_BUTTON_DOMINANT) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Dm");
    }
    else if (event_type == NDOF_BUTTON_PLUS) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "+");
    }
    else if (event_type == NDOF_BUTTON_MINUS) {
      icon_draw_rect_input_text(&rect, inverted, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "-");
    }
  }
}
