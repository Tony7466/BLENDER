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

#include "BLT_translation.hh"

#include "UI_interface.hh"

#include "interface_intern.hh"

static void icon_draw_icon(float x, float y, int icon_id, bool inverted, float alpha)
{
  UI_icon_draw_ex(x,
                  y,
                  inverted ? icon_id + 1 : icon_id,
                  U.inv_scale_factor,
                  alpha,
                  0.0f,
                  nullptr,
                  false,
                  nullptr,
                  false);
}

static void icon_draw_rect_input_text(
    rctf *rect, const char *str, bool inverted, float alpha, EventIconWidth icon_width)
{
  float color[4];
  UI_GetThemeColor4fv(inverted ? TH_BACK : TH_TEXT, color);
  if (alpha < 1.0f) {
    color[3] *= alpha;
  }

  if (icon_width == EventIconWidth::Wide) {
    rect->xmax = rect->xmin + (rect->xmax - rect->xmin) * 1.5f;
    float offset = 4.0f * UI_SCALE_FAC;
    icon_draw_icon(rect->xmin + offset, rect->ymin, ICON_KEY_EMPTY2, inverted, alpha);
  }
  else if (icon_width == EventIconWidth::Widest) {
    rect->xmax = rect->xmin + (rect->xmax - rect->xmin) * 2.0f;
    float offset = 8.0f * UI_SCALE_FAC;
    icon_draw_icon(rect->xmin + offset, rect->ymin, ICON_KEY_EMPTY3, inverted, alpha);
  }
  else {
    icon_draw_icon(rect->xmin, rect->ymin, ICON_KEY_EMPTY1, inverted, alpha);
  }

  const int font_id = BLF_default();
  BLF_color4fv(font_id, color);

  float available_width = rect->xmax - rect->xmin - (3.0f * UI_SCALE_FAC);

  const uiFontStyle *fstyle = UI_FSTYLE_WIDGET;
  float font_size = std::min(15.0f, fstyle->points) * UI_SCALE_FAC;

  float width, height;
  BLF_size(font_id, font_size);
  BLF_width_and_height(font_id, str, BLF_DRAW_STR_DUMMY_MAX, &width, &height);

  if (width > available_width) {
    font_size *= available_width / width;
  }

  BLF_size(font_id, font_size);
  BLF_width_and_height(font_id, str, BLF_DRAW_STR_DUMMY_MAX, &width, &height);
  const float x = rect->xmin + UI_SCALE_FAC + ((available_width - width) / 2.0f);

  const float y = rect->ymin + (font_size * 0.4f);
  BLF_position(font_id, x, y, 0.0f);
  BLF_draw(font_id, str, BLF_DRAW_STR_DUMMY_MAX);
}

EventIconWidth ui_event_icon_width(const int icon)
{
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

  if (ELEM(icon,
           ICON_EVENT_ESC,
           ICON_EVENT_DEL,
           ICON_EVENT_HOME,
           ICON_EVENT_END,
           ICON_EVENT_PAUSE,
           ICON_EVENT_INSERT,
           ICON_EVENT_APP))
  {
    return EventIconWidth::Wide;
  }

  if (platform != MACOS && ELEM(icon, ICON_EVENT_CTRL, ICON_EVENT_ALT, ICON_EVENT_OS)) {
    return EventIconWidth::Wide;
  }

  if (icon == ICON_EVENT_OS && platform != MACOS && platform != MSWIN) {
    return EventIconWidth::Wide;
  }

  if (icon == ICON_EVENT_SPACEKEY) {
    return EventIconWidth::Widest;
  }

  return EventIconWidth::Normal;
}

void icon_draw_rect_input(float x, float y, int w, int h, int icon, float alpha, bool inverted)
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

  EventIconWidth icon_width = ui_event_icon_width(icon);

  if ((icon >= ICON_EVENT_A) && (icon <= ICON_EVENT_Z)) {
    const char str[2] = {char('A' + (icon - ICON_EVENT_A)), '\0'};
    icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
  }
  else if ((icon >= ICON_EVENT_ZEROKEY) && (icon <= ICON_EVENT_NINEKEY)) {
    const char str[2] = {char('0' + (icon - ICON_EVENT_ZEROKEY)), '\0'};
    icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
  }
  else if ((icon >= ICON_EVENT_F1) && (icon <= ICON_EVENT_F12)) {
    char str[4];
    SNPRINTF(str, "F%d", 1 + (icon - ICON_EVENT_F1));
    icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
  }
  else if ((icon >= ICON_EVENT_F13) && (icon <= ICON_EVENT_F24)) {
    char str[4];
    SNPRINTF(str, "F%d", 1 + (icon - ICON_EVENT_F13));
    icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
  }
  if (icon == ICON_EVENT_SHIFT) {
    icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_SHIFT, inverted, alpha);
  }
  else if (icon == ICON_EVENT_CTRL) {
    if (platform == MACOS) {
      icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_CONTROL, inverted, alpha);
    }
    else {
      icon_draw_rect_input_text(&rect, IFACE_("Ctrl"), inverted, alpha, icon_width);
    }
  }
  else if (icon == ICON_EVENT_ALT) {
    if (platform == MACOS) {
      icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_OPTION, inverted, alpha);
    }
    else {
      icon_draw_rect_input_text(&rect, IFACE_("Alt"), inverted, alpha, icon_width);
    }
  }
  else if (icon == ICON_EVENT_OS) {
    if (platform == MACOS) {
      icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_COMMAND, inverted, alpha);
    }
    else if (platform == MSWIN) {
      icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_WINDOWS, inverted, alpha);
    }
    else {
      icon_draw_rect_input_text(&rect, IFACE_("OS"), inverted, alpha, icon_width);
    }
  }
  else if (icon == ICON_EVENT_DEL) {
    icon_draw_rect_input_text(&rect, IFACE_("Del"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_TAB) {
    icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_TAB, inverted, alpha);
  }
  else if (icon == ICON_EVENT_HOME) {
    icon_draw_rect_input_text(&rect, IFACE_("Home"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_END) {
    icon_draw_rect_input_text(&rect, IFACE_("End"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_RETURN) {
    icon_draw_icon(rect.xmin, rect.ymin, ICON_KEY_RETURN, inverted, alpha);
  }
  else if (icon == ICON_EVENT_ESC) {
    icon_draw_rect_input_text(&rect, IFACE_("Esc"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PAGEUP) {
    icon_draw_rect_input_text(&rect, "P" BLI_STR_UTF8_UPWARDS_ARROW, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PAGEDOWN) {
    icon_draw_rect_input_text(
        &rect, "P" BLI_STR_UTF8_DOWNWARDS_ARROW, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_LEFT_ARROW) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_LEFTWARDS_ARROW, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_UP_ARROW) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_UPWARDS_ARROW, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_RIGHT_ARROW) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_RIGHTWARDS_ARROW, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_DOWN_ARROW) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_DOWNWARDS_ARROW, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_SPACEKEY) {
    icon_draw_rect_input_text(&rect, IFACE_("Space"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MOUSE_4) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "4", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MOUSE_5) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "5", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MOUSE_6) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "6", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MOUSE_7) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_BLACK_VERTICAL_ELLIPSE "7", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_TABLET_STYLUS) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_LOWER_RIGHT_PENCIL, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_TABLET_ERASER) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_UPPER_RIGHT_PENCIL, inverted, alpha, icon_width);
  }
  else if ((icon >= ICON_EVENT_PAD0) && (icon <= ICON_EVENT_PAD9)) {
    char str[5];
    SNPRINTF(str, "%s%i", BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH, icon - ICON_EVENT_PAD0);
    icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PADASTER) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "6", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PADSLASH) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "/", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PADMINUS) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "-", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PADENTER) {
    icon_draw_rect_input_text(
        &rect,
        BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH BLI_STR_UTF8_RETURN_SYMBOL,
        inverted,
        alpha,
        icon_width);
  }
  else if (icon == ICON_EVENT_PADPLUS) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH "+", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PADPERIOD) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_SQUARE_WITH_ORTHOGONAL_CROSSHATCH ".", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PAUSE) {
    icon_draw_rect_input_text(&rect, IFACE_("Pause"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_INSERT) {
    icon_draw_rect_input_text(&rect, IFACE_("Insert"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_UNKNOWN) {
    icon_draw_rect_input_text(&rect, " ", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_GRLESS) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_GREATER_THAN_OR_LESS_THAN, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MEDIAPLAY) {
    icon_draw_rect_input_text(&rect,
                              BLI_STR_UTF8_BLACK_RIGHT_POINTING_TRIANGLE_WITH_DOUBLE_VERTICAL_BAR,
                              inverted,
                              alpha,
                              icon_width);
  }
  else if (icon == ICON_EVENT_MEDIASTOP) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_BLACK_SQUARE_FOR_STOP, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MEDIAFIRST) {
    icon_draw_rect_input_text(&rect,
                              BLI_STR_UTF8_BLACK_LEFT_POINTING_DOUBLE_TRIANGLE_WITH_VERTICAL_BAR,
                              inverted,
                              alpha,
                              icon_width);
  }
  else if (icon == ICON_EVENT_MEDIALAST) {
    icon_draw_rect_input_text(&rect,
                              BLI_STR_UTF8_BLACK_RIGHT_POINTING_DOUBLE_TRIANGLE_WITH_VERTICAL_BAR,
                              inverted,
                              alpha,
                              icon_width);
  }
  else if (icon == ICON_EVENT_APP) {
    icon_draw_rect_input_text(&rect, IFACE_("App"), inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_CAPSLOCK) {
    icon_draw_rect_input_text(
        &rect, BLI_STR_UTF8_UPWARDS_UP_ARROW_FROM_BAR, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_BACKSPACE) {
    icon_draw_rect_input_text(&rect, BLI_STR_UTF8_ERASE_TO_THE_LEFT, inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_SEMICOLON) {
    icon_draw_rect_input_text(&rect, ";", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PERIOD) {
    icon_draw_rect_input_text(&rect, ".", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_COMMA) {
    icon_draw_rect_input_text(&rect, ",", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_QUOTE) {
    icon_draw_rect_input_text(&rect, "'", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_ACCENTGRAVE) {
    icon_draw_rect_input_text(&rect, "`", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_MINUS) {
    icon_draw_rect_input_text(&rect, "-", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_PLUS) {
    icon_draw_rect_input_text(&rect, "+", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_SLASH) {
    icon_draw_rect_input_text(&rect, "/", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_BACKSLASH) {
    icon_draw_rect_input_text(&rect, "\\", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_EQUAL) {
    icon_draw_rect_input_text(&rect, "=", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_LEFTBRACKET) {
    icon_draw_rect_input_text(&rect, "[", inverted, alpha, icon_width);
  }
  else if (icon == ICON_EVENT_RIGHTBRACKET) {
    icon_draw_rect_input_text(&rect, "]", inverted, alpha, icon_width);
  }
  else if (icon >= ICON_EVENT_NDOF_BUTTON_V1 && icon <= ICON_EVENT_NDOF_BUTTON_MINUS) {
    if ((icon >= ICON_EVENT_NDOF_BUTTON_V1) && (icon <= ICON_EVENT_NDOF_BUTTON_V3)) {
      char str[7];
      SNPRINTF(
          str, "%sv%i", BLI_STR_UTF8_CIRCLED_WHITE_BULLET, (icon + 1) - ICON_EVENT_NDOF_BUTTON_V1);
      icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
    }
    if ((icon >= ICON_EVENT_NDOF_BUTTON_SAVE_V1) && (icon <= ICON_EVENT_NDOF_BUTTON_SAVE_V3)) {
      char str[7];
      SNPRINTF(str,
               "%ss%i",
               BLI_STR_UTF8_CIRCLED_WHITE_BULLET,
               (icon + 1) - ICON_EVENT_NDOF_BUTTON_SAVE_V1);
      icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
    }
    else if ((icon >= ICON_EVENT_NDOF_BUTTON_1) && (icon <= ICON_EVENT_NDOF_BUTTON_12)) {
      char str[7];
      SNPRINTF(
          str, "%s%i", BLI_STR_UTF8_CIRCLED_WHITE_BULLET, (1 + icon) - ICON_EVENT_NDOF_BUTTON_1);
      icon_draw_rect_input_text(&rect, str, inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_MENU) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Me", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_FIT) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ft", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_TOP) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Tp", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_BOTTOM) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Bt", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_LEFT) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Le", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_RIGHT) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ri", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_FRONT) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Fr", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_BACK) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Bk", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_ISO1) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "I1", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_ISO2) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "I2", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_ROLL_CW) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Rl", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_ROLL_CCW) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Rc", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_SPIN_CW) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Sp", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_SPIN_CCW) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Sc", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_TILT_CW) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ti", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_TILT_CCW) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Tc", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_ROTATE) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Ro", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_PANZOOM) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "PZ", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_DOMINANT) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "Dm", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_PLUS) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "+", inverted, alpha, icon_width);
    }
    else if (icon == ICON_EVENT_NDOF_BUTTON_MINUS) {
      icon_draw_rect_input_text(
          &rect, BLI_STR_UTF8_CIRCLED_WHITE_BULLET "-", inverted, alpha, icon_width);
    }
  }
}
