/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "UI_interface_c.hh"

struct Image;

namespace blender {
class StringRef;
}  // namespace blender

namespace blender::nodes {

struct NodeExtraInfoRow {
  std::string text;
  int icon = 0;
  const char *tooltip = nullptr;

  uiButToolTipFunc tooltip_fn = nullptr;
  void *tooltip_fn_arg = nullptr;
  void (*tooltip_fn_free_arg)(void *) = nullptr;
};

struct NodeExtraInfoParams {
  Vector<NodeExtraInfoRow> &rows;
  const bNode &node;
  const bContext &C;
};

struct NodeTooltipTextLine {
  StringRef line;
  uiTooltipColorID color = UI_TIP_LC_MAIN;
  int indentation = 0;
  Image *image = nullptr;

  enum class Type : int8_t {
    Line = 0,
    Space = 1,
    Image = 2,
  };

  Type type = Type::Line;

  NodeTooltipTextLine() = default;
  NodeTooltipTextLine(StringRef src_line) : line(src_line) {}
  NodeTooltipTextLine(StringRef src_line, uiTooltipColorID src_color)
      : line(src_line), color(src_color)
  {
  }

  static NodeTooltipTextLine empty_space();
};

}  // namespace blender::nodes
