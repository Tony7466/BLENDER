#ifndef WITH_HEADLESS

#  include <algorithm>
#  include <fstream>
#  include <sstream>

#  include "BLF_api.hh"
#  include "blf_internal.hh"

#  include "svg_icons.h"

namespace blender::blf::icon {

static blender::Vector<SVG_Icon> &icons_storage()
{
  static blender::Vector<SVG_Icon> icons = blender_default_icons();
  return icons;
}

const char *get_icon_svg(int icon)
{
  blender::Span<SVG_Icon> icons = icons_storage();
  if (icons.size() < icon) {
    return datatoc_none_svg;
  }
  return icons[icon].svg.c_str();
}

void set(blender::StringRefNull name, blender::StringRefNull filepath)
{
  blender::Vector<SVG_Icon> &icons = icons_storage();
  SVG_Icon *icon = std::find_if(
      icons.begin(), icons.end(), [name](const SVG_Icon &icon) { return name == icon.name; });
  if (icon == icons.end()) {
    return;
  }
  std::ifstream svg_file(filepath);
  if (svg_file.fail()) {
    return;
  }
  svg_file.seekg(0, std::ios::beg);
  std::ostringstream ss;
  ss << svg_file.rdbuf();
  icon->svg = ss.str();
}

void reset(blender::StringRefNull name)
{
  blender::Vector<SVG_Icon> &icons = icons_storage();
  SVG_Icon *icon = std::find_if(
      icons.begin(), icons.end(), [name](const SVG_Icon &icon) { return name == icon.name; });
  if (icon == icons.end()) {
    return;
  }
  icon->svg = blender_default_icons()[icon - icons.begin()].svg;
}
}  // namespace blender::blf::icon

#endif
