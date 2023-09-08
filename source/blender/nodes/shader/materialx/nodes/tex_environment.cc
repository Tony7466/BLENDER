/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

#include "hydra/image.h"

#include "DEG_depsgraph_query.h"

namespace blender::nodes::materialx {

NodeItem TexEnvironmentNodeParser::compute()
{
  NodeItem res = val(MaterialX::Color4(1.0f, 0.0f, 1.0f, 1.0f));

  Image *image = (Image *)node_->id;
  if (!image) {
    return res;
  }

  NodeTexEnvironment *tex_env = static_cast<NodeTexEnvironment *>(node_->storage);
  Scene *scene = DEG_get_input_scene(depsgraph_);
  Main *bmain = DEG_get_bmain(depsgraph_);

  /* TODO: What if Blender built without Hydra? Also io::hydra::cache_or_get_image_file contains
   * pretty general code, so could be moved from bf_usd project. */
  std::string image_path = io::hydra::cache_or_get_image_file(
      bmain, scene, image, &tex_env->iuser);

  NodeItem vector = get_input_link("Vector", NodeItem::Type::Vector2);
  if (!vector) {
    vector = texcoord_node();
  }
  /* TODO: texcoords should be translated to spherical coordinates */

  std::string filtertype;
  switch (tex_env->interpolation) {
    case SHD_INTERP_LINEAR:
      filtertype = "linear";
      break;
    case SHD_INTERP_CLOSEST:
      filtertype = "closest";
      break;
    case SHD_INTERP_CUBIC:
    case SHD_INTERP_SMART:
      filtertype = "cubic";
      break;
    default:
      BLI_assert_unreachable();
  }

  res = create_node("image", NodeItem::Type::Color4);
  res.set_input("file", image_path, NodeItem::Type::Filename);
  res.set_input("texcoord", vector);
  res.set_input("filtertype", val(filtertype));

  return res;
}

}  // namespace blender::nodes::materialx
