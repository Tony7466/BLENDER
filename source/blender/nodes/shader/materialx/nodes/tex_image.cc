/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

#include "hydra/image.h"

#include "DEG_depsgraph_query.h"

namespace blender::nodes::materialx {

NodeItem TexImageNodeParser::compute()
{
  Image *image = (Image *)node_->id;
  NodeTexImage *tex = static_cast<NodeTexImage *>(node_->storage);
  Scene *scene = DEG_get_input_scene(depsgraph_);
  Main *bmain = DEG_get_bmain(depsgraph_);
  std::string image_path;
  /* TODO: What if Blender built without Hydra? Also io::hydra::cache_or_get_image_file contains
   * pretty general code, so could be moved from bf_usd project. */
#ifdef WITH_HYDRA
  image_path = io::hydra::cache_or_get_image_file(bmain, scene, image, &tex->iuser);
#endif

  NodeItem texcoord = create_node("texcoord", NodeItem::Type::Vector2);
  NodeItem res = create_node("image", NodeItem::Type::Color3);
  res.set_input("file", image_path, NodeItem::Type::Filename);
  res.set_input("texcoord", texcoord);
  return res;
}

}  // namespace blender::nodes::materialx
