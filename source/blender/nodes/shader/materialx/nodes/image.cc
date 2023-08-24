/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node.h"
#include "image.h"

#include "hydra/image.h"

#include "DEG_depsgraph_query.h"

namespace blender::nodes::materialx {

const MaterialX::Color3 MaterialXTexImageNode::texture_error_color_{1.0, 0.0, 1.0};

MaterialXTexImageNode::MaterialXTexImageNode(MaterialX::DocumentPtr doc,
                                             const Depsgraph *depsgraph,
                                             const Material *material,
                                             const bNode *node)
    : MaterialXNode(doc, depsgraph, material, node)
{
  matx_node = doc->addNode("image", MaterialX::createValidName(node->name), "color3");
}

MaterialX::NodePtr MaterialXTexImageNode::convert()
{
  Image *image = (Image *)node->id;
  NodeTexImage *tex = static_cast<NodeTexImage *>(node->storage);
  Scene *scene = DEG_get_input_scene(depsgraph);
  Main *bmain = DEG_get_bmain(depsgraph);
  std::string image_path;
  /* TODO: What if Blender built without Hydra? Also io::hydra::cache_or_get_image_file contain
   * pretty general code, so could be moved from bf_usd project. */
#ifdef WITH_HYDRA
  image_path = io::hydra::cache_or_get_image_file(bmain, scene, image, &tex->iuser);
#endif
  MaterialX::NodePtr uv_node = doc->addNode("texcoord", MaterialX::EMPTY_STRING, "vector2");

  matx_node->addInput("file", "filename")->setValue(image_path);
  matx_node->addInput("texcoord", "vector2")->setNodeName(uv_node->getName());

  return matx_node;
}

}  // namespace blender::nodes::materialx
