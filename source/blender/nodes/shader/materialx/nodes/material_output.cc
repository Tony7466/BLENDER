/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "material_output.h"
#include "principled_bsdf.h"

namespace blender::nodes::materialx {

MaterialXMaterialOutputNode::MaterialXMaterialOutputNode(MaterialX::DocumentPtr doc,
                                                         const Depsgraph *depsgraph,
                                                         const Material *material,
                                                         const bNode *node)
    : MaterialXNode(doc, depsgraph, material, node)
{
  matx_node = doc->addNode("surfacematerial", MaterialX::createValidName(node->name), "material");
}

MaterialX::NodePtr MaterialXMaterialOutputNode::convert()
{
  LISTBASE_FOREACH (const bNodeSocket *, sock, &node->inputs) {
    if (!sock->link) {
      continue;
    }
    if (STREQ(sock->name, "Surface")) {
      const bNode *inode = sock->link->fromnode;
      MaterialXPrincipledBSDFNode surface_node(doc, depsgraph, material, inode);
      surface_node.convert();
      matx_node->addInput("surfaceshader", "surfaceshader")->setNodeName(inode->name);
    }
  }
  return matx_node;
}

}  // namespace blender::nodes::materialx
