/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node.h"

namespace blender::nodes::materialx {

MaterialXNode::MaterialXNode(MaterialX::DocumentPtr doc,
                             const Depsgraph *depsgraph,
                             const Material *material,
                             const bNode *node)
    : depsgraph(depsgraph), material(material), node(node), doc(doc)
{
}

}  // namespace blender::nodes::materialx
