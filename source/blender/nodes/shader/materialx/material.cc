/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "material.h"
#include "nodes/material_output.h"

#include <MaterialXCore/Node.h>

#include "NOD_shader.h"

namespace blender::nodes::materialx {

static void export_nodegraph(MaterialX::DocumentPtr doc, Depsgraph *depsgraph, Material *material)
{
  material->nodetree->ensure_topology_cache();

  bNode *output_node = ntreeShaderOutputNode(material->nodetree, SHD_OUTPUT_ALL);
  MaterialXMaterialOutputNode material_node(doc, depsgraph, material, output_node);
  material_node.convert();
}

static void create_standard_surface(MaterialX::DocumentPtr doc, Material *material)
{
  MaterialX::NodePtr surfacematerial = doc->addNode(
      "surfacematerial", MaterialX::EMPTY_STRING, "material");
  MaterialX::NodePtr standard_surface = doc->addNode(
      "standard_surface", MaterialX::EMPTY_STRING, "surfaceshader");

  standard_surface->addInput("base", "float")->setValue(1.0);
  standard_surface->addInput("base_color", "color3")
      ->setValue(MaterialX::Color3(material->r, material->g, material->b));

  surfacematerial->addInput(standard_surface->getType(), standard_surface->getType())
      ->setNodeName(standard_surface->getName());
}

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph, Material *material)
{
  MaterialX::DocumentPtr doc = MaterialX::createDocument();
  if (material->use_nodes) {
    export_nodegraph(doc, depsgraph, material);
  }
  else {
    create_standard_surface(doc, material);
  }
  return doc;
}

}  // namespace blender::nodes::materialx
