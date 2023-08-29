/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "material.h"
#include "nodes/node_parser.h"

#include <MaterialXCore/Node.h>
#include <MaterialXFormat/XmlIo.h>

#include "NOD_shader.h"

namespace blender::nodes::materialx {

static void export_nodegraph(MaterialX::GraphElement *graph,
                             Depsgraph *depsgraph,
                             Material *material)
{
  material->nodetree->ensure_topology_cache();

  bNode *output_node = ntreeShaderOutputNode(material->nodetree, SHD_OUTPUT_ALL);
  OutputMaterialNodeParser parser(graph, depsgraph, material, output_node);
  parser.compute();
}

static void create_standard_surface(MaterialX::GraphElement *graph, Material *material)
{
  MaterialX::NodePtr standard_surface = graph->addNode(
      "standard_surface", MaterialX::EMPTY_STRING, "surfaceshader");

  standard_surface->addInput("base", "float")->setValue(1.0);
  standard_surface->addInput("base_color", "color3")
      ->setValue(MaterialX::Color3(material->r, material->g, material->b));

  MaterialX::NodePtr surfacematerial = graph->addNode(
      "surfacematerial", MaterialX::EMPTY_STRING, "material");
  surfacematerial->addInput(standard_surface->getType(), standard_surface->getType())
      ->setNodeName(standard_surface->getName());
}

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph, Material *material)
{
  MaterialX::DocumentPtr doc = MaterialX::createDocument();
  if (material->use_nodes) {
    export_nodegraph(doc.get(), depsgraph, material);
  }
  else {
    create_standard_surface(doc.get(), material);
  }
  std::string str = MaterialX::writeToXmlString(doc);
  printf("\nMaterial: %s\n%s\n", material->id.name, str.c_str());
  return doc;
}

}  // namespace blender::nodes::materialx
