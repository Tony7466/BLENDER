/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "output_material.h"

namespace blender::nodes::materialx {
OutputMaterialNodeParser::OutputMaterialNodeParser(MaterialX::GraphElement *graph,
                                                   const Depsgraph *depsgraph,
                                                   const Material *material,
                                                   const bNode *node)
    : NodeParser(graph, depsgraph, material, node, nullptr)
{
}

NodeItem OutputMaterialNodeParser::compute()
{
  return empty();
}

NodeItem OutputMaterialNodeParser::compute(const std::string &socket_name)
{
  NodeItem surface = empty();
  if (node_) {
    surface = get_input_link(socket_name);
  }
  else {
    surface = create_node("standard_surface", "surfaceshader");
    surface.set_input("base_color", value(MaterialX::Color3(1.0f, 0.0f, 1.0f)));
  }
  NodeItem res = create_node("surfacematerial", "material");
  res.node->setName(node_name(node_, nullptr));
  res.set_input("surfaceshader", surface);
  return res;
}

NodeItem OutputMaterialNodeParser::compute_default()
{
  NodeItem surface = create_node("standard_surface", "surfaceshader");
  surface.set_input("base_color",
                    value(MaterialX::Color3(material_->r, material_->g, material_->b)));
  surface.set_input("diffuse_roughness", value(material_->roughness));
  if (material_->metallic > 0.0f) {
    surface.set_input("metalness", value(material_->metallic));
  }
  if (material_->spec) {
    surface.set_input("specular", value(material_->spec));
    surface.set_input("specular_color", value(material_->spec));
    surface.set_input("specular_roughness", value(material_->roughness));
  }

  NodeItem res = create_node("surfacematerial", "material");
  res.node->setName("Material_Default");
  res.set_input("surfaceshader", surface);
  return res;
}

}  // namespace blender::nodes::materialx
