/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "output_material.h"

namespace blender::nodes::materialx {
OutputMaterialNodeParser::OutputMaterialNodeParser(MaterialX::GraphElement *graph,
                                                   const Depsgraph *depsgraph,
                                                   const Material *material,
                                                   const bNode *node)
    : ShaderNodeParser(graph, depsgraph, material, node, nullptr, NodeItem::Type::Material)
{
}

NodeItem OutputMaterialNodeParser::compute()
{
  NodeItem surface = empty();
  if (node_) {
    NodeItem bsdf = get_input_shader("Surface", NodeItem::Type::BSDF);
    NodeItem edf = get_input_shader("Surface", NodeItem::Type::EDF);
    if (bsdf || edf) {
      surface = create_node("surface", "surfaceshader");
      if (bsdf) {
        surface.set_input("bsdf", bsdf);
      }
      if (edf) {
        surface.set_input("edf", edf);
      }
    }
    else {
      surface = get_input_shader("Surface", NodeItem::Type::SurfaceShader);
    }
  }
  else {
    surface = create_node("standard_surface", "surfaceshader");
    surface.set_input("base_color", value(MaterialX::Color3(1.0f, 0.0f, 1.0f)));
  }
  NodeItem res = create_node("surfacematerial", "material");
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

std::string OutputMaterialNodeParser::node_name()
{
  return NodeParser::node_name();
}

}  // namespace blender::nodes::materialx
