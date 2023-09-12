/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "material.h"
#include "node_parser.h"

#include <MaterialXFormat/XmlIo.h>

#include "DEG_depsgraph.h"

#include "DNA_material_types.h"

#include "NOD_shader.h"

namespace blender::nodes::materialx {

class DefaultMaterialNodeParser : public NodeParser {
 public:
  using NodeParser::NodeParser;

  NodeItem compute() override
  {
    NodeItem surface = create_node("standard_surface", NodeItem::Type::SurfaceShader);
    surface.set_input("base_color",
                      val(MaterialX::Color3(material_->r, material_->g, material_->b)));
    surface.set_input("diffuse_roughness", val(material_->roughness));
    if (material_->metallic > 0.0f) {
      surface.set_input("metalness", val(material_->metallic));
    }
    if (material_->spec) {
      surface.set_input("specular", val(material_->spec));
      surface.set_input("specular_color", val(material_->spec));
      surface.set_input("specular_roughness", val(material_->roughness));
    }

    NodeItem res = create_node("surfacematerial", NodeItem::Type::Material);
    res.node->setName("Material_Default");
    res.set_input("surfaceshader", surface);
    return res;
  }

  NodeItem compute_error()
  {
    NodeItem surface = create_node("standard_surface", NodeItem::Type::SurfaceShader);
    surface.set_input("base_color", val(MaterialX::Color3(1.0f, 0.0f, 1.0f)));

    NodeItem res = create_node("surfacematerial", NodeItem::Type::Material);
    res.node->setName("Material_Error");
    res.set_input("surfaceshader", surface);
    return res;
  }
};

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph, Material *material)
{
  CLOG_INFO(LOG_MATERIALX_SHADER, 0, "Material: %s", material->id.name);

  MaterialX::DocumentPtr doc = MaterialX::createDocument();
  if (material->use_nodes) {
    material->nodetree->ensure_topology_cache();
    bNode *output_node = ntreeShaderOutputNode(material->nodetree, SHD_OUTPUT_ALL);
    if (output_node) {
      NodeParserData data = {
          doc.get(), depsgraph, material, NodeItem::Type::Material, NodeItem(doc.get())};
      output_node->typeinfo->materialx_fn(&data, output_node, nullptr);
    }
    else {
      DefaultMaterialNodeParser(
          doc.get(), depsgraph, material, nullptr, nullptr, NodeItem::Type::Material)
          .compute_error();
    }
  }
  else {
    DefaultMaterialNodeParser(
        doc.get(), depsgraph, material, nullptr, nullptr, NodeItem::Type::Material)
        .compute();
  }

  CLOG_INFO(LOG_MATERIALX_SHADER,
            1,
            "Material: %s\n%s",
            material->id.name,
            MaterialX::writeToXmlString(doc).c_str());
  return doc;
}

}  // namespace blender::nodes::materialx
