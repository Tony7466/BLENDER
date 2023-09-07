/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "material.h"
#include "nodes/output_material.h"

#include <MaterialXFormat/XmlIo.h>

#include "DEG_depsgraph.h"

#include "DNA_material_types.h"

#include "NOD_shader.h"

namespace blender::nodes::materialx {

CLG_LOGREF_DECLARE_GLOBAL(LOG_MATERIALX_SHADER, "materialx.shader");

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph, Material *material)
{
  CLOG_INFO(LOG_MATERIALX_SHADER, 0, "Material: %s", material->id.name);

  MaterialX::DocumentPtr doc = MaterialX::createDocument();
  if (material->use_nodes) {
    material->nodetree->ensure_topology_cache();
    bNode *output_node = ntreeShaderOutputNode(material->nodetree, SHD_OUTPUT_ALL);
    OutputMaterialNodeParser(doc.get(), depsgraph, material, output_node).compute_full();
  }
  else {
    OutputMaterialNodeParser(doc.get(), depsgraph, material, nullptr).compute_default();
  }

  CLOG_INFO(LOG_MATERIALX_SHADER,
            1,
            "Material: %s\n%s",
            material->id.name,
            MaterialX::writeToXmlString(doc).c_str());
  return doc;
}

}  // namespace blender::nodes::materialx
