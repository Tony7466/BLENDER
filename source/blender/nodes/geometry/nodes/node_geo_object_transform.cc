/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "DNA_object_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_object_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object").hide_label();
  b.add_output<decl::Matrix>("Transform");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Object *object = params.get_input<Object *>("Object");
  if (!object) {
    params.set_default_remaining_outputs();
    return;
  }
  const Object *self_object = params.self_object();
  params.set_output("Transform",
                    float4x4(self_object->world_to_object) * float4x4(object->object_to_world));
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_OBJECT_TRANSFORM, "Object Transform", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_object_transform_cc
