/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_modifier_types.h"

#include "BKE_context.h"
#include "BKE_idprop.hh"
#include "BKE_lib_id.h"
#include "BKE_node.h"
#include "BKE_object.h"

#include "RNA_prototypes.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"

#include "WM_api.h"

#include "ED_object.h"
#include "ED_screen.h"

#include "NOD_socket.h"

#include "MOD_nodes.h"

#include "object_intern.h"

/* ------------------------------------------------------------------- */
/** \name Create Geometry Nodes Wrapper Object
 * \{ */

namespace blender::ed::object {

static eNodeSocketDatatype custom_data_type_to_socket_type(const eCustomDataType type)
{
  switch (type) {
    case CD_PROP_FLOAT:
      return SOCK_FLOAT;
    case CD_PROP_INT32:
      return SOCK_INT;
    case CD_PROP_FLOAT3:
      return SOCK_VECTOR;
    case CD_PROP_BOOL:
      return SOCK_BOOLEAN;
    case CD_PROP_COLOR:
      return SOCK_RGBA;
    default:
      BLI_assert_unreachable();
      return SOCK_FLOAT;
  }
}

static eCustomDataType socket_type_to_custom_data_type(const eNodeSocketDatatype socket_type)
{
  switch (socket_type) {
    case SOCK_FLOAT:
      return CD_PROP_FLOAT;
    case SOCK_INT:
      return CD_PROP_INT32;
    case SOCK_VECTOR:
      return CD_PROP_FLOAT3;
    case SOCK_BOOLEAN:
      return CD_PROP_BOOL;
    case SOCK_RGBA:
      return CD_PROP_COLOR;
    default:
      /* Fallback. */
      return CD_AUTO_FROM_NAME;
  }
}

static bool create_group_wrapper_poll(bContext *C)
{
  return edit_modifier_poll_generic(C, &RNA_NodesModifier, 0, true, false);
}

static int create_group_wrapper_exec(bContext *C, wmOperator *op)
{
  // if context.area.type == 'PROPERTIES':
  //     modifier = context.modifier
  // else:
  //     modifier = context.object.modifiers.active

  // if not modifier:
  //     return {'CANCELLED'}
  // old_group = modifier.node_group
  // if not old_group:
  //     return {'CANCELLED'}

  // group = geometry_node_group_empty_new(add_link=False)
  // new_group_node = group.nodes.new("GeometryNodeGroup")
  // new_group_node.node_tree = old_group

  // group_input_node = group.nodes["Group Input"]
  // group_output_node = group.nodes["Group Output"]

  // for input in old_group.inputs:
  //     group.inputs.new()
  //     if hasattr(input, "default_value"):
  //         new_group_node.inputs[input.identifier].default_value = modifier[input.identifier]
  //     else:
  //         group.links.new(group_input_node.outputs[input.identifier],
  //         new_group_node.inputs[input.identifier])

  // for output in old_group.outputs:
  //     group.links.new(new_group_node.outputs[input.identifier],
  //     group_output_node.inputs[input.identifier])

  // modifier.node_group = group

  // return {'FINISHED'}

  Main *bmain = CTX_data_main(C);
  Object *object = CTX_data_active_object(C);
  PointerRNA ptr = CTX_data_pointer_get_type(C, "modifier", &RNA_NodesModifier);
  NodesModifierData *nmd = static_cast<NodesModifierData *>(ptr.data);
  if (!nmd) {
    return OPERATOR_CANCELLED;
  }
  const bNodeTree *old_group = nmd->node_group;
  if (!old_group) {
    return OPERATOR_CANCELLED;
  }

  bNodeTree *new_group = reinterpret_cast<bNodeTree *>(
      BKE_id_new(bmain, ID_NT, nmd->modifier.name));

  old_group->ensure_topology_cache();

  bNode *new_group_node = nodeAddNode(C, new_group, "GeometryNodeGroup");
  new_group->id = nmd->node_group;
  nodes::update_node_declaration_and_sockets(*new_group, *new_group_node);

  for (const bNodeSocket *input : old_group->interface_inputs()) {
    bNodeSocket *group_node_socket = nodeFindSocket(new_group_node, SOCK_IN, input->identifier);
    ntreeAddSocketInterfaceFromSocket(new_group, new_group_node, group_node_socket);
  }
  for (const bNodeSocket *output : old_group->interface_outputs()) {
    bNodeSocket *group_node_socket = nodeFindSocket(new_group_node, SOCK_IN, output->identifier);
    ntreeAddSocketInterfaceFromSocket(new_group, new_group_node, group_node_socket);
  }

  bNode *new_group_input_node = nodeAddNode(C, new_group, "NodeGroupInput");
  bNode *new_group_output_node = nodeAddNode(C, new_group, "NodeGroupInput");

  for (const bNodeSocket *input : old_group->interface_inputs()) {
    if (const std::optional<StringRef> attribute = MOD_nodes_property_try_get_attribute(
            *nmd, input->identifier)) {
      bNode *named_attribute_node = nodeAddNode(C, new_group, "NodeGeometryInputNamedAttribute");
      const auto &storage = *static_cast<const NodeGeometryInputNamedAttribute *>(
          named_attribute_node->storage);

      storage.data_type = socket_type_to_custom_data_type(input->type);
      named_attribute_node->typeinfo->updatefunc(new_group, named_attribute_node);

      bNodeSocket *name_socket = nodeFindSocket(named_attribute_node, SOCK_IN, "Name");
      BLI_strncpy(name_socket->default_value_typed<bNodeSocketValueString>()->value,
                  attribute->data(),
                  attribute->size());
      nodeAddLink(new_group,
                  named_attribute_node,
                  bke::node_find_enabled_output_socket(*named_attribute_node, "Attribute"),
                  new_group_node,
                  nodeFindSocket(new_group_node, SOCK_IN, input->identifier));
    }
    else {
      IDProperty *prop = IDP_GetPropertyFromGroup(nmd->settings.properties, input->identifier);
      switch (eNodeSocketDatatype(input->type)) {
        case SOCK_FLOAT:
          break;
        case SOCK_VECTOR:
          break;
        case SOCK_RGBA:
          break;
        case SOCK_SHADER:
          break;
        case SOCK_BOOLEAN:
          break;
        case SOCK_INT:
          break;
        case SOCK_STRING:
          break;
        case SOCK_OBJECT:
          break;
        case SOCK_IMAGE:
          break;
        case SOCK_GEOMETRY:
          break;
        case SOCK_COLLECTION:
          break;
        case SOCK_TEXTURE:
          break;
        case SOCK_MATERIAL:
          break;
      }
    }
  }

  for (const bNodeSocket *output : old_group->interface_outputs()) {
  }

  DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);

  return OPERATOR_FINISHED;
}

}  // namespace blender::ed::object

void OBJECT_OT_new_geometry_node_group_wrapper(wmOperatorType *ot)
{
  ot->name = "Create Wrapper Geometry Group";
  ot->description = "Create a new node group that contains the modifier's node group as a node";
  ot->idname = "OBJECT_OT_new_geometry_node_group_wrapper";

  ot->exec = blender::ed::object::create_group_wrapper_exec;
  ot->poll = blender::ed::object::create_group_wrapper_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  edit_modifier_properties(ot);
}

/** \} */
