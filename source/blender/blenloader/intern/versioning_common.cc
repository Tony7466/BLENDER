/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blenloader
 */
/* allow readfile to use deprecated functionality */
#define DNA_DEPRECATED_ALLOW

#include <cstring>

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"

#include "BLI_listbase.h"
#include "BLI_map.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_string.h"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "NOD_socket.h"

#include "BKE_animsys.h"
#include "BKE_lib_id.h"
#include "BKE_main.h"
#include "BKE_main_namemap.h"
#include "BKE_modifier.h"
#include "BKE_node.h"
#include "BKE_node_runtime.hh"

#include "MEM_guardedalloc.h"

#include "versioning_common.h"

using blender::Map;
using blender::StringRef;

ARegion *do_versions_add_region_if_not_found(ListBase *regionbase,
                                             int region_type,
                                             const char *allocname,
                                             int link_after_region_type)
{
  ARegion *link_after_region = nullptr;
  LISTBASE_FOREACH (ARegion *, region, regionbase) {
    if (region->regiontype == region_type) {
      return nullptr;
    }
    if (region->regiontype == link_after_region_type) {
      link_after_region = region;
    }
  }

  ARegion *new_region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), allocname));
  new_region->regiontype = region_type;
  BLI_insertlinkafter(regionbase, link_after_region, new_region);
  return new_region;
}

ARegion *do_versions_ensure_region(ListBase *regionbase,
                                   int region_type,
                                   const char *allocname,
                                   int link_after_region_type)
{
  ARegion *link_after_region = nullptr;
  LISTBASE_FOREACH (ARegion *, region, regionbase) {
    if (region->regiontype == region_type) {
      return region;
    }
    if (region->regiontype == link_after_region_type) {
      link_after_region = region;
    }
  }

  ARegion *new_region = MEM_cnew<ARegion>(allocname);
  new_region->regiontype = region_type;
  BLI_insertlinkafter(regionbase, link_after_region, new_region);
  return new_region;
}

ID *do_versions_rename_id(Main *bmain,
                          const short id_type,
                          const char *name_src,
                          const char *name_dst)
{
  /* We can ignore libraries */
  ListBase *lb = which_libbase(bmain, id_type);
  ID *id = nullptr;
  LISTBASE_FOREACH (ID *, idtest, lb) {
    if (!ID_IS_LINKED(idtest)) {
      if (STREQ(idtest->name + 2, name_src)) {
        id = idtest;
      }
      if (STREQ(idtest->name + 2, name_dst)) {
        return nullptr;
      }
    }
  }
  if (id != nullptr) {
    BKE_main_namemap_remove_name(bmain, id, id->name + 2);
    BLI_strncpy(id->name + 2, name_dst, sizeof(id->name) - 2);
    /* We know it's unique, this just sorts. */
    BLI_libblock_ensure_unique_name(bmain, id->name);
  }
  return id;
}

static void change_node_socket_name(ListBase *sockets, const char *old_name, const char *new_name)
{
  LISTBASE_FOREACH (bNodeSocket *, socket, sockets) {
    if (STREQ(socket->name, old_name)) {
      BLI_strncpy(socket->name, new_name, sizeof(socket->name));
    }
    if (STREQ(socket->identifier, old_name)) {
      BLI_strncpy(socket->identifier, new_name, sizeof(socket->name));
    }
  }
}

void version_node_socket_id_delim(bNodeSocket *socket)
{
  StringRef name = socket->name;
  StringRef id = socket->identifier;

  if (!id.startswith(name)) {
    /* We only need to affect the case where the identifier starts with the name. */
    return;
  }

  StringRef id_number = id.drop_known_prefix(name);
  if (id_number.is_empty()) {
    /* The name was already unique, and didn't need numbers at the end for the id. */
    return;
  }

  if (id_number.startswith(".")) {
    socket->identifier[name.size()] = '_';
  }
}

void version_node_socket_name(bNodeTree *ntree,
                              const int node_type,
                              const char *old_name,
                              const char *new_name)
{
  for (bNode *node : ntree->all_nodes()) {
    if (node->type == node_type) {
      change_node_socket_name(&node->inputs, old_name, new_name);
      change_node_socket_name(&node->outputs, old_name, new_name);
    }
  }
}

void version_node_input_socket_name(bNodeTree *ntree,
                                    const int node_type,
                                    const char *old_name,
                                    const char *new_name)
{
  for (bNode *node : ntree->all_nodes()) {
    if (node->type == node_type) {
      change_node_socket_name(&node->inputs, old_name, new_name);
    }
  }
}

void version_node_output_socket_name(bNodeTree *ntree,
                                     const int node_type,
                                     const char *old_name,
                                     const char *new_name)
{
  for (bNode *node : ntree->all_nodes()) {
    if (node->type == node_type) {
      change_node_socket_name(&node->outputs, old_name, new_name);
    }
  }
}

bNodeSocket *version_node_add_socket_if_not_exist(bNodeTree *ntree,
                                                  bNode *node,
                                                  eNodeSocketInOut in_out,
                                                  int type,
                                                  int subtype,
                                                  const char *identifier,
                                                  const char *name)
{
  bNodeSocket *sock = nodeFindSocket(node, in_out, identifier);
  if (sock != nullptr) {
    return sock;
  }
  return nodeAddStaticSocket(ntree, node, in_out, type, subtype, identifier, name);
}

void version_node_id(bNodeTree *ntree, const int node_type, const char *new_name)
{
  for (bNode *node : ntree->all_nodes()) {
    if (node->type == node_type) {
      if (!STREQ(node->idname, new_name)) {
        strcpy(node->idname, new_name);
      }
    }
  }
}

void version_node_socket_index_animdata(Main *bmain,
                                        const int node_tree_type,
                                        const int node_type,
                                        const int socket_index_orig,
                                        const int socket_index_offset,
                                        const int total_number_of_sockets)
{

  /* The for loop for the input ids is at the top level otherwise we lose the animation
   * keyframe data. Not sure what causes that, so I (Sybren) moved the code here from
   * versioning_290.cc as-is (structure-wise). */
  for (int input_index = total_number_of_sockets - 1; input_index >= socket_index_orig;
       input_index--) {
    FOREACH_NODETREE_BEGIN (bmain, ntree, owner_id) {
      if (ntree->type != node_tree_type) {
        continue;
      }

      for (bNode *node : ntree->all_nodes()) {
        if (node->type != node_type) {
          continue;
        }

        const size_t node_name_length = strlen(node->name);
        const size_t node_name_escaped_max_length = (node_name_length * 2);
        char *node_name_escaped = (char *)MEM_mallocN(node_name_escaped_max_length + 1,
                                                      "escaped name");
        BLI_str_escape(node_name_escaped, node->name, node_name_escaped_max_length);
        char *rna_path_prefix = BLI_sprintfN("nodes[\"%s\"].inputs", node_name_escaped);

        const int new_index = input_index + socket_index_offset;
        BKE_animdata_fix_paths_rename_all_ex(
            bmain, owner_id, rna_path_prefix, nullptr, nullptr, input_index, new_index, false);
        MEM_freeN(rna_path_prefix);
        MEM_freeN(node_name_escaped);
      }
    }
    FOREACH_NODETREE_END;
  }
}

void version_socket_update_is_used(bNodeTree *ntree)
{
  for (bNode *node : ntree->all_nodes()) {
    LISTBASE_FOREACH (bNodeSocket *, socket, &node->inputs) {
      socket->flag &= ~SOCK_IS_LINKED;
    }
    LISTBASE_FOREACH (bNodeSocket *, socket, &node->outputs) {
      socket->flag &= ~SOCK_IS_LINKED;
    }
  }
  LISTBASE_FOREACH (bNodeLink *, link, &ntree->links) {
    link->fromsock->flag |= SOCK_IS_LINKED;
    link->tosock->flag |= SOCK_IS_LINKED;
  }
}

ARegion *do_versions_add_region(int regiontype, const char *name)
{
  ARegion *region = (ARegion *)MEM_callocN(sizeof(ARegion), name);
  region->regiontype = regiontype;
  return region;
}

void node_tree_relink_with_socket_id_map(bNodeTree &ntree,
                                         bNode &old_node,
                                         bNode &new_node,
                                         const Map<std::string, std::string> &map)
{
  LISTBASE_FOREACH_MUTABLE (bNodeLink *, link, &ntree.links) {
    if (link->tonode == &old_node) {
      bNodeSocket *old_socket = link->tosock;
      if (const std::string *new_identifier = map.lookup_ptr_as(old_socket->identifier)) {
        bNodeSocket *new_socket = nodeFindSocket(&new_node, SOCK_IN, new_identifier->c_str());
        link->tonode = &new_node;
        link->tosock = new_socket;
        old_socket->link = nullptr;
      }
    }
    if (link->fromnode == &old_node) {
      bNodeSocket *old_socket = link->fromsock;
      if (const std::string *new_identifier = map.lookup_ptr_as(old_socket->identifier)) {
        bNodeSocket *new_socket = nodeFindSocket(&new_node, SOCK_OUT, new_identifier->c_str());
        link->fromnode = &new_node;
        link->fromsock = new_socket;
        old_socket->link = nullptr;
      }
    }
  }
}

namespace replace_legacy_instances {

using namespace blender;

static bNodeSocket &node_input_by_name(const StringRefNull name, bNode *node)
{
  LISTBASE_FOREACH (bNodeSocket *, socket, &node->inputs) {
    if (!socket->is_available()) {
      continue;
    }
    if (StringRefNull(socket->name) == name) {
      return *socket;
    }
  }
  BLI_assert_unreachable();
  return *reinterpret_cast<bNodeSocket *>(node->inputs.first);
}

static bNodeSocket &node_output_by_name(const StringRefNull name, bNode *node)
{
  LISTBASE_FOREACH (bNodeSocket *, socket, &node->outputs) {
    if (!socket->is_available()) {
      continue;
    }
    if (StringRefNull(socket->name) == name) {
      return *socket;
    }
  }
  BLI_assert_unreachable();
  return *reinterpret_cast<bNodeSocket *>(node->outputs.first);
}

struct RegularNodeTrees {
  bNodeTree *instances_on_points = nullptr;
  bNodeTree *instances_on_faces = nullptr;

  bNodeTree *view_geometry = nullptr;

  bNodeTree *face_scale = nullptr;
  bNodeTree *face_aling = nullptr;
};

static bNodeTree *builtin_instancing_node_group(Main *bmain,
                                                const bool /*on_vertices*/,
                                                RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.instances_on_points != nullptr) {
    return cached_node_trees.instances_on_points;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, "Legacy Point Instances", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Instance");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Instancer");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Aling to Vertex Normal");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Instances");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  bNode *normal_input = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_NORMAL);

  bNode *aling_y_z = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ALIGN_EULER_TO_VECTOR);
  aling_y_z->custom1 = FN_NODE_ALIGN_EULER_TO_VECTOR_AXIS_Y;
  aling_y_z->custom2 = FN_NODE_ALIGN_EULER_TO_VECTOR_PIVOT_AXIS_Z;
  connect(normal_input, "Normal", aling_y_z, "Vector");

  bNode *aling_y_x = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ALIGN_EULER_TO_VECTOR);
  aling_y_x->custom1 = FN_NODE_ALIGN_EULER_TO_VECTOR_AXIS_Y;
  aling_y_x->custom2 = FN_NODE_ALIGN_EULER_TO_VECTOR_PIVOT_AXIS_X;
  connect(aling_y_z, "Rotation", aling_y_x, "Rotation");
  connect(normal_input, "Normal", aling_y_x, "Vector");

  bNode *aling_selection_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *switch_vector = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SWITCH);
  NodeSwitch &switch_data = *reinterpret_cast<NodeSwitch *>(switch_vector->storage);
  switch_data.input_type = SOCK_VECTOR;
  switch_vector->typeinfo->updatefunc(node_tree, switch_vector);
  connect(aling_selection_in, "Aling to Vertex Normal", switch_vector, "Switch");
  connect(aling_y_x, "Rotation", switch_vector, "True");

  bNode *instance_on_point = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INSTANCE_ON_POINTS);
  connect(switch_vector, "Output", instance_on_point, "Rotation");

  bNode *parent_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(parent_in, "Instancer", instance_on_point, "Points");

  bNode *geometry_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(geometry_in, "Instance", instance_on_point, "Instance");

  bNode *instances_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(instance_on_point, "Instances", instances_out, "Instances");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.instances_on_points = node_tree;
  return node_tree;
}

static bNodeTree *builtin_view_geometry_group(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.view_geometry != nullptr) {
    return cached_node_trees.view_geometry;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, "ViewGeometry", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Geometry");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Viewport");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Render");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Geometry");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  bNode *is_viewport = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_IS_VIEWPORT);

  bNode *bool_invert = nodeAddStaticNode(nullptr, node_tree, FN_NODE_BOOLEAN_MATH);
  bool_invert->custom1 = NODE_BOOLEAN_MATH_NOT;
  bool_invert->typeinfo->updatefunc(node_tree, bool_invert);
  connect(is_viewport, "Is Viewport", bool_invert, "Boolean");

  bNode *viewport_render_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *viewport_render_switch = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SWITCH);
  NodeSwitch &switch_data_vr = *reinterpret_cast<NodeSwitch *>(viewport_render_switch->storage);
  switch_data_vr.input_type = SOCK_BOOLEAN;
  viewport_render_switch->typeinfo->updatefunc(node_tree, viewport_render_switch);
  connect(bool_invert, "Boolean", viewport_render_switch, "Switch");
  connect(viewport_render_in, "Viewport", viewport_render_switch, "False");
  connect(viewport_render_in, "Render", viewport_render_switch, "True");

  bNode *geometry_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *geometry_switch = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SWITCH);
  NodeSwitch &switch_data_g = *reinterpret_cast<NodeSwitch *>(geometry_switch->storage);
  switch_data_g.input_type = SOCK_GEOMETRY;
  geometry_switch->typeinfo->updatefunc(node_tree, geometry_switch);
  connect(viewport_render_switch, "Output", geometry_switch, "Switch");
  connect(geometry_in, "Geometry", geometry_switch, "True");

  bNode *geometry_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(geometry_switch, "Output", geometry_out, "Geometry");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.view_geometry = node_tree;
  return node_tree;
}

static bNodeTree *childrens_combine_node_group(Main *bmain,
                                               bNodeTree *instancer_node_group,
                                               const Span<Object *> objects,
                                               const StringRefNull name,
                                               RegularNodeTrees &cached_node_trees)
{
  const std::string tree_name = std::string(name) + std::string("_childrens");
  bNodeTree *node_tree = ntreeAddTree(bmain, tree_name.c_str(), "GeometryNodeTree");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Geometry");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Viewport");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Render");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Aling to Vertex Normal");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Geometry");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  const auto add_node_group = [node_tree](bNodeTree *tree) -> bNode * {
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
  };

  bNode *join_geometrys = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_JOIN_GEOMETRY);

  for (Object *object : objects) {
    bNode *object_info = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_OBJECT_INFO);
    bNodeSocket &object_input = node_input_by_name("Object", object_info);
    bNodeSocket &as_instance = node_input_by_name("As Instance", object_info);
    object_input.default_value_typed<bNodeSocketValueObject>()->value = object;
    as_instance.default_value_typed<bNodeSocketValueBoolean>()->value = true;
    connect(object_info, "Geometry", join_geometrys, "Geometry");
  }

  bNode *instansing_group = add_node_group(instancer_node_group);
  connect(join_geometrys, "Geometry", instansing_group, "Instance");

  bNode *parent_aling_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(parent_aling_input, "Geometry", instansing_group, "Instancer");
  connect(
      parent_aling_input, "Aling to Vertex Normal", instansing_group, "Aling to Vertex Normal");

  bNode *view_switch_group = add_node_group(builtin_view_geometry_group(bmain, cached_node_trees));
  bNode *geometry_viewport_render_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(geometry_viewport_render_input, "Geometry", view_switch_group, "Geometry");
  connect(geometry_viewport_render_input, "Viewport", view_switch_group, "Viewport");
  connect(geometry_viewport_render_input, "Render", view_switch_group, "Render");

  bNode *join_parent = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_JOIN_GEOMETRY);
  connect(instansing_group, "Instances", join_parent, "Geometry");
  connect(view_switch_group, "Geometry", join_parent, "Geometry");

  bNode *group_output = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(join_parent, "Geometry", group_output, "Geometry");
  // version_socket_update_is_used(node_tree); ??
  return node_tree;
}

static void object_push_instances_modifier(Main * /*bmain*/, Object *object, bNodeTree *node_tree)
{
  ModifierData *new_modifier = BKE_modifier_new(eModifierType_Nodes);
  NodesModifierData &node_modifier = *reinterpret_cast<NodesModifierData *>(new_modifier);
  node_modifier.node_group = node_tree;
  BLI_addtail(&object->modifiers, new_modifier);
}

}  //  namespace replace_legacy_instances

void remove_legacy_instances_on(Main *bmain, ListBase &lb_objects)
{
  using namespace blender;

  MultiValueMap<Object *, Object *> parents_map;

  LISTBASE_FOREACH (Object *, object, &lb_objects) {
    Object *emitter = object->parent;
    if (emitter == nullptr) {
      continue;
    }
    /* Object, that may have children, but that not an instance emitter. */
    if (emitter->transflag == 0) {
      continue;
    }
    parents_map.add(emitter, object);
  }

  if (parents_map.is_empty()) {
    return;
  }

  /* Not a static. Pointers are valid only during one project session. */
  replace_legacy_instances::RegularNodeTrees cached_node_trees;

  for (auto &&[parent, objects] : parents_map.items()) {

    /* Or on faces (OB_DUPLIFACES). */
    const bool on_vertices = (parent->transflag & OB_DUPLIVERTS) != 0;

    const StringRefNull parent_name(parent->id.name + 2);

    using namespace replace_legacy_instances;
    bNodeTree *instancer_node_group = builtin_instancing_node_group(
        bmain, on_vertices, cached_node_trees);
    bNodeTree *childrens_combine_node = childrens_combine_node_group(
        bmain, instancer_node_group, objects, parent_name, cached_node_trees);
    object_push_instances_modifier(bmain, parent, childrens_combine_node);

    parent->transflag = 0;
  }

  printf("HELLO!\n");
}