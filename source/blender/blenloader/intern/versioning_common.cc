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

#include "BLT_translation.h"

#include "NOD_socket.h"

#include "MOD_nodes.h"

#include "DEG_depsgraph.h"

#include "BKE_animsys.h"
#include "BKE_attribute.h"
#include "BKE_geometry_set.h"
#include "BKE_idprop.hh"
#include "BKE_lib_id.h"
#include "BKE_main.h"
#include "BKE_main_namemap.h"
#include "BKE_modifier.h"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_object.h"

#include "MEM_guardedalloc.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

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
      STRNCPY(socket->name, new_name);
    }
    if (STREQ(socket->identifier, old_name)) {
      STRNCPY(socket->identifier, new_name);
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
       input_index--)
  {
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

static blender::Vector<bNodeLink *> find_connected_links(bNodeTree *ntree, bNodeSocket *in_socket)
{
  blender::Vector<bNodeLink *> links;
  LISTBASE_FOREACH (bNodeLink *, link, &ntree->links) {
    if (link->tosock == in_socket) {
      links.append(link);
    }
  }
  return links;
}

void add_realize_instances_before_socket(bNodeTree *ntree,
                                         bNode *node,
                                         bNodeSocket *geometry_socket)
{
  BLI_assert(geometry_socket->type == SOCK_GEOMETRY);
  blender::Vector<bNodeLink *> links = find_connected_links(ntree, geometry_socket);
  for (bNodeLink *link : links) {
    /* If the realize instances node is already before this socket, no need to continue. */
    if (link->fromnode->type == GEO_NODE_REALIZE_INSTANCES) {
      return;
    }

    bNode *realize_node = nodeAddStaticNode(nullptr, ntree, GEO_NODE_REALIZE_INSTANCES);
    realize_node->parent = node->parent;
    realize_node->locx = node->locx - 100;
    realize_node->locy = node->locy;
    nodeAddLink(ntree,
                link->fromnode,
                link->fromsock,
                realize_node,
                static_cast<bNodeSocket *>(realize_node->inputs.first));
    link->fromnode = realize_node;
    link->fromsock = static_cast<bNodeSocket *>(realize_node->outputs.first);
  }
}

namespace replace_legacy_instances_imp {

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
  LISTBASE_FOREACH (bNodeSocket *, socket, &node->inputs) {
    if (!socket->is_available()) {
      continue;
    }
    if (StringRefNull(socket->identifier) == name) {
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
  LISTBASE_FOREACH (bNodeSocket *, socket, &node->outputs) {
    if (!socket->is_available()) {
      continue;
    }
    if (StringRefNull(socket->identifier) == name) {
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
  bNodeTree *invert_rotation = nullptr;

  bNodeTree *reset_instances_transform = nullptr;
  bNodeTree *sample_apply_instances_transform = nullptr;

  bNodeTree *face_scale = nullptr;
  bNodeTree *face_aling = nullptr;
  bNodeTree *first_face_tris_points = nullptr;
};

template<typename StorageT>
static void for_node_storage(bNodeTree *tree,
                             bNode *node,
                             const FunctionRef<void(StorageT &)> &func)
{
  func(*reinterpret_cast<StorageT *>(node->storage));

  if (node->typeinfo->updatefunc != nullptr) {
    node->typeinfo->updatefunc(tree, node);
  }
}

static bNodeTree *invert_rotation_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.invert_rotation != nullptr) {
    return cached_node_trees.invert_rotation;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, "InvertRotation", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketVector", "Rotation");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketVector", "Rotation");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    BLI_assert(node_out != node_in);
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  bNode *rotation_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *negative_rotation = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  negative_rotation->custom1 = NODE_VECTOR_MATH_SCALE;
  negative_rotation->typeinfo->updatefunc(node_tree, negative_rotation);
  node_input_by_name("Scale", negative_rotation)
      .default_value_typed<bNodeSocketValueFloat>()
      ->value = -1.0f;
  connect(rotation_in, "Rotation", negative_rotation, "Vector");

  bNode *separate_rotation = nodeAddStaticNode(nullptr, node_tree, SH_NODE_SEPXYZ);
  connect(negative_rotation, "Vector", separate_rotation, "Vector");

  bNode *combine_rotation_on_z = nodeAddStaticNode(nullptr, node_tree, SH_NODE_COMBXYZ);
  connect(separate_rotation, "Z", combine_rotation_on_z, "Z");
  bNode *rotate_z = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ROTATE_EULER);
  connect(combine_rotation_on_z, "Vector", rotate_z, "Rotate By");

  bNode *combine_rotation_on_y = nodeAddStaticNode(nullptr, node_tree, SH_NODE_COMBXYZ);
  connect(separate_rotation, "Y", combine_rotation_on_y, "Y");
  bNode *rotate_y = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ROTATE_EULER);
  connect(combine_rotation_on_y, "Vector", rotate_y, "Rotate By");

  bNode *combine_rotation_on_x = nodeAddStaticNode(nullptr, node_tree, SH_NODE_COMBXYZ);
  connect(separate_rotation, "X", combine_rotation_on_x, "X");
  bNode *rotate_x = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ROTATE_EULER);
  connect(combine_rotation_on_x, "Vector", rotate_x, "Rotate By");

  bNode *rotation_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);

  connect(rotate_z, "Rotation", rotate_y, "Rotation");
  connect(rotate_y, "Rotation", rotate_x, "Rotation");
  connect(rotate_x, "Rotation", rotation_out, "Rotation");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.invert_rotation = node_tree;
  return node_tree;
}

static bNodeTree *view_geometry_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
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
  for_node_storage<NodeSwitch>(node_tree, viewport_render_switch, [&](auto &storage) {
    storage.input_type = SOCK_BOOLEAN;
  });
  connect(bool_invert, "Boolean", viewport_render_switch, "Switch");
  connect(viewport_render_in, "Viewport", viewport_render_switch, "False");
  connect(viewport_render_in, "Render", viewport_render_switch, "True");

  bNode *geometry_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *geometry_switch = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SWITCH);
  for_node_storage<NodeSwitch>(
      node_tree, geometry_switch, [&](auto &storage) { storage.input_type = SOCK_GEOMETRY; });
  connect(viewport_render_switch, "Output", geometry_switch, "Switch");
  connect(geometry_in, "Geometry", geometry_switch, "True");

  bNode *geometry_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(geometry_switch, "Output", geometry_out, "Geometry");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.view_geometry = node_tree;
  return node_tree;
}

static bNode *join_objects_as_instances(const Span<Object *> objects, bNodeTree *node_tree)
{
  bNode *join_geometrys = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_JOIN_GEOMETRY);
  bNodeSocket &in = node_input_by_name("Geometry", join_geometrys);

  for (Object *object : objects) {
    bNode *object_info = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_OBJECT_INFO);
    for_node_storage<NodeGeometryObjectInfo>(node_tree, object_info, [&](auto &storage) {
      storage.transform_space = GEO_NODE_TRANSFORM_SPACE_RELATIVE;
    });
    bNodeSocket &object_input = node_input_by_name("Object", object_info);
    bNodeSocket &as_instance = node_input_by_name("As Instance", object_info);
    object_input.default_value_typed<bNodeSocketValueObject>()->value = object;
    as_instance.default_value_typed<bNodeSocketValueBoolean>()->value = true;

    bNodeSocket &out = node_output_by_name("Geometry", object_info);
    nodeAddLink(node_tree, object_info, &out, join_geometrys, &in);
  }

  return join_geometrys;
}

static bNodeTree *reset_instances_transform_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.reset_instances_transform != nullptr) {
    return cached_node_trees.reset_instances_transform;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, ".Reset Instances Trasform", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Instances");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Instances");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    BLI_assert(node_out != node_in);
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  const auto add_node_group =
      [&](const FunctionRef<bNodeTree *(Main * bmain, RegularNodeTrees & cached_node_trees)>
              tree_func) -> bNode * {
    bNodeTree *tree = tree_func(bmain, cached_node_trees);
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    id_us_plus(&tree->id);
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
  };

  const auto set_in = [](bNode *node, StringRefNull input_name, auto value) {
    if constexpr (std::is_same_v<decltype(value), bool>) {
      node_input_by_name(input_name, node).default_value_typed<bNodeSocketValueBoolean>()->value =
          value;
    }
    else if constexpr (std::is_same_v<decltype(value), float>) {
      node_input_by_name(input_name, node).default_value_typed<bNodeSocketValueFloat>()->value =
          value;
    }
    else if constexpr (std::is_same_v<decltype(value), float3>) {
      float(&values)[3] = node_input_by_name(input_name, node)
                              .default_value_typed<bNodeSocketValueVector>()
                              ->value;
      values[0] = value[0];
      values[1] = value[1];
      values[2] = value[2];
    }
    else {
      static_assert(true);
    }
  };

  bNode *instances_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *scale_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SCALE_INSTANCES);
  set_in(scale_instances, "Local Space", false);
  connect(instances_in, "Instances", scale_instances, "Instances");

  bNode *rotate_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_ROTATE_INSTANCES);
  set_in(rotate_instances, "Local Space", false);
  connect(scale_instances, "Instances", rotate_instances, "Instances");

  bNode *translate_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_TRANSLATE_INSTANCES);
  connect(rotate_instances, "Instances", translate_instances, "Instances");
  set_in(translate_instances, "Local Space", false);

  bNode *geometry_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(translate_instances, "Instances", geometry_out, "Instances");

  bNode *invert_size = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  invert_size->custom1 = NODE_VECTOR_MATH_DIVIDE;
  invert_size->typeinfo->updatefunc(node_tree, invert_size);
  set_in(invert_size, "Vector", float3{1.0f});
  connect(invert_size, "Vector", scale_instances, "Scale");

  bNode *invert_rotation = add_node_group(invert_rotation_tree);
  connect(invert_rotation, "Rotation", rotate_instances, "Rotation");

  bNode *invert_position = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  invert_position->custom1 = NODE_VECTOR_MATH_SCALE;
  invert_position->typeinfo->updatefunc(node_tree, invert_position);
  set_in(invert_position, "Scale", -1.0f);
  connect(invert_position, "Vector", translate_instances, "Translation");

  bNode *set_scale = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_INSTANCE_SCALE);
  connect(set_scale, "Scale", invert_size, "Vector_001");
  bNode *set_rotation = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_INSTANCE_ROTATION);
  connect(set_rotation, "Rotation", invert_rotation, "Rotation");
  bNode *set_position = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_POSITION);
  connect(set_position, "Position", invert_position, "Vector");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.reset_instances_transform = node_tree;
  return node_tree;
}

static bNodeTree *sample_apply_instances_transform_tree(Main *bmain,
                                                        RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.sample_apply_instances_transform != nullptr) {
    return cached_node_trees.sample_apply_instances_transform;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, ".Apply Instances Trasform", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Instances");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Transform Source");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketInt", "Instances Index");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Instances");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  const auto set_in = [](bNode *node, StringRefNull input_name, auto value) {
    if constexpr (std::is_same_v<decltype(value), bool>) {
      node_input_by_name(input_name, node).default_value_typed<bNodeSocketValueBoolean>()->value =
          value;
    }
    else {
      static_assert(true);
    }
  };

  bNode *instances_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *pivot_porition = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_POSITION);

  bNode *scale_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SCALE_INSTANCES);
  connect(instances_in, "Instances", scale_instances, "Instances");
  connect(pivot_porition, "Position", scale_instances, "Center");
  set_in(scale_instances, "Local Space", false);

  bNode *rotate_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_ROTATE_INSTANCES);
  connect(scale_instances, "Instances", rotate_instances, "Instances");
  connect(pivot_porition, "Position", rotate_instances, "Pivot Point");
  set_in(rotate_instances, "Local Space", false);

  bNode *translate_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_TRANSLATE_INSTANCES);
  connect(rotate_instances, "Instances", translate_instances, "Instances");
  set_in(translate_instances, "Local Space", false);

  bNode *geometry_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(translate_instances, "Instances", geometry_out, "Instances");

  bNode *source_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *indices_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *scale = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_INSTANCE_SCALE);
  bNode *sample_scale = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SAMPLE_INDEX);
  for_node_storage<NodeGeometrySampleIndex>(node_tree, sample_scale, [&](auto &storage) {
    storage.data_type = CD_PROP_FLOAT3;
    storage.domain = ATTR_DOMAIN_INSTANCE;
  });
  connect(source_in, "Transform Source", sample_scale, "Geometry");
  connect(scale, "Scale", sample_scale, "Value");
  connect(indices_in, "Instances Index", sample_scale, "Index");
  connect(sample_scale, "Value", scale_instances, "Scale");

  bNode *rotation = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_INSTANCE_ROTATION);
  bNode *sample_rotation = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SAMPLE_INDEX);
  for_node_storage<NodeGeometrySampleIndex>(node_tree, sample_rotation, [&](auto &storage) {
    storage.data_type = CD_PROP_FLOAT3;
    storage.domain = ATTR_DOMAIN_INSTANCE;
  });
  connect(source_in, "Transform Source", sample_rotation, "Geometry");
  connect(rotation, "Rotation", sample_rotation, "Value");
  connect(indices_in, "Instances Index", sample_rotation, "Index");
  connect(sample_rotation, "Value", rotate_instances, "Rotation");

  bNode *position = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_POSITION);
  bNode *sample_position = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SAMPLE_INDEX);
  for_node_storage<NodeGeometrySampleIndex>(node_tree, sample_position, [&](auto &storage) {
    storage.data_type = CD_PROP_FLOAT3;
    storage.domain = ATTR_DOMAIN_INSTANCE;
  });
  connect(source_in, "Transform Source", sample_position, "Geometry");
  connect(position, "Position", sample_position, "Value");
  connect(indices_in, "Instances Index", sample_position, "Index");
  connect(sample_position, "Value", translate_instances, "Translation");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.sample_apply_instances_transform = node_tree;
  return node_tree;
}

static bNodeTree *instances_on_points_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
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

  const auto add_node_group =
      [&](const FunctionRef<bNodeTree *(Main * bmain, RegularNodeTrees & cached_node_trees)>
              tree_func) -> bNode * {
    bNodeTree *tree = tree_func(bmain, cached_node_trees);
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    id_us_plus(&tree->id);
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
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
  for_node_storage<NodeSwitch>(
      node_tree, switch_vector, [&](auto &storage) { storage.input_type = SOCK_VECTOR; });
  connect(aling_selection_in, "Aling to Vertex Normal", switch_vector, "Switch");
  connect(aling_y_x, "Rotation", switch_vector, "True");

  bNode *instance_on_point = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INSTANCE_ON_POINTS);

  bNode *parent_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *capture_result = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_CAPTURE_ATTRIBUTE);
  for_node_storage<NodeGeometryAttributeCapture>(
      node_tree, capture_result, [&](auto &storage) { storage.data_type = CD_PROP_FLOAT3; });
  connect(parent_in, "Instancer", capture_result, "Geometry");
  connect(switch_vector, "Output", capture_result, "Value");
  connect(capture_result, "Attribute", instance_on_point, "Rotation");

  bNode *mesh_to_points = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_MESH_TO_POINTS);
  connect(capture_result, "Geometry", mesh_to_points, "Mesh");

  bNode *duplicate_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_DUPLICATE_ELEMENTS);
  connect(mesh_to_points, "Points", duplicate_instances, "Geometry");
  connect(duplicate_instances, "Geometry", instance_on_point, "Points");

  bNode *instance_total_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *instance_total = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_ATTRIBUTE_DOMAIN_SIZE);
  instance_total->custom1 = int(GEO_COMPONENT_TYPE_INSTANCES);
  instance_total->typeinfo->updatefunc(node_tree, instance_total);
  connect(instance_total_in, "Instance", instance_total, "Geometry");
  connect(instance_total, "Instance Count", duplicate_instances, "Amount");

  bNode *geometry_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *reset_transform = add_node_group(reset_instances_transform_tree);
  connect(geometry_in, "Instance", reset_transform, "Instances");
  connect(reset_transform, "Instances", instance_on_point, "Instance");

  bNode *instances_source_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *apply_transform = add_node_group(sample_apply_instances_transform_tree);
  connect(instance_on_point, "Instances", apply_transform, "Instances");
  connect(instances_source_in, "Instance", apply_transform, "Transform Source");
  connect(duplicate_instances, "Duplicate Index", apply_transform, "Instances Index");

  bNode *instances_out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(apply_transform, "Instances", instances_out, "Instances");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.instances_on_points = node_tree;
  return node_tree;
}

static bNodeTree *first_face_tris_points_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.first_face_tris_points != nullptr) {
    return cached_node_trees.first_face_tris_points;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, ".First Face Tris Points", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketInt", "Vertex Index 1");
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketInt", "Vertex Index 2");
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketInt", "Vertex Index 3");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };
  bNode *out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);

  bNode *verfex_of_corner_1 = nodeAddStaticNode(
      nullptr, node_tree, GEO_NODE_MESH_TOPOLOGY_VERTEX_OF_CORNER);
  connect(verfex_of_corner_1, "Vertex Index", out, "Vertex Index 1");
  bNode *corner_of_face_1 = nodeAddStaticNode(
      nullptr, node_tree, GEO_NODE_MESH_TOPOLOGY_CORNERS_OF_FACE);
  connect(corner_of_face_1, "Corner Index", verfex_of_corner_1, "Corner Index");
  node_input_by_name("Sort Index", corner_of_face_1)
      .default_value_typed<bNodeSocketValueInt>()
      ->value = 0;

  bNode *verfex_of_corner_2 = nodeAddStaticNode(
      nullptr, node_tree, GEO_NODE_MESH_TOPOLOGY_VERTEX_OF_CORNER);
  connect(verfex_of_corner_2, "Vertex Index", out, "Vertex Index 2");
  bNode *corner_of_face_2 = nodeAddStaticNode(
      nullptr, node_tree, GEO_NODE_MESH_TOPOLOGY_CORNERS_OF_FACE);
  connect(corner_of_face_2, "Corner Index", verfex_of_corner_2, "Corner Index");
  node_input_by_name("Sort Index", corner_of_face_2)
      .default_value_typed<bNodeSocketValueInt>()
      ->value = 1;

  bNode *verfex_of_corner_3 = nodeAddStaticNode(
      nullptr, node_tree, GEO_NODE_MESH_TOPOLOGY_VERTEX_OF_CORNER);
  connect(verfex_of_corner_3, "Vertex Index", out, "Vertex Index 3");
  bNode *corner_of_face_3 = nodeAddStaticNode(
      nullptr, node_tree, GEO_NODE_MESH_TOPOLOGY_CORNERS_OF_FACE);
  connect(corner_of_face_3, "Corner Index", verfex_of_corner_3, "Corner Index");
  node_input_by_name("Sort Index", corner_of_face_3)
      .default_value_typed<bNodeSocketValueInt>()
      ->value = 2;

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.first_face_tris_points = node_tree;
  return node_tree;
}

static bNodeTree *face_aling_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.face_aling != nullptr) {
    return cached_node_trees.face_aling;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, "Face Aling", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Geometry");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Geometry");
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketVector", "Rotation");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  const auto add_node_group =
      [&](const FunctionRef<bNodeTree *(Main * bmain, RegularNodeTrees & cached_node_trees)>
              tree_func) -> bNode * {
    bNodeTree *tree = tree_func(bmain, cached_node_trees);
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    id_us_plus(&tree->id);
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
  };

  bNode *triangle_indices_group = add_node_group(first_face_tris_points_tree);

  bNode *position = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_POSITION);

  bNode *at_index_1 = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_EVALUATE_AT_INDEX);
  at_index_1->custom1 = ATTR_DOMAIN_POINT;
  at_index_1->custom2 = CD_PROP_FLOAT3;
  at_index_1->typeinfo->updatefunc(node_tree, at_index_1);
  connect(triangle_indices_group, "Vertex Index 1", at_index_1, "Index");
  connect(position, "Position", at_index_1, "Value");

  bNode *at_index_2 = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_EVALUATE_AT_INDEX);
  at_index_2->custom1 = ATTR_DOMAIN_POINT;
  at_index_2->custom2 = CD_PROP_FLOAT3;
  at_index_2->typeinfo->updatefunc(node_tree, at_index_2);
  connect(triangle_indices_group, "Vertex Index 2", at_index_2, "Index");
  connect(position, "Position", at_index_2, "Value");

  bNode *at_index_3 = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_EVALUATE_AT_INDEX);
  at_index_3->custom1 = ATTR_DOMAIN_POINT;
  at_index_3->custom2 = CD_PROP_FLOAT3;
  at_index_3->typeinfo->updatefunc(node_tree, at_index_3);
  connect(triangle_indices_group, "Vertex Index 3", at_index_3, "Index");
  connect(position, "Position", at_index_3, "Value");

  bNode *vector_substruct_1 = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  vector_substruct_1->custom1 = NODE_VECTOR_MATH_SUBTRACT;
  vector_substruct_1->typeinfo->updatefunc(node_tree, vector_substruct_1);
  connect(at_index_2, "Value", vector_substruct_1, "Vector_001");
  connect(at_index_3, "Value", vector_substruct_1, "Vector");

  bNode *vector_substruct_2 = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  vector_substruct_2->custom1 = NODE_VECTOR_MATH_SUBTRACT;
  vector_substruct_2->typeinfo->updatefunc(node_tree, vector_substruct_2);
  connect(at_index_1, "Value", vector_substruct_2, "Vector_001");
  connect(at_index_2, "Value", vector_substruct_2, "Vector");

  bNode *vector_cros_prod_1 = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  vector_cros_prod_1->custom1 = NODE_VECTOR_MATH_CROSS_PRODUCT;
  vector_cros_prod_1->typeinfo->updatefunc(node_tree, vector_cros_prod_1);
  connect(vector_substruct_2, "Vector", vector_cros_prod_1, "Vector");
  connect(vector_substruct_1, "Vector", vector_cros_prod_1, "Vector_001");

  bNode *vector_cros_prod_2 = nodeAddStaticNode(nullptr, node_tree, SH_NODE_VECTOR_MATH);
  vector_cros_prod_2->custom1 = NODE_VECTOR_MATH_CROSS_PRODUCT;
  vector_cros_prod_2->typeinfo->updatefunc(node_tree, vector_cros_prod_2);
  connect(vector_cros_prod_1, "Vector", vector_cros_prod_2, "Vector");
  connect(vector_substruct_2, "Vector", vector_cros_prod_2, "Vector_001");

  bNode *aling_z_auto = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ALIGN_EULER_TO_VECTOR);
  aling_z_auto->custom1 = FN_NODE_ALIGN_EULER_TO_VECTOR_AXIS_Z;
  aling_z_auto->custom2 = FN_NODE_ALIGN_EULER_TO_VECTOR_PIVOT_AXIS_AUTO;
  connect(vector_cros_prod_1, "Vector", aling_z_auto, "Vector");

  bNode *aling_y_auto = nodeAddStaticNode(nullptr, node_tree, FN_NODE_ALIGN_EULER_TO_VECTOR);
  aling_y_auto->custom1 = FN_NODE_ALIGN_EULER_TO_VECTOR_AXIS_Y;
  aling_y_auto->custom2 = FN_NODE_ALIGN_EULER_TO_VECTOR_PIVOT_AXIS_AUTO;
  connect(aling_z_auto, "Rotation", aling_y_auto, "Rotation");
  connect(vector_cros_prod_2, "Vector", aling_y_auto, "Vector");

  bNode *in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *capture_result = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_CAPTURE_ATTRIBUTE);
  for_node_storage<NodeGeometryAttributeCapture>(node_tree, capture_result, [&](auto &storage) {
    storage.domain = ATTR_DOMAIN_FACE;
    storage.data_type = CD_PROP_FLOAT3;
  });
  connect(in, "Geometry", capture_result, "Geometry");
  connect(aling_y_auto, "Rotation", capture_result, "Value");

  bNode *out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(capture_result, "Geometry", out, "Geometry");
  connect(capture_result, "Attribute", out, "Rotation");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.face_aling = node_tree;
  return node_tree;
}

static bNodeTree *faces_scale_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.face_scale != nullptr) {
    return cached_node_trees.face_scale;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, "Face Size", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Geometry");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Geometry");
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketFloat", "Size");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  bNode *face_area_input = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INPUT_MESH_FACE_AREA);

  bNode *math_sqrt = nodeAddStaticNode(nullptr, node_tree, SH_NODE_MATH);
  math_sqrt->custom1 = NODE_MATH_SQRT;
  math_sqrt->typeinfo->updatefunc(node_tree, math_sqrt);
  connect(face_area_input, "Area", math_sqrt, "Value");

  bNode *in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *capture_result = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_CAPTURE_ATTRIBUTE);
  for_node_storage<NodeGeometryAttributeCapture>(node_tree, capture_result, [&](auto &storage) {
    storage.domain = ATTR_DOMAIN_FACE;
    storage.data_type = CD_PROP_FLOAT;
  });
  connect(in, "Geometry", capture_result, "Geometry");
  connect(math_sqrt, "Value", capture_result, "Value");

  bNode *out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(capture_result, "Geometry", out, "Geometry");
  connect(capture_result, "Attribute", out, "Size");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.face_scale = node_tree;
  return node_tree;
}

static bNodeTree *instances_on_faces_tree(Main *bmain, RegularNodeTrees &cached_node_trees)
{
  if (cached_node_trees.instances_on_faces != nullptr) {
    return cached_node_trees.instances_on_faces;
  }

  bNodeTree *node_tree = ntreeAddTree(bmain, "Legacy Face Instances", "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Instance");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Instancer");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Instances");
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketFloat", "Instances Size");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  const auto add_node_group =
      [&](const FunctionRef<bNodeTree *(Main * bmain, RegularNodeTrees & cached_node_trees)>
              tree_func) -> bNode * {
    bNodeTree *tree = tree_func(bmain, cached_node_trees);
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    id_us_plus(&tree->id);
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
  };

  bNode *instancer_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *instance_total_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *instance_total = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_ATTRIBUTE_DOMAIN_SIZE);
  instance_total->custom1 = int(GEO_COMPONENT_TYPE_INSTANCES);
  instance_total->typeinfo->updatefunc(node_tree, instance_total);
  connect(instance_total_in, "Instance", instance_total, "Geometry");

  bNode *face_size_group = add_node_group(faces_scale_tree);
  connect(instancer_in, "Instancer", face_size_group, "Geometry");

  bNode *face_aling_group = add_node_group(face_aling_tree);
  connect(face_size_group, "Geometry", face_aling_group, "Geometry");

  bNode *mesh_to_points = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_MESH_TO_POINTS);
  for_node_storage<NodeGeometryMeshToPoints>(node_tree, mesh_to_points, [&](auto &storage) {
    storage.mode = GEO_NODE_MESH_TO_POINTS_FACES;
  });
  connect(face_aling_group, "Geometry", mesh_to_points, "Mesh");

  bNode *duplicate_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_DUPLICATE_ELEMENTS);
  connect(mesh_to_points, "Points", duplicate_instances, "Geometry");
  connect(instance_total, "Instance Count", duplicate_instances, "Amount");

  bNode *instance_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *reset_transform = add_node_group(reset_instances_transform_tree);
  connect(instance_in, "Instance", reset_transform, "Instances");

  bNode *instance_on_points = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_INSTANCE_ON_POINTS);
  connect(duplicate_instances, "Geometry", instance_on_points, "Points");
  connect(reset_transform, "Instances", instance_on_points, "Instance");
  connect(face_aling_group, "Rotation", instance_on_points, "Rotation");

  bNode *instances_source_in = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  bNode *apply_transform = add_node_group(sample_apply_instances_transform_tree);
  connect(instance_on_points, "Instances", apply_transform, "Instances");
  connect(instances_source_in, "Instance", apply_transform, "Transform Source");
  connect(duplicate_instances, "Duplicate Index", apply_transform, "Instances Index");

  bNode *out = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(apply_transform, "Instances", out, "Instances");
  connect(face_size_group, "Size", out, "Instances Size");

  bke::node_field_inferencing::update_field_inferencing(*node_tree);
  cached_node_trees.instances_on_faces = node_tree;
  return node_tree;
}

static bNodeTree *instances_on_points(const Span<Object *> objects,
                                      const StringRefNull name,
                                      Main *bmain,
                                      RegularNodeTrees &cached_node_trees)
{
  bNodeTree *node_tree = ntreeAddTree(bmain, name.c_str(), "GeometryNodeTree");

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

  const auto add_node_group =
      [&](const FunctionRef<bNodeTree *(Main * bmain, RegularNodeTrees & cached_node_trees)>
              tree_func) -> bNode * {
    bNodeTree *tree = tree_func(bmain, cached_node_trees);
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    id_us_plus(&tree->id);
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
  };

  bNode *join_geometrys = join_objects_as_instances(objects, node_tree);

  bNode *instansing_group = add_node_group(instances_on_points_tree);
  connect(join_geometrys, "Geometry", instansing_group, "Instance");

  bNode *parent_aling_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(parent_aling_input, "Geometry", instansing_group, "Instancer");
  connect(
      parent_aling_input, "Aling to Vertex Normal", instansing_group, "Aling to Vertex Normal");

  bNode *view_switch_group = add_node_group(view_geometry_tree);
  bNode *geometry_viewport_render_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(geometry_viewport_render_input, "Geometry", view_switch_group, "Geometry");
  connect(geometry_viewport_render_input, "Viewport", view_switch_group, "Viewport");
  connect(geometry_viewport_render_input, "Render", view_switch_group, "Render");

  bNode *join_parent = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_JOIN_GEOMETRY);
  connect(instansing_group, "Instances", join_parent, "Geometry");
  connect(view_switch_group, "Geometry", join_parent, "Geometry");

  bNode *group_output = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(join_parent, "Geometry", group_output, "Geometry");

  return node_tree;
}

static bNodeTree *instances_on_faces(const Span<Object *> objects,
                                     const StringRefNull name,
                                     Main *bmain,
                                     RegularNodeTrees &cached_node_trees)
{
  bNodeTree *node_tree = ntreeAddTree(bmain, name.c_str(), "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Geometry");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Viewport");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Render");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketBool", "Scale by Face Size");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketFloat", "Factor");

  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Geometry");

  const auto connect = [node_tree](bNode *node_out,
                                   const StringRefNull name_out,
                                   bNode *node_in,
                                   const StringRefNull name_in) {
    bNodeSocket &out = node_output_by_name(name_out, node_out);
    bNodeSocket &in = node_input_by_name(name_in, node_in);
    nodeAddLink(node_tree, node_out, &out, node_in, &in);
  };

  const auto add_node_group =
      [&](const FunctionRef<bNodeTree *(Main * bmain, RegularNodeTrees & cached_node_trees)>
              tree_func) -> bNode * {
    bNodeTree *tree = tree_func(bmain, cached_node_trees);
    bNode *group = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP);
    group->id = &tree->id;
    id_us_plus(&tree->id);
    bke::node_field_inferencing::update_field_inferencing(*node_tree);
    nodes::update_node_declaration_and_sockets(*node_tree, *group);
    return group;
  };

  bNode *join_geometrys = join_objects_as_instances(objects, node_tree);

  bNode *instansing_group = add_node_group(instances_on_faces_tree);
  connect(join_geometrys, "Geometry", instansing_group, "Instance");

  bNode *geometry_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(geometry_input, "Geometry", instansing_group, "Instancer");

  bNode *scale_instances = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_SCALE_INSTANCES);
  connect(instansing_group, "Instances", scale_instances, "Instances");

  bNode *math_apply_scale_factor = nodeAddStaticNode(nullptr, node_tree, SH_NODE_MATH);
  math_apply_scale_factor->custom1 = NODE_MATH_MULTIPLY;
  math_apply_scale_factor->typeinfo->updatefunc(node_tree, math_apply_scale_factor);
  connect(instansing_group, "Instances Size", math_apply_scale_factor, "Value_001");
  connect(math_apply_scale_factor, "Value", scale_instances, "Scale");

  bNode *scale_factor_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(scale_factor_input, "Factor", math_apply_scale_factor, "Value");

  bNode *scale_use_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  connect(scale_use_input, "Scale by Face Size", scale_instances, "Selection");

  bNode *geometry_viewport_render_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);

  bNode *view_switch_group = add_node_group(view_geometry_tree);
  connect(geometry_viewport_render_input, "Geometry", view_switch_group, "Geometry");
  connect(geometry_viewport_render_input, "Viewport", view_switch_group, "Viewport");
  connect(geometry_viewport_render_input, "Render", view_switch_group, "Render");

  bNode *join_parent = nodeAddStaticNode(nullptr, node_tree, GEO_NODE_JOIN_GEOMETRY);
  connect(scale_instances, "Instances", join_parent, "Geometry");
  connect(view_switch_group, "Geometry", join_parent, "Geometry");

  bNode *group_output = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  connect(join_parent, "Geometry", group_output, "Geometry");
  return node_tree;
}

static void copy_rna_to_id(PropertyRNA &src_prop, PointerRNA &src_ptr, IDProperty &dst)
{
  switch (IDP_ui_data_type(&dst)) {
    case IDP_UI_DATA_TYPE_INT: {
      const int values = RNA_property_int_get(&src_ptr, &src_prop);
      IDP_Int(&dst) = values;
      break;
    }
    case IDP_UI_DATA_TYPE_FLOAT: {
      const int values = RNA_property_float_get(&src_ptr, &src_prop);
      IDP_Float(&dst) = values;
      break;
    }
    case IDP_UI_DATA_TYPE_BOOLEAN: {
      const bool values = RNA_property_boolean_get(&src_ptr, &src_prop);
      IDP_Bool(&dst) = values;
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }
}

template<typename ItemT> static void span_to_list(MutableSpan<ItemT> src, ListBase &dst)
{
  for (ItemT &item : src) {
    BLI_addtail(&dst, &item);
  }
}

static void object_push_instances_modifier(const StringRefNull modifier_name,
                                           Main *bmain,
                                           Object &object,
                                           bNodeTree &node_tree)
{
  ModifierData *md = BKE_modifier_new(eModifierType_Nodes);
  NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(md);

  nmd.node_group = &node_tree;
  id_us_plus(&node_tree.id);

  STRNCPY(nmd.modifier.name, modifier_name.c_str());
  BKE_modifier_unique_name(&object.modifiers, md);

  BLI_addtail(&object.modifiers, md);
  BKE_object_modifier_set_active(&object, md);

  MOD_nodes_update_interface(&object, &nmd);

  PointerRNA object_ptr;
  RNA_pointer_create(&object.id, &RNA_Object, &object, &object_ptr);

  static const Map<StringRef, StringRef> socket_legacy_name_maping = []() {
    Map<StringRef, StringRef> name_mapping;

    name_mapping.add("Viewport", "show_instancer_for_viewport");
    name_mapping.add("Render", "show_instancer_for_render");

    name_mapping.add("Scale by Face Size", "use_instance_faces_scale");
    name_mapping.add("Factor", "instance_faces_scale");

    name_mapping.add("Aling to Vertex Normal", "use_instance_vertices_rotation");
    return name_mapping;
  }();

  Vector<std::string> legacy_rna_paths;
  Vector<AnimationBasePathChange> animation_data;
  node_tree.ensure_topology_cache();
  for (const bNodeSocket *socket : node_tree.interface_inputs().drop_front(1)) {
    IDProperty *modifier_prop = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                         socket->identifier);
    BLI_assert(modifier_prop != nullptr);

    const StringRef legacy_prop_name = socket_legacy_name_maping.lookup(socket->name);
    PropertyRNA *object_prop = RNA_struct_find_property(&object_ptr, legacy_prop_name.data());
    BLI_assert(object_prop != nullptr);

    copy_rna_to_id(*object_prop, object_ptr, *modifier_prop);

    std::stringstream new_rna_path_stream;
    new_rna_path_stream << "modifiers[\"" << nmd.modifier.name << "\"][\"" << socket->identifier
                        << "\"]";
    std::string new_rna_path = new_rna_path_stream.str();

    AnimationBasePathChange animation_to_move;
    animation_to_move.src_basepath = legacy_prop_name.data();
    animation_to_move.dst_basepath = new_rna_path.c_str();
    legacy_rna_paths.append(std::move(new_rna_path));
    animation_data.append(animation_to_move);
  }

  ListBase change_list = {nullptr, nullptr};
  span_to_list<AnimationBasePathChange>(animation_data, change_list);
  BKE_animdata_transfer_by_basepath(bmain, &object.id, &object.id, &change_list);
}

}  // namespace replace_legacy_instances_imp

void remove_legacy_instances_on(Main *bmain, ListBase &lb_objects)
{
  using namespace blender;
  using namespace replace_legacy_instances_imp;

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
  RegularNodeTrees cached_node_trees;

  for (auto &&[parent, objects] : parents_map.items()) {

    /* Or on faces (OB_DUPLIFACES). */
    const bool on_vertices = (parent->transflag & OB_DUPLIVERTS) != 0;

    const StringRefNull parent_name(parent->id.name + 2);
    const StringRefNull domain_name = on_vertices ? N_("Points") : N_("Faces");

    char tree_name[64];
    BLI_snprintf(tree_name,
                 sizeof(tree_name),
                 N_("Instances on %s of %s"),
                 domain_name.c_str(),
                 parent_name.c_str());

    char modifier_name[64];
    BLI_snprintf(
        modifier_name, sizeof(modifier_name), N_("Instances on %s 3.6"), domain_name.c_str());

    bNodeTree *instances_node_tree;
    if (on_vertices) {
      instances_node_tree = instances_on_points(objects, tree_name, bmain, cached_node_trees);
    }
    else {
      instances_node_tree = instances_on_faces(objects, tree_name, bmain, cached_node_trees);
    }
    object_push_instances_modifier(modifier_name, bmain, *parent, *instances_node_tree);

    parent->transflag = 0;
  }
}
