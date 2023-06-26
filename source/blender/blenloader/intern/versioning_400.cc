/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blenloader
 */

#define DNA_DEPRECATED_ALLOW

#include "CLG_log.h"

#include "DNA_lightprobe_types.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_movieclip_types.h"

#include "DNA_genfile.h"

#include "BLI_assert.h"
#include "BLI_listbase.h"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"

#include "BKE_idprop.hh"
#include "BKE_main.h"
#include "BKE_mesh_legacy_convert.h"
#include "BKE_modifier.h"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.h"
#include "BKE_tracking.h"

#include "BLO_readfile.h"

#include "BLT_translation.h"

#include "readfile.h"

#include "versioning_common.h"

// static CLG_LogRef LOG = {"blo.readfile.doversion"};

static bNodeTree *add_realize_node_tree(Main &bmain)
{
  bNodeTree *node_tree = ntreeAddTree(&bmain, DATA_("Auto Smooth"), "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", DATA_("Geometry"));
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketFloatAngle", DATA_("Angle"));
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", DATA_("Geometry"));

  bNode *group_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  group_input->locx = -380.0f;
  bNode *group_output = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  group_output->locx = 260.0f;
  group_output->flag |= NODE_DO_OUTPUT;

  bNode *shade_smooth = nodeAddNode(nullptr, node_tree, "GeometryNodeSetShadeSmooth");
  shade_smooth->locx = -160.0f;
  shade_smooth->locy = 40.0f;

  bNode *store_node = nodeAddNode(nullptr, node_tree, "GeometryNodeStoreNamedAttribute");
  store_node->locx = 40.0f;
  store_node->locy = 40.0f;
  static_cast<NodeGeometryStoreNamedAttribute *>(store_node->storage)->data_type = CD_PROP_BOOL;
  static_cast<NodeGeometryStoreNamedAttribute *>(store_node->storage)->domain = ATTR_DOMAIN_EDGE;
  bNodeSocket *name = nodeFindSocket(store_node, SOCK_IN, DATA_("Name"));
  STRNCPY(name->default_value_typed<bNodeSocketValueString>()->value, "sharp_edge");

  bNode *greater_node = nodeAddNode(nullptr, node_tree, "FunctionNodeCompare");
  greater_node->locx = -160.0f;
  greater_node->locy = -100.0f;
  static_cast<NodeFunctionCompare *>(greater_node->storage)->operation = NODE_COMPARE_GREATER_THAN;
  static_cast<NodeFunctionCompare *>(greater_node->storage)->data_type = SOCK_FLOAT;

  bNode *edge_angle_node = nodeAddNode(nullptr, node_tree, "GeometryNodeInputMeshEdgeAngle");
  edge_angle_node->locx = -380.0f;
  edge_angle_node->locy = -180.0f;

  nodeAddLink(node_tree,
              group_input,
              static_cast<bNodeSocket *>(group_input->outputs.first),
              shade_smooth,
              nodeFindSocket(shade_smooth, SOCK_IN, "Geometry"));
  nodeAddLink(node_tree,
              shade_smooth,
              nodeFindSocket(shade_smooth, SOCK_OUT, "Geometry"),
              store_node,
              nodeFindSocket(store_node, SOCK_IN, "Geometry"));
  nodeAddLink(node_tree,
              edge_angle_node,
              nodeFindSocket(edge_angle_node, SOCK_OUT, "Unsigned Angle"),
              greater_node,
              nodeFindSocket(greater_node, SOCK_IN, "A"));
  nodeAddLink(node_tree,
              group_input,
              static_cast<bNodeSocket *>(BLI_findlink(&group_input->outputs, 1)),
              greater_node,
              nodeFindSocket(greater_node, SOCK_IN, "B"));
  nodeAddLink(node_tree,
              greater_node,
              nodeFindSocket(greater_node, SOCK_OUT, "Result"),
              store_node,
              nodeFindSocket(store_node, SOCK_IN, "Value_Bool"));
  nodeAddLink(node_tree,
              store_node,
              nodeFindSocket(store_node, SOCK_OUT, "Geometry"),
              group_output,
              static_cast<bNodeSocket *>(group_output->inputs.first));

  LISTBASE_FOREACH (bNode *, node, &node_tree->nodes) {
    nodeSetSelected(node, false);
  }

  version_socket_update_is_used(node_tree);

  return node_tree;
}

static void version_mesh_objects_replace_auto_smooth(Main &bmain)
{
  using namespace blender;
  bNodeTree *auto_smooth_node_tree = nullptr;
  LISTBASE_FOREACH (Object *, object, &bmain.objects) {
    if (object->type != OB_MESH) {
      continue;
    }
    Mesh *mesh = static_cast<Mesh *>(object->data);
    if (!(mesh->flag & ME_AUTOSMOOTH)) {
      continue;
    }
    if (CustomData_has_layer(&mesh->ldata, CD_CUSTOMLOOPNORMAL)) {
      continue;
    }
    if (!auto_smooth_node_tree) {
      auto_smooth_node_tree = add_realize_node_tree(bmain);
    }
    auto *md = reinterpret_cast<NodesModifierData *>(BKE_modifier_new(eModifierType_Nodes));
    STRNCPY(md->modifier.name, DATA_("Auto Smooth"));
    BKE_modifier_unique_name(&object->modifiers, &md->modifier);
    md->node_group = auto_smooth_node_tree;
    if (!BLI_listbase_is_empty(&object->modifiers) &&
        static_cast<ModifierData *>(object->modifiers.last)->type == eModifierType_Subsurf)
    {
      /* Add the auto smooth node group before the last subdivision surface modifier if possible.
       * Subdivision surface modifiers have special handling for interpolating face corner normals,
       * and recalculating them afterwards isn't usually helpful and can be much slower. */
      BLI_insertlinkbefore(&object->modifiers, object->modifiers.last, md);
    }
    else {
      BLI_addtail(&object->modifiers, md);
    }

    md->settings.properties = bke::idprop::create_group("Nodes Modifier Settings").release();
    IDProperty *angle_prop = bke::idprop::create(DATA_("Input_1"), mesh->smoothresh).release();
    IDP_AddToGroup(md->settings.properties, angle_prop);
  }
}

void do_versions_after_linking_400(FileData * /*fd*/, Main *bmain)
{
  if (!MAIN_VERSION_ATLEAST(bmain, 400, 8)) {
    version_mesh_objects_replace_auto_smooth(*bmain);
  }

  /**
   * Versioning code until next subversion bump goes here.
   *
   * \note Be sure to check when bumping the version:
   * - #blo_do_versions_400 in this file.
   * - "versioning_userdef.c", #blo_do_versions_userdef
   * - "versioning_userdef.c", #do_versions_theme
   *
   * \note Keep this message at the bottom of the function.
   */
  {
    /* Keep this block, even when empty. */
  }
}

static void version_mesh_legacy_to_struct_of_array_format(Mesh &mesh)
{
  BKE_mesh_legacy_convert_flags_to_selection_layers(&mesh);
  BKE_mesh_legacy_convert_flags_to_hide_layers(&mesh);
  BKE_mesh_legacy_convert_uvs_to_generic(&mesh);
  BKE_mesh_legacy_convert_mpoly_to_material_indices(&mesh);
  BKE_mesh_legacy_sharp_faces_from_flags(&mesh);
  BKE_mesh_legacy_bevel_weight_to_layers(&mesh);
  BKE_mesh_legacy_sharp_edges_from_flags(&mesh);
  BKE_mesh_legacy_face_set_to_generic(&mesh);
  BKE_mesh_legacy_edge_crease_to_layers(&mesh);
  BKE_mesh_legacy_uv_seam_from_flags(&mesh);
  BKE_mesh_legacy_convert_verts_to_positions(&mesh);
  BKE_mesh_legacy_attribute_flags_to_strings(&mesh);
  BKE_mesh_legacy_convert_loops_to_corners(&mesh);
  BKE_mesh_legacy_convert_polys_to_offsets(&mesh);
  BKE_mesh_legacy_convert_edges_to_generic(&mesh);
}

static void version_motion_tracking_legacy_camera_object(MovieClip &movieclip)
{
  MovieTracking &tracking = movieclip.tracking;
  MovieTrackingObject *active_tracking_object = BKE_tracking_object_get_active(&tracking);
  MovieTrackingObject *tracking_camera_object = BKE_tracking_object_get_camera(&tracking);

  BLI_assert(tracking_camera_object != nullptr);

  if (BLI_listbase_is_empty(&tracking_camera_object->tracks)) {
    tracking_camera_object->tracks = tracking.tracks_legacy;
    active_tracking_object->active_track = tracking.act_track_legacy;
  }

  if (BLI_listbase_is_empty(&tracking_camera_object->plane_tracks)) {
    tracking_camera_object->plane_tracks = tracking.plane_tracks_legacy;
    active_tracking_object->active_plane_track = tracking.act_plane_track_legacy;
  }

  if (tracking_camera_object->reconstruction.cameras == nullptr) {
    tracking_camera_object->reconstruction = tracking.reconstruction_legacy;
  }

  /* Clear pointers in the legacy storage.
   * Always do it, in the case something got missed in the logic above, so that the legacy storage
   * is always ensured to be empty after load. */
  BLI_listbase_clear(&tracking.tracks_legacy);
  BLI_listbase_clear(&tracking.plane_tracks_legacy);
  tracking.act_track_legacy = nullptr;
  tracking.act_plane_track_legacy = nullptr;
  memset(&tracking.reconstruction_legacy, 0, sizeof(tracking.reconstruction_legacy));
}

static void version_movieclips_legacy_camera_object(Main *bmain)
{
  LISTBASE_FOREACH (MovieClip *, movieclip, &bmain->movieclips) {
    version_motion_tracking_legacy_camera_object(*movieclip);
  }
}

static void version_geometry_nodes_add_realize_instance_nodes(bNodeTree *ntree)
{
  LISTBASE_FOREACH_MUTABLE (bNode *, node, &ntree->nodes) {
    if (STREQ(node->idname, "GeometryNodeMeshBoolean")) {
      add_realize_instances_before_socket(ntree, node, nodeFindSocket(node, SOCK_IN, "Mesh 2"));
    }
  }
}

static void version_mesh_crease_generic(Main &bmain)
{
  LISTBASE_FOREACH (Mesh *, mesh, &bmain.meshes) {
    BKE_mesh_legacy_crease_to_generic(mesh);
  }

  LISTBASE_FOREACH (bNodeTree *, ntree, &bmain.nodetrees) {
    if (ntree->type == NTREE_GEOMETRY) {
      LISTBASE_FOREACH (bNode *, node, &ntree->nodes) {
        if (STR_ELEM(node->idname,
                     "GeometryNodeStoreNamedAttribute",
                     "GeometryNodeInputNamedAttribute")) {
          bNodeSocket *socket = nodeFindSocket(node, SOCK_IN, "Name");
          if (STREQ(socket->default_value_typed<bNodeSocketValueString>()->value, "crease")) {
            STRNCPY(socket->default_value_typed<bNodeSocketValueString>()->value, "crease_edge");
          }
        }
      }
    }
  }

  LISTBASE_FOREACH (Object *, object, &bmain.objects) {
    LISTBASE_FOREACH (ModifierData *, md, &object->modifiers) {
      if (md->type != eModifierType_Nodes) {
        continue;
      }
      if (IDProperty *settings = reinterpret_cast<NodesModifierData *>(md)->settings.properties) {
        LISTBASE_FOREACH (IDProperty *, prop, &settings->data.group) {
          if (blender::StringRef(prop->name).endswith("_attribute_name")) {
            if (STREQ(IDP_String(prop), "crease")) {
              IDP_AssignString(prop, "crease_edge");
            }
          }
        }
      }
    }
  }
}

static void versioning_replace_legacy_glossy_node(bNodeTree *ntree)
{
  LISTBASE_FOREACH (bNode *, node, &ntree->nodes) {
    if (node->type == SH_NODE_BSDF_GLOSSY_LEGACY) {
      STRNCPY(node->idname, "ShaderNodeBsdfAnisotropic");
      node->type = SH_NODE_BSDF_GLOSSY;
    }
  }
}

static void versioning_remove_microfacet_sharp_distribution(bNodeTree *ntree)
{
  /* Find all glossy, glass and refraction BSDF nodes that have their distribution
   * set to SHARP and set them to GGX, disconnect any link to the Roughness input
   * and set its value to zero. */
  LISTBASE_FOREACH (bNode *, node, &ntree->nodes) {
    if (!ELEM(node->type, SH_NODE_BSDF_GLOSSY, SH_NODE_BSDF_GLASS, SH_NODE_BSDF_REFRACTION)) {
      continue;
    }
    if (node->custom1 != SHD_GLOSSY_SHARP_DEPRECATED) {
      continue;
    }

    node->custom1 = SHD_GLOSSY_GGX;
    LISTBASE_FOREACH (bNodeSocket *, socket, &node->inputs) {
      if (!STREQ(socket->identifier, "Roughness")) {
        continue;
      }

      if (socket->link != nullptr) {
        nodeRemLink(ntree, socket->link);
      }
      bNodeSocketValueFloat *socket_value = (bNodeSocketValueFloat *)socket->default_value;
      socket_value->value = 0.0f;

      break;
    }
  }
}

void blo_do_versions_400(FileData *fd, Library * /*lib*/, Main *bmain)
{
  if (!MAIN_VERSION_ATLEAST(bmain, 400, 1)) {
    LISTBASE_FOREACH (Mesh *, mesh, &bmain->meshes) {
      version_mesh_legacy_to_struct_of_array_format(*mesh);
    }
    version_movieclips_legacy_camera_object(bmain);
  }

  if (!MAIN_VERSION_ATLEAST(bmain, 400, 2)) {
    LISTBASE_FOREACH (Mesh *, mesh, &bmain->meshes) {
      BKE_mesh_legacy_bevel_weight_to_generic(mesh);
    }
  }

  if (!MAIN_VERSION_ATLEAST(bmain, 400, 3)) {
    LISTBASE_FOREACH (bNodeTree *, ntree, &bmain->nodetrees) {
      if (ntree->type == NTREE_GEOMETRY) {
        version_geometry_nodes_add_realize_instance_nodes(ntree);
      }
    }
  }

  /* 400 4 did not require any do_version here. */

  if (!MAIN_VERSION_ATLEAST(bmain, 400, 5)) {
    LISTBASE_FOREACH (Scene *, scene, &bmain->scenes) {
      ToolSettings *ts = scene->toolsettings;
      if (ts->snap_mode_tools != SCE_SNAP_TO_NONE) {
        ts->snap_mode_tools = SCE_SNAP_TO_GEOM;
      }

#define SCE_SNAP_PROJECT (1 << 3)
      if (ts->snap_flag & SCE_SNAP_PROJECT) {
        ts->snap_mode |= SCE_SNAP_INDIVIDUAL_PROJECT;
      }
#undef SCE_SNAP_PROJECT
    }
  }

  if (!MAIN_VERSION_ATLEAST(bmain, 400, 6)) {
    LISTBASE_FOREACH (Mesh *, mesh, &bmain->meshes) {
      BKE_mesh_legacy_face_map_to_generic(mesh);
    }
    FOREACH_NODETREE_BEGIN (bmain, ntree, id) {
      versioning_replace_legacy_glossy_node(ntree);
      versioning_remove_microfacet_sharp_distribution(ntree);
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_ATLEAST(bmain, 400, 7)) {
    LISTBASE_FOREACH (Mesh *, mesh, &bmain->meshes) {
      version_mesh_crease_generic(*bmain);
    }
  }

  /**
   * Versioning code until next subversion bump goes here.
   *
   * \note Be sure to check when bumping the version:
   * - #do_versions_after_linking_400 in this file.
   * - "versioning_userdef.c", #blo_do_versions_userdef
   * - "versioning_userdef.c", #do_versions_theme
   *
   * \note Keep this message at the bottom of the function.
   */
  {
    /* Convert anisotropic BSDF node to glossy BSDF. */

    /* Keep this block, even when empty. */

    if (!DNA_struct_elem_find(fd->filesdna, "LightProbe", "int", "grid_bake_sample_count")) {
      LISTBASE_FOREACH (LightProbe *, lightprobe, &bmain->lightprobes) {
        lightprobe->grid_bake_samples = 2048;
        lightprobe->surfel_density = 1.0f;
      }
    }
  }
}
