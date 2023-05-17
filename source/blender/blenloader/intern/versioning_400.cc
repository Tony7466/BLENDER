/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blenloader
 */

#define DNA_DEPRECATED_ALLOW

#include "CLG_log.h"

#include "DNA_mesh_types.h"
#include "DNA_movieclip_types.h"

#include "BLI_assert.h"
#include "BLI_listbase.h"

#include "BKE_idprop.hh"
#include "BKE_main.h"
#include "BKE_mesh_legacy_convert.h"
#include "BKE_modifier.h"
#include "BKE_node.h"
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
  version_mesh_objects_replace_auto_smooth(*bmain);
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

void blo_do_versions_400(FileData * /*fd*/, Library * /*lib*/, Main *bmain)
{
  if (!MAIN_VERSION_ATLEAST(bmain, 400, 1)) {
    LISTBASE_FOREACH (Mesh *, mesh, &bmain->meshes) {
      version_mesh_legacy_to_struct_of_array_format(*mesh);
    }
    version_movieclips_legacy_camera_object(bmain);
  }

  /**
   * Versioning code until next subversion bump goes here.
   *
   * \note Be sure to check when bumping the version:
   * - "versioning_userdef.c", #blo_do_versions_userdef
   * - "versioning_userdef.c", #do_versions_theme
   *
   * \note Keep this message at the bottom of the function.
   */
  {
    /* Keep this block, even when empty. */
  }
}
