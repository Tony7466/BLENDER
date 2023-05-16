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

#include "BKE_main.h"
#include "BKE_mesh_legacy_convert.h"
#include "BKE_modifier.h"
#include "BKE_node.h"
#include "BKE_tracking.h"

#include "BLO_readfile.h"

#include "BLT_translation.h"

#include "readfile.h"

#include "versioning_common.h"

// static CLG_LogRef LOG = {"blo.readfile.doversion"};

static bNodeTree *add_realize_node_tree(Main &bmain)
{
  bNodeTree *node_tree = ntreeAddTree(&bmain, DATA_("Auto Smooth 4.0 Legacy"), "GeometryNodeTree");

  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketGeometry", "Geometry");
  ntreeAddSocketInterface(node_tree, SOCK_IN, "NodeSocketFloatAngle", "Angle");
  ntreeAddSocketInterface(node_tree, SOCK_OUT, "NodeSocketGeometry", "Geometry");

  bNode *group_input = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_INPUT);
  group_input->locx = -400.0f;
  bNode *group_output = nodeAddStaticNode(nullptr, node_tree, NODE_GROUP_OUTPUT);
  group_output->locx = 500.0f;
  group_output->flag |= NODE_DO_OUTPUT;

  bNode *shade_smooth = nodeAddNode(nullptr, node_tree, "GeometryNodeSetShadeSmooth");

  bNode *store_node = nodeAddNode(nullptr, node_tree, "GeometryNodeStoreNamedAttribute");
  static_cast<NodeGeometryStoreNamedAttribute *>(store_node->storage)->data_type = CD_PROP_BOOL;
  static_cast<NodeGeometryStoreNamedAttribute *>(store_node->storage)->domain = ATTR_DOMAIN_EDGE;
  bNodeSocket *name = nodeFindSocket(store_node, SOCK_IN, DATA_("Name"));
  STRNCPY(name->default_value_typed<bNodeSocketValueString>()->value, "sharp_face");

  bNode *greater_node = nodeAddNode(nullptr, node_tree, "FunctionNodeCompare");
  static_cast<NodeFunctionCompare *>(store_node->storage)->operation = NODE_COMPARE_GREATER_THAN;
  static_cast<NodeFunctionCompare *>(store_node->storage)->data_type = CD_PROP_FLOAT;
}

void do_versions_after_linking_400(FileData * /*fd*/, Main *bmain)
{
  bNodeTree *auto_smooth_node_tree = nullptr;
  LISTBASE_FOREACH (Object *, object, &bmain->objects) {
    if (object->type == OB_MESH) {
      Mesh *mesh = static_cast<Mesh *>(object->data);
      if (mesh->flag & ME_AUTOSMOOTH) {
        if (!auto_smooth_node_tree) {
          auto_smooth_node_tree = add_realize_node_tree(*bmain);
        }
        auto *md = reinterpret_cast<NodesModifierData *>(BKE_modifier_new(eModifierType_Nodes));
        STRNCPY(md->modifier.name, DATA_("Auto Smooth 4.0 Legacy"));
        BKE_modifier_unique_name(&object->modifiers, &md->modifier);
        md->node_group = auto_smooth_node_tree;

        BLI_addtail(&object->modifiers, md);
      }
    }
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

  /* NOTE: The regular .blend file saving converts the new format to the legacy format, but the
   * auto-save one does not do this. Likely, the regular saving clears the new storage before
   * write, so it can be used to make a decision here.
   *
   * The idea is basically to not override the new storage if it exists. This is only supposed to
   * happen for auto-save files. */

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
  // if (!MAIN_VERSION_ATLEAST(bmain, 400, 0)) {
  /* This is done here because we will continue to write with the old format until 4.0, so we need
   * to convert even "current" files. Keep the check commented out for now so the versioning isn't
   * turned off right after the 4.0 bump. */
  LISTBASE_FOREACH (Mesh *, mesh, &bmain->meshes) {
    version_mesh_legacy_to_struct_of_array_format(*mesh);
  }
  version_movieclips_legacy_camera_object(bmain);
  // }

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
