/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "DNA_space_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"
#include "BLI_rect.h"

#include "BKE_context.h"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.h"
#include "BKE_report.h"

#include "ED_node.hh"
#include "ED_space_api.hh"

#include "UI_interface.hh"
#include "UI_view2d.hh"

#include "transform.hh"
#include "transform_convert.hh"
#include "transform_snap.hh"

#include "WM_api.hh"

struct TransInfoCustomDataNode {
  View2DEdgePanData edgepan_data;

  /* Compare if the view has changed so we can update with `transformViewUpdate`. */
  rctf viewrect_prev;

  eTModifier attachment_state;

  blender::float2 selection_center;
  bool indicate_frame_joining;
  void *draw_handle;
};

struct TransCustomDataNode {
  /* For reversible unparenting during transform. */
  bNode *parent;
};

/* -------------------------------------------------------------------- */
/** \name Node Transform Creation
 * \{ */

static void create_transform_data_for_node(
    TransData &td, TransData2D &td2d, TransCustomDataNode &tdc, bNode &node, const float dpi_fac)
{
  /* account for parents (nested nodes) */
  const blender::float2 node_offset = {node.offsetx, node.offsety};
  blender::float2 loc = blender::bke::nodeToView(&node, blender::math::round(node_offset));
  loc *= dpi_fac;

  /* use top-left corner as the transform origin for nodes */
  /* Weirdo - but the node system is a mix of free 2d elements and DPI sensitive UI. */
  td2d.loc[0] = loc.x;
  td2d.loc[1] = loc.y;
  td2d.loc[2] = 0.0f;
  td2d.loc2d = td2d.loc; /* current location */

  td.loc = td2d.loc;
  copy_v3_v3(td.iloc, td.loc);
  /* use node center instead of origin (top-left corner) */
  td.center[0] = td2d.loc[0];
  td.center[1] = td2d.loc[1];
  td.center[2] = 0.0f;

  memset(td.axismtx, 0, sizeof(td.axismtx));
  td.axismtx[2][2] = 1.0f;

  td.ext = nullptr;
  td.val = nullptr;

  td.flag = TD_SELECTED;
  td.dist = 0.0f;

  unit_m3(td.mtx);
  unit_m3(td.smtx);

  td.extra = &node;

  tdc.parent = node.parent;
}

static bool is_node_parent_select(const bNode *node)
{
  while ((node = node->parent)) {
    if (node->flag & NODE_SELECT) {
      return true;
    }
  }
  return false;
}

static void node_transform_restore_parenting_hierarchy(TransDataContainer *tc,
                                                       bNodeTree &node_tree)
{
  for (int i = 0; i < tc->data_len; i++) {
    TransData *td = &tc->data[i];
    bNode *node = static_cast<bNode *>(td->extra);

    TransCustomDataNode *tdc = static_cast<TransCustomDataNode *>(tc->custom.type.data);
    bNode *parent_node = tdc[i].parent;

    if (parent_node == nullptr) {
      continue;
    }

    nodeAttachNode(&node_tree, node, parent_node);
  }
}

static void draw_hitzone(const bContext * /*C*/, ARegion *region, void *arg)
{
  using namespace blender;

  TransInfoCustomDataNode *td = static_cast<TransInfoCustomDataNode *>(arg);
  const View2D *v2d = &region->v2d;

  float2 sc_view = td->selection_center * UI_SCALE_FAC;
  float radius = 0.2f * U.widget_unit;
  float2 center;
  UI_view2d_view_to_region_fl(v2d, sc_view.x, sc_view.y, &center.x, &center.y);

  rctf rect;
  BLI_rctf_init_pt_radius(&rect, center, radius);

  float color_select[4];
  UI_GetThemeColor4fv(TH_SELECT, color_select);

  float color_active[4];
  UI_GetThemeColor4fv(TH_ACTIVE, color_active);

  color_select[3] = 0.6f;
  color_active[3] = 0.6f;

  UI_draw_roundbox_4fv_ex(&rect,
                          color_select,
                          nullptr,
                          1.0f,
                          td->indicate_frame_joining ? color_active : color_select,
                          U.pixelsize,
                          radius);
}

static void draw_hitzone_activate(ARegion &region, TransInfoCustomDataNode *customdata)
{
  if (customdata->draw_handle == nullptr) {
    customdata->draw_handle = ED_region_draw_cb_activate(
        region.type, draw_hitzone, customdata, REGION_DRAW_POST_PIXEL);
  }
}

static void draw_hitzone_deactivate(const ARegion &region, TransInfoCustomDataNode *customdata)
{
  if (customdata->draw_handle) {
    ED_region_draw_cb_exit(region.type, customdata->draw_handle);
    customdata->draw_handle = nullptr;
  }
}

static blender::float2 get_center_of_selection(bNodeTree &node_tree)
{
  using namespace blender;
  using namespace blender::ed::space_node;
  rctf bounds_rect;
  BLI_rctf_init_minmax(&bounds_rect);

  for (const bNode *node : get_selected_nodes(node_tree)) {
    /* We get the selection center in node space, because by the time the selection indicator is
     * drawn `totr` hasn't been updated, yet. */
    rctf node_rect;
    float2 node_loc = blender::bke::nodeToView(node,
                                               math::round(float2(node->offsetx, node->offsety)));
    node_rect.xmin = node_loc.x;
    node_rect.xmax = node_loc.x + node->width;
    node_rect.ymin = node_loc.y - node->height;
    node_rect.ymax = node_loc.y;
    BLI_rctf_union(&bounds_rect, &node_rect);
  }

  return float2(BLI_rctf_cent_x(&bounds_rect), BLI_rctf_cent_y(&bounds_rect));
}

static void createTransNodeData(bContext *C, TransInfo *t)
{
  using namespace blender;
  using namespace blender::ed;
  using namespace blender::ed::space_node;
  SpaceNode *snode = static_cast<SpaceNode *>(t->area->spacedata.first);
  bNodeTree *node_tree = snode->edittree;
  if (!node_tree) {
    return;
  }

  /* Custom data to enable edge panning during the node transform */
  TransInfoCustomDataNode *customdata = MEM_cnew<TransInfoCustomDataNode>(__func__);
  UI_view2d_edge_pan_init(t->context,
                          &customdata->edgepan_data,
                          NODE_EDGE_PAN_INSIDE_PAD,
                          NODE_EDGE_PAN_OUTSIDE_PAD,
                          NODE_EDGE_PAN_SPEED_RAMP,
                          NODE_EDGE_PAN_MAX_SPEED,
                          NODE_EDGE_PAN_DELAY,
                          NODE_EDGE_PAN_ZOOM_INFLUENCE);
  customdata->viewrect_prev = customdata->edgepan_data.initial_rect;
  customdata->attachment_state = t->modifiers & (MOD_NODE_LINK_ATTACH | MOD_NODE_FRAME_DETACH);

  if (t->modifiers & MOD_NODE_LINK_ATTACH) {
    space_node::node_insert_on_link_flags_set(*snode, *t->region);
  }
  else {
    space_node::node_insert_on_link_flags_clear(*node_tree);
  }

  t->custom.type.data = customdata;
  t->custom.type.use_free = true;

  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);

  /* Nodes don't support proportional editing and probably never will. */
  t->flag = t->flag & ~T_PROP_EDIT_ALL;

  VectorSet<bNode *> nodes = space_node::get_selected_nodes(*node_tree);
  nodes.remove_if([&](bNode *node) { return is_node_parent_select(node); });
  if (nodes.is_empty()) {
    return;
  }

  float2 selection_center = get_center_of_selection(*node_tree);
  customdata->selection_center = selection_center;
  customdata->indicate_frame_joining = space_node::node_insert_on_frame_flags_set(
      *node_tree, selection_center);

  tc->data_len = nodes.size();
  tc->data = MEM_cnew_array<TransData>(tc->data_len, __func__);
  tc->data_2d = MEM_cnew_array<TransData2D>(tc->data_len, __func__);
  tc->custom.type.data = MEM_cnew_array<TransCustomDataNode>(tc->data_len, __func__);
  tc->custom.type.use_free = true;

  for (const int i : nodes.index_range()) {
    TransCustomDataNode *data_node = static_cast<TransCustomDataNode *>(tc->custom.type.data);
    create_transform_data_for_node(
        tc->data[i], tc->data_2d[i], data_node[i], *nodes[i], UI_SCALE_FAC);
  }

  if (nodes.size() > 1 || nodes[0]->is_frame()) {
    draw_hitzone_activate(*CTX_wm_region(C), customdata);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Flush Transform Nodes
 * \{ */

static void node_snap_grid_apply(TransInfo *t)
{
  using namespace blender;

  if (!(transform_snap_is_active(t) &&
        (t->tsnap.mode & (SCE_SNAP_TO_INCREMENT | SCE_SNAP_TO_GRID)))) {
    return;
  }

  float2 grid_size = t->snap_spatial;
  if (t->modifiers & MOD_PRECISION) {
    grid_size *= t->snap_spatial_precision;
  }

  /* Early exit on unusable grid size. */
  if (math::is_zero(grid_size)) {
    return;
  }

  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    for (const int i : IndexRange(tc->data_len)) {
      TransData &td = tc->data[i];
      float iloc[2], loc[2], tvec[2];
      if (td.flag & TD_SKIP) {
        continue;
      }

      if ((t->flag & T_PROP_EDIT) && (td.factor == 0.0f)) {
        continue;
      }

      copy_v2_v2(iloc, td.loc);

      loc[0] = roundf(iloc[0] / grid_size[0]) * grid_size[0];
      loc[1] = roundf(iloc[1] / grid_size[1]) * grid_size[1];

      sub_v2_v2v2(tvec, loc, iloc);
      add_v2_v2(td.loc, tvec);
    }
  }
}

static void node_transform_detach_parents(bNodeTree &node_tree)
{
  for (bNode *node : blender::ed::space_node::get_selected_nodes(node_tree)) {
    if (node->parent == nullptr) {
      continue;
    }
    if (is_node_parent_select(node)) {
      /* When a parent frame is transformed together with the node we don't need to clear
       * the parenting. */
      continue;
    }

    nodeDetachNode(&node_tree, node);
  }
}

static void flushTransNodes(TransInfo *t)
{
  using namespace blender::ed;
  const float dpi_fac = UI_SCALE_FAC;
  SpaceNode *snode = static_cast<SpaceNode *>(t->area->spacedata.first);
  bNodeTree &node_tree = *snode->edittree;

  TransInfoCustomDataNode *customdata = (TransInfoCustomDataNode *)t->custom.type.data;

  if (t->options & CTX_VIEW2D_EDGE_PAN) {
    if (t->state == TRANS_CANCEL) {
      UI_view2d_edge_pan_cancel(t->context, &customdata->edgepan_data);
    }
    else {
      /* Edge panning functions expect window coordinates, mval is relative to region */
      const int xy[2] = {
          t->region->winrct.xmin + int(t->mval[0]),
          t->region->winrct.ymin + int(t->mval[1]),
      };
      UI_view2d_edge_pan_apply(t->context, &customdata->edgepan_data, xy);
    }
  }

  float offset[2] = {0.0f, 0.0f};
  if (t->state != TRANS_CANCEL) {
    if (!BLI_rctf_compare(&customdata->viewrect_prev, &t->region->v2d.cur, FLT_EPSILON)) {
      /* Additional offset due to change in view2D rect. */
      BLI_rctf_transform_pt_v(&t->region->v2d.cur, &customdata->viewrect_prev, offset, offset);
      tranformViewUpdate(t);
      customdata->viewrect_prev = t->region->v2d.cur;
    }
  }

  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    node_snap_grid_apply(t);

    /* flush to 2d vector from internally used 3d vector */
    for (int i = 0; i < tc->data_len; i++) {
      TransData *td = &tc->data[i];
      TransData2D *td2d = &tc->data_2d[i];
      bNode *node = static_cast<bNode *>(td->extra);

      blender::float2 loc;
      add_v2_v2v2(loc, td2d->loc, offset);

      /* Weirdo - but the node system is a mix of free 2d elements and DPI sensitive UI. */
      loc /= dpi_fac;

      /* account for parents (nested nodes) */
      const blender::float2 node_offset = {node->offsetx, node->offsety};
      const blender::float2 new_node_location = loc - blender::math::round(node_offset);
      const blender::float2 location = blender::bke::nodeFromView(node->parent, new_node_location);
      node->locx = location.x;
      node->locy = location.y;
    }
  }

  TransDataContainer *tc = t->data_container;
  /* Handle intersection with node links. */
  if (tc->data_len == 1) {
    if (t->modifiers & MOD_NODE_LINK_ATTACH) {
      space_node::node_insert_on_link_flags_set(*snode, *t->region);
    }
    else {
      space_node::node_insert_on_link_flags_clear(node_tree);
    }
  }

  /* Handle intersection with frame nodes. */

  const bool frame_attachment_state_has_changed = (t->modifiers & MOD_NODE_FRAME_DETACH) !=
                                                  (customdata->attachment_state &
                                                   MOD_NODE_FRAME_DETACH);

  // TODO(Leon): This needs to update frame sizes, when the childs are attached.
  if (t->modifiers & MOD_NODE_FRAME_DETACH) {
    if (frame_attachment_state_has_changed) {
      node_transform_detach_parents(node_tree);
    }
    customdata->attachment_state = MOD_NODE_FRAME_DETACH;
  }
  else {
    if (frame_attachment_state_has_changed) {
      node_transform_restore_parenting_hierarchy(tc, node_tree);
    }
    customdata->attachment_state &= ~MOD_NODE_FRAME_DETACH;
  }

  blender::float2 selection_center = get_center_of_selection(node_tree);
  customdata->selection_center = selection_center;
  customdata->indicate_frame_joining = space_node::node_insert_on_frame_flags_set(
      node_tree, selection_center);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Special After Transform Node
 * \{ */

static void special_aftertrans_update__node(bContext *C, TransInfo *t)
{
  using namespace blender::ed;
  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = (SpaceNode *)t->area->spacedata.first;
  bNodeTree *ntree = snode->edittree;

  const bool canceled = (t->state == TRANS_CANCEL);

  if (canceled && t->remove_on_cancel) {
    /* remove selected nodes on cancel */
    if (ntree) {
      LISTBASE_FOREACH_MUTABLE (bNode *, node, &ntree->nodes) {
        if (node->flag & NODE_SELECT) {
          nodeRemoveNode(bmain, ntree, node, true);
        }
      }
      ED_node_tree_propagate_change(C, bmain, ntree);
    }
  }

  if (!canceled) {
    ED_node_post_apply_transform(C, snode->edittree);
    if (t->modifiers & MOD_NODE_LINK_ATTACH) {
      space_node::node_insert_on_link_flags(*bmain, *snode);
    }

    space_node::node_insert_on_frame_flags(*ntree);
    ED_node_tree_propagate_change(C, bmain, ntree);
  }

  space_node::node_insert_on_link_flags_clear(*ntree);
  space_node::node_insert_on_frame_flags_clear(*ntree);

  TransInfoCustomDataNode *customdata = static_cast<TransInfoCustomDataNode *>(
      t->custom.type.data);
  draw_hitzone_deactivate(*CTX_wm_region(C), customdata);

  wmOperatorType *ot = WM_operatortype_find("NODE_OT_insert_offset", true);
  BLI_assert(ot);
  PointerRNA ptr;
  WM_operator_properties_create_ptr(&ptr, ot);
  WM_operator_name_call_ptr(C, ot, WM_OP_INVOKE_DEFAULT, &ptr, nullptr);
  WM_operator_properties_free(&ptr);
}

/** \} */

TransConvertTypeInfo TransConvertType_Node = {
    /*flags*/ (T_POINTS | T_2D_EDIT),
    /*create_trans_data*/ createTransNodeData,
    /*recalc_data*/ flushTransNodes,
    /*special_aftertrans_update*/ special_aftertrans_update__node,
};
