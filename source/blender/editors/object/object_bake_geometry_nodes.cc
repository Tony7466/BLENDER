/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "WM_api.h"
#include "WM_types.h"

#include "ED_object.h"
#include "ED_screen.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "DNA_modifier_types.h"

#include "MOD_nodes.hh"

#include "BKE_bake_geometry_nodes.hh"
#include "BKE_context.h"
#include "BKE_scene.h"

#include "DEG_depsgraph.h"

#include "object_intern.h"

namespace blender::ed::object::bake_geometry_nodes {

static int geometry_node_bake_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = ED_object_active_context(C);
  ModifierData *md = edit_modifier_property_get(op, ob, 0);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  if (!(md && md->type == eModifierType_Nodes)) {
    return OPERATOR_CANCELLED;
  }

  NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(md);
  const int32_t bake_index = RNA_int_get(op->ptr, "bake_index");

  if (bake_index < 0 || bake_index >= nmd.bakes_num) {
    return OPERATOR_CANCELLED;
  }

  NodesModifierBake &bake = nmd.bakes[bake_index];

  if (!nmd.runtime->bakes) {
    nmd.runtime->bakes = std::make_shared<bke::GeometryNodesModifierBakes>();
  }
  bke::GeometryNodesModifierBakes &bakes = *nmd.runtime->bakes;
  bke::BakeNodeStorage &bake_storage = *bakes.storage_by_id.lookup_or_add_cb(
      bake.id, []() { return std::make_unique<bke::BakeNodeStorage>(); });
  bake_storage.geometry.reset();
  bake_storage.newly_baked_geometry.reset();

  bakes.requested_bake_ids.add(bake.id);

  DEG_id_tag_update(&ob->id, ID_RECALC_GEOMETRY);
  BKE_scene_graph_update_tagged(depsgraph, bmain);

  bakes.requested_bake_ids.clear();

  bake_storage.geometry = std::move(bake_storage.newly_baked_geometry);
  bake_storage.newly_baked_geometry.reset();

  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);

  return OPERATOR_CANCELLED;
}

static int geometry_node_bake_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  if (edit_modifier_invoke_properties(C, op)) {
    return geometry_node_bake_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

static int geometry_node_bake_delete_exec(bContext *C, wmOperator *op)
{
  Object *ob = ED_object_active_context(C);
  ModifierData *md = edit_modifier_property_get(op, ob, 0);
  if (!(md && md->type == eModifierType_Nodes)) {
    return OPERATOR_CANCELLED;
  }

  NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(md);
  const int32_t bake_index = RNA_int_get(op->ptr, "bake_index");

  if (bake_index < 0 || bake_index >= nmd.bakes_num) {
    return OPERATOR_CANCELLED;
  }

  NodesModifierBake &bake = nmd.bakes[bake_index];
  if (!nmd.runtime->bakes) {
    return OPERATOR_FINISHED;
  }

  nmd.runtime->bakes->storage_by_id.remove(bake.id);

  DEG_id_tag_update(&ob->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

static int geometry_node_bake_delete_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  if (edit_modifier_invoke_properties(C, op)) {
    return geometry_node_bake_delete_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

}  // namespace blender::ed::object::bake_geometry_nodes

void OBJECT_OT_geometry_node_bake(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_geometry_nodes;

  ot->name = "Bake Geometry Node";
  ot->description = "Bake geometry in a Bake node in geometry nodes";
  ot->idname = __func__;

  ot->invoke = geometry_node_bake_invoke;
  ot->exec = geometry_node_bake_exec;
  ot->poll = ED_operator_object_active_editable;

  PropertyRNA *prop;

  edit_modifier_properties(ot);

  prop = RNA_def_int(ot->srna, "bake_index", 0, 0, INT32_MAX, "Bake Index", "", 0, INT32_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

void OBJECT_OT_geometry_node_bake_delete(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_geometry_nodes;

  ot->name = "Delete Geometry Node Bake";
  ot->description = "Delete baked data";
  ot->idname = __func__;

  ot->invoke = geometry_node_bake_delete_invoke;
  ot->exec = geometry_node_bake_delete_exec;
  ot->poll = ED_operator_object_active_editable;

  PropertyRNA *prop;

  edit_modifier_properties(ot);

  prop = RNA_def_int(ot->srna, "bake_index", 0, 0, INT32_MAX, "Bake Index", "", 0, INT32_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}
