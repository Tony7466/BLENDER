/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_object.hh"
#include "ED_screen.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "DNA_modifier_types.h"

#include "MOD_nodes.hh"

#include "BKE_bake_items_serialize.hh"
#include "BKE_context.h"
#include "BKE_main.h"
#include "BKE_modifier.h"
#include "BKE_scene.h"

#include "DEG_depsgraph.h"

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string_utils.h"

#include "object_intern.h"

namespace blender::ed::object::bake_geometry_nodes {

static void bake_operator_props(wmOperatorType *ot)
{
  PropertyRNA *prop;

  WM_operator_properties_id_lookup(ot, false);

  prop = RNA_def_string(
      ot->srna, "modifier", nullptr, MAX_NAME, "Modifier", "Name of the modifier");
  RNA_def_property_flag(prop, PROP_HIDDEN);

  prop = RNA_def_int(ot->srna, "bake_id", 0, 0, INT32_MAX, "Bake ID", "", 0, INT32_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

[[nodiscard]] static bool find_edit_bake(Main *bmain,
                                         wmOperator *op,
                                         Object **r_object,
                                         NodesModifierData **r_modifier,
                                         NodesModifierBake **r_bake)
{
  Object *object = reinterpret_cast<Object *>(
      WM_operator_properties_id_lookup_from_name_or_session_uuid(bmain, op->ptr, ID_OB));
  if (object == nullptr) {
    return false;
  }
  char modifier_name[MAX_NAME];
  RNA_string_get(op->ptr, "modifier", modifier_name);
  ModifierData *md = BKE_modifiers_findby_name(object, modifier_name);
  if (md == nullptr || md->type != eModifierType_Nodes) {
    return false;
  }
  NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
  const int bake_id = RNA_int_get(op->ptr, "bake_id");
  NodesModifierBake *bake = nullptr;
  for (NodesModifierBake &bake_iter : MutableSpan(nmd->bakes, nmd->bakes_num)) {
    if (bake_iter.id == bake_id) {
      bake = &bake_iter;
    }
  }
  if (bake == nullptr) {
    return false;
  }

  *r_object = object;
  *r_modifier = nmd;
  *r_bake = bake;
  return true;
}

static int geometry_node_bake_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);

  Object *object;
  NodesModifierData *modifier;
  NodesModifierBake *bake;
  if (!find_edit_bake(bmain, op, &object, &modifier, &bake)) {
    return OPERATOR_CANCELLED;
  }
  NodesModifierData &nmd = *modifier;
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);

  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);

  return OPERATOR_CANCELLED;
}

static int geometry_node_bake_delete_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);

  Object *object;
  NodesModifierData *modifier;
  NodesModifierBake *bake;
  if (!find_edit_bake(bmain, op, &object, &modifier, &bake)) {
    return OPERATOR_CANCELLED;
  }
  NodesModifierData &nmd = *modifier;

  DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
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

  ot->exec = geometry_node_bake_exec;
  ot->poll = ED_operator_object_active_editable;

  bake_operator_props(ot);
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

  bake_operator_props(ot);
}
