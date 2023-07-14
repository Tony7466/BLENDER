/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "WM_api.h"
#include "WM_types.h"

#include "ED_screen.h"

#include "object_intern.h"

namespace blender::ed::object::bake_geometry_nodes {

static int geometry_node_bake_exec(bContext * /*C*/, wmOperator * /*op*/)
{
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
}
