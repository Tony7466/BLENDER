/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_vector.hh"

#include "WM_types.h"

#include "ED_screen.h"

#include "DNA_windowmanager_types.h"

#include "object_intern.h"

namespace blender::ed::object::bake_simulation {

static int bake_simulation_exec(bContext *C, wmOperator *op)
{
  std::cout << "Hello\n";
  return OPERATOR_FINISHED;
}

}  // namespace blender::ed::object::bake_simulation

void OBJECT_OT_bake_simulation(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_simulation;

  ot->name = "Bake Simulation";
  ot->description = "Bake simulations in geometry nodes modifiers";
  ot->idname = __func__;

  ot->exec = bake_simulation_exec;
  ot->poll = ED_operator_object_active;
}
