/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_vector.hh"

#include "WM_types.h"

#include "ED_screen.h"

#include "DNA_windowmanager_types.h"

#include "BKE_context.h"
#include "BKE_scene.h"

#include "DEG_depsgraph.h"

#include "object_intern.h"

namespace blender::ed::object::bake_simulation {

static int bake_simulation_exec(bContext *C, wmOperator *op)
{
  Object *object = CTX_data_active_object(C);
  Scene *scene = CTX_data_scene(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Main *bmain = CTX_data_main(C);

  const int old_frame = scene->r.cfra;

  for (const int frame : IndexRange(1, 10)) {
    scene->r.cfra = frame;
    scene->r.subframe = 0.0f;

    BKE_scene_graph_update_for_newframe(depsgraph);
  }

  scene->r.cfra = old_frame;
  DEG_time_tag_update(bmain);

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
