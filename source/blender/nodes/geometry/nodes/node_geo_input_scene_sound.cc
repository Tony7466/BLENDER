/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_scene.h"
#include "DNA_sound_types.h" //Bsound
#include "DEG_depsgraph_query.h" //DEG_get_input_scene

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_scene_sound_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>(N_("Volume"));
  b.add_output<decl::Float>(N_("Pitch"));
}

static void node_exec(GeoNodeExecParams params)
{
  Scene *scene = DEG_get_input_scene(params.depsgraph());
  bSound *sound = static_cast<bSound*>(scene->sound_scene);

  if (sound != nullptr) {
    params.set_output("Volume", sound->volume);
    params.set_output("Pitch", sound->pitch);
  } else {
    params.set_output("Volume", 69.420f);
    params.set_output("Pitch", 69.420f);
  }
}

}  // namespace blender::nodes::node_geo_input_scene_sound_cc

void register_node_type_geo_input_scene_sound()
{
  static bNodeType ntype;
  namespace file_ns = blender::nodes::node_geo_input_scene_sound_cc;
  geo_node_type_base(&ntype, GEO_NODE_INPUT_SCENE_SOUND, "Scene Sound", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
