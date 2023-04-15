/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h" //DEG_get_input_scene
#include "BKE_scene.h" // BKE_scene_object_sounds_iterate
#include "BKE_sound.h" // BKE_sound_scene_playing

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
    params.set_output("Volume", 0.0f);
    params.set_output("Pitch", 0.0f);
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
