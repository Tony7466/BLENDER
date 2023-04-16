/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_main.h"
#include "BKE_scene.h"
#include "BKE_sound.h"

#include "DNA_sound_types.h"
#include "DNA_sequence_types.h"
#include "DEG_depsgraph_query.h"
#include "DEG_depsgraph.h"

#include "SEQ_sequencer.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_scene_sound_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>(N_("Volume"));
  b.add_output<decl::Float>(N_("Pitch"));
}

static void node_exec(GeoNodeExecParams params)
{
  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  const float scene_ctime = BKE_scene_ctime_get(scene);

  Editing *ed = static_cast<Editing *>(SEQ_editing_get(scene)); // scene->ed ?? 
  float volume = 0.0f;
  float pitch = 0.0f;

  if (ed) {
    Sequence *seq;
    for (seq = static_cast<Sequence *>(ed->seqbasep->first); seq; seq = seq->next) {
      if (seq->type == SEQ_TYPE_SOUND_RAM && seq->start <= scene_ctime && seq->start + seq->len > scene_ctime) {
        // TODO: does sound accumulate like that? 
        volume += seq->sound->volume;
        pitch += seq->sound->pitch;
      }
    }
  }

  params.set_output("Volume", volume);
  params.set_output("Pitch", pitch);
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
