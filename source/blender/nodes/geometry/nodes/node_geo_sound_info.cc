/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_sound_types.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sound_info_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Sound>("Sound").hide_label();
  b.add_output<decl::Int>("Audio Channels").description("Total channel amount of the sound");
  b.add_output<decl::Int>("Sample Rate")
      .subtype(PropertySubType::PROP_FREQUENCY)
      .description("Sample rate of the sound");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  bSound *sound = params.get_input<bSound *>("Sound");
  if (!sound) {
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Audio Channels", sound->audio_channels);
  params.set_output("Sample Rate", sound->samplerate);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SOUND_INFO, "Sound Info", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sound_info_cc
