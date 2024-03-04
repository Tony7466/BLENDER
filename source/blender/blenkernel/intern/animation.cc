/* SPDX-FileCopyrightText: 2023 Blender Foundation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file Animation data-block.
 * \ingroup bke
 */

#include "BLI_listbase.h"
#include "BLI_listbase_wrapper.hh"
#include "BLI_map.hh"
#include "BLI_string_utf8.h"

#include "BLO_read_write.hh"

#include "BKE_animation.hh"
#include "BKE_fcurve.h"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"

#include "ANIM_animation.hh"

#include "DNA_anim_types.h"
#include "DNA_defaults.h"

#include "BLT_translation.hh"

using namespace blender;

struct BlendWriter;
struct BlendDataReader;

/** Deep copy an Animation data-block. */
static void animation_copy_data(Main * /*bmain*/, ID *id_dst, const ID *id_src, const int /*flag*/)
{
  Animation *dna_anim_dst = reinterpret_cast<Animation *>(id_dst);
  animrig::Animation &anim_dst = dna_anim_dst->wrap();

  const Animation *dna_anim_src = reinterpret_cast<const Animation *>(id_src);
  const animrig::Animation &anim_src = dna_anim_src->wrap();

  anim_dst.copy_from(anim_src);
}

/** Free (or release) any data used by this animation (does not free the animation itself). */
static void animation_free_data(ID *id)
{
  animrig::Animation &anim = reinterpret_cast<Animation *>(id)->wrap();
  anim.free_data();
}

void BKE_anim_channels_for_output_free_data(AnimationChannelBag *channels)
{
  for (FCurve *fcu : channels->wrap().fcurves()) {
    BKE_fcurve_free(fcu);
  }
  MEM_SAFE_FREE(channels->fcurve_array);
  channels->fcurve_array_num = 0;
}

static void animation_foreach_id(ID *id, LibraryForeachIDData *data)
{
  animrig::Animation &anim = reinterpret_cast<Animation *>(id)->wrap();

  for (animrig::Layer *layer : anim.layers()) {
    for (animrig::Strip *strip : layer->strips()) {
      switch (strip->type()) {
        case animrig::Strip::Type::Keyframe: {
          auto &key_strip = strip->as<animrig::KeyframeStrip>();
          for (animrig::ChannelBag *channelbag_for_output : key_strip.channelbags()) {
            for (FCurve *fcurve : channelbag_for_output->fcurves()) {
              BKE_LIB_FOREACHID_PROCESS_FUNCTION_CALL(data, BKE_fcurve_foreach_id(fcurve, data));
            }
          }
        }
      }
    }
  }
}

static void write_channelbag(BlendWriter *writer, animrig::ChannelBag &channelbag)
{
  BLO_write_struct(writer, AnimationChannelBag, &channelbag);

  Span<FCurve *> fcurves = channelbag.fcurves();
  BLO_write_pointer_array(writer, fcurves.size(), fcurves.data());

  for (FCurve *fcurve : fcurves) {
    BLO_write_struct(writer, FCurve, fcurve);
    BKE_fcurve_blend_write_data(writer, fcurve);
  }
}

static void write_keyframe_strip(BlendWriter *writer, animrig::KeyframeStrip &key_strip)
{
  BLO_write_struct(writer, KeyframeAnimationStrip, &key_strip);

  auto channelbags = key_strip.channelbags();
  BLO_write_pointer_array(writer, channelbags.size(), channelbags.data());

  for (animrig::ChannelBag *channelbag : channelbags) {
    write_channelbag(writer, *channelbag);
  }
}

static void write_strips(BlendWriter *writer, Span<animrig::Strip *> strips)
{
  BLO_write_pointer_array(writer, strips.size(), strips.data());

  for (animrig::Strip *strip : strips) {
    switch (strip->type()) {
      case animrig::Strip::Type::Keyframe: {
        auto &key_strip = strip->as<animrig::KeyframeStrip>();
        write_keyframe_strip(writer, key_strip);
      }
    }
  }
}

static void write_layers(BlendWriter *writer, Span<animrig::Layer *> layers)
{
  BLO_write_pointer_array(writer, layers.size(), layers.data());

  for (animrig::Layer *layer : layers) {
    BLO_write_struct(writer, AnimationLayer, layer);
    write_strips(writer, layer->strips());
  }
}

static void write_outputs(BlendWriter *writer, Span<animrig::Output *> outputs)
{
  BLO_write_pointer_array(writer, outputs.size(), outputs.data());
  for (animrig::Output *output : outputs) {
    BLO_write_struct(writer, AnimationOutput, output);
  }
}

static void animation_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  animrig::Animation &anim = reinterpret_cast<Animation *>(id)->wrap();

  BLO_write_id_struct(writer, Animation, id_address, &anim.id);
  BKE_id_blend_write(writer, &anim.id);

  write_layers(writer, anim.layers());
  write_outputs(writer, anim.outputs());
}

static void read_channelbag(BlendDataReader *reader, animrig::ChannelBag &channelbag)
{
  BLO_read_pointer_array(reader, reinterpret_cast<void **>(&channelbag.fcurve_array));

  for (int i = 0; i < channelbag.fcurve_array_num; i++) {
    BLO_read_data_address(reader, &channelbag.fcurve_array[i]);
    BKE_fcurve_blend_read_data(reader, channelbag.fcurve_array[i]);
  }
}

static void read_keyframe_strip(BlendDataReader *reader, animrig::KeyframeStrip &strip)
{
  BLO_read_pointer_array(reader, reinterpret_cast<void **>(&strip.channelbags_array));

  for (int i = 0; i < strip.channelbags_array_num; i++) {
    BLO_read_data_address(reader, &strip.channelbags_array[i]);
    AnimationChannelBag *channelbag = strip.channelbags_array[i];
    read_channelbag(reader, channelbag->wrap());
  }
}

static void read_animation_layers(BlendDataReader *reader, animrig::Animation &anim)
{
  BLO_read_pointer_array(reader, reinterpret_cast<void **>(&anim.layer_array));

  for (int layer_idx = 0; layer_idx < anim.layer_array_num; layer_idx++) {
    BLO_read_data_address(reader, &anim.layer_array[layer_idx]);
    AnimationLayer *layer = anim.layer_array[layer_idx];

    BLO_read_pointer_array(reader, reinterpret_cast<void **>(&layer->strip_array));
    for (int strip_idx = 0; strip_idx < layer->strip_array_num; strip_idx++) {
      BLO_read_data_address(reader, &layer->strip_array[strip_idx]);
      AnimationStrip *dna_strip = layer->strip_array[strip_idx];
      animrig::Strip &strip = dna_strip->wrap();

      switch (strip.type()) {
        case animrig::Strip::Type::Keyframe: {
          read_keyframe_strip(reader, strip.as<animrig::KeyframeStrip>());
        }
      }
    }
  }
}

static void read_animation_outputs(BlendDataReader *reader, animrig::Animation &anim)
{
  BLO_read_pointer_array(reader, reinterpret_cast<void **>(&anim.output_array));

  for (int i = 0; i < anim.output_array_num; i++) {
    BLO_read_data_address(reader, &anim.output_array[i]);
  }
}

static void animation_blend_read_data(BlendDataReader *reader, ID *id)
{
  animrig::Animation &animation = reinterpret_cast<Animation *>(id)->wrap();
  read_animation_layers(reader, animation);
  read_animation_outputs(reader, animation);
}

IDTypeInfo IDType_ID_AN = {
    /*id_code*/ ID_AN,
    /*id_filter*/ FILTER_ID_AN,
    /*dependencies_id_types*/ 0,
    /*main_listbase_index*/ INDEX_ID_AN,
    /*struct_size*/ sizeof(Animation),
    /*name*/ "Animation",
    /*name_plural*/ "animations",
    /*translation_context*/ BLT_I18NCONTEXT_ID_ANIMATION,
    /*flags*/ IDTYPE_FLAGS_NO_ANIMDATA,
    /*asset_type_info*/ nullptr,

    /*init_data*/ nullptr,
    /*copy_data*/ animation_copy_data,
    /*free_data*/ animation_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ animation_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ animation_blend_write,
    /*blend_read_data*/ animation_blend_read_data,
    /*blend_read_after_liblink*/ nullptr,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};

Animation *BKE_animation_add(Main *bmain, const char name[])
{
  Animation *anim = static_cast<Animation *>(BKE_id_new(bmain, ID_AN, name));
  return anim;
}
