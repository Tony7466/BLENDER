/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include <cstdlib>

#include "DNA_anim_types.h"

#include "ANIM_animation.hh"

#include "BLI_utildefines.h"

#include "BLT_translation.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "rna_internal.hh"

#include "WM_api.hh"
#include "WM_types.hh"

using namespace blender;

const EnumPropertyItem rna_enum_layer_mix_mode_items[] = {
    {int(animrig::Layer::MixMode::Replace),
     "REPLACE",
     0,
     "Replace",
     "Channels in this layer override the same channels from underlying layers"},
    {int(animrig::Layer::MixMode::Offset),
     "OFFSET",
     0,
     "Offset",
     "Channels in this layer are added to underlying layers as sequential operations"},
    {int(animrig::Layer::MixMode::Add),
     "ADD",
     0,
     "Add",
     "Channels in this layer are added to underlying layers on a per-channel basis"},
    {int(animrig::Layer::MixMode::Subtract),
     "SUBTRACT",
     0,
     "Subtract",
     "Channels in this layer are subtracted to underlying layers on a per-channel basis"},
    {int(animrig::Layer::MixMode::Multiply),
     "MULTIPLY",
     0,
     "Multiply",
     "Channels in this layer are multiplied with underlying layers on a per-channel basis"},
    {0, nullptr, 0, nullptr, nullptr},
};

const EnumPropertyItem rna_enum_strip_type_items[] = {
    {int(animrig::Strip::Type::Keyframe),
     "KEYFRAME",
     0,
     "Keyframe",
     "Strip containing keyframes on F-Curves"},
    {0, nullptr, 0, nullptr, nullptr},
};

#ifdef RNA_RUNTIME

#  include "ANIM_animation.hh"

#  include "DEG_depsgraph.hh"

#  include <fmt/format.h>

static animrig::Animation &rna_animation(const PointerRNA *ptr)
{
  return reinterpret_cast<Animation *>(ptr->owner_id)->wrap();
}

static animrig::Output &rna_data_output(const PointerRNA *ptr)
{
  return reinterpret_cast<AnimationOutput *>(ptr->data)->wrap();
}

static animrig::Layer &rna_data_layer(const PointerRNA *ptr)
{
  return reinterpret_cast<AnimationLayer *>(ptr->data)->wrap();
}

static animrig::Strip &rna_data_strip(const PointerRNA *ptr)
{
  return reinterpret_cast<AnimationStrip *>(ptr->data)->wrap();
}

static void rna_Animation_tag_animupdate(Main *, Scene *, PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  DEG_id_tag_update(&anim.id, ID_RECALC_ANIMATION);
}

static animrig::KeyframeStrip &rna_data_keyframe_strip(const PointerRNA *ptr)
{
  animrig::Strip &strip = reinterpret_cast<AnimationStrip *>(ptr->data)->wrap();
  return strip.as<animrig::KeyframeStrip>();
}

static animrig::ChannelBag &rna_data_channelbag(const PointerRNA *ptr)
{
  return reinterpret_cast<AnimationChannelBag *>(ptr->data)->wrap();
}

template<typename T>
static void rna_iterator_array_begin(CollectionPropertyIterator *iter, Span<T *> items)
{
  rna_iterator_array_begin(iter, (void *)items.data(), sizeof(T *), items.size(), 0, nullptr);
}

template<typename T>
static void rna_iterator_array_begin(CollectionPropertyIterator *iter, MutableSpan<T *> items)
{
  rna_iterator_array_begin(iter, (void *)items.data(), sizeof(T *), items.size(), 0, nullptr);
}

static AnimationOutput *rna_Animation_outputs_new(Animation *anim_id,
                                                  bContext *C,
                                                  ReportList *reports,
                                                  ID *animated_id)
{
  if (animated_id == nullptr) {
    BKE_report(reports,
               RPT_ERROR,
               "An output without animated ID cannot be created at the moment; if you need it, "
               "please file a bug report");
    return nullptr;
  }

  animrig::Animation &anim = anim_id->wrap();
  animrig::Output &output = anim.output_add();
  /* TODO: actually set output->idtype to this ID's type. */
  anim.output_name_set(output, animated_id->name);

  WM_event_add_notifier(C, NC_ANIMATION | ND_ANIMCHAN, nullptr);
  return &output;
}

static void rna_iterator_animation_layers_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  rna_iterator_array_begin(iter, anim.layers());
}

static int rna_iterator_animation_layers_length(PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  return anim.layers().size();
}

static AnimationLayer *rna_Animation_layers_new(Animation *dna_animation,
                                                bContext *C,
                                                ReportList *reports,
                                                const char *name)
{
  animrig::Animation &anim = dna_animation->wrap();

  if (anim.layers().size() >= 1) {
    /* Not allowed to have more than one layer, for now. This limitation is in
     * place until working with multiple animated IDs is fleshed out better. */
    BKE_report(reports, RPT_ERROR, "An Animation may not have more than one layer");
    return nullptr;
  }

  animrig::Layer &layer = anim.layer_add(name);

  WM_event_add_notifier(C, NC_ANIMATION | ND_ANIMCHAN, nullptr);
  return &layer;
}

void rna_Animation_layers_remove(Animation *dna_animation,
                                 bContext *C,
                                 ReportList *reports,
                                 AnimationLayer *dna_layer)
{
  animrig::Animation &anim = dna_animation->wrap();
  animrig::Layer &layer = dna_layer->wrap();
  if (!anim.layer_remove(layer)) {
    BKE_report(reports, RPT_ERROR, "This layer does not belong to this animation");
    return;
  }

  WM_event_add_notifier(C, NC_ANIMATION | ND_ANIMCHAN, nullptr);
  DEG_id_tag_update(&anim.id, ID_RECALC_ANIMATION);
}

static void rna_iterator_animation_outputs_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  rna_iterator_array_begin(iter, anim.outputs());
}

static int rna_iterator_animation_outputs_length(PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  return anim.outputs().size();
}

static std::optional<std::string> rna_AnimationOutput_path(const PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  animrig::Output &output_to_find = rna_data_output(ptr);

  Span<animrig::Output *> outputs = anim.outputs();
  for (int i = 0; i < outputs.size(); ++i) {
    animrig::Output &output = *outputs[i];
    if (&output != &output_to_find) {
      continue;
    }

    return fmt::format("outputs[{}]", i);
  }
  return std::nullopt;
}

static void rna_AnimationOutput_name_set(PointerRNA *ptr, const char *name)
{
  animrig::Animation &anim = rna_animation(ptr);
  animrig::Output &output = rna_data_output(ptr);

  anim.output_name_set(output, name);
}

static void rna_AnimationOutput_name_update(Main *bmain, Scene *, PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  animrig::Output &output = rna_data_output(ptr);

  anim.output_name_propagate(*bmain, output);
}

static std::optional<std::string> rna_AnimationLayer_path(const PointerRNA *ptr)
{
  animrig::Layer &layer = rna_data_layer(ptr);

  char name_esc[sizeof(layer.name) * 2];
  BLI_str_escape(name_esc, layer.name, sizeof(name_esc));
  return fmt::format("layers[\"{}\"]", name_esc);
}

static void rna_iterator_animationlayer_strips_begin(CollectionPropertyIterator *iter,
                                                     PointerRNA *ptr)
{
  animrig::Layer &layer = rna_data_layer(ptr);
  rna_iterator_array_begin(iter, layer.strips());
}

static int rna_iterator_animationlayer_strips_length(PointerRNA *ptr)
{
  animrig::Layer &layer = rna_data_layer(ptr);
  return layer.strips().size();
}

AnimationStrip *rna_AnimationStrips_new(AnimationLayer *dna_layer,
                                        bContext *C,
                                        ReportList *reports,
                                        const int type)
{
  const animrig::Strip::Type strip_type = animrig::Strip::Type(type);

  animrig::Layer &layer = dna_layer->wrap();

  if (layer.strips().size() >= 1) {
    /* Not allowed to have more than one strip, for now. This limitation is in
     * place until working with layers is fleshed out better. */
    BKE_report(reports, RPT_ERROR, "A layer may not have more than one strip");
    return nullptr;
  }

  animrig::Strip &strip = layer.strip_add(strip_type);

  WM_event_add_notifier(C, NC_ANIMATION | ND_ANIMCHAN, nullptr);
  return &strip;
}

void rna_AnimationStrips_remove(ID *animation_id,
                                AnimationLayer *dna_layer,
                                bContext *C,
                                ReportList *reports,
                                AnimationStrip *dna_strip)
{
  animrig::Layer &layer = dna_layer->wrap();
  animrig::Strip &strip = dna_strip->wrap();
  if (!layer.strip_remove(strip)) {
    BKE_report(reports, RPT_ERROR, "this strip does not belong to this layer");
    return;
  }

  WM_event_add_notifier(C, NC_ANIMATION | ND_ANIMCHAN, nullptr);
  DEG_id_tag_update(animation_id, ID_RECALC_ANIMATION);
}

static StructRNA *rna_AnimationStrip_refine(PointerRNA *ptr)
{
  animrig::Strip &strip = rna_data_strip(ptr);
  switch (strip.type()) {
    case animrig::Strip::Type::Keyframe:
      return &RNA_KeyframeAnimationStrip;
  }
  return &RNA_UnknownType;
}

static std::optional<std::string> rna_AnimationStrip_path(const PointerRNA *ptr)
{
  animrig::Animation &anim = rna_animation(ptr);
  animrig::Strip &strip_to_find = rna_data_strip(ptr);

  for (animrig::Layer *layer : anim.layers()) {
    Span<animrig::Strip *> strips = layer->strips();
    for (int i = 0; i < strips.size(); ++i) {
      animrig::Strip &strip = *strips[i];
      if (&strip != &strip_to_find) {
        continue;
      }

      PointerRNA layer_ptr = RNA_pointer_create(&anim.id, &RNA_AnimationLayer, layer);
      const std::optional<std::string> layer_path = rna_AnimationLayer_path(&layer_ptr);
      BLI_assert_msg(layer_path, "Every animation layer should have a valid RNA path.");
      const std::string strip_path = fmt::format("{}.strips[{}]", *layer_path, i);
      return strip_path;
    }
  }

  return std::nullopt;
}

static void rna_iterator_keyframestrip_channelbags_begin(CollectionPropertyIterator *iter,
                                                         PointerRNA *ptr)
{
  animrig::KeyframeStrip &key_strip = rna_data_keyframe_strip(ptr);
  rna_iterator_array_begin(iter, key_strip.channelbags());
}

static int rna_iterator_keyframestrip_channelbags_length(PointerRNA *ptr)
{
  animrig::KeyframeStrip &key_strip = rna_data_keyframe_strip(ptr);
  return key_strip.channelbags().size();
}

static void rna_iterator_ChannelBag_fcurves_begin(CollectionPropertyIterator *iter,
                                                  PointerRNA *ptr)
{
  animrig::ChannelBag &bag = rna_data_channelbag(ptr);
  rna_iterator_array_begin(iter, bag.fcurves());
}

static int rna_iterator_ChannelBag_fcurves_length(PointerRNA *ptr)
{
  animrig::ChannelBag &bag = rna_data_channelbag(ptr);
  return bag.fcurves().size();
}

static AnimationChannelBag *rna_KeyframeAnimationStrip_channels(
    KeyframeAnimationStrip *self, const animrig::output_handle_t output_handle)
{
  animrig::KeyframeStrip &key_strip = self->wrap();
  return key_strip.channelbag_for_output(output_handle);
}

#else

static void rna_def_animation_outputs(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;

  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "AnimationOutputs");
  srna = RNA_def_struct(brna, "AnimationOutputs", nullptr);
  RNA_def_struct_sdna(srna, "Animation");
  RNA_def_struct_ui_text(srna, "Animation Outputs", "Collection of animation outputs");

  /* Animation.outputs.new(...) */
  func = RNA_def_function(srna, "new", "rna_Animation_outputs_new");
  RNA_def_function_ui_description(func, "Add an output to the animation");
  RNA_def_function_flag(func, FUNC_USE_CONTEXT | FUNC_USE_REPORTS);
  parm = RNA_def_pointer(
      func, "animated_id", "ID", "Data-Block", "Data-block that will be animated by this output");

  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_pointer(func, "output", "AnimationOutput", "", "Newly created animation output");
  RNA_def_function_return(func, parm);
}

static void rna_def_animation_layers(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;

  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "AnimationLayers");
  srna = RNA_def_struct(brna, "AnimationLayers", nullptr);
  RNA_def_struct_sdna(srna, "Animation");
  RNA_def_struct_ui_text(srna, "Animation Layers", "Collection of animation layers");

  /* Animation.layers.new(...) */
  func = RNA_def_function(srna, "new", "rna_Animation_layers_new");
  RNA_def_function_flag(func, FUNC_USE_CONTEXT | FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Add a layer to the animation");
  parm = RNA_def_string(func,
                        "name",
                        nullptr,
                        sizeof(AnimationLayer::name) - 1,
                        "Name",
                        "Name of the layer, unique within the Animation data-block");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_pointer(func, "layer", "AnimationLayer", "", "Newly created animation layer");
  RNA_def_function_return(func, parm);

  /* Animation.layers.remove(layer) */
  func = RNA_def_function(srna, "remove", "rna_Animation_layers_remove");
  RNA_def_function_flag(func, FUNC_USE_CONTEXT | FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Remove the layer from the animation");
  parm = RNA_def_pointer(
      func, "anim_layer", "AnimationLayer", "Animation Layer", "The layer to remove");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
}

static void rna_def_animation(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "Animation", "ID");
  RNA_def_struct_sdna(srna, "Animation");
  RNA_def_struct_ui_text(srna, "Animation", "A collection of animation layers");
  RNA_def_struct_ui_icon(srna, ICON_ACTION);

  prop = RNA_def_property(srna, "last_output_handle", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);

  /* Collection properties .*/
  prop = RNA_def_property(srna, "outputs", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "AnimationOutput");
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_animation_outputs_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_animation_outputs_length",
                                    nullptr,
                                    nullptr,
                                    nullptr);
  RNA_def_property_ui_text(prop, "Outputs", "The list of data-blocks animated by this Animation");
  rna_def_animation_outputs(brna, prop);

  prop = RNA_def_property(srna, "layers", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "AnimationLayer");
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_animation_layers_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_animation_layers_length",
                                    nullptr,
                                    nullptr,
                                    nullptr);
  RNA_def_property_ui_text(prop, "Layers", "The list of layers that make up this Animation");
  rna_def_animation_layers(brna, prop);
}

static void rna_def_animation_output(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "AnimationOutput", nullptr);
  RNA_def_struct_path_func(srna, "rna_AnimationOutput_path");
  RNA_def_struct_ui_text(srna,
                         "Animation Output",
                         "Reference to a data-block that will be animated by this Animation");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_string_funcs(prop, nullptr, nullptr, "rna_AnimationOutput_name_set");
  RNA_def_property_update(prop, NC_ANIMATION | ND_ANIMCHAN, "rna_AnimationOutput_name_update");

  prop = RNA_def_property(srna, "handle", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
}

static void rna_def_animationlayer_strips(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;

  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "AnimationStrips");
  srna = RNA_def_struct(brna, "AnimationStrips", nullptr);
  RNA_def_struct_sdna(srna, "AnimationLayer");
  RNA_def_struct_ui_text(srna, "Animation Strips", "Collection of animation strips");

  /* Layer.strips.new(type='...') */
  func = RNA_def_function(srna, "new", "rna_AnimationStrips_new");
  RNA_def_function_ui_description(func, "Add a new infinite strip to the layer");
  RNA_def_function_flag(func, FUNC_USE_CONTEXT | FUNC_USE_REPORTS);
  parm = RNA_def_enum(func,
                      "type",
                      rna_enum_strip_type_items,
                      (int)animrig::Strip::Type::Keyframe,
                      "Type",
                      "The type of strip to create");
  /* Return value. */
  parm = RNA_def_pointer(func, "strip", "AnimationStrip", "", "Newly created animation strip");
  RNA_def_function_return(func, parm);

  /* Layer.strips.remove(strip) */
  func = RNA_def_function(srna, "remove", "rna_AnimationStrips_remove");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_CONTEXT | FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Remove the strip from the animation layer");
  parm = RNA_def_pointer(
      func, "anim_strip", "AnimationStrip", "Animation Strip", "The strip to remove");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
}

static void rna_def_animation_layer(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "AnimationLayer", nullptr);
  RNA_def_struct_ui_text(srna, "Animation Layer", "");
  RNA_def_struct_path_func(srna, "rna_AnimationLayer_path");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_struct_name_property(srna, prop);

  prop = RNA_def_property(srna, "influence", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_text(
      prop,
      "Influence",
      "How much of this layer is used when blending into the output of lower layers");
  RNA_def_property_ui_range(prop, 0.0, 1.0, 3, 2);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_update(prop, NC_ANIMATION | ND_ANIMCHAN, "rna_Animation_tag_animupdate");

  prop = RNA_def_property(srna, "mix_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "layer_mix_mode");
  RNA_def_property_ui_text(
      prop, "Mix Mode", "How animation of this layer is blended into the output of lower layers");
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_enum_items(prop, rna_enum_layer_mix_mode_items);
  RNA_def_property_update(prop, NC_ANIMATION | ND_ANIMCHAN, "rna_Animation_tag_animupdate");

  /* Collection properties .*/
  prop = RNA_def_property(srna, "strips", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "AnimationStrip");
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_animationlayer_strips_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_animationlayer_strips_length",
                                    nullptr,
                                    nullptr,
                                    nullptr);
  RNA_def_property_ui_text(prop, "Strips", "The list of strips that are on this animation layer");

  rna_def_animationlayer_strips(brna, prop);
}

static void rna_def_keyframestrip_channelbags(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;

  RNA_def_property_srna(cprop, "AnimationChannelBags");
  srna = RNA_def_struct(brna, "AnimationChannelBags", nullptr);
  RNA_def_struct_sdna(srna, "KeyframeAnimationStrip");
  RNA_def_struct_ui_text(srna,
                         "Animation Channels for Outputs",
                         "For each animation output, a list of animation channels");
}

static void rna_def_animation_keyframe_strip(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "KeyframeAnimationStrip", "AnimationStrip");
  RNA_def_struct_ui_text(
      srna, "Keyframe Animation Strip", "Strip with a set of FCurves for each animation output");

  prop = RNA_def_property(srna, "channelbags", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "AnimationChannelBag");
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_keyframestrip_channelbags_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_keyframestrip_channelbags_length",
                                    nullptr,
                                    nullptr,
                                    nullptr);
  rna_def_keyframestrip_channelbags(brna, prop);

  {
    FunctionRNA *func;
    PropertyRNA *parm;

    /* KeyframeStrip.channels(...). */
    func = RNA_def_function(srna, "channels", "rna_KeyframeAnimationStrip_channels");
    parm = RNA_def_int(func,
                       "output_handle",
                       0,
                       0,
                       INT_MAX,
                       "Output Handle",
                       "Number that identifies a specific animation output",
                       0,
                       INT_MAX);
    RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
    parm = RNA_def_pointer(func, "channels", "AnimationChannelBag", "Channels", "");
    RNA_def_function_return(func, parm);
  }
}

static void rna_def_animation_strip(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "AnimationStrip", nullptr);
  RNA_def_struct_ui_text(srna, "Animation Strip", "");
  RNA_def_struct_path_func(srna, "rna_AnimationStrip_path");
  RNA_def_struct_refine_func(srna, "rna_AnimationStrip_refine");

  static const EnumPropertyItem prop_type_items[] = {
      {int(animrig::Strip::Type::Keyframe),
       "KEYFRAME",
       0,
       "Keyframe",
       "Strip with a set of FCurves for each animation output"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "strip_type");
  RNA_def_property_enum_items(prop, prop_type_items);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);

  /* Define Strip subclasses. */
  rna_def_animation_keyframe_strip(brna);
}

static void rna_def_channelbag_for_output_fcurves(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;

  RNA_def_property_srna(cprop, "AnimationChannelBagFCurves");
  srna = RNA_def_struct(brna, "AnimationChannelBagFCurves", nullptr);
  RNA_def_struct_sdna(srna, "bAnimationChannelBag");
  RNA_def_struct_ui_text(
      srna, "F-Curves", "Collection of F-Curves for a specific animation output");
}

static void rna_def_animation_channelbags(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "AnimationChannelBag", nullptr);
  RNA_def_struct_ui_text(
      srna,
      "Animation Channel Bag",
      "Collection of animation channels, typically associated with an animation output");

  prop = RNA_def_property(srna, "output_handle", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);

  prop = RNA_def_property(srna, "fcurves", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_ChannelBag_fcurves_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_ChannelBag_fcurves_length",
                                    nullptr,
                                    nullptr,
                                    nullptr);
  RNA_def_property_struct_type(prop, "FCurve");
  RNA_def_property_ui_text(prop, "F-Curves", "The individual F-Curves that animate the output");
  rna_def_channelbag_for_output_fcurves(brna, prop);
}

void RNA_def_animation_id(BlenderRNA *brna)
{
  rna_def_animation(brna);
  rna_def_animation_output(brna);
  rna_def_animation_layer(brna);
  rna_def_animation_strip(brna);
  rna_def_animation_channelbags(brna);
}

#endif
