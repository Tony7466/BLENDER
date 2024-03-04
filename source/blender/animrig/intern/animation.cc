/* SPDX-FileCopyrightText: 2023 Blender Developers
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_anim_defaults.h"
#include "DNA_anim_types.h"
#include "DNA_array_utils.hh"
#include "DNA_defaults.h"

#include "BLI_listbase.h"
#include "BLI_listbase_wrapper.hh"
#include "BLI_math_base.h"
#include "BLI_string.h"
#include "BLI_string_utf8.h"
#include "BLI_string_utils.hh"

#include "BKE_anim_data.hh"
#include "BKE_animation.hh"
#include "BKE_fcurve.h"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"

#include "ED_keyframing.hh"

#include "MEM_guardedalloc.h"

#include "atomic_ops.h"

#include "ANIM_animation.hh"
#include "ANIM_fcurve.hh"

#include <cstdio>
#include <cstring>

namespace blender::animrig {

static animrig::Layer &animationlayer_alloc()
{
  AnimationLayer *layer = DNA_struct_default_alloc(AnimationLayer);
  return layer->wrap();
}
static animrig::Strip &animationstrip_alloc_infinite(const Strip::Type type)
{
  AnimationStrip *strip;
  switch (type) {
    case Strip::Type::Keyframe: {
      KeyframeAnimationStrip *key_strip = MEM_new<KeyframeAnimationStrip>(__func__);
      strip = &key_strip->strip;
      break;
    }
  }

  BLI_assert_msg(strip, "unsupported strip type");

  /* Copy the default AnimationStrip fields into the allocated data-block. */
  memcpy(strip, DNA_struct_default_get(AnimationStrip), sizeof(*strip));
  return strip->wrap();
}

/* Copied from source/blender/blenkernel/intern/grease_pencil.cc. It also has a shrink_array()
 * function, if we ever need one (we will).
 * Keep an eye on DNA_array_utils.hh; we may want to move these functions in there. */
template<typename T> static void grow_array(T **array, int *num, const int add_num)
{
  BLI_assert(add_num > 0);
  const int new_array_num = *num + add_num;
  T *new_array = reinterpret_cast<T *>(
      MEM_cnew_array<T *>(new_array_num, "animrig::animation/grow_array"));

  blender::uninitialized_relocate_n(*array, *num, new_array);
  if (*array != nullptr) {
    MEM_freeN(*array);
  }

  *array = new_array;
  *num = new_array_num;
}

template<typename T> static void grow_array_and_append(T **array, int *num, T item)
{
  grow_array(array, num, 1);
  (*array)[*num - 1] = item;
}

template<typename T> static void shrink_array(T **array, int *num, const int shrink_num)
{
  BLI_assert(shrink_num > 0);
  const int new_array_num = *num - shrink_num;
  T *new_array = reinterpret_cast<T *>(MEM_cnew_array<T *>(new_array_num, __func__));

  blender::uninitialized_move_n(*array, new_array_num, new_array);
  MEM_freeN(*array);

  *array = new_array;
  *num = new_array_num;
}

/* ----- Animation implementation ----------- */

void Animation::copy_from(const Animation &other)
{
  /* Copy all simple properties. */
  this->layer_array_num = other.layer_array_num;
  this->layer_active_index = other.layer_active_index;
  this->output_array_num = other.output_array_num;
  this->last_output_handle = other.last_output_handle;

  /* Layers. */
  this->layer_array = MEM_cnew_array<AnimationLayer *>(other.layer_array_num, __func__);
  for (int i : other.layers().index_range()) {
    this->layer_array[i] = MEM_new<animrig::Layer>(__func__, *other.layer(i));
  }

  /* Outputs. */
  this->output_array = MEM_cnew_array<AnimationOutput *>(other.output_array_num, __func__);
  for (int i : other.outputs().index_range()) {
    this->output_array[i] = MEM_new<animrig::Output>(__func__, *other.output(i));
  }
}

blender::Span<const Layer *> Animation::layers() const
{
  return blender::Span<Layer *>{reinterpret_cast<Layer **>(this->layer_array),
                                this->layer_array_num};
}
blender::MutableSpan<Layer *> Animation::layers()
{
  return blender::MutableSpan<Layer *>{reinterpret_cast<Layer **>(this->layer_array),
                                       this->layer_array_num};
}
const Layer *Animation::layer(const int64_t index) const
{
  return &this->layer_array[index]->wrap();
}
Layer *Animation::layer(const int64_t index)
{
  return &this->layer_array[index]->wrap();
}

Layer &Animation::layer_add(const StringRefNull name)
{
  using namespace blender::animrig;

  Layer &new_layer = animationlayer_alloc();
  STRNCPY_UTF8(new_layer.name, name.c_str());

  grow_array_and_append<::AnimationLayer *>(
      &this->layer_array, &this->layer_array_num, &new_layer);
  this->layer_active_index = this->layer_array_num - 1;

  return new_layer;
}

static void layer_ptr_destructor(AnimationLayer **dna_layer_ptr)
{
  Layer &layer = (*dna_layer_ptr)->wrap();
  MEM_delete(&layer);
};

bool Animation::layer_remove(Layer &layer_to_remove)
{
  const int64_t layer_index = this->find_layer_index(layer_to_remove);
  if (layer_index < 0) {
    return false;
  }

  dna::array::remove_index(&this->layer_array,
                           &this->layer_array_num,
                           &this->layer_active_index,
                           layer_index,
                           layer_ptr_destructor);
  return true;
}

int64_t Animation::find_layer_index(const Layer &layer) const
{
  for (const int64_t layer_index : this->layers().index_range()) {
    const Layer *visit_layer = this->layer(layer_index);
    if (visit_layer == &layer) {
      return layer_index;
    }
  }
  return -1;
}

blender::Span<const Output *> Animation::outputs() const
{
  return blender::Span<Output *>{reinterpret_cast<Output **>(this->output_array),
                                 this->output_array_num};
}
blender::MutableSpan<Output *> Animation::outputs()
{
  return blender::MutableSpan<Output *>{reinterpret_cast<Output **>(this->output_array),
                                        this->output_array_num};
}
const Output *Animation::output(const int64_t index) const
{
  return &this->output_array[index]->wrap();
}
Output *Animation::output(const int64_t index)
{
  return &this->output_array[index]->wrap();
}

Output *Animation::output_for_handle(const output_handle_t handle)
{
  const Output *out = const_cast<const Animation *>(this)->output_for_handle(handle);
  return const_cast<Output *>(out);
}

const Output *Animation::output_for_handle(const output_handle_t handle) const
{
  /* TODO: implement hashmap lookup. */
  for (const Output *out : outputs()) {
    if (out->handle == handle) {
      return out;
    }
  }
  return nullptr;
}

static void anim_output_name_ensure_unique(Animation &animation, Output &out)
{
  /* Cannot capture parameters by reference in the lambda, as that would change its signature
   * and no longer be compatible with BLI_uniquename_cb(). That's why this struct is necessary. */
  struct DupNameCheckData {
    Animation &anim;
    Output &out;
  };
  DupNameCheckData check_data = {animation, out};

  auto check_name_is_used = [](void *arg, const char *name) -> bool {
    DupNameCheckData *data = static_cast<DupNameCheckData *>(arg);
    for (const Output *output : data->anim.outputs()) {
      if (output == &data->out) {
        /* Don't compare against the output that's being renamed. */
        continue;
      }
      if (STREQ(output->name, name)) {
        return true;
      }
    }
    return false;
  };

  BLI_uniquename_cb(check_name_is_used, &check_data, "", '.', out.name, sizeof(out.name));
}

void Animation::output_name_set(Output &out, const StringRefNull new_name)
{
  STRNCPY_UTF8(out.name, new_name.c_str());
  anim_output_name_ensure_unique(*this, out);
}

void Animation::output_name_propagate(Main &bmain, const Output &out)
{
  /* Just loop over all animatable IDs in the main dataabase. */
  ListBase *lb;
  ID *id;
  FOREACH_MAIN_LISTBASE_BEGIN (&bmain, lb) {
    FOREACH_MAIN_LISTBASE_ID_BEGIN (lb, id) {
      if (!id_can_have_animdata(id)) {
        /* This ID type cannot have any animation, so ignore all and continue to
         * the next ID type. */
        break;
      }

      AnimData *adt = BKE_animdata_from_id(id);
      if (!adt || adt->animation != this) {
        /* Not animated by this Animation. */
        continue;
      }
      if (adt->output_handle != out.handle) {
        /* Not animated by this Output. */
        continue;
      }

      /* Ensure the Output name on the AnimData is correct. */
      STRNCPY_UTF8(adt->output_name, out.name);
    }
    FOREACH_MAIN_LISTBASE_ID_END;
  }
  FOREACH_MAIN_LISTBASE_END;
}

Output *Animation::output_find_by_name(const StringRefNull output_name)
{
  for (Output *out : outputs()) {
    if (STREQ(out->name, output_name.c_str())) {
      return out;
    }
  }
  return nullptr;
}

Output *Animation::output_for_id(const ID &animated_id)
{
  const Output *out = const_cast<const Animation *>(this)->output_for_id(animated_id);
  return const_cast<Output *>(out);
}

const Output *Animation::output_for_id(const ID &animated_id) const
{
  const AnimData *adt = BKE_animdata_from_id(&animated_id);

  /* Note that there is no check that `adt->animation` is actually `this`. */

  const Output *out = this->output_for_handle(adt->output_handle);
  if (!out) {
    return nullptr;
  }
  if (!out->is_suitable_for(animated_id)) {
    return nullptr;
  }
  return out;
}

Output &Animation::output_allocate()
{
  Output &output = MEM_new<AnimationOutput>(__func__)->wrap();
  this->last_output_handle++;
  BLI_assert_msg(this->last_output_handle > 0, "Animation Output handle 32-bit overflow");
  output.handle = this->last_output_handle;
  return output;
}

Output &Animation::output_add()
{
  Output &output = this->output_allocate();

  /* Append the Output to the animation data-block. */
  grow_array_and_append<::AnimationOutput *>(
      &this->output_array, &this->output_array_num, &output);

  return output;
}

void Animation::free_data()
{
  /* Free layers. */
  for (Layer *layer : this->layers()) {
    MEM_delete(layer);
  }
  MEM_SAFE_FREE(this->layer_array);
  this->layer_array_num = 0;

  /* Free outputs. */
  for (Output *output : this->outputs()) {
    MEM_delete(output);
  }
  MEM_SAFE_FREE(this->output_array);
  this->output_array_num = 0;
}

/* ----- AnimationLayer implementation ----------- */

Layer::Layer(const Layer &other)
{
  memcpy(this, &other, sizeof(*this));

  /* Strips. */
  this->strip_array = MEM_cnew_array<AnimationStrip *>(other.strip_array_num, __func__);
  for (int i : other.strips().index_range()) {
    this->strip_array[i] = other.strip(i)->duplicate(__func__);
  }
}

Layer::~Layer()
{
  for (Strip *strip : this->strips()) {
    MEM_delete(strip);
  }
  MEM_SAFE_FREE(this->strip_array);
  this->strip_array_num = 0;
}

blender::Span<const Strip *> Layer::strips() const
{
  return blender::Span<Strip *>{reinterpret_cast<Strip **>(this->strip_array),
                                this->strip_array_num};
}
blender::MutableSpan<Strip *> Layer::strips()
{
  return blender::MutableSpan<Strip *>{reinterpret_cast<Strip **>(this->strip_array),
                                       this->strip_array_num};
}
const Strip *Layer::strip(const int64_t index) const
{
  return &this->strip_array[index]->wrap();
}
Strip *Layer::strip(const int64_t index)
{
  return &this->strip_array[index]->wrap();
}

Strip &Layer::strip_add(const Strip::Type strip_type)
{
  Strip &strip = animationstrip_alloc_infinite(strip_type);

  /* Add the new strip to the strip array. */
  grow_array_and_append<::AnimationStrip *>(&this->strip_array, &this->strip_array_num, &strip);

  return strip;
}

static void strip_ptr_destructor(AnimationStrip **dna_strip_ptr)
{
  Strip &strip = (*dna_strip_ptr)->wrap();
  MEM_delete(&strip);
};

bool Layer::strip_remove(Strip &strip_to_remove)
{
  const int64_t strip_index = this->find_strip_index(strip_to_remove);
  if (strip_index < 0) {
    return false;
  }

  dna::array::remove_index(
      &this->strip_array, &this->strip_array_num, nullptr, strip_index, strip_ptr_destructor);

  return true;
}

int64_t Layer::find_strip_index(const Strip &strip) const
{
  for (const int64_t strip_index : this->strips().index_range()) {
    const Strip *visit_strip = this->strip(strip_index);
    if (visit_strip == &strip) {
      return strip_index;
    }
  }
  return -1;
}

/* ----- AnimationOutput implementation ----------- */

bool Output::is_suitable_for(const ID &animated_id) const
{
  /* Check that the ID type is compatible with this output. */
  const int animated_idtype = GS(animated_id.name);
  return this->idtype == 0 || this->idtype == animated_idtype;
}

/* ----- AnimationStrip implementation ----------- */

Strip *Strip::duplicate(const StringRefNull allocation_name) const
{
  switch (this->type()) {
    case Type::Keyframe: {
      const KeyframeStrip &source = this->as<KeyframeStrip>();
      KeyframeStrip *copy = MEM_new<KeyframeStrip>(allocation_name.c_str(), source);
      return &copy->strip.wrap();
    }
  }
  BLI_assert_unreachable();
}

Strip::~Strip()
{
  switch (this->type()) {
    case Type::Keyframe:
      this->as<KeyframeStrip>().~KeyframeStrip();
      return;
  }
  BLI_assert_unreachable();
}

/* ----- KeyframeAnimationStrip implementation ----------- */

KeyframeStrip::KeyframeStrip(const KeyframeStrip &other)
{
  memcpy(this, &other, sizeof(*this));

  this->channelbags_array = MEM_cnew_array<AnimationChannelBag *>(other.channelbags_array_num,
                                                                  __func__);
  Span<const ChannelBag *> channelbags_src = other.channelbags();
  for (int i : channelbags_src.index_range()) {
    this->channelbags_array[i] = MEM_new<animrig::ChannelBag>(__func__, *other.channelbag(i));
  }
}

KeyframeStrip::~KeyframeStrip()
{
  for (ChannelBag *channelbag_for_output : this->channelbags()) {
    MEM_delete(channelbag_for_output);
  }
  MEM_SAFE_FREE(this->channelbags_array);
  this->channelbags_array_num = 0;
}

template<> bool Strip::is<KeyframeStrip>() const
{
  return type() == Type::Keyframe;
}

template<> KeyframeStrip &Strip::as<KeyframeStrip>()
{
  BLI_assert_msg(this->is<KeyframeStrip>(), "Strip is not a KeyframeStrip");
  return *reinterpret_cast<KeyframeStrip *>(this);
}

template<> const KeyframeStrip &Strip::as<KeyframeStrip>() const
{
  BLI_assert_msg(this->is<KeyframeStrip>(), "Strip is not a KeyframeStrip");
  return *reinterpret_cast<const KeyframeStrip *>(this);
}

blender::Span<const ChannelBag *> KeyframeStrip::channelbags() const
{
  return blender::Span<ChannelBag *>{reinterpret_cast<ChannelBag **>(this->channelbags_array),
                                     this->channelbags_array_num};
}
blender::MutableSpan<ChannelBag *> KeyframeStrip::channelbags()
{
  return blender::MutableSpan<ChannelBag *>{
      reinterpret_cast<ChannelBag **>(this->channelbags_array), this->channelbags_array_num};
}
const ChannelBag *KeyframeStrip::channelbag(const int64_t index) const
{
  return &this->channelbags_array[index]->wrap();
}
ChannelBag *KeyframeStrip::channelbag(const int64_t index)
{
  return &this->channelbags_array[index]->wrap();
}
const ChannelBag *KeyframeStrip::channelbag_for_output(const output_handle_t output_handle) const
{
  for (const ChannelBag *channels : this->channelbags()) {
    if (channels->output_handle == output_handle) {
      return channels;
    }
  }
  return nullptr;
}
ChannelBag *KeyframeStrip::channelbag_for_output(const output_handle_t output_handle)
{
  const auto *const_this = const_cast<const KeyframeStrip *>(this);
  const auto *const_channels = const_this->channelbag_for_output(output_handle);
  return const_cast<ChannelBag *>(const_channels);
}
const ChannelBag *KeyframeStrip::channelbag_for_output(const Output &out) const
{
  return this->channelbag_for_output(out.handle);
}
ChannelBag *KeyframeStrip::channelbag_for_output(const Output &out)
{
  return this->channelbag_for_output(out.handle);
}

ChannelBag &KeyframeStrip::channelbag_for_output_add(const Output &out)
{
#ifndef NDEBUG
  BLI_assert_msg(channelbag_for_output(out) == nullptr,
                 "Cannot add chans-for-out for already-registered output");
#endif

  ChannelBag &channels = MEM_new<AnimationChannelBag>(__func__)->wrap();
  channels.output_handle = out.handle;

  grow_array_and_append<AnimationChannelBag *>(
      &this->channelbags_array, &this->channelbags_array_num, &channels);

  return channels;
}

/* AnimationChannelBag implementation. */

ChannelBag::ChannelBag(const ChannelBag &other)
{
  this->output_handle = other.output_handle;
  this->fcurve_array_num = other.fcurve_array_num;

  this->fcurve_array = MEM_cnew_array<FCurve *>(other.fcurve_array_num, __func__);
  for (int i = 0; i < other.fcurve_array_num; i++) {
    const FCurve *fcu_src = other.fcurve_array[i];
    this->fcurve_array[i] = BKE_fcurve_copy(fcu_src);
  }
}

ChannelBag::~ChannelBag()
{
  for (FCurve *fcu : this->fcurves()) {
    BKE_fcurve_free(fcu);
  }
  MEM_SAFE_FREE(this->fcurve_array);
  this->fcurve_array_num = 0;
}

blender::Span<const FCurve *> ChannelBag::fcurves() const
{
  return blender::Span<FCurve *>{reinterpret_cast<FCurve **>(this->fcurve_array),
                                 this->fcurve_array_num};
}
blender::MutableSpan<FCurve *> ChannelBag::fcurves()
{
  return blender::MutableSpan<FCurve *>{reinterpret_cast<FCurve **>(this->fcurve_array),
                                        this->fcurve_array_num};
}
const FCurve *ChannelBag::fcurve(const int64_t index) const
{
  return this->fcurve_array[index];
}
FCurve *ChannelBag::fcurve(const int64_t index)
{
  return this->fcurve_array[index];
}

}  // namespace blender::animrig
