/* SPDX-FileCopyrightText: 2023 Blender Developers
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 *
 * \brief Animation data-block functionality.
 */
#pragma once

#include "ANIM_fcurve.hh"

#include "DNA_anim_types.h"

#include "BLI_math_vector.hh"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"

struct AnimationEvalContext;
struct FCurve;
struct ID;
struct Main;
struct PointerRNA;

namespace blender::animrig {

/* Forward declarations for the types defined later in this file. */
class Layer;
class Strip;
class Output;

/* Use an alias for the Output handle type to help disambiguate function parameters. */
using output_handle_t = decltype(::AnimationOutput::handle);

/**
 * Container of animation data for one or more animated IDs.
 *
 * Broadly an Animation consists of Layers, each Layer has Strips, and it's the
 * Strips that eventually contain the animation data.
 *
 * Temporary limitation: each Animation can only contain one Layer.
 *
 * Which sub-set of that data drives the animation of which ID is determined by
 * which Output is associated with that ID.
 *
 * \see AnimData::animation
 * \see AnimData::output_handle
 */
class Animation : public ::Animation {
 public:
  Animation() = default;
  /**
   * Copy constructor is deleted, as code should use regular ID library
   * management functions to duplicate this data-block.
   */
  Animation(const Animation &other) = delete;

  /**
   * Duplicate all Animation data from 'other' into `this`.
   *
   * Only used by the ID datablock management, other code should use regular ID
   * library management functions to duplicate this data-block.
   */
  void copy_from(const Animation &other);

  /* Animation Layers access. */
  blender::Span<const Layer *> layers() const;
  blender::MutableSpan<Layer *> layers();
  const Layer *layer(int64_t index) const;
  Layer *layer(int64_t index);

  Layer &layer_add(StringRefNull name);

  /**
   * Remove the layer from this animation.
   *
   * After this call, the passed reference is no longer valid, as the memory
   * will have been freed. Any strips on the layer will be freed too.
   *
   * \return true when the layer was found & removed, false if it wasn't found. */
  bool layer_remove(Layer &layer_to_remove);

  /* Animation Output access. */
  blender::Span<const Output *> outputs() const;
  blender::MutableSpan<Output *> outputs();
  const Output *output(int64_t index) const;
  Output *output(int64_t index);

  Output *output_for_handle(output_handle_t handle);
  const Output *output_for_handle(output_handle_t handle) const;

  /**
   * Set the output name.
   *
   * This has to be done on the Animation level to ensure each output has a
   * unique name within the Animation.
   *
   * \see Animation::output_name_propagate
   */
  void output_name_set(Output &out, StringRefNull new_name);

  /**
   * Update the `AnimData::animation_output_name` field of any ID that is animated by this.Output.
   *
   * Should be called after `output_name_set(out)`. This is implemented as a separate function due
   * to the need to access bmain, which is available in the RNA on-property-update handler, but not
   * in the RNA property setter. */
  void output_name_propagate(Main &bmain, const Output &out);

  Output *output_find_by_name(StringRefNull output_name);

  Output *output_for_id(const ID &animated_id);
  const Output *output_for_id(const ID &animated_id) const;

  Output &output_add();
  /** Free all data in the `Animation`. Doesn't delete the `Animation` itself. */
  void free_data();

 protected:
  /** Return the layer's index, or -1 if not found in this animation. */
  int64_t find_layer_index(const Layer &layer) const;

 private:
  Output &output_allocate();
};
static_assert(sizeof(Animation) == sizeof(::Animation),
              "DNA struct and its C++ wrapper must have the same size");

/**
 * Strips contain the actual animation data.
 *
 * Although the data model allows for different strip types, currently only a
 * single type is implemented: keyframe strips.
 */
class Strip : public ::AnimationStrip {
 public:
  Strip() = default;
  /**
   * Strip cannot be duplicated via the copy constructor. Either use a concrete
   * strip type's copy constructor, or use Strip::duplicate().
   */
  Strip(const Strip &other) = delete;
  ~Strip();

  Strip *duplicate(StringRefNull allocation_name) const;

  enum class Type : int8_t { Keyframe = 0 };

  /**
   * Strip type, so it's known which subclass this can be wrapped in without
   * having to rely on C++ RTTI.
   */
  Type type() const
  {
    return static_cast<Type>(this->strip_type);
  }

  template<typename T> bool is() const;
  template<typename T> T &as();
  template<typename T> const T &as() const;
};
static_assert(sizeof(Strip) == sizeof(::AnimationStrip),
              "DNA struct and its C++ wrapper must have the same size");

/**
 * Layers can be stacked on top of each other to define the animation. Each
 * layer has a mix mode and an influence (0-1), which define how it is mixed
 * with the layers below it.
 *
 * Layers contain one or more Strips, which in turn contain the animation data
 * itself.
 *
 * Temporary limitation: at most one strip may exist on a layer, and it extends
 * from negative to positive infinity.
 */
class Layer : public ::AnimationLayer {
 public:
  Layer() = default;
  Layer(const Layer &other);
  ~Layer();

  enum class Flags : uint8_t {
    /* Set by default, cleared to mute. */
    Enabled = (1 << 0),
    /* When adding/removing a flag, also update the ENUM_OPERATORS() invocation below. */
  };

  Flags flags() const
  {
    return static_cast<Flags>(this->layer_flags);
  }

  enum class MixMode : int8_t {
    Replace = 0,
    Offset = 1,
    Add = 2,
    Subtract = 3,
    Multiply = 4,
  };

  MixMode mix_mode() const
  {
    return static_cast<MixMode>(this->layer_mix_mode);
  }

  /* Strip access. */
  blender::Span<const Strip *> strips() const;
  blender::MutableSpan<Strip *> strips();
  const Strip *strip(int64_t index) const;
  Strip *strip(int64_t index);
  Strip &strip_add(Strip::Type strip_type);

  /**
   * Remove the strip from this layer.
   *
   * After this call, the passed reference is no longer valid, as the memory
   * will have been freed.
   *
   * \return true when the strip was found & removed, false if it wasn't found. */
  bool strip_remove(Strip &strip);

 protected:
  /** Return the strip's index, or -1 if not found in this layer. */
  int64_t find_strip_index(const Strip &strip) const;
};
static_assert(sizeof(Layer) == sizeof(::AnimationLayer),
              "DNA struct and its C++ wrapper must have the same size");

ENUM_OPERATORS(Layer::Flags, Layer::Flags::Enabled);

/**
 * Identifier for a sub-set of the animation data inside an Animation data-block.
 *
 * An animatable ID specifies both an `Animation*` and an `AnimationOutput::handle`
 * to identify which F-Curves (and in the future other animation data) it will
 * be animated by.
 *
 * This is called an 'output' because it acts like an output socket of the
 * Animation data-block, into which an animatable ID can be noodled.
 *
 * \see AnimData::output_handle
 */
class Output : public ::AnimationOutput {
 public:
  Output() = default;
  Output(const Output &other) = default;
  ~Output() = default;

  /** Return whether this Output is usable by this ID type. */
  bool is_suitable_for(const ID &animated_id) const;
};
static_assert(sizeof(Output) == sizeof(::AnimationOutput),
              "DNA struct and its C++ wrapper must have the same size");

/**
 * KeyframeStrips effectively contain a bag of F-Curves for each Output.
 */
class KeyframeStrip : public ::KeyframeAnimationStrip {
 public:
  KeyframeStrip() = default;
  KeyframeStrip(const KeyframeStrip &other);
  ~KeyframeStrip();

  /* ChannelBag array access. */
  blender::Span<const ChannelBag *> channelbags() const;
  blender::MutableSpan<ChannelBag *> channelbags();
  const ChannelBag *channelbag(int64_t index) const;
  ChannelBag *channelbag(int64_t index);

  /**
   * Find the animation channels for this output.
   *
   * \return nullptr if there is none yet for this output.
   */
  const ChannelBag *channelbag_for_output(const Output &out) const;
  ChannelBag *channelbag_for_output(const Output &out);
  const ChannelBag *channelbag_for_output(output_handle_t output_handle) const;
  ChannelBag *channelbag_for_output(output_handle_t output_handle);

  /**
   * Add the animation channels for this output.
   *
   * Should only be called when there is no `ChannelBag` for this output yet.
   */
  ChannelBag &channelbag_for_output_add(const Output &out);
};
static_assert(sizeof(KeyframeStrip) == sizeof(::KeyframeAnimationStrip),
              "DNA struct and its C++ wrapper must have the same size");

template<> KeyframeStrip &Strip::as<KeyframeStrip>();
template<> const KeyframeStrip &Strip::as<KeyframeStrip>() const;

/**
 * Bag of F-Curves for a specific Output handle.
 */
class ChannelBag : public ::AnimationChannelBag {
 public:
  ChannelBag() = default;
  ChannelBag(const ChannelBag &other);
  ~ChannelBag();

  /* FCurves access. */
  blender::Span<const FCurve *> fcurves() const;
  blender::MutableSpan<FCurve *> fcurves();
  const FCurve *fcurve(int64_t index) const;
  FCurve *fcurve(int64_t index);

  const FCurve *fcurve_find(StringRefNull rna_path, int array_index) const;
};
static_assert(sizeof(ChannelBag) == sizeof(::AnimationChannelBag),
              "DNA struct and its C++ wrapper must have the same size");

}  // namespace blender::animrig

/* Wrap functions for the DNA structs. */

inline blender::animrig::Animation &Animation::wrap()
{
  return *reinterpret_cast<blender::animrig::Animation *>(this);
}
inline const blender::animrig::Animation &Animation::wrap() const
{
  return *reinterpret_cast<const blender::animrig::Animation *>(this);
}

inline blender::animrig::Layer &AnimationLayer::wrap()
{
  return *reinterpret_cast<blender::animrig::Layer *>(this);
}
inline const blender::animrig::Layer &AnimationLayer::wrap() const
{
  return *reinterpret_cast<const blender::animrig::Layer *>(this);
}

inline blender::animrig::Output &AnimationOutput::wrap()
{
  return *reinterpret_cast<blender::animrig::Output *>(this);
}
inline const blender::animrig::Output &AnimationOutput::wrap() const
{
  return *reinterpret_cast<const blender::animrig::Output *>(this);
}

inline blender::animrig::Strip &AnimationStrip::wrap()
{
  return *reinterpret_cast<blender::animrig::Strip *>(this);
}
inline const blender::animrig::Strip &AnimationStrip::wrap() const
{
  return *reinterpret_cast<const blender::animrig::Strip *>(this);
}

inline blender::animrig::KeyframeStrip &KeyframeAnimationStrip::wrap()
{
  return *reinterpret_cast<blender::animrig::KeyframeStrip *>(this);
}
inline const blender::animrig::KeyframeStrip &KeyframeAnimationStrip::wrap() const
{
  return *reinterpret_cast<const blender::animrig::KeyframeStrip *>(this);
}

inline blender::animrig::ChannelBag &AnimationChannelBag::wrap()
{
  return *reinterpret_cast<blender::animrig::ChannelBag *>(this);
}
inline const blender::animrig::ChannelBag &AnimationChannelBag::wrap() const
{
  return *reinterpret_cast<const blender::animrig::ChannelBag *>(this);
}
