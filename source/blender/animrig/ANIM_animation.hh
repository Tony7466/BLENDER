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

  /* Animation Output access. */
  blender::Span<const Output *> outputs() const;
  blender::MutableSpan<Output *> outputs();
  const Output *output(int64_t index) const;
  Output *output(int64_t index);

  /** Free all data in the `Animation`. Doesn't delete the `Animation` itself. */
  void free_data();
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
