/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_AUDASPACE
#  include <AUD_Device.h>
#  include <AUD_Handle.h>
#  include <AUD_Sound.h>
#endif

#include "BLI_array.hh"
#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_task.hh"

#include "BKE_attribute_math.hh"
#include "BKE_main.hh"
#include "BKE_sound.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_socket_search_link.hh"

#include "DNA_sound_types.h"

#include "DNA_ID.h"
#include "DNA_packedFile_types.h"

#include "node_geometry_util.hh"
#include <BKE_scene.hh>
#include <DEG_depsgraph_query.hh>

namespace blender::nodes {

}  // namespace blender::nodes

namespace blender::nodes::node_geo_sample_sound_cc {

NODE_STORAGE_FUNCS(NodeGeometrySampleSound);

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  // TODO: complete all functionalities
  b.add_input<decl::Sound>("Sound");
  b.add_input<decl::Float>("Time").min(0).supports_field();
  // b.add_input<decl::Float>("Start Frequency").min(0).supports_field();
  // b.add_input<decl::Float>("End Frequency").min(0).supports_field();
  b.add_output<decl::Float>("Amplitude").dependent_field({1});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  // TODO: figure out what needs to make a switch
  // uiItemR(layout, ptr, "spec_chan", UI_ITEM_NONE, nullptr, ICON_NONE);
  // uiItemR(layout, ptr, "spec_freq", UI_ITEM_NONE, nullptr, ICON_NONE);
  UNUSED_VARS(layout);
  UNUSED_VARS(ptr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySampleSound *data = MEM_cnew<NodeGeometrySampleSound>(__func__);
  data->spec_chan = 0;
  data->spec_freq = 0;
  node->storage = data;
}

#ifdef WITH_AUDASPACE

class SampleSoundFunction : public mf::MultiFunction {
  const bSound *sound_;
  const bool spec_chan_;
  const bool spec_freq_;
  const double frame_rate_;

  AUD_Device *device_;
  AUD_Handle *handle_;

 public:
  SampleSoundFunction(const bSound *sound,
                      const bool spec_chan,
                      const bool spec_freq,
                      const bool frame_rate)
      : sound_(sound), spec_chan_(spec_chan), spec_freq_(spec_freq), frame_rate_(frame_rate)
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Sample Sound", signature};
      builder.single_input<float>("Time");
      // TODO: frequency domain
      // builder.single_input<float>("Start Frequency");
      // builder.single_input<float>("End Frequency");
      builder.single_output<float>("Amplitude");
      return signature;
    }();
    this->set_signature(&signature);

    this->init();
  }

  ~SampleSoundFunction()
  {
    if (!sound_) {
      return;
    }

    AUD_Handle_free(handle_);
    AUD_Device_free(device_);
  }

  void init()
  {
    if (!sound_) {
      return;
    }

    AUD_DeviceSpecs specs{
        .format = AUD_FORMAT_FLOAT32,
        .specs{
            .rate = AUD_SampleRate(sound_->samplerate), .channels = AUD_CHANNELS_MONO,
            // TODO: support specifying channels
            // .channels = spec_chan_ ? AUD_Channels(sound_->audio_channels) : AUD_CHANNELS_MONO,
        }};

    device_ = AUD_Device_open("read", specs, 1024, "Sample Sound");
    handle_ = AUD_Device_play(device_, sound_->handle, true);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &times = params.readonly_single_input<float>(0, "Time");
    MutableSpan<float> dst = params.uninitialized_single_output<float>(1, "Amplitude");

    mask.foreach_index([&](int index) {
      if (!sound_) {
        dst[index] = 0;
        return;
      }

      // XXX: clamp should probably be handled by user
      if (times[index] < 0) {
        dst[index] = 0;
        return;
      }

      // TODO: support variable sampling step instead of simply rely on scene fps and the magic
      // number (i.e. the `5` here); maybe combine with temporal smoothing and/or ADSR envelopes?
      const int n_sample = sound_->samplerate / frame_rate_ / 5;
      Array<float> buf(n_sample);

      AUD_Device_lock(device_);
      AUD_Handle_setPosition(handle_, times[index]);
      AUD_Device_read(device_, (unsigned char *)buf.data(), n_sample);
      AUD_Device_unlock(device_);

      if (spec_freq_) {
        // TODO: fft, windowing, temporal smoothing, etc.
        dst[index] = buf[0];
      }
      else {
        // TODO: find a better averaging method, and try to vectorize
        float avg = 0;
        for (float sample : buf) {
          avg += math::abs(sample) / n_sample;
        }
        dst[index] = avg;
      }
    });
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const NodeGeometrySampleSound &storage = node_storage(params.node());
  const bool spec_chan = bool(storage.spec_chan);
  const bool spec_freq = bool(storage.spec_freq);

  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  const double frame_rate = double(scene->r.frs_sec) / double(scene->r.frs_sec_base);

  bSound *sound = params.extract_input<bSound *>("Sound");
  Field<float> times = params.extract_input<Field<float>>("Time");

  auto fn = std::make_shared<SampleSoundFunction>(sound, spec_chan, spec_freq, frame_rate);
  auto op = FieldOperation::Create(std::move(fn), {std::move(times)});
  params.set_output("Amplitude", std::move(Field<float>(op)));
}

#else  // WITH_AUDASPACE

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Amplitude", fn::make_constant_field<float>(0));
}

#endif

static void node_register()
{
  static blender::bke::bNodeType ntype;

  // TODO: new SOUND node class
  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_SOUND, "Sample Sound", NODE_CLASS_SOUND);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  blender::bke::node_type_storage(
      &ntype, "NodeGeometrySampleSound", node_free_standard_storage, node_copy_standard_storage);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sample_sound_cc
