/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_AUDASPACE
#  include <AUD_Device.h>
#  include <AUD_Handle.h>
#  include <AUD_Sound.h>
#endif

#ifdef WITH_FFTW3
#  include <fftw3.h>
#endif

#include "BLI_array.hh"
#include "BLI_fftw.hh"
#include "BLI_task.hh"

#include "BKE_attribute_math.hh"
#include "BKE_main.hh"
#include "BKE_scene.hh"
#include "BKE_sound.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_socket_search_link.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_sound_types.h"

#include "node_geometry_util.hh"

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
  b.add_input<decl::Float>("Smoothness").min(0).max(1).default_value(0.1);
  // b.add_input<decl::Int>("Channel Index").min(0).max(7).supports_field();
  b.add_input<decl::Float>("Start Frequency").min(0).supports_field();
  b.add_input<decl::Float>("End Frequency").min(0).supports_field();
  b.add_output<decl::Float>("Amplitude").dependent_field({1, 3, 4});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "specify_channel", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "specify_frequency", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "window", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "fft_size", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySampleSound *data = MEM_cnew<NodeGeometrySampleSound>(__func__);
  data->specify_channel = false;
  data->specify_frequency = true;
  data->window = GEO_NODE_SAMPLE_SOUND_WINDOW_HANN;
  node->storage = data;
}

#ifdef WITH_AUDASPACE

class SampleSoundFunction : public mf::MultiFunction {
  bSound *sound_;
  const float smoothness_;
  const bool specify_channel_;
  const bool specify_frequency_;
  const NodeGeometrySampleSoundWindow window_;
  const NodeGeometrySampleSoundFFTSize fft_size_;
  const double frame_rate_;
  const float length_;

  AUD_Device *device_;
  AUD_Handle *handle_;

 public:
  SampleSoundFunction(bSound *sound,
                      const float smoothness,
                      const bool specify_channel,
                      const bool specify_frequency,
                      const NodeGeometrySampleSoundWindow window,
                      const NodeGeometrySampleSoundFFTSize fft_size,
                      const double frame_rate,
                      const float length)
      : sound_(sound),
        smoothness_(smoothness),
        specify_channel_(specify_channel),
        specify_frequency_(specify_frequency),
        window_(window),
        fft_size_(fft_size),
        frame_rate_(frame_rate),
        length_(length)
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Sample Sound", signature};
      builder.single_input<float>("Time");
      builder.single_input<float>("Start Frequency");
      builder.single_input<float>("End Frequency");
      builder.single_output<float>("Amplitude");
      return signature;
    }();
    this->set_signature(&signature);

    this->init();
  }

  ~SampleSoundFunction()
  {
    AUD_Handle_free(handle_);
    AUD_Device_free(device_);
  }

  void init()
  {
    AUD_DeviceSpecs specs = {
        .format = AUD_FORMAT_FLOAT32,
        .specs = {
            .rate = AUD_SampleRate(sound_->samplerate), .channels = AUD_CHANNELS_MONO,
            // TODO: support specifying channels
            // .channels = specify_channel_ ? AUD_Channels(sound_->audio_channels) :
            // AUD_CHANNELS_MONO,
        }};

    device_ = AUD_Device_open("read", specs, 1024, "Sample Sound");
    // FIXME: `assert(sound)` fails if a new sound is specified while still playing
    handle_ = AUD_Device_play(device_, sound_->handle, true);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &times = params.readonly_single_input<float>(0, "Time");
    const VArray<float> &freqs_start = params.readonly_single_input<float>(1, "Start Frequency");
    const VArray<float> &freqs_end = params.readonly_single_input<float>(2, "End Frequency");
    MutableSpan<float> dst = params.uninitialized_single_output<float>(3, "Amplitude");

    mask.foreach_index([&](int64_t i) {
      const int64_t sample_index = times[i] * sound_->samplerate;

      if (specify_frequency_) {
#  ifdef WITH_FFTW3
        const int fft_size = 1 << fft_size_;
        const int64_t aligned_index = (sample_index / fft_size) * fft_size;
        const int64_t bin_index = aligned_index / 2;
        const int bin_start = math::clamp(
            int(freqs_start[i] / sound_->samplerate * fft_size), 0, fft_size / 2 - 1);
        const int bin_end = math::clamp(
            int(freqs_end[i] / sound_->samplerate * fft_size), 0, fft_size / 2 - 1);
        // TODO: temporal smoothing (interpolation)
        // double offset_factor = float(sample_index % (fft_size / 2)) / float(fft_size / 2);

        FFTParameter parameter = {.fft_size = fft_size_, .window = window_};
        int result_index = BKE_sound_findindex_fft_result(sound_, &parameter);
        if (result_index != -1) {
          const FFTResults *results = static_cast<FFTResults *>(sound_->fft_results);
          const float *bins = results->arr[result_index].fft;

          if (bins[bin_index] == 0) {
            dst[i] = bins[bin_index + bin_end] - bins[bin_index + bin_start];
            return;
          }
        }

        if (aligned_index < 0) {
          dst[i] = 0;
          return;
        }

        Array<float> buf(fft_size);

        AUD_Device_lock(device_);
        AUD_Handle_setPosition(handle_, double(aligned_index) / double(sound_->samplerate));
        AUD_Device_read(device_, reinterpret_cast<unsigned char *>(buf.data()), fft_size);
        AUD_Device_unlock(device_);

        switch (window_) {
          case GEO_NODE_SAMPLE_SOUND_WINDOW_NONE:
            break;
          case GEO_NODE_SAMPLE_SOUND_WINDOW_HANN:
            for (size_t i = 0; i < buf.size(); ++i) {
              buf[i] *= 0.5 - 0.5 * math::cos((2 * math::numbers::pi * i) / buf.size());
            }
            break;
          case GEO_NODE_SAMPLE_SOUND_WINDOW_HAMMING:
            for (size_t i = 0; i < buf.size(); ++i) {
              buf[i] *= 0.54 - 0.46 * math::cos((2 * math::numbers::pi * i) / buf.size());
            }
            break;
          case GEO_NODE_SAMPLE_SOUND_WINDOW_BLACKMAN:
            for (size_t i = 0; i < buf.size(); ++i) {
              buf[i] *= 0.42 - 0.5 * math::cos((2 * math::numbers::pi * i) / buf.size()) +
                        0.08 * math::cos((4 * math::numbers::pi * i) / buf.size());
            }
            break;
        }

        fftw::initialize_float();
        fftwf_complex *fft_buf = static_cast<fftwf_complex *>(
            fftwf_malloc(sizeof(*fft_buf) * fft_size / 2 + 1));
        fftwf_plan plan = fftwf_plan_dft_r2c_1d(fft_size, buf.data(), fft_buf, FFTW_ESTIMATE);
        fftwf_execute(plan);

        if (result_index == -1) {
          const int64_t total_bins = int64_t(math::ceil(length_) * sound_->samplerate) / 2 + 1;

          FFTResult fft_result = {.parameter = parameter,
                                  .fft = static_cast<float *>(MEM_malloc_arrayN(
                                      total_bins, sizeof(float), "FFTResult.fft"))};
          for (int64_t i = 0; i < total_bins; ++i) {
            // XXX: get rid of special values
            fft_result.fft[i] = -1;
          }

          result_index = BKE_sound_add_fft_result(sound_, &fft_result);
        }

        const FFTResults *results = static_cast<FFTResults *>(sound_->fft_results);
        float *bins = results->arr[result_index].fft;

        // TODO: parallel scan
        bins[bin_index] = 0;
        for (int i = 1; i < fft_size / 2; ++i) {
          bins[bin_index + i] = bins[bin_index + i - 1] +
                                (math::abs(fft_buf[i - 1][0]) / (fft_size / 2));
        }

        dst[i] = bins[bin_index + bin_end] - bins[bin_index + bin_start];

        fftwf_destroy_plan(plan);
        fftwf_free(fft_buf);
#  else  // WITH_FFTW3
        dst[i] = 0;
#  endif
      }
      else {
        const int sample_length = sound_->samplerate * smoothness_ + 1;
        Array<float> buf(sample_length);

        AUD_Device_lock(device_);

        // FIXME: eliminate floating-point errors
        const double position = double(sample_index) / double(sound_->samplerate) - smoothness_;
        if (position < 0) {
          const int empty_n = (-position * sound_->samplerate);
          const int read_n = sample_length - empty_n;

          buf.fill(0);
          AUD_Handle_setPosition(handle_, 0);
          AUD_Device_read(
              device_, reinterpret_cast<unsigned char *>(buf.data() + empty_n), read_n);
        }
        else {
          AUD_Handle_setPosition(handle_, position);
          AUD_Device_read(device_, reinterpret_cast<unsigned char *>(buf.data()), sample_length);
        }

        AUD_Device_unlock(device_);

        float result = 0;
        for (float sample : buf) {
          result += math::abs(sample) / sample_length;
        }
        dst[i] = result;
      }
    });
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  bSound *sound = params.extract_input<bSound *>("Sound");
  if (!sound) {
    params.set_output("Amplitude", fn::make_constant_field<float>(0));
    return;
  }

  const NodeGeometrySampleSound &storage = node_storage(params.node());
  const bool specify_channel = bool(storage.specify_channel);
  const bool specify_frequency = bool(storage.specify_frequency);
  const NodeGeometrySampleSoundWindow window = NodeGeometrySampleSoundWindow(storage.window);
  const NodeGeometrySampleSoundFFTSize fft_size = NodeGeometrySampleSoundFFTSize(storage.fft_size);

  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  Main *bmain = params.bmain();
  const double frame_rate = double(scene->r.frs_sec) / double(scene->r.frs_sec_base);
  const float length = BKE_sound_get_length(bmain, sound);

  const float smoothness = params.extract_input<float>("Smoothness");
  Field<float> times = params.extract_input<Field<float>>("Time");
  Field<float> freqs_start = params.extract_input<Field<float>>("Start Frequency");
  Field<float> freqs_end = params.extract_input<Field<float>>("End Frequency");

  auto fn = std::make_shared<SampleSoundFunction>(
      sound, smoothness, specify_channel, specify_frequency, window, fft_size, frame_rate, length);
  auto op = FieldOperation::Create(
      std::move(fn), {std::move(times), std::move(freqs_start), std::move(freqs_end)});
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
