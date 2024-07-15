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
#include "BKE_sound_fft_cache.hh"

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

  // TODO: complete all functionalities.
  // TODO: make descriptions more concise.
  b.add_input<decl::Sound>("Sound");
  b.add_input<decl::Float>("Time")
      .min(0)
      .subtype(PropertySubType::PROP_TIME_ABSOLUTE)
      .supports_field()
      .description("Sample time of the sound");
  b.add_input<decl::Float>("Smoothness")
      .min(0)
      .max(1)
      .subtype(PropertySubType::PROP_TIME_ABSOLUTE)
      .default_value(0.1)
      .description("Temporal smoothness");
  // b.add_input<decl::Int>("Channel Index")
  //     .min(1)
  //     .max(7)
  //     .default_value(1)
  //     .supports_field()
  //     .description("Selected channel of the sound");
  b.add_input<decl::Float>("Low")
      .min(0)
      .max(1)
      .subtype(PropertySubType::PROP_FACTOR)
      .supports_field()
      .description("The lower bound of frequency (in percentage) to be sampled");
  b.add_input<decl::Float>("High")
      .min(0)
      .max(1)
      .subtype(PropertySubType::PROP_FACTOR)
      .supports_field()
      .description("The upper bound of frequency (in percentage) to be sampled");
  b.add_output<decl::Float>("Amplitude").dependent_field({1, 3, 4}).description("Sample result");

#ifdef WITH_FFTW3
  // Make FFTW routines thread-safe.
  fftw::initialize_float();
#endif
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "specify_channel", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "specify_frequency", UI_ITEM_NONE, nullptr, ICON_NONE);
}

static void node_layout_ex(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "specify_channel", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "specify_frequency", UI_ITEM_NONE, nullptr, ICON_NONE);

  uiItemR(layout, ptr, "window", UI_ITEM_NONE, IFACE_("Window"), ICON_NONE);
  uiItemR(layout, ptr, "fft_size", UI_ITEM_NONE, IFACE_("FFT Size"), ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySampleSound *data = MEM_cnew<NodeGeometrySampleSound>(__func__);
  data->specify_channel = false;
  data->specify_frequency = true;
  data->window = GEO_NODE_SAMPLE_SOUND_WINDOW_HANN;
  data->fft_size = GEO_NODE_SAMPLE_SOUND_FFT_SIZE_2048;
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
      builder.single_input<float>("Low");
      builder.single_input<float>("High");
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
    AUD_DeviceSpecs device_specs{};
    device_specs.format = AUD_FORMAT_FLOAT32;
    device_specs.rate = sound_->samplerate;
    // TODO: support specifying channels.
    device_specs.channels = AUD_CHANNELS_MONO;

    device_ = AUD_Device_open("read", device_specs, 1024, "Sample Sound");
    handle_ = AUD_Device_play(device_, sound_->handle, true);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &times = params.readonly_single_input<float>(0, "Time");
    const VArray<float> &lows = params.readonly_single_input<float>(1, "Low");
    const VArray<float> &highs = params.readonly_single_input<float>(2, "High");
    MutableSpan<float> dst = params.uninitialized_single_output<float>(3, "Amplitude");

    mask.foreach_index([&](int64_t i) {
      const int64_t sample_index = times[i] * sound_->samplerate;

      if (specify_frequency_) {
#  ifdef WITH_FFTW3
        using namespace bke::sound::fft_cache;
        FFTCache &cache = *sound_->fft_cache->cache;

        const int fft_size = 1 << fft_size_;
        const int bin_size = fft_size / 2;

        const int low = math::clamp(int(lows[i] * (bin_size - 1)), 0, bin_size - 1);
        const int high = math::clamp(int(highs[i] * (bin_size - 1)), 0, bin_size - 1);

        const int64_t aligned_sample_index = (sample_index / fft_size) * fft_size;
        const int64_t bin_index = aligned_sample_index / 2;

        // TODO: temporal smoothing (interpolation)
        // double offset_factor = float(sample_index % (bin_size)) / float(bin_size);

        Parameter parameter{};
        parameter.fft_size = fft_size;
        parameter.window = window_;
        parameter.bin_index = bin_index;

        // Try to read from cache
        {
          std::shared_lock lock{cache.mutex};

          if (cache.map.contains(parameter)) {
            const Array<float> &bins = cache.map.lookup(parameter);

            if (high == low) {
              dst[i] = bins[high + 1] - bins[low];
            }
            else {
              dst[i] = bins[high] - bins[low];
            }
            return;
          }
        }

        if (bin_index < 0) {
          dst[i] = 0;
          return;
        }

        Array<float> buf(fft_size);

        AUD_Device_lock(device_);
        AUD_Handle_setPosition(handle_, double(aligned_sample_index) / double(sound_->samplerate));
        AUD_Device_read(device_, reinterpret_cast<unsigned char *>(buf.data()), buf.size());
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

        fftwf_complex *fft_buf = static_cast<fftwf_complex *>(
            fftwf_malloc(sizeof(*fft_buf) * fft_size / 2 + 1));
        fftwf_plan plan = fftwf_plan_dft_r2c_1d(fft_size, buf.data(), fft_buf, FFTW_ESTIMATE);
        fftwf_execute(plan);

        Array<float> bins(bin_size);

        // TODO: parallel scan
        bins[0] = 0;
        for (int i = 1; i < bin_size; ++i) {
          bins[i] = bins[i - 1] + math::abs(fft_buf[i - 1][0]) / bin_size;
        }

        fftwf_destroy_plan(plan);
        fftwf_free(fft_buf);

        if (high == low) {
          dst[i] = bins[high + 1] - bins[low];
        }
        else {
          dst[i] = bins[high] - bins[low];
        }

        // Write into cache
        {
          std::unique_lock lock{cache.mutex};
          cache.map.add(std::move(parameter), std::move(bins));
        }
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

#  ifdef WITH_FFTW3
  static std::mutex mutex;

  if (specify_frequency) {
    std::lock_guard lock{mutex};
    if (!sound->fft_cache) {
      BKE_sound_fft_cache_new(sound);
    }
  }
#  endif

  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  Main *bmain = params.bmain();
  const double frame_rate = FPS;
  const float length = BKE_sound_get_length(bmain, sound);

  const float smoothness = params.extract_input<float>("Smoothness");
  Field<float> times = params.extract_input<Field<float>>("Time");
  Field<float> lows = params.extract_input<Field<float>>("Low");
  Field<float> highs = params.extract_input<Field<float>>("High");

  auto fn = std::make_shared<SampleSoundFunction>(
      sound, smoothness, specify_channel, specify_frequency, window, fft_size, frame_rate, length);
  auto op = FieldOperation::Create(std::move(fn),
                                   {std::move(times), std::move(lows), std::move(highs)});
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

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_SOUND, "Sample Sound", NODE_CLASS_CONVERTER);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  blender::bke::node_type_storage(
      &ntype, "NodeGeometrySampleSound", node_free_standard_storage, node_copy_standard_storage);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sample_sound_cc
