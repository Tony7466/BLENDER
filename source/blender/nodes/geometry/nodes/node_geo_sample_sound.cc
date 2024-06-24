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

#include <list>
#include <map>

#include "BLI_array.hh"
#include "BLI_listbase.h"
#include "BLI_map.hh"
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

struct Sample {
  int time;
  float freq_start, freq_end;
  float smoothness;
  NodeGeometrySampleSoundWindow window;

  friend bool operator==(const Sample &lhs, const Sample &rhs)
  {
    return lhs.time == rhs.time && lhs.freq_start == rhs.freq_start &&
           lhs.freq_end == rhs.freq_end && lhs.smoothness == rhs.smoothness &&
           lhs.window == rhs.window;
  }

  uint64_t hash() const
  {
    uint64_t temp = get_default_hash(
        this->time, this->freq_start, this->freq_end, this->smoothness);
    return get_default_hash(temp, this->window);
  }
};

// An LRU cache implementation similar to boost's lru_cache, but uses BLI_map and BLI_listbase.
// FIXME: this is currently not thread safe, neither is boost's lru_cache.
template<class K, class V> class LRUCache {
 public:
  LRUCache(size_t capacity) : capacity_(capacity)
  {
    static_assert(std::is_trivial_v<K>, "LRUCache only allows trivial key type");

    list_.first = nullptr;
    list_.last = nullptr;
  }
  ~LRUCache()
  {
    BLI_freelistN(&list_);
  }

  bool contains(const K &key)
  {
    return map_.contains(key);
  }

  void insert(const K &key, const V &value)
  {
    if (!contains(key)) {
      if (map_.size() >= capacity_) {
        evict();
      }

      auto node = MEM_cnew<LinkData>("LRUCache ListBase Node");
      auto data = MEM_cnew<K>("LRUCache ListBase Node Data");
      memcpy(data, &key, sizeof(*data));
      node->data = data;

      BLI_addhead(&list_, node);
      map_.add(key, std::make_pair(value, list_.first));
    }
  }

  V get(const K &key)
  {
    auto i = map_.lookup(key);
    auto j = i.second;
    if (j != list_.first) {
      // FIXME: would crash here
      BLI_remlink(&list_, j);
      BLI_addhead(&list_, j);

      const V &value = i.first;
      map_.add(key, std::make_pair(value, list_.first));

      return value;
    }
    else {
      return i.first;
    }
  }

  void clear()
  {
    map_.clear();
    BLI_freelistN(&list_);
    BLI_listbase_clear(&list_);
  }

 private:
  ListBase list_;
  Map<K, std::pair<V, void *>> map_;
  size_t capacity_;

  void evict()
  {
    auto i = static_cast<LinkData *>(list_.last);
    map_.remove(*static_cast<K *>(i->data));
    BLI_freelinkN(&list_, i);
  }
};

using CacheType = LRUCache<Sample, float>;

// TODO: redesign the whole caching system when everything's settled down.
static Map<const bSound *, std::shared_ptr<CacheType>> g_caches;

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
  uiItemR(layout, ptr, "window", UI_ITEM_NONE, nullptr, ICON_NONE);
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
  const bSound *sound_;
  const float smoothness_;
  const bool specify_channel_;
  const bool specify_frequency_;
  const NodeGeometrySampleSoundWindow window_;
  const double frame_rate_;

  std::shared_ptr<CacheType> cache_;

  AUD_Device *device_;
  AUD_Handle *handle_;

 public:
  SampleSoundFunction(const bSound *sound,
                      const float smoothness,
                      const bool specify_channel,
                      const bool specify_frequency,
                      const NodeGeometrySampleSoundWindow window_,
                      const double frame_rate,
                      std::shared_ptr<CacheType> cache)
      : sound_(sound),
        smoothness_(smoothness),
        specify_channel_(specify_channel),
        specify_frequency_(specify_frequency),
        window_(window_),
        frame_rate_(frame_rate),
        cache_(cache)
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
    AUD_DeviceSpecs specs{
        .format = AUD_FORMAT_FLOAT32,
        .specs{
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
      const int sample_i = times[i] * sound_->samplerate;

      {
        Sample key;
        if (specify_frequency_) {
          key = {sample_i, freqs_start[i], freqs_end[i], smoothness_, window_};
        }
        else {
          // XXX: better get rid of special values (i.e. `-1` here)
          key = {sample_i, -1, -1, smoothness_, GEO_NODE_SAMPLE_SOUND_WINDOW_NONE};
        }

        if (cache_->contains(key)) {
          dst[i] = cache_->get(key);
          return;
        }
      }

      const int sample_n = sound_->samplerate * smoothness_ + 1;
      Array<float> buf(sample_n);

      AUD_Device_lock(device_);

      // FIXME: eliminate floating-point errors
      const double position = double(sample_i) / double(sound_->samplerate) - smoothness_;
      if (position < 0) {
        const int empty_n = (-position * sound_->samplerate);
        const int read_n = sample_n - empty_n;

        buf.fill(0);
        AUD_Handle_setPosition(handle_, 0);
        // XXX: pointer arithmetic
        AUD_Device_read(device_, reinterpret_cast<unsigned char *>(buf.data() + empty_n), read_n);
      }
      else {
        AUD_Handle_setPosition(handle_, position);
        AUD_Device_read(device_, reinterpret_cast<unsigned char *>(buf.data()), sample_n);
      }

      if (specify_frequency_) {
#  ifdef WITH_FFTW3
        switch (window_) {
          case GEO_NODE_SAMPLE_SOUND_WINDOW_NONE:
            break;
          case GEO_NODE_SAMPLE_SOUND_WINDOW_HANN:
            for (size_t i = 0; i < buf.size(); ++i) {
              buf[i] *= 0.5 - 0.5 * math::cos((2 * math::numbers::pi * i) / sample_n);
            }
            break;
          case GEO_NODE_SAMPLE_SOUND_WINDOW_HAMMING:
            for (size_t i = 0; i < buf.size(); ++i) {
              buf[i] *= 0.54 - 0.46 * math::cos((2 * math::numbers::pi * i) / sample_n);
            }
            break;
          case GEO_NODE_SAMPLE_SOUND_WINDOW_BLACKMAN:
            for (size_t i = 0; i < buf.size(); ++i) {
              buf[i] *= 0.42 - 0.5 * math::cos((2 * math::numbers::pi * i) / sample_n) +
                        0.08 * math::cos((4 * math::numbers::pi * i) / sample_n);
            }
            break;
        }

        fftwf_complex *fft_buf = (fftwf_complex *)fftwf_malloc(sizeof(*fft_buf) *
                                                               (sample_n / 2 + 1));
        fftwf_plan plan = fftwf_plan_dft_r2c_1d(sample_n, buf.data(), fft_buf, FFTW_ESTIMATE);
        fftwf_execute(plan);

        const int bin_start = math::clamp(
            int(freqs_start[i] / sound_->samplerate * sample_n / 2), 0, sample_n / 2);
        const int bin_end = math::clamp(
            int(freqs_end[i] / sound_->samplerate * sample_n / 2), 0, sample_n / 2);
        float result = 0;
        for (size_t i = bin_start; i < bin_end; ++i) {
          result += math::abs(fft_buf[i][0]) / (bin_end - bin_start + 1);
        }
        dst[i] = result;

        fftwf_destroy_plan(plan);
        fftwf_free(fft_buf);

        cache_->insert({sample_i, freqs_start[i], freqs_end[i], smoothness_, window_}, result);
#  else  // WITH_FFTW3
        dst[i] = 0;
#  endif
      }
      else {
        // TODO: find better averaging algorithms & temporal smoothing
        float result = 0;
        for (float sample : buf) {
          result += math::abs(sample) / sample_n;
        }
        dst[i] = result;

        cache_->insert({sample_i, -1, -1, smoothness_, GEO_NODE_SAMPLE_SOUND_WINDOW_NONE}, result);
      }

      AUD_Device_unlock(device_);
    });
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const bSound *sound = params.extract_input<bSound *>("Sound");
  if (!sound) {
    params.set_output("Amplitude", fn::make_constant_field<float>(0));
    return;
  }

  // TODO: dynamic/configurable cache size?
  auto cache = g_caches.lookup_or_add(sound, std::make_shared<CacheType>(16384));

  const NodeGeometrySampleSound &storage = node_storage(params.node());
  const bool specify_channel = bool(storage.specify_channel);
  const bool specify_frequency = bool(storage.specify_frequency);
  const NodeGeometrySampleSoundWindow window = NodeGeometrySampleSoundWindow(storage.window);

  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  const double frame_rate = double(scene->r.frs_sec) / double(scene->r.frs_sec_base);

  const float smoothness = params.extract_input<float>("Smoothness");
  Field<float> times = params.extract_input<Field<float>>("Time");
  Field<float> freqs_start = params.extract_input<Field<float>>("Start Frequency");
  Field<float> freqs_end = params.extract_input<Field<float>>("End Frequency");

  auto fn = std::make_shared<SampleSoundFunction>(
      sound, smoothness, specify_channel, specify_frequency, window, frame_rate, cache);
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
