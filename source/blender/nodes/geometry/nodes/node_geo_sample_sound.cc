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

  friend bool operator==(const Sample &lhs, const Sample &rhs)
  {
    return lhs.time == rhs.time && lhs.freq_start == rhs.freq_start &&
           lhs.freq_end == rhs.freq_end;
  }
};

struct SampleHash {
  size_t operator()(const Sample &s) const noexcept
  {
    size_t h1 = std::hash<int>{}(s.time);
    size_t h2 = std::hash<float>{}(s.freq_start);
    size_t h3 = std::hash<float>{}(s.freq_end);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

// An LRU cache implementation similar to boost's lru_cache, but uses unordered_map.
// FIXME: this is currently not thread safe, neither is boost's lru_cache. However for some reason
// this crashes at a significantly lower rate than what boost's (ordered)map implementation does.
template<class K, class V, class H> class LRUCache {
 public:
  LRUCache(size_t capacity) : capacity_(capacity) {}
  ~LRUCache() {}

  bool contains(const K &key)
  {
    return map_.find(key) != map_.end();
  }

  void insert(const K &key, const V &value)
  {
    if (!contains(key)) {
      if (map_.size() >= capacity_) {
        evict();
      }

      list_.push_front(key);
      map_[key] = std::make_pair(value, list_.begin());
    }
  }

  V get(const K &key)
  {
    auto i = map_.find(key);
    auto j = i->second.second;
    if (j != list_.begin()) {
      // FIXME: would crash here
      list_.erase(j);
      list_.push_front(key);

      j = list_.begin();
      const V &value = i->second.first;
      map_[key] = std::make_pair(value, j);

      return value;
    }
    else {
      return i->second.first;
    }
  }

  void clear()
  {
    map_.clear();
    list_.clear();
  }

 private:
  std::list<K> list_;
  std::unordered_map<K, std::pair<V, typename std::list<K>::iterator>, H> map_;
  size_t capacity_;

  void evict()
  {
    auto i = --list_.end();
    map_.erase(*i);
    list_.erase(i);
  }
};

using CacheType = LRUCache<Sample, float, SampleHash>;

// TODO: redesign the whole caching system when everything's settled down.
static std::map<const bSound *, std::shared_ptr<CacheType>> g_caches;

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  // TODO: complete all functionalities
  b.add_input<decl::Sound>("Sound");
  b.add_input<decl::Float>("Time").min(0).supports_field();
  // b.add_input<decl::Float>("Smoothness").min(0).max(1);
  // b.add_input<decl::Int>("Channel Index").min(0).max(7).supports_field();
  b.add_input<decl::Float>("Start Frequency").min(0).supports_field();
  b.add_input<decl::Float>("End Frequency").min(0).supports_field();
  b.add_output<decl::Float>("Amplitude").dependent_field({1, 2, 3});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "spec_chan", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "spec_freq", UI_ITEM_NONE, nullptr, ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySampleSound *data = MEM_cnew<NodeGeometrySampleSound>(__func__);
  data->spec_chan = false;
  data->spec_freq = true;
  node->storage = data;
}

#ifdef WITH_AUDASPACE

class SampleSoundFunction : public mf::MultiFunction {
  const bSound *sound_;
  std::shared_ptr<CacheType> cache_;
  const bool spec_chan_;
  const bool spec_freq_;
  const double frame_rate_;

  AUD_Device *device_;
  AUD_Handle *handle_;

 public:
  SampleSoundFunction(const bSound *sound,
                      std::shared_ptr<CacheType> cache,
                      const bool spec_chan,
                      const bool spec_freq,
                      const double frame_rate)
      : sound_(sound),
        cache_(cache),
        spec_chan_(spec_chan),
        spec_freq_(spec_freq),
        frame_rate_(frame_rate)
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
            // .channels = spec_chan_ ? AUD_Channels(sound_->audio_channels) : AUD_CHANNELS_MONO,
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
      const int sample_idx = times[i] * sound_->samplerate;

      {
        Sample key;
        if (spec_freq_) {
          key = {sample_idx, freqs_start[i], freqs_end[i]};
        }
        else {
          // XXX: better get rid of special values (i.e. `-1` here)
          key = {sample_idx, -1, -1};
        }

        if (cache_->contains(key)) {
          dst[i] = cache_->get(key);
          return;
        }
      }

      // TODO: support variable sampling step instead of simply rely on scene fps. This actually
      // causes problems when the time step is smaller than one frame_duration. Maybe combine
      // with temporal smoothing and/or ADSR envelopes?
      const int n = sound_->samplerate / frame_rate_;
      Array<float> buf(n);

      AUD_Device_lock(device_);

      // FIXME: eliminate floating-point errors
      // XXX: would seek to negative position if times[i] is less than 0. Should we clamp here or
      // let user handle this?
      AUD_Handle_setPosition(handle_, double(sample_idx) / double(sound_->samplerate));
      AUD_Device_read(device_, (unsigned char *)buf.data(), n);
      if (spec_freq_) {
#  ifdef WITH_FFTW3
        // TODO: windowing, temporal smoothing, etc.
        fftwf_complex *fft_buf = (fftwf_complex *)fftwf_malloc(sizeof(*fft_buf) * (n / 2 + 1));
        fftwf_plan plan = fftwf_plan_dft_r2c_1d(n, buf.data(), fft_buf, FFTW_ESTIMATE);
        fftwf_execute(plan);

        const int bin_start = math::clamp(
            int(freqs_start[i] / sound_->samplerate * n / 2), 0, n / 2);
        const int bin_end = math::clamp(int(freqs_end[i] / sound_->samplerate * n / 2), 0, n / 2);
        float result = 0;
        for (size_t i = bin_start; i < bin_end; ++i) {
          result += math::abs(fft_buf[i][0]) / (bin_end - bin_start + 1);
        }
        dst[i] = result;

        fftwf_destroy_plan(plan);
        fftwf_free(fft_buf);

        cache_->insert({sample_idx, freqs_start[i], freqs_end[i]}, result);
#  else  // WITH_FFTW3
        dst[i] = 0;
#  endif
      }
      else {
        // TODO: find better averaging algorithms & temporal smoothing
        float result = 0;
        for (float sample : buf) {
          result += math::abs(sample) / n;
        }
        dst[i] = result;

        cache_->insert({sample_idx, -1, -1}, result);
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

  if (g_caches.find(sound) == g_caches.end()) {
    // TODO: dynamic/configurable cache size?
    g_caches.insert(std::make_pair(sound, std::make_shared<CacheType>(16384)));
  }

  const NodeGeometrySampleSound &storage = node_storage(params.node());
  const bool spec_chan = bool(storage.spec_chan);
  const bool spec_freq = bool(storage.spec_freq);

  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  const double frame_rate = double(scene->r.frs_sec) / double(scene->r.frs_sec_base);

  Field<float> times = params.extract_input<Field<float>>("Time");
  Field<float> freqs_start = params.extract_input<Field<float>>("Start Frequency");
  Field<float> freqs_end = params.extract_input<Field<float>>("End Frequency");

  auto fn = std::make_shared<SampleSoundFunction>(
      sound, g_caches[sound], spec_chan, spec_freq, frame_rate);
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
