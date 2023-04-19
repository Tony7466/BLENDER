/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_geometry_set.hh"

#include "BLI_map.hh"

namespace blender::bke::sim {

class BDataSharing;
class ModifierSimulationCache;

class SimulationStateItem {
 public:
  virtual ~SimulationStateItem() = default;
};

class GeometrySimulationStateItem : public SimulationStateItem {
 private:
  GeometrySet geometry_;

 public:
  GeometrySimulationStateItem(GeometrySet geometry) : geometry_(std::move(geometry)) {}

  const GeometrySet &geometry() const
  {
    return geometry_;
  }
};

class SimulationZoneState {
 public:
  Vector<std::unique_ptr<SimulationStateItem>> items;
};

struct SimulationZoneID {
  Vector<int> node_ids;

  uint64_t hash() const
  {
    return get_default_hash(this->node_ids);
  }

  friend bool operator==(const SimulationZoneID &a, const SimulationZoneID &b)
  {
    return a.node_ids == b.node_ids;
  }
};

class ModifierSimulationState {
 private:
  mutable bool bake_loaded_;

 public:
  ModifierSimulationCache *owner_;
  mutable std::mutex mutex_;
  Map<SimulationZoneID, std::unique_ptr<SimulationZoneState>> zone_states_;
  std::optional<std::string> meta_path_;
  std::optional<std::string> bdata_dir_;

  const SimulationZoneState *get_zone_state(const SimulationZoneID &zone_id) const
  {
    std::lock_guard lock{mutex_};
    if (auto *ptr = zone_states_.lookup_ptr(zone_id)) {
      return ptr->get();
    }
    return nullptr;
  }

  SimulationZoneState &get_zone_state_for_write(const SimulationZoneID &zone_id)
  {
    std::lock_guard lock{mutex_};
    return *zone_states_.lookup_or_add_cb(
        zone_id, []() { return std::make_unique<SimulationZoneState>(); });
  }

  void ensure_bake_loaded() const;
};

struct SubFrame {
 private:
  int frame_;
  float subframe_;

 public:
  SubFrame(const int frame = 0, float subframe = 0.0f) : frame_(frame), subframe_(subframe)
  {
    BLI_assert(subframe >= 0.0f);
    BLI_assert(subframe < 1.0f);
  }

  SubFrame(const float frame) : SubFrame(int(floorf(frame)), fractf(frame)) {}

  int frame() const
  {
    return frame_;
  }

  float subframe() const
  {
    return subframe_;
  }

  explicit operator float() const
  {
    return float(frame_) + float(subframe_);
  }

  explicit operator double() const
  {
    return double(frame_) + double(subframe_);
  }

  static SubFrame min()
  {
    return {INT32_MIN, 0.0f};
  }

  static SubFrame max()
  {
    return {INT32_MAX, std::nexttowardf(1.0f, 0.0)};
  }

  friend bool operator==(const SubFrame &a, const SubFrame &b)
  {
    return a.frame_ == b.frame_ && a.subframe_ == b.subframe_;
  }

  friend bool operator!=(const SubFrame &a, const SubFrame &b)
  {
    return !(a == b);
  }

  friend bool operator<(const SubFrame &a, const SubFrame &b)
  {
    return a.frame_ < b.frame_ || (a.frame_ == b.frame_ && a.subframe_ < b.subframe_);
  }

  friend bool operator<=(const SubFrame &a, const SubFrame &b)
  {
    return a.frame_ <= b.frame_ || (a.frame_ == b.frame_ && a.subframe_ <= b.subframe_);
  }

  friend bool operator>(const SubFrame &a, const SubFrame &b)
  {
    return a.frame_ > b.frame_ || (a.frame_ == b.frame_ && a.subframe_ > b.subframe_);
  }

  friend bool operator>=(const SubFrame &a, const SubFrame &b)
  {
    return a.frame_ >= b.frame_ || (a.frame_ == b.frame_ && a.subframe_ >= b.subframe_);
  }

  friend std::ostream &operator<<(std::ostream &stream, const SubFrame &a)
  {
    return stream << float(a);
  }
};

struct ModifierSimulationStateAtFrame {
  SubFrame frame;
  ModifierSimulationState state;
};

enum class CacheState {
  Valid,
  Invalid,
  Baked,
};

struct StatesAroundFrame {
  const ModifierSimulationStateAtFrame *prev = nullptr;
  const ModifierSimulationStateAtFrame *current = nullptr;
  const ModifierSimulationStateAtFrame *next = nullptr;
};

class ModifierSimulationCache {
 private:
  Vector<std::unique_ptr<ModifierSimulationStateAtFrame>> states_at_frames_;
  std::unique_ptr<BDataSharing> bdata_sharing_;

  friend ModifierSimulationState;

 public:
  CacheState cache_state_ = CacheState::Valid;
  bool failed_finding_bake_ = false;

  void try_discover_bake(StringRefNull meta_dir, StringRefNull bdata_dir);

  bool has_state_at_frame(const SubFrame &frame) const
  {
    for (const auto &item : states_at_frames_) {
      if (item->frame == frame) {
        return true;
      }
    }
    return false;
  }

  bool has_states() const
  {
    return !states_at_frames_.is_empty();
  }

  const ModifierSimulationState *get_state_at_exact_frame(const SubFrame &frame) const
  {
    for (const auto &item : states_at_frames_) {
      if (item->frame == frame) {
        return &item->state;
      }
    }
    return nullptr;
  }

  ModifierSimulationState &get_state_at_frame_for_write(const SubFrame &frame)
  {
    for (const auto &item : states_at_frames_) {
      if (item->frame == frame) {
        return item->state;
      }
    }
    states_at_frames_.append(std::make_unique<ModifierSimulationStateAtFrame>());
    states_at_frames_.last()->frame = frame;
    states_at_frames_.last()->state.owner_ = this;
    return states_at_frames_.last()->state;
  }

  StatesAroundFrame get_states_around_frame(const SubFrame &frame) const
  {
    StatesAroundFrame states_around_frame;
    for (const auto &item : states_at_frames_) {
      if (item->frame < frame) {
        if (states_around_frame.prev == nullptr || item->frame > states_around_frame.prev->frame) {
          states_around_frame.prev = item.get();
        }
      }
      if (item->frame == frame) {
        if (states_around_frame.current == nullptr) {
          states_around_frame.current = item.get();
        }
      }
      if (item->frame > frame) {
        if (states_around_frame.next == nullptr || item->frame < states_around_frame.next->frame) {
          states_around_frame.next = item.get();
        }
      }
    }
    return states_around_frame;
  }

  void invalidate()
  {
    cache_state_ = CacheState::Invalid;
  }

  CacheState cache_state() const
  {
    return cache_state_;
  }

  void reset();
};

}  // namespace blender::bke::sim
