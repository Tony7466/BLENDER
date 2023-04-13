/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_geometry_set.hh"

#include "BLI_map.hh"

namespace blender::bke::sim {

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
 public:
  mutable std::mutex mutex_;
  Map<SimulationZoneID, std::unique_ptr<SimulationZoneState>> zone_states_;

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
  std::unique_ptr<ModifierSimulationState> state;
};

class ModifierSimulationCache {
 private:
  Vector<ModifierSimulationStateAtFrame> states_at_frames_;
  bool invalid_ = false;

 public:
  void load_baked_states(StringRefNull meta_dir, StringRefNull bdata_dir);

  bool has_state_at_frame(const SubFrame &frame) const
  {
    for (const ModifierSimulationStateAtFrame &item : states_at_frames_) {
      if (item.frame == frame) {
        return true;
      }
    }
    return false;
  }

  const ModifierSimulationState *get_state_at_exact_frame(const SubFrame &frame) const
  {
    for (const ModifierSimulationStateAtFrame &item : states_at_frames_) {
      if (item.frame == frame) {
        return item.state.get();
      }
    }
    return nullptr;
  }

  ModifierSimulationState &get_state_at_frame_for_write(const SubFrame &frame)
  {
    for (const ModifierSimulationStateAtFrame &item : states_at_frames_) {
      if (item.frame == frame) {
        return *item.state.get();
      }
    }
    states_at_frames_.append({frame, std::make_unique<ModifierSimulationState>()});
    return *states_at_frames_.last().state;
  }

  const ModifierSimulationStateAtFrame *try_get_last_state_before_frame(
      const SubFrame &frame) const
  {
    SubFrame last_frame = SubFrame::min();
    const ModifierSimulationStateAtFrame *last_item = nullptr;
    for (const auto &item : states_at_frames_) {
      if (item.frame < frame && item.frame > last_frame) {
        last_frame = item.frame;
        last_item = &item;
      }
    }
    return last_item;
  }

  void invalidate()
  {
    invalid_ = true;
  }

  bool is_invalid() const
  {
    return invalid_;
  }

  void reset()
  {
    states_at_frames_.clear();
    invalid_ = false;
  }
};

}  // namespace blender::bke::sim
