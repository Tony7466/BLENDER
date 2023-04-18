/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_simulation_state.hh"

#include "BLI_serialize.hh"

namespace blender::bke::sim {

struct BDataSlice {
  std::string name;
  IndexRange range;

  std::shared_ptr<io::serialize::DictionaryValue> serialize() const;
  static std::optional<BDataSlice> deserialize(const io::serialize::DictionaryValue &io_slice);
};

class BDataReader {
 public:
  [[nodiscard]] virtual bool read(const BDataSlice &slice, void *r_data) const = 0;
};

class BDataWriter {
 public:
  virtual BDataSlice write(const void *data, const int64_t size) = 0;

  virtual std::shared_ptr<io::serialize::DictionaryValue> write_shared(
      const ImplicitSharingInfo *sharing_info,
      const FunctionRef<std::shared_ptr<io::serialize::DictionaryValue>()> write_fn) = 0;
};

void serialize_modifier_simulation_state(const ModifierSimulationState &state,
                                         BDataWriter &bdata_writer,
                                         io::serialize::DictionaryValue &r_io_root);
void deserialize_modifier_simulation_state(const io::serialize::DictionaryValue &io_root,
                                           const BDataReader &bdata_reader,
                                           ModifierSimulationState &r_state);

}  // namespace blender::bke::sim
