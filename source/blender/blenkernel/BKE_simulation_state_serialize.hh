/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_simulation_state.hh"

#include "BLI_serialize.hh"

struct Main;
struct ModifierData;

namespace blender::bke::sim {

using DictionaryValue = io::serialize::DictionaryValue;
using DictionaryValuePtr = std::shared_ptr<DictionaryValue>;

struct BDataSlice {
  std::string name;
  IndexRange range;

  DictionaryValuePtr serialize() const;
  static std::optional<BDataSlice> deserialize(const io::serialize::DictionaryValue &io_slice);
};

class BDataReader {
 public:
  [[nodiscard]] virtual bool read(const BDataSlice &slice, void *r_data) const = 0;
};

class BDataWriter {
 public:
  virtual BDataSlice write(const void *data, int64_t size) = 0;
};

class BDataSharing {
 private:
  struct StoredByRuntimeValue {
    int64_t sharing_info_version;
    DictionaryValuePtr io_data;
  };

  /** The #ImplicitSharingInfo pointer is a weak user. */
  Map<const ImplicitSharingInfo *, StoredByRuntimeValue> stored_by_runtime_;
  /* TODO: Protect by mutex. */
  mutable Map<std::string, ImplicitSharingInfoAndData> runtime_by_stored_;

 public:
  ~BDataSharing();

  [[nodiscard]] DictionaryValuePtr write_shared(const ImplicitSharingInfo *sharing_info,
                                                FunctionRef<DictionaryValuePtr()> write_fn);

  [[nodiscard]] std::optional<ImplicitSharingInfoAndData> read_shared(
      const DictionaryValue &io_data,
      FunctionRef<std::optional<ImplicitSharingInfoAndData>()> read_fn) const;
};

class DiskBDataReader : public BDataReader {
 private:
  const std::string bdata_dir_;

 public:
  DiskBDataReader(std::string bdata_dir);
  [[nodiscard]] bool read(const BDataSlice &slice, void *r_data) const override;
};

class DiskBDataWriter : public BDataWriter {
 private:
  std::string bdata_name_;
  std::ostream &bdata_file_;
  int64_t current_offset_;

 public:
  DiskBDataWriter(std::string bdata_name, std::ostream &bdata_file, int64_t current_offset);

  BDataSlice write(const void *data, int64_t size) override;
};

std::string get_bake_directory(const Main &bmain, const Object &object, const ModifierData &md);
std::string get_bdata_directory(const Main &bmain, const Object &object, const ModifierData &md);
std::string get_meta_directory(const Main &bmain, const Object &object, const ModifierData &md);

void serialize_modifier_simulation_state(const ModifierSimulationState &state,
                                         BDataWriter &bdata_writer,
                                         BDataSharing &bdata_sharing,
                                         DictionaryValue &r_io_root);
void deserialize_modifier_simulation_state(const DictionaryValue &io_root,
                                           const BDataReader &bdata_reader,
                                           const BDataSharing &bdata_sharing,
                                           ModifierSimulationState &r_state);

}  // namespace blender::bke::sim
