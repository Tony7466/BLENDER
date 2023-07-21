/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_bake_items.hh"
#include "BKE_bake_items_serialize.hh"

#include "BLI_path_util.h"

namespace blender::bke {

using namespace io::serialize;
using DictionaryValuePtr = std::shared_ptr<DictionaryValue>;

GeometryBakeItem::GeometryBakeItem(GeometrySet geometry) : geometry(std::move(geometry)) {}

PrimitiveBakeItem::PrimitiveBakeItem(const CPPType &type, const void *value) : type_(type)
{
  value_ = MEM_mallocN_aligned(type.size(), type.alignment(), __func__);
  type.copy_construct(value, value_);
}

PrimitiveBakeItem::~PrimitiveBakeItem()
{
  type_.destruct(value_);
  MEM_freeN(value_);
}

StringBakeItem::StringBakeItem(std::string value) : value_(std::move(value)) {}

std::shared_ptr<DictionaryValue> BDataSlice::serialize() const
{
  auto io_slice = std::make_shared<DictionaryValue>();
  io_slice->append_str("name", this->name);
  io_slice->append_int("start", range.start());
  io_slice->append_int("size", range.size());
  return io_slice;
}

std::optional<BDataSlice> BDataSlice::deserialize(const DictionaryValue &io_slice)
{
  const std::optional<StringRefNull> name = io_slice.lookup_str("name");
  const std::optional<int64_t> start = io_slice.lookup_int("start");
  const std::optional<int64_t> size = io_slice.lookup_int("size");
  if (!name || !start || !size) {
    return std::nullopt;
  }

  return BDataSlice{*name, {*start, *size}};
}

DiskBDataReader::DiskBDataReader(std::string bdata_dir) : bdata_dir_(std::move(bdata_dir)) {}

[[nodiscard]] bool DiskBDataReader::read(const BDataSlice &slice, void *r_data) const
{
  if (slice.range.is_empty()) {
    return true;
  }

  char bdata_path[FILE_MAX];
  BLI_path_join(bdata_path, sizeof(bdata_path), bdata_dir_.c_str(), slice.name.c_str());

  std::lock_guard lock{mutex_};
  std::unique_ptr<fstream> &bdata_file = open_input_streams_.lookup_or_add_cb_as(
      bdata_path,
      [&]() { return std::make_unique<fstream>(bdata_path, std::ios::in | std::ios::binary); });
  bdata_file->seekg(slice.range.start());
  bdata_file->read(static_cast<char *>(r_data), slice.range.size());
  if (bdata_file->gcount() != slice.range.size()) {
    return false;
  }
  return true;
}

DiskBDataWriter::DiskBDataWriter(std::string bdata_name,
                                 std::ostream &bdata_file,
                                 const int64_t current_offset)
    : bdata_name_(std::move(bdata_name)), bdata_file_(bdata_file), current_offset_(current_offset)
{
}

BDataSlice DiskBDataWriter::write(const void *data, const int64_t size)
{
  const int64_t old_offset = current_offset_;
  bdata_file_.write(static_cast<const char *>(data), size);
  current_offset_ += size;
  return {bdata_name_, {old_offset, size}};
}

BDataSharing::~BDataSharing()
{
  for (const ImplicitSharingInfo *sharing_info : stored_by_runtime_.keys()) {
    sharing_info->remove_weak_user_and_delete_if_last();
  }
  for (const ImplicitSharingInfoAndData &value : runtime_by_stored_.values()) {
    if (value.sharing_info) {
      value.sharing_info->remove_user_and_delete_if_last();
    }
  }
}

DictionaryValuePtr BDataSharing::write_shared(const ImplicitSharingInfo *sharing_info,
                                              FunctionRef<DictionaryValuePtr()> write_fn)
{
  if (sharing_info == nullptr) {
    return write_fn();
  }
  return stored_by_runtime_.add_or_modify(
      sharing_info,
      /* Create new value. */
      [&](StoredByRuntimeValue *value) {
        new (value) StoredByRuntimeValue();
        value->io_data = write_fn();
        value->sharing_info_version = sharing_info->version();
        sharing_info->add_weak_user();
        return value->io_data;
      },
      /* Potentially modify existing value. */
      [&](StoredByRuntimeValue *value) {
        const int64_t new_version = sharing_info->version();
        BLI_assert(value->sharing_info_version <= new_version);
        if (value->sharing_info_version < new_version) {
          value->io_data = write_fn();
          value->sharing_info_version = new_version;
        }
        return value->io_data;
      });
}

std::optional<ImplicitSharingInfoAndData> BDataSharing::read_shared(
    const DictionaryValue &io_data,
    FunctionRef<std::optional<ImplicitSharingInfoAndData>()> read_fn) const
{
  std::lock_guard lock{mutex_};

  io::serialize::JsonFormatter formatter;
  std::stringstream ss;
  formatter.serialize(ss, io_data);
  const std::string key = ss.str();

  if (const ImplicitSharingInfoAndData *shared_data = runtime_by_stored_.lookup_ptr(key)) {
    shared_data->sharing_info->add_user();
    return *shared_data;
  }
  std::optional<ImplicitSharingInfoAndData> data = read_fn();
  if (!data) {
    return std::nullopt;
  }
  if (data->sharing_info != nullptr) {
    data->sharing_info->add_user();
    runtime_by_stored_.add_new(key, *data);
  }
  return data;
}

}  // namespace blender::bke
