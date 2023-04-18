/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <fstream>
#include <iomanip>
#include <random>

#include "BLI_endian_defines.h"
#include "BLI_endian_switch.h"
#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_serialize.hh"
#include "BLI_vector.hh"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_screen.h"

#include "DNA_curves_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_context.h"
#include "BKE_curves.hh"
#include "BKE_instances.hh"
#include "BKE_lib_id.h"
#include "BKE_main.h"
#include "BKE_mesh.hh"
#include "BKE_object.h"
#include "BKE_pointcloud.h"
#include "BKE_scene.h"
#include "BKE_simulation_state.hh"

#include "RNA_access.h"
#include "RNA_enum_types.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"

#include "object_intern.h"

namespace blender::ed::object::bake_simulation {

static std::string escape_name(StringRef name)
{
  std::stringstream ss;
  for (const char c : name) {
    if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
      ss << c;
    }
    else {
      ss << int(c);
    }
  }
  return ss.str();
}

static std::string get_blendcache_directory(const Main &bmain)
{
  StringRefNull blend_file_path = BKE_main_blendfile_path(&bmain);
  char blend_directory[FILE_MAX];
  char blend_name[FILE_MAX];
  BLI_split_dirfile(blend_file_path.c_str(),
                    blend_directory,
                    blend_name,
                    sizeof(blend_directory),
                    sizeof(blend_name));
  blend_name[StringRef(blend_name).rfind(".")] = '\0';
  const std::string blendcache_name = "blendcache_" + StringRef(blend_name);

  char blendcache_dir[FILE_MAX];
  BLI_path_join(blendcache_dir, sizeof(blendcache_dir), blend_directory, blendcache_name.c_str());
  return blendcache_dir;
}

static std::string get_modifier_sim_name(const Object &ob, const ModifierData &md)
{
  const std::string object_name_escaped = escape_name(ob.id.name + 2);
  const std::string modifier_name_escaped = escape_name(md.name);
  return "sim_" + object_name_escaped + "_" + modifier_name_escaped;
}

std::string get_bdata_directory(const Main &bmain, const Object &ob, const ModifierData &md);
std::string get_bdata_directory(const Main &bmain, const Object &ob, const ModifierData &md)
{
  char bdata_dir[FILE_MAX];
  BLI_path_join(bdata_dir,
                sizeof(bdata_dir),
                get_blendcache_directory(bmain).c_str(),
                get_modifier_sim_name(ob, md).c_str(),
                "bdata");
  return bdata_dir;
}

std::string get_meta_directory(const Main &bmain, const Object &ob, const ModifierData &md);
std::string get_meta_directory(const Main &bmain, const Object &ob, const ModifierData &md)
{
  char meta_dir[FILE_MAX];
  BLI_path_join(meta_dir,
                sizeof(meta_dir),
                get_blendcache_directory(bmain).c_str(),
                get_modifier_sim_name(ob, md).c_str(),
                "meta");
  return meta_dir;
}

static StringRefNull get_endian_io_name(const int endian)
{
  if (endian == L_ENDIAN) {
    return "little";
  }
  BLI_assert(endian == B_ENDIAN);
  return "big";
}

static StringRefNull get_domain_io_name(const eAttrDomain domain)
{
  const char *io_name = "unknown";
  RNA_enum_id_from_value(rna_enum_attribute_domain_items, domain, &io_name);
  return io_name;
}

static StringRefNull get_data_type_io_name(const eCustomDataType data_type)
{
  const char *io_name = "unknown";
  RNA_enum_id_from_value(rna_enum_attribute_type_items, data_type, &io_name);
  return io_name;
}

static eAttrDomain get_domain_from_io_name(const StringRefNull io_name)
{
  int domain;
  RNA_enum_value_from_identifier(rna_enum_attribute_domain_items, io_name.c_str(), &domain);
  return eAttrDomain(domain);
}

static eCustomDataType get_data_type_from_io_name(const StringRefNull io_name)
{
  int domain;
  RNA_enum_value_from_identifier(rna_enum_attribute_type_items, io_name.c_str(), &domain);
  return eCustomDataType(domain);
}

struct BDataSlice {
  std::string name;
  IndexRange range;

  std::shared_ptr<io::serialize::DictionaryValue> serialize() const
  {
    auto io_slice = std::make_shared<io::serialize::DictionaryValue>();
    io_slice->append_str("name", this->name);
    io_slice->append_int("start", range.start());
    io_slice->append_int("size", range.size());
    return io_slice;
  }

  static std::optional<BDataSlice> deserialize(const io::serialize::DictionaryValue &io_slice)
  {
    const std::optional<StringRefNull> name = io_slice.lookup_str("name");
    const std::optional<int64_t> start = io_slice.lookup_int("start");
    const std::optional<int64_t> size = io_slice.lookup_int("size");
    if (!name || !start || !size) {
      return std::nullopt;
    }

    return BDataSlice{*name, {*start, *size}};
  }
};

class BDataReader {
 private:
  std::string bdata_dir_;

 public:
  Map<std::string, std::pair<const ImplicitSharingInfo *, const void *>> &shared_data_;

  BDataReader(std::string bdata_dir,
              Map<std::string, std::pair<const ImplicitSharingInfo *, const void *>> &shared_data)
      : bdata_dir_(std::move(bdata_dir)), shared_data_(shared_data)
  {
  }

  [[nodiscard]] bool read(const BDataSlice &slice, void *r_data) const
  {
    if (slice.range.is_empty()) {
      return true;
    }

    char bdata_path[FILE_MAX];
    BLI_path_join(bdata_path, sizeof(bdata_path), bdata_dir_.c_str(), slice.name.c_str());

    fstream bdata_file{bdata_path, std::ios::in | std::ios::binary};
    bdata_file.seekg(slice.range.start());
    bdata_file.read(static_cast<char *>(r_data), slice.range.size());
    return true;
  }
};

class BDataWriter {
 private:
  std::string bdata_name_;
  std::ostream &bdata_file_;
  int64_t current_offset_;
  Map<const ImplicitSharingInfo *, std::shared_ptr<io::serialize::DictionaryValue>> &shared_data_;

 public:
  BDataWriter(std::string bdata_name,
              std::ostream &bdata_file,
              const int64_t current_offset,
              Map<const ImplicitSharingInfo *, std::shared_ptr<io::serialize::DictionaryValue>>
                  &shared_data)
      : bdata_name_(std::move(bdata_name)),
        bdata_file_(bdata_file),
        current_offset_(current_offset),
        shared_data_(shared_data)
  {
  }

  BDataSlice write(const void *data, const int64_t size)
  {
    const int64_t old_offset = current_offset_;
    bdata_file_.write(static_cast<const char *>(data), size);
    current_offset_ += size;
    return {bdata_name_, {old_offset, size}};
  }

  std::shared_ptr<io::serialize::DictionaryValue> write_shared(
      const ImplicitSharingInfo *sharing_info,
      const FunctionRef<std::shared_ptr<io::serialize::DictionaryValue>()> write_fn)
  {
    if (sharing_info == nullptr) {
      return write_fn();
    }
    return shared_data_.lookup_or_add_cb(sharing_info, [&]() {
      sharing_info->add_weak_user();
      return write_fn();
    });
  }
};

static std::optional<std::pair<const ImplicitSharingInfo *, const void *>> read_bdata_shared(
    BDataReader &bdata_reader,
    const io::serialize::DictionaryValue &io_data,
    const FunctionRef<std::optional<std::pair<const ImplicitSharingInfo *, const void *>>()>
        read_fn)
{
  io::serialize::JsonFormatter formatter;
  std::stringstream ss;
  formatter.serialize(ss, io_data);
  const std::string key = ss.str();
  if (const auto *value_ptr = bdata_reader.shared_data_.lookup_ptr_as(key)) {
    if (value_ptr->first->add_user_if_not_expired()) {
      return *value_ptr;
    }
    return std::nullopt;
  }
  std::optional<std::pair<const ImplicitSharingInfo *, const void *>> data = read_fn();
  if (!data) {
    return std::nullopt;
  }
  BLI_assert(data->first != nullptr || data->second == nullptr);
  if (data->first != nullptr) {
    data->first->add_weak_user();
  }
  bdata_reader.shared_data_.add_new(key, *data);
  return *data;
}

static std::shared_ptr<io::serialize::DictionaryValue> write_bdata_raw_data_with_endian(
    BDataWriter &bdata_writer, const void *data, const int64_t size_in_bytes)
{
  auto io_data = bdata_writer.write(data, size_in_bytes).serialize();
  if (ENDIAN_ORDER == B_ENDIAN) {
    io_data->append_str("endian", get_endian_io_name(ENDIAN_ORDER));
  }
  return io_data;
}

[[nodiscard]] static bool read_bdata_raw_data_with_endian(
    const BDataReader &bdata_reader,
    const io::serialize::DictionaryValue &io_data,
    const int64_t element_size,
    const int64_t elements_num,
    void *r_data)
{
  const std::optional<BDataSlice> slice = BDataSlice::deserialize(io_data);
  if (!slice) {
    return false;
  }
  if (slice->range.size() != element_size * elements_num) {
    return false;
  }
  if (!bdata_reader.read(*slice, r_data)) {
    return false;
  }
  const StringRefNull stored_endian = io_data.lookup_str("endian").value_or("little");
  const StringRefNull current_endian = get_endian_io_name(ENDIAN_ORDER);
  const bool need_endian_switch = stored_endian != current_endian;
  if (need_endian_switch) {
    switch (element_size) {
      case 1:
        break;
      case 2:
        BLI_endian_switch_uint16_array(static_cast<uint16_t *>(r_data), elements_num);
        break;
      case 4:
        BLI_endian_switch_uint32_array(static_cast<uint32_t *>(r_data), elements_num);
        break;
      case 8:
        BLI_endian_switch_uint64_array(static_cast<uint64_t *>(r_data), elements_num);
        break;
      default:
        return false;
    }
  }
  return true;
}

static std::shared_ptr<io::serialize::DictionaryValue> write_bdata_raw_bytes(
    BDataWriter &bdata_writer, const void *data, const int64_t size_in_bytes)
{
  return bdata_writer.write(data, size_in_bytes).serialize();
}

[[nodiscard]] static bool read_bdata_raw_bytes(const BDataReader &bdata_reader,
                                               const io::serialize::DictionaryValue &io_data,
                                               const int64_t bytes_num,
                                               void *r_data)
{
  const std::optional<BDataSlice> slice = BDataSlice::deserialize(io_data);
  if (!slice) {
    return false;
  }
  if (slice->range.size() != bytes_num) {
    return false;
  }
  return bdata_reader.read(*slice, r_data);
}

static std::shared_ptr<io::serialize::DictionaryValue> write_bdata_int_array(
    BDataWriter &bdata_writer, const Span<int> data)
{
  return write_bdata_raw_data_with_endian(bdata_writer, data.data(), data.size_in_bytes());
}

[[nodiscard]] static bool read_bdata_int_array(const BDataReader &bdata_reader,
                                               const io::serialize::DictionaryValue &io_data,
                                               MutableSpan<int> r_data)
{
  return read_bdata_raw_data_with_endian(
      bdata_reader, io_data, sizeof(int), r_data.size(), r_data.data());
}

static std::shared_ptr<io::serialize::DictionaryValue> write_bdata_simple_gspan(
    BDataWriter &bdata_writer, const GSpan data)
{
  const CPPType &type = data.type();
  BLI_assert(type.is_trivial());
  if (type.size() == 1 || type.is<ColorGeometry4b>()) {
    return write_bdata_raw_bytes(bdata_writer, data.data(), data.size_in_bytes());
  }
  return write_bdata_raw_data_with_endian(bdata_writer, data.data(), data.size_in_bytes());
}

[[nodiscard]] static bool read_bdata_simple_gspan(const BDataReader &bdata_reader,
                                                  const io::serialize::DictionaryValue &io_data,
                                                  GMutableSpan r_data)
{
  const CPPType &type = r_data.type();
  BLI_assert(type.is_trivial());
  if (type.size() == 1 || type.is<ColorGeometry4b>()) {
    return read_bdata_raw_bytes(bdata_reader, io_data, r_data.size_in_bytes(), r_data.data());
  }
  if (type.is_any<int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t, float>()) {
    return read_bdata_raw_data_with_endian(
        bdata_reader, io_data, type.size(), r_data.size(), r_data.data());
  }
  if (type.is_any<float2, int2>()) {
    return read_bdata_raw_data_with_endian(
        bdata_reader, io_data, sizeof(float), r_data.size() * 2, r_data.data());
  }
  if (type.is<float3>()) {
    return read_bdata_raw_data_with_endian(
        bdata_reader, io_data, sizeof(float), r_data.size() * 3, r_data.data());
  }
  if (type.is<float4x4>()) {
    return read_bdata_raw_data_with_endian(
        bdata_reader, io_data, sizeof(float), r_data.size() * 16, r_data.data());
  }
  if (type.is<ColorGeometry4f>()) {
    return read_bdata_raw_data_with_endian(
        bdata_reader, io_data, sizeof(float), r_data.size() * 4, r_data.data());
  }
  return false;
}

template<typename T>
static std::shared_ptr<io::serialize::DictionaryValue> write_bdata_simple_span(
    BDataWriter &bdata_writer, const Span<T> data)
{
  return write_bdata_simple_gspan(bdata_writer, data);
}

template<typename T>
[[nodiscard]] static bool read_bdata_simple_span(const BDataReader &bdata_reader,
                                                 const io::serialize::DictionaryValue &io_data,
                                                 MutableSpan<T> r_data)
{
  return read_bdata_simple_gspan(bdata_reader, io_data, r_data);
}

static void load_attributes(const io::serialize::ArrayValue &io_attributes,
                            bke::MutableAttributeAccessor &attributes,
                            const BDataReader &bdata_reader)
{
  for (const auto &io_attribute_value : io_attributes.elements()) {
    const auto *io_attribute = io_attribute_value->as_dictionary_value();
    if (!io_attribute) {
      continue;
    }
    const std::optional<StringRefNull> name = io_attribute->lookup_str("name");
    const std::optional<StringRefNull> domain_str = io_attribute->lookup_str("domain");
    const std::optional<StringRefNull> type_str = io_attribute->lookup_str("type");
    auto io_data = io_attribute->lookup_dict("data");
    if (!name || !domain_str || !type_str || !io_data) {
      continue;
    }

    const eAttrDomain domain = get_domain_from_io_name(*domain_str);
    const eCustomDataType data_type = get_data_type_from_io_name(*type_str);

    bke::GSpanAttributeWriter attribute = attributes.lookup_or_add_for_write_only_span(
        *name, domain, data_type);
    if (!attribute) {
      continue;
    }
    const CPPType &cpp_type = attribute.span.type();
    if (!read_bdata_simple_gspan(bdata_reader, *io_data, attribute.span)) {
      cpp_type.value_initialize_n(attribute.span.data(), attribute.span.size());
    }

    attribute.finish();
  }
}

static PointCloud *try_load_pointcloud(const io::serialize::DictionaryValue &io_geometry,
                                       const BDataReader &bdata_reader)
{
  const io::serialize::DictionaryValue *io_pointcloud = io_geometry.lookup_dict("pointcloud");
  if (!io_pointcloud) {
    return nullptr;
  }
  const int num_points = io_pointcloud->lookup_int("num_points").value_or(0);
  const io::serialize::ArrayValue *io_attributes = io_pointcloud->lookup_array("attributes");
  if (!io_attributes) {
    return nullptr;
  }
  PointCloud *pointcloud = BKE_pointcloud_new_nomain(num_points);
  bke::MutableAttributeAccessor attributes = pointcloud->attributes_for_write();
  load_attributes(*io_attributes, attributes, bdata_reader);
  return pointcloud;
}

static Curves *try_load_curves(const io::serialize::DictionaryValue &io_geometry,
                               const BDataReader &bdata_reader)
{
  const io::serialize::DictionaryValue *io_curves = io_geometry.lookup_dict("curves");
  if (!io_curves) {
    return nullptr;
  }
  const int num_points = io_curves->lookup_int("num_points").value_or(0);
  const int num_curves = io_curves->lookup_int("num_curves").value_or(0);

  const io::serialize::ArrayValue *io_attributes = io_curves->lookup_array("attributes");
  if (!io_attributes) {
    return nullptr;
  }

  Curves *curves_id = bke::curves_new_nomain(num_points, num_curves);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();

  auto cancel = [&]() {
    BKE_id_free(nullptr, curves_id);
    return nullptr;
  };

  if (num_curves > 0) {
    const auto io_curve_offsets = io_curves->lookup_dict("curve_offsets");
    if (!io_curve_offsets) {
      return cancel();
    }
    if (!read_bdata_int_array(bdata_reader, *io_curve_offsets, curves.offsets_for_write())) {
      return cancel();
    }
  }

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  load_attributes(*io_attributes, attributes, bdata_reader);

  return curves_id;
}

static Mesh *try_load_mesh(const io::serialize::DictionaryValue &io_geometry,
                           const BDataReader &bdata_reader)
{
  const io::serialize::DictionaryValue *io_mesh = io_geometry.lookup_dict("mesh");
  if (!io_mesh) {
    return nullptr;
  }
  const int num_verts = io_mesh->lookup_int("num_vertices").value_or(0);
  const int num_edges = io_mesh->lookup_int("num_edges").value_or(0);
  const int num_polys = io_mesh->lookup_int("num_polygons").value_or(0);
  const int num_corners = io_mesh->lookup_int("num_corners").value_or(0);

  const io::serialize::ArrayValue *io_attributes = io_mesh->lookup_array("attributes");
  if (!io_attributes) {
    return nullptr;
  }

  Mesh *mesh = BKE_mesh_new_nomain(num_verts, num_edges, num_corners, num_polys);

  auto cancel = [&]() {
    BKE_id_free(nullptr, mesh);
    return nullptr;
  };

  if (num_polys > 0) {
    const auto io_poly_offsets = io_mesh->lookup_dict("poly_offsets");
    if (!io_poly_offsets) {
      return cancel();
    }
    if (!read_bdata_int_array(bdata_reader, *io_poly_offsets, mesh->poly_offsets_for_write())) {
      return cancel();
    }
  }

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  load_attributes(*io_attributes, attributes, bdata_reader);

  return mesh;
}

static GeometrySet load_geometry(const io::serialize::DictionaryValue &io_geometry,
                                 const BDataReader &bdata_reader);

static bke::Instances *try_load_instances(const io::serialize::DictionaryValue &io_geometry,
                                          const BDataReader &bdata_reader)
{
  const io::serialize::DictionaryValue *io_instances = io_geometry.lookup_dict("instances");
  if (!io_instances) {
    return nullptr;
  }
  const int num_instances = io_instances->lookup_int("num_instances").value_or(0);
  if (num_instances == 0) {
    return nullptr;
  }
  const io::serialize::ArrayValue *io_attributes = io_instances->lookup_array("attributes");
  if (!io_attributes) {
    return nullptr;
  }
  const io::serialize::ArrayValue *io_references = io_instances->lookup_array("references");
  if (!io_references) {
    return nullptr;
  }

  bke::Instances *instances = new bke::Instances();
  instances->resize(num_instances);

  const auto cancel = [&]() {
    delete instances;
    return nullptr;
  };

  for (const auto &io_reference_value : io_references->elements()) {
    const io::serialize::DictionaryValue *io_reference = io_reference_value->as_dictionary_value();
    GeometrySet reference_geometry;
    if (io_reference) {
      reference_geometry = load_geometry(*io_reference, bdata_reader);
    }
    instances->add_reference(std::move(reference_geometry));
  }

  const auto io_transforms = io_instances->lookup_dict("transforms");
  if (!io_transforms) {
    return cancel();
  }
  if (!read_bdata_simple_span(bdata_reader, *io_transforms, instances->transforms())) {
    return cancel();
  }

  const auto io_handles = io_instances->lookup_dict("handles");
  if (!io_handles) {
    return cancel();
  }
  if (!read_bdata_simple_span(bdata_reader, *io_handles, instances->reference_handles())) {
    return cancel();
  }

  bke::MutableAttributeAccessor attributes = instances->attributes_for_write();
  load_attributes(*io_attributes, attributes, bdata_reader);

  return instances;
}

static GeometrySet load_geometry(const io::serialize::DictionaryValue &io_geometry,
                                 const BDataReader &bdata_reader)
{
  GeometrySet geometry;
  if (Mesh *mesh = try_load_mesh(io_geometry, bdata_reader)) {
    geometry.replace_mesh(mesh);
  }
  if (PointCloud *pointcloud = try_load_pointcloud(io_geometry, bdata_reader)) {
    geometry.replace_pointcloud(pointcloud);
  }
  if (Curves *curves = try_load_curves(io_geometry, bdata_reader)) {
    geometry.replace_curves(curves);
  }
  if (bke::Instances *instances = try_load_instances(io_geometry, bdata_reader)) {
    geometry.replace_instances(instances);
  }
  return geometry;
}

void load_simulation_state(const StringRefNull meta_path,
                           const StringRefNull bdata_dir,
                           bke::sim::ModifierSimulationState &r_state);
void load_simulation_state(const StringRefNull meta_path,
                           const StringRefNull bdata_dir,
                           bke::sim::ModifierSimulationState &r_state)
{
  Map<std::string, std::pair<const ImplicitSharingInfo *, const void *>> shared_data;
  const BDataReader bdata_reader{bdata_dir, shared_data};

  std::unique_ptr<io::serialize::Value> io_root_value;
  {
    io::serialize::JsonFormatter formatter;
    fstream meta_file{meta_path.c_str(), std::ios::in};
    io_root_value = formatter.deserialize(meta_file);
  }
  if (!io_root_value) {
    return;
  }
  const io::serialize::DictionaryValue *io_root = io_root_value->as_dictionary_value();
  if (!io_root) {
    return;
  }
  const std::optional<int> version = io_root->lookup_int("version");
  if (!version) {
    return;
  }
  if (*version != 1) {
    return;
  }
  const io::serialize::ArrayValue *io_zones = io_root->lookup_array("zones");
  if (!io_zones) {
    return;
  }
  for (const auto &io_zone_value : io_zones->elements()) {
    const io::serialize::DictionaryValue *io_zone = io_zone_value->as_dictionary_value();
    if (!io_zone) {
      continue;
    }
    const io::serialize::ArrayValue *io_zone_id = io_zone->lookup_array("zone_id");
    bke::sim::SimulationZoneID zone_id;
    for (const auto &io_zone_id_element : io_zone_id->elements()) {
      const io::serialize::IntValue *io_node_id = io_zone_id_element->as_int_value();
      if (!io_node_id) {
        continue;
      }
      zone_id.node_ids.append(io_node_id->value());
    }

    const io::serialize::ArrayValue *io_state_items = io_zone->lookup_array("state_items");
    if (!io_state_items) {
      continue;
    }

    auto zone_state = std::make_unique<bke::sim::SimulationZoneState>();

    for (const auto &io_state_item_value : io_state_items->elements()) {
      const io::serialize::DictionaryValue *io_state_item =
          io_state_item_value->as_dictionary_value();
      if (!io_state_item) {
        continue;
      }
      const std::optional<StringRefNull> state_item_type = io_state_item->lookup_str("type");
      if (!state_item_type) {
        continue;
      }
      if (*state_item_type == "geometry") {
        const io::serialize::DictionaryValue *io_geometry = io_state_item->lookup_dict("data");
        if (!io_geometry) {
          continue;
        }
        GeometrySet geometry = load_geometry(*io_geometry, bdata_reader);
        auto state_item = std::make_unique<bke::sim::GeometrySimulationStateItem>(
            std::move(geometry));
        zone_state->items.append(std::move(state_item));
      }
    }

    r_state.zone_states_.add_overwrite(zone_id, std::move(zone_state));
  }
}

static std::shared_ptr<io::serialize::ArrayValue> serialize_material_slots(
    const Span<const Material *> material_slots)
{
  auto io_materials = std::make_shared<io::serialize::ArrayValue>();
  for (const Material *material : material_slots) {
    if (material == nullptr) {
      io_materials->append_null();
    }
    else {
      auto io_material = io_materials->append_dict();
      io_material->append_str("name", material->id.name + 2);
      if (material->id.lib != nullptr) {
        io_material->append_str("lib_name", material->id.lib->id.name + 2);
      }
    }
  }
  return io_materials;
}

static std::shared_ptr<io::serialize::ArrayValue> serialize_attributes(
    const bke::AttributeAccessor &attributes,
    BDataWriter &bdata_writer,
    const Set<std::string> &attributes_to_ignore)
{
  auto io_attributes = std::make_shared<io::serialize::ArrayValue>();
  attributes.for_all(
      [&](const bke::AttributeIDRef &attribute_id, const bke::AttributeMetaData &meta_data) {
        if (attributes_to_ignore.contains_as(attribute_id.name())) {
          return true;
        }

        auto io_attribute = io_attributes->append_dict();

        io_attribute->append_str("name", attribute_id.name());

        const StringRefNull domain_name = get_domain_io_name(meta_data.domain);
        io_attribute->append_str("domain", domain_name);

        const StringRefNull type_name = get_data_type_io_name(meta_data.data_type);
        io_attribute->append_str("type", type_name);

        const bke::GAttributeReader attribute = attributes.lookup(attribute_id);
        const GVArraySpan attribute_span(attribute.varray);
        io_attribute->append("data", bdata_writer.write_shared(attribute.sharing_info, [&]() {
          return write_bdata_simple_gspan(bdata_writer, attribute_span);
        }));

        return true;
      });
  return io_attributes;
}

static std::shared_ptr<io::serialize::DictionaryValue> serialize_geometry_set(
    const GeometrySet &geometry, BDataWriter &bdata_writer)
{
  auto io_geometry = std::make_shared<io::serialize::DictionaryValue>();
  if (geometry.has_mesh()) {
    const Mesh &mesh = *geometry.get_mesh_for_read();
    auto io_mesh = io_geometry->append_dict("mesh");

    io_mesh->append_int("num_vertices", mesh.totvert);
    io_mesh->append_int("num_edges", mesh.totedge);
    io_mesh->append_int("num_polygons", mesh.totpoly);
    io_mesh->append_int("num_corners", mesh.totloop);

    if (mesh.totpoly > 0) {
      io_mesh->append("poly_offsets",
                      bdata_writer.write_shared(mesh.runtime->poly_offsets_sharing_info, [&]() {
                        return write_bdata_int_array(bdata_writer, mesh.poly_offsets());
                      }));
    }

    auto io_materials = serialize_material_slots({mesh.mat, mesh.totcol});
    io_mesh->append("materials", io_materials);

    auto io_attributes = serialize_attributes(mesh.attributes(), bdata_writer, {});
    io_mesh->append("attributes", io_attributes);
  }
  if (geometry.has_pointcloud()) {
    const PointCloud &pointcloud = *geometry.get_pointcloud_for_read();
    auto io_pointcloud = io_geometry->append_dict("pointcloud");

    io_pointcloud->append_int("num_points", pointcloud.totpoint);

    auto io_materials = serialize_material_slots({pointcloud.mat, pointcloud.totcol});
    io_pointcloud->append("materials", io_materials);

    auto io_attributes = serialize_attributes(pointcloud.attributes(), bdata_writer, {});
    io_pointcloud->append("attributes", io_attributes);
  }
  if (geometry.has_curves()) {
    const Curves &curves_id = *geometry.get_curves_for_read();
    const bke::CurvesGeometry &curves = curves_id.geometry.wrap();

    auto io_curves = io_geometry->append_dict("curves");

    io_curves->append_int("num_points", curves.point_num);
    io_curves->append_int("num_curves", curves.curve_num);

    if (curves.curve_num > 0) {
      io_curves->append(
          "curve_offsets",
          bdata_writer.write_shared(curves.runtime->curve_offsets_sharing_info, [&]() {
            return write_bdata_int_array(bdata_writer, curves.offsets());
          }));
    }

    auto io_materials = serialize_material_slots({curves_id.mat, curves_id.totcol});
    io_curves->append("materials", io_materials);

    auto io_attributes = serialize_attributes(curves.attributes(), bdata_writer, {});
    io_curves->append("attributes", io_attributes);
  }
  if (geometry.has_instances()) {
    const bke::Instances &instances = *geometry.get_instances_for_read();
    auto io_instances = io_geometry->append_dict("instances");

    io_instances->append_int("num_instances", instances.instances_num());

    auto io_references = io_instances->append_array("references");
    for (const bke::InstanceReference &reference : instances.references()) {
      BLI_assert(reference.type() == bke::InstanceReference::Type::GeometrySet);
      io_references->append(serialize_geometry_set(reference.geometry_set(), bdata_writer));
    }

    io_instances->append("transforms",
                         write_bdata_simple_span(bdata_writer, instances.transforms()));
    io_instances->append("handles",
                         write_bdata_simple_span(bdata_writer, instances.reference_handles()));

    auto io_attributes = serialize_attributes(instances.attributes(), bdata_writer, {"position"});
    io_instances->append("attributes", io_attributes);
  }
  return io_geometry;
}

static bool bake_simulation_poll(bContext *C)
{
  if (!ED_operator_object_active(C)) {
    return false;
  }
  Main *bmain = CTX_data_main(C);
  const char *path = BKE_main_blendfile_path(bmain);
  if (path == nullptr || path[0] == '\0') {
    CTX_wm_operator_poll_msg_set(C, "File has to be saved");
    return false;
  }
  return true;
}

struct ModifierBakeData {
  NodesModifierData *nmd;
  std::string meta_dir;
  std::string bdata_dir;
  Map<const ImplicitSharingInfo *, std::shared_ptr<io::serialize::DictionaryValue>> shared_data;

  ~ModifierBakeData()
  {
    for (const ImplicitSharingInfo *sharing_info : shared_data.keys()) {
      sharing_info->remove_weak_user_and_delete_if_last();
    };
  }
};

struct ObjectBakeData {
  Object *object;
  Vector<ModifierBakeData> modifiers;
};

static int bake_simulation_exec(bContext *C, wmOperator * /*op*/)
{
  using namespace bke::sim;

  Scene *scene = CTX_data_scene(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Main *bmain = CTX_data_main(C);

  Vector<ObjectBakeData> objects_to_bake;
  CTX_DATA_BEGIN (C, Object *, object, selected_objects) {
    if (!BKE_id_is_editable(bmain, &object->id)) {
      continue;
    }
    ObjectBakeData bake_data;
    bake_data.object = object;
    LISTBASE_FOREACH (ModifierData *, md, &object->modifiers) {
      if (md->type == eModifierType_Nodes) {
        NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
        if (nmd->simulation_cache != nullptr) {
          nmd->simulation_cache->reset();
        }
        bake_data.modifiers.append({nmd,
                                    get_meta_directory(*bmain, *object, *md),
                                    get_bdata_directory(*bmain, *object, *md)});
      }
    }
    objects_to_bake.append(std::move(bake_data));
  }
  CTX_DATA_END;

  const int old_frame = scene->r.cfra;

  for (float frame_f = scene->r.sfra; frame_f <= scene->r.efra; frame_f += 1.0f) {
    const SubFrame frame{frame_f};

    scene->r.cfra = frame.frame();
    scene->r.subframe = frame.subframe();

    char frame_file_c_str[64];
    BLI_snprintf(frame_file_c_str, sizeof(frame_file_c_str), "%011.5f", double(frame));
    BLI_str_replace_char(frame_file_c_str, '.', '_');
    const StringRefNull frame_file_str = frame_file_c_str;

    BKE_scene_graph_update_for_newframe(depsgraph);

    for (ObjectBakeData &object_bake_data : objects_to_bake) {
      for (ModifierBakeData &modifier_bake_data : object_bake_data.modifiers) {
        NodesModifierData &nmd = *modifier_bake_data.nmd;
        if (nmd.simulation_cache == nullptr) {
          continue;
        }

        const std::string bdata_file_name = frame_file_str + ".bdata";

        char bdata_path[FILE_MAX];
        BLI_path_join(bdata_path,
                      sizeof(bdata_path),
                      modifier_bake_data.bdata_dir.c_str(),
                      bdata_file_name.c_str());
        char meta_path[FILE_MAX];
        BLI_path_join(meta_path,
                      sizeof(meta_path),
                      modifier_bake_data.meta_dir.c_str(),
                      (frame_file_str + ".json").c_str());

        BLI_make_existing_file(bdata_path);
        fstream bdata_file{bdata_path, std::ios::out | std::ios::binary};
        BDataWriter bdata_writer{bdata_file_name, bdata_file, 0, modifier_bake_data.shared_data};

        io::serialize::DictionaryValue io_root;
        io_root.append_int("version", 1);
        auto io_zones = io_root.append_array("zones");

        ModifierSimulationCache &sim_cache = *nmd.simulation_cache;
        const ModifierSimulationState *sim_state = sim_cache.get_state_at_exact_frame(frame);
        if (sim_state == nullptr) {
          continue;
        }
        {
          std::lock_guard lock{sim_state->mutex_};
          for (const auto item : sim_state->zone_states_.items()) {
            const SimulationZoneID &zone_id = item.key;
            const SimulationZoneState &zone_state = *item.value;

            auto io_zone = io_zones->append_dict();

            auto io_zone_id = io_zone->append_array("zone_id");

            for (const int node_id : zone_id.node_ids) {
              io_zone_id->append_int(node_id);
            }

            auto io_state_items = io_zone->append_array("state_items");
            for (const std::unique_ptr<SimulationStateItem> &state_item : zone_state.items) {
              /* TODO: Use better id. */
              const std::string state_item_id = std::to_string(&state_item -
                                                               zone_state.items.begin());

              auto io_state_item = io_state_items->append_dict();

              io_state_item->append_str("id", state_item_id);

              if (const GeometrySimulationStateItem *geometry_state_item =
                      dynamic_cast<const GeometrySimulationStateItem *>(state_item.get())) {
                io_state_item->append_str("type", "geometry");

                const GeometrySet &geometry = geometry_state_item->geometry();

                auto io_geometry = serialize_geometry_set(geometry, bdata_writer);
                io_state_item->append("data", io_geometry);
              }
            }
          }
        }

        BLI_make_existing_file(meta_path);
        fstream meta_file{meta_path, std::ios::out};
        io::serialize::JsonFormatter json_formatter;
        json_formatter.serialize(meta_file, io_root);
      }
    }
  }

  for (ObjectBakeData &object_bake_data : objects_to_bake) {
    for (ModifierBakeData &modifier_bake_data : object_bake_data.modifiers) {
      NodesModifierData &nmd = *modifier_bake_data.nmd;
      if (nmd.simulation_cache) {
        nmd.simulation_cache->cache_state_ = CacheState::Baked;
      }
    }
    DEG_id_tag_update(&object_bake_data.object->id, ID_RECALC_GEOMETRY);
  }

  scene->r.cfra = old_frame;
  DEG_time_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);

  return OPERATOR_FINISHED;
}

static int delete_baked_simulation_exec(bContext *C, wmOperator * /*op*/)
{
  Main *bmain = CTX_data_main(C);

  CTX_DATA_BEGIN (C, Object *, object, selected_objects) {
    LISTBASE_FOREACH (ModifierData *, md, &object->modifiers) {
      if (md->type == eModifierType_Nodes) {
        NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
        char bake_directory[FILE_MAX];
        BLI_path_join(bake_directory,
                      sizeof(bake_directory),
                      get_blendcache_directory(*bmain).c_str(),
                      get_modifier_sim_name(*object, *md).c_str());
        BLI_delete(bake_directory, true, true);
        if (nmd->simulation_cache != nullptr) {
          nmd->simulation_cache->reset();
        }
      }
    }

    DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
  }
  CTX_DATA_END;

  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);

  return OPERATOR_FINISHED;
}

}  // namespace blender::ed::object::bake_simulation

void OBJECT_OT_bake_simulation(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_simulation;

  ot->name = "Bake Simulation";
  ot->description = "Bake simulations in geometry nodes modifiers";
  ot->idname = __func__;

  ot->exec = bake_simulation_exec;
  ot->poll = bake_simulation_poll;
}

void OBJECT_OT_delete_baked_simulation(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_simulation;

  ot->name = "Delete Baked Simulation";
  ot->description = "Delete baked simulation";
  ot->idname = __func__;

  ot->exec = delete_baked_simulation_exec;
  ot->poll = bake_simulation_poll;
}
