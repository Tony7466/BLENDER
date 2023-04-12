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
#include "BKE_main.h"
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

static std::string escape_name(StringRef name)
{
  std::stringstream ss;
  for (const char c : name) {
    if (('0' <= c && c <= '9') || ('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
      ss << c;
    }
    else {
      ss << int(c);
    }
  }
  return ss.str();
}

static GeometrySet load_geometry(const io::serialize::DictionaryValue &io_geometry,
                                 const StringRefNull bdata_dir)
{
  const auto io_geometry_lookup = io_geometry.create_lookup();
  GeometrySet geometry;
  if (io_geometry_lookup.contains("pointcloud")) {
    const auto &io_pointcloud = *io_geometry_lookup.lookup("pointcloud")->as_dictionary_value();
    const auto io_pointcloud_lookup = io_pointcloud.create_lookup();
    const int num_points = io_pointcloud_lookup.lookup("num_points")->as_int_value()->value();
    PointCloud *pointcloud = BKE_pointcloud_new_nomain(num_points);
    const auto &attributes_io = *io_pointcloud_lookup.lookup("attributes")->as_array_value();

    bke::MutableAttributeAccessor attributes = pointcloud->attributes_for_write();
    for (const auto &attribute_io : attributes_io.elements()) {
      const auto attribute_io_lookup = attribute_io->as_dictionary_value()->create_lookup();
      const StringRefNull name = attribute_io_lookup.lookup("name")->as_string_value()->value();
      const StringRefNull domain_str =
          attribute_io_lookup.lookup("domain")->as_string_value()->value();
      const eAttrDomain domain = get_domain_from_io_name(domain_str);
      const StringRefNull type_str =
          attribute_io_lookup.lookup("type")->as_string_value()->value();
      const eCustomDataType data_type = get_data_type_from_io_name(type_str);

      std::cout << name << " " << domain_str << " " << type_str << "\n";
      bke::GSpanAttributeWriter attribute = attributes.lookup_or_add_for_write_only_span(
          name, domain, data_type);

      auto io_data = attribute_io_lookup.lookup("data")->as_dictionary_value();

      const StringRefNull bdata_name = io_data->lookup_linear("name")->as_string_value()->value();
      const int start = io_data->lookup_linear("start")->as_int_value()->value();
      const StringRefNull endian_str =
          io_data->lookup_linear("endian")->as_string_value()->value();
      const bool is_same_endian = endian_str == get_endian_io_name(ENDIAN_ORDER);

      char bdata_path[FILE_MAX];
      BLI_path_join(bdata_path, sizeof(bdata_path), bdata_dir.c_str(), bdata_name.c_str());
      {
        fstream bdata_file{bdata_path, std::ios::in | std::ios::binary};
        bdata_file.seekg(start);
        bdata_file.read(static_cast<char *>(attribute.span.data()),
                        int64_t(num_points) * attribute.span.type().size());
      }

      if (!is_same_endian) {
        switch (data_type) {
          case CD_PROP_INT32:
          case CD_PROP_FLOAT:
          case CD_PROP_FLOAT2:
          case CD_PROP_FLOAT3:
          case CD_PROP_COLOR: {
            BLI_endian_switch_uint32_array(reinterpret_cast<uint32_t *>(attribute.span.data()),
                                           attribute.span.size_in_bytes() / sizeof(uint32_t));
            break;
          }
        }
      }

      attribute.finish();
    }

    geometry.replace_pointcloud(pointcloud);
  }
  return geometry;
}

using WriteBDataFn = FunctionRef<std::shared_ptr<io::serialize::DictionaryValue>(Span<uint8_t>)>;

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
    const bke::AttributeAccessor &attributes, const WriteBDataFn write_bdata)
{
  auto io_attributes = std::make_shared<io::serialize::ArrayValue>();
  attributes.for_all(
      [&](const bke::AttributeIDRef &attribute_id, const bke::AttributeMetaData &meta_data) {
        auto io_attribute = io_attributes->append_dict();

        io_attribute->append_str("name", attribute_id.name());

        const StringRefNull domain_name = get_domain_io_name(meta_data.domain);
        io_attribute->append_str("domain", domain_name);

        const StringRefNull type_name = get_data_type_io_name(meta_data.data_type);
        io_attribute->append_str("type", type_name);

        const bke::GAttributeReader attribute = attributes.lookup(attribute_id);
        const GVArraySpan attribute_span(attribute.varray);
        const int64_t binary_size = attribute_span.size() * attribute_span.type().size();

        auto io_attribute_data = write_bdata(
            Span<uint8_t>(reinterpret_cast<const uint8_t *>(attribute_span.data()), binary_size));
        io_attribute_data->append_str("endian", get_endian_io_name(ENDIAN_ORDER));
        io_attribute->append("data", io_attribute_data);

        return true;
      });
  return io_attributes;
}

static std::shared_ptr<io::serialize::DictionaryValue> serialize_geometry_set(
    const GeometrySet &geometry, const WriteBDataFn write_bdata)
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
      auto io_polygon_indices = write_bdata(
          Span<uint8_t>({reinterpret_cast<const uint8_t *>(mesh.poly_offset_indices),
                         int64_t(sizeof(*mesh.poly_offset_indices)) * mesh.totpoly + 1}));
      io_polygon_indices->append_str("endian", get_endian_io_name(ENDIAN_ORDER));
      io_mesh->append("poly_offset_indices", io_polygon_indices);
    }

    auto io_materials = serialize_material_slots({mesh.mat, mesh.totcol});
    io_mesh->append("materials", io_materials);

    auto io_attributes = serialize_attributes(mesh.attributes(), write_bdata);
    io_mesh->append("attributes", io_attributes);
  }
  if (geometry.has_pointcloud()) {
    const PointCloud &pointcloud = *geometry.get_pointcloud_for_read();
    auto io_pointcloud = io_geometry->append_dict("pointcloud");

    io_pointcloud->append_int("num_points", pointcloud.totpoint);

    auto io_materials = serialize_material_slots({pointcloud.mat, pointcloud.totcol});
    io_pointcloud->append("materials", io_materials);

    auto io_attributes = serialize_attributes(pointcloud.attributes(), write_bdata);
    io_pointcloud->append("attributes", io_attributes);
  }
  if (geometry.has_curves()) {
    const Curves &curves = *geometry.get_curves_for_read();
    auto io_curves = io_geometry->append_dict("curves");

    io_curves->append_int("num_points", curves.geometry.point_num);
    io_curves->append_int("num_curves", curves.geometry.curve_num);

    auto io_materials = serialize_material_slots({curves.mat, curves.totcol});
    io_curves->append("materials", io_materials);

    auto io_attributes = serialize_attributes(curves.geometry.wrap().attributes(), write_bdata);
    io_curves->append("attributes", io_attributes);
  }
  return io_geometry;
}

static bool bake_simulation_poll(bContext *C)
{
  if (!ED_operator_object_active(C)) {
    return false;
  }
  Main *bmain = CTX_data_main(C);
  if (BKE_main_blendfile_path(bmain) == nullptr) {
    CTX_wm_operator_poll_msg_set(C, "File has to be saved");
    return false;
  }
  return true;
}

static int bake_simulation_exec(bContext *C, wmOperator * /*op*/)
{
  using namespace bke::sim;

  Object *object = CTX_data_active_object(C);
  Scene *scene = CTX_data_scene(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Main *bmain = CTX_data_main(C);

  StringRefNull blend_file_path = BKE_main_blendfile_path(bmain);
  char blend_directory[FILE_MAX];
  char blend_name[FILE_MAX];
  BLI_split_dirfile(blend_file_path.c_str(),
                    blend_directory,
                    blend_name,
                    sizeof(blend_directory),
                    sizeof(blend_name));
  blend_name[StringRef(blend_name).rfind(".")] = '\0';
  const std::string bake_directory_name = "blendcache_" + StringRef(blend_name);
  char bake_directory[FILE_MAX];
  BLI_path_join(
      bake_directory, sizeof(bake_directory), blend_directory, bake_directory_name.c_str());
  char bdata_directory[FILE_MAX];
  BLI_path_join(bdata_directory, sizeof(bdata_directory), bake_directory, "bdata");
  char meta_directory[FILE_MAX];
  BLI_path_join(meta_directory, sizeof(meta_directory), bake_directory, "meta");

  const int old_frame = scene->r.cfra;

  const std::string object_name_escaped = escape_name(object->id.name + 2);

  Vector<NodesModifierData *> modifiers;
  Vector<std::string> modifier_meta_dirs;
  Vector<std::string> modifier_bdata_dirs;
  LISTBASE_FOREACH (ModifierData *, md, &object->modifiers) {
    if (md->type == eModifierType_Nodes) {
      modifiers.append(reinterpret_cast<NodesModifierData *>(md));
      const std::string id_escaped = object_name_escaped + "_" + escape_name(md->name);
      char modifier_bake_directory[FILE_MAX];
      BLI_path_join(modifier_bake_directory,
                    sizeof(modifier_bake_directory),
                    bake_directory,
                    id_escaped.c_str());
      char modifier_meta_directory[FILE_MAX];
      BLI_path_join(modifier_meta_directory,
                    sizeof(modifier_meta_directory),
                    modifier_bake_directory,
                    "meta");
      char modifier_bdata_directory[FILE_MAX];
      BLI_path_join(modifier_bdata_directory,
                    sizeof(modifier_bdata_directory),
                    modifier_bake_directory,
                    "bdata");
      modifier_meta_dirs.append(modifier_meta_directory);
      modifier_bdata_dirs.append(modifier_bdata_directory);
    }
  }

  for (const int frame : IndexRange(1, 10)) {
    scene->r.cfra = frame;
    scene->r.subframe = 0.0f;

    std::stringstream frame_ss;
    frame_ss << std::setfill('0') << std::setw(5) << frame;
    const std::string frame_str = frame_ss.str();

    BKE_scene_graph_update_for_newframe(depsgraph);

    for (const int modifier_i : modifiers.index_range()) {
      NodesModifierData &nmd = *modifiers[modifier_i];
      if (nmd.simulation_cache == nullptr) {
        continue;
      }

      const std::string bdata_file_name = frame_str + ".bdata";

      char bdata_path[FILE_MAX];
      BLI_path_join(bdata_path,
                    sizeof(bdata_path),
                    modifier_bdata_dirs[modifier_i].c_str(),
                    bdata_file_name.c_str());
      char meta_path[FILE_MAX];
      BLI_path_join(meta_path,
                    sizeof(meta_path),
                    modifier_meta_dirs[modifier_i].c_str(),
                    (frame_str + ".json").c_str());

      int64_t binary_file_offset = 0;
      BLI_make_existing_file(bdata_path);
      fstream binary_data_file{bdata_path, std::ios::out | std::ios::binary};

      io::serialize::DictionaryValue io_root;
      io_root.append_int("version", 1);
      auto io_zones = io_root.append_array("zones");

      ModifierSimulationCache &sim_cache = *nmd.simulation_cache;
      const ModifierSimulationState *sim_state = sim_cache.get_state_at_time(frame);
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

              auto io_geometry = serialize_geometry_set(geometry, [&](const Span<uint8_t> data) {
                binary_data_file.write(reinterpret_cast<const char *>(data.data()), data.size());
                auto io_data_ref = std::make_shared<io::serialize::DictionaryValue>();
                io_data_ref->append_str("name", bdata_file_name);
                io_data_ref->append_int("start", binary_file_offset);
                io_data_ref->append_int("size", data.size());
                binary_file_offset += data.size();
                return io_data_ref;
              });
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

  scene->r.cfra = old_frame;
  DEG_time_tag_update(bmain);

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
