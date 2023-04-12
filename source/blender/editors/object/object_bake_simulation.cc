/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <fstream>
#include <iomanip>
#include <random>

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_serialize.hh"
#include "BLI_vector.hh"

#include "WM_types.h"

#include "ED_screen.h"

#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_context.h"
#include "BKE_main.h"
#include "BKE_scene.h"
#include "BKE_simulation_state.hh"

#include "RNA_enum_types.h"

#include "DEG_depsgraph.h"

#include "object_intern.h"

namespace blender::ed::object::bake_simulation {

static StringRefNull get_domain_io_name(const eAttrDomain domain)
{
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      return "point";
    case ATTR_DOMAIN_EDGE:
      return "edge";
    case ATTR_DOMAIN_FACE:
      return "face";
    case ATTR_DOMAIN_CORNER:
      return "corner";
    case ATTR_DOMAIN_CURVE:
      return "curve";
    case ATTR_DOMAIN_INSTANCE:
      return "instance";
    case ATTR_DOMAIN_AUTO:
      break;
  }
  BLI_assert_unreachable();
  return "";
}

static StringRefNull get_data_type_io_name(const eCustomDataType data_type)
{
  for (const EnumPropertyItem *item = rna_enum_attribute_type_items; item->identifier != nullptr;
       item++) {
    if (item->value == data_type) {
      return item->identifier;
    }
  }
  return "unkown";
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

static int bake_simulation_exec(bContext *C, wmOperator *op)
{
  using namespace bke::sim;

  Object *object = CTX_data_active_object(C);
  Scene *scene = CTX_data_scene(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Main *bmain = CTX_data_main(C);

  StringRefNull blend_file_path = BKE_main_blendfile_path(bmain);
  char blend_directory[FILE_MAX];
  BLI_split_dir_part(blend_file_path.c_str(), blend_directory, sizeof(blend_directory));
  char bake_directory[FILE_MAX];
  BLI_path_join(bake_directory, sizeof(bake_directory), blend_directory, "blend_bake");
  char bdata_directory[FILE_MAX];
  BLI_path_join(bdata_directory, sizeof(bdata_directory), bake_directory, "bdata");
  char meta_directory[FILE_MAX];
  BLI_path_join(meta_directory, sizeof(meta_directory), bake_directory, "meta");

  std::random_device random_device;
  std::mt19937 rng(random_device());
  std::uniform_int_distribution<uint32_t> rng_distribution;
  const uint32_t bake_id = rng_distribution(rng);
  const std::string bake_id_str = std::to_string(bake_id);

  char current_meta_directory[FILE_MAX];
  BLI_path_join(
      current_meta_directory, sizeof(current_meta_directory), meta_directory, bake_id_str.c_str());

  BLI_dir_create_recursive(bdata_directory);
  BLI_dir_create_recursive(current_meta_directory);

  const int old_frame = scene->r.cfra;

  Vector<NodesModifierData *> modifiers;
  LISTBASE_FOREACH (ModifierData *, md, &object->modifiers) {
    if (md->type == eModifierType_Nodes) {
      modifiers.append(reinterpret_cast<NodesModifierData *>(md));
    }
  }

  for (const int frame : IndexRange(1, 10)) {
    scene->r.cfra = frame;
    scene->r.subframe = 0.0f;

    std::stringstream bdata_file_name_ss;
    bdata_file_name_ss << bake_id_str << "_frame_" << std::setfill('0') << std::setw(5) << frame
                       << ".bdata";
    const std::string bdata_file_name = bdata_file_name_ss.str();
    char bdata_path[FILE_MAX];
    BLI_path_join(bdata_path, sizeof(bdata_path), bdata_directory, bdata_file_name.c_str());

    int64_t binary_file_offset = 0;
    fstream binary_data_file{bdata_path, std::ios::out | std::ios::binary};

    BKE_scene_graph_update_for_newframe(depsgraph);

    for (NodesModifierData *md : modifiers) {
      if (md->simulation_cache == nullptr) {
        continue;
      }

      io::serialize::DictionaryValue io_root;
      auto &io_root_elements = io_root.elements();
      io_root_elements.append({"version", std::make_shared<io::serialize::IntValue>(1)});
      auto io_zones = std::make_shared<io::serialize::ArrayValue>();
      io_root_elements.append({"zones", io_zones});

      ModifierSimulationCache &sim_cache = *md->simulation_cache;
      const ModifierSimulationState *sim_state = sim_cache.get_state_at_time(frame);
      if (sim_state == nullptr) {
        continue;
      }
      {
        std::lock_guard lock{sim_state->mutex_};
        for (const auto item : sim_state->zone_states_.items()) {
          const SimulationZoneID &zone_id = item.key;
          const SimulationZoneState &zone_state = *item.value;

          auto io_zone = std::make_shared<io::serialize::DictionaryValue>();
          io_zones->elements().append(io_zone);

          auto io_zone_id = std::make_shared<io::serialize::ArrayValue>();
          io_zone->elements().append({"zone_id", io_zone_id});

          for (const int node_id : zone_id.node_ids) {
            io_zone_id->elements().append(std::make_shared<io::serialize::IntValue>(node_id));
          }

          auto io_state_items = std::make_shared<io::serialize::ArrayValue>();
          io_zone->elements().append({"state_items", io_state_items});
          for (const std::unique_ptr<SimulationStateItem> &state_item : zone_state.items) {
            /* TODO: Use better id. */
            const std::string state_item_id = std::to_string(&state_item -
                                                             zone_state.items.begin());

            auto io_state_item = std::make_shared<io::serialize::DictionaryValue>();
            io_state_items->elements().append(io_state_item);

            io_state_item->elements().append(
                {"id", std::make_shared<io::serialize::StringValue>(state_item_id)});

            if (const GeometrySimulationStateItem *geometry_state_item =
                    dynamic_cast<const GeometrySimulationStateItem *>(state_item.get())) {
              io_state_item->elements().append(
                  {"type", std::make_shared<io::serialize::StringValue>("geometry")});

              const GeometrySet &geometry = geometry_state_item->geometry();

              auto io_geometry = std::make_shared<io::serialize::DictionaryValue>();
              io_state_item->elements().append({"data", io_geometry});

              if (geometry.has_mesh()) {
                const Mesh &mesh = *geometry.get_mesh_for_read();
                auto io_mesh = std::make_shared<io::serialize::DictionaryValue>();
                io_geometry->elements().append({"mesh", io_mesh});

                io_mesh->elements().append(
                    {"num_vertices", std::make_shared<io::serialize::IntValue>(mesh.totvert)});
                io_mesh->elements().append(
                    {"num_edges", std::make_shared<io::serialize::IntValue>(mesh.totedge)});
                io_mesh->elements().append(
                    {"num_polygons", std::make_shared<io::serialize::IntValue>(mesh.totpoly)});
                io_mesh->elements().append(
                    {"num_corners", std::make_shared<io::serialize::IntValue>(mesh.totloop)});

                auto io_materials = std::make_shared<io::serialize::ArrayValue>();
                io_mesh->elements().append({"materials", io_materials});

                for (const int material_index : IndexRange(mesh.totcol)) {
                  const Material *material = mesh.mat[material_index];

                  if (material == nullptr) {
                    io_materials->elements().append(std::make_shared<io::serialize::NullValue>());
                  }
                  else {
                    auto io_material = std::make_shared<io::serialize::DictionaryValue>();
                    io_materials->elements().append(io_material);
                    io_material->elements().append(
                        {"name",
                         std::make_shared<io::serialize::StringValue>(material->id.name + 2)});
                    if (material->id.lib != nullptr) {
                      io_material->elements().append({"lib_name",
                                                      std::make_shared<io::serialize::StringValue>(
                                                          material->id.lib->id.name + 2)});
                    }
                  }
                }

                auto io_attributes = std::make_shared<io::serialize::ArrayValue>();
                io_mesh->elements().append({"attributes", io_attributes});

                const bke::AttributeAccessor attributes = mesh.attributes();
                attributes.for_all([&](const bke::AttributeIDRef &attribute_id,
                                       const bke::AttributeMetaData &meta_data) {
                  auto io_attribute = std::make_shared<io::serialize::DictionaryValue>();
                  io_attributes->elements().append(io_attribute);

                  io_attribute->elements().append(
                      {"name", std::make_shared<io::serialize::StringValue>(attribute_id.name())});

                  const StringRefNull domain_name = get_domain_io_name(meta_data.domain);
                  io_attribute->elements().append(
                      {"domain", std::make_shared<io::serialize::StringValue>(domain_name)});

                  const StringRefNull type_name = get_data_type_io_name(meta_data.data_type);
                  io_attribute->elements().append(
                      {"type", std::make_shared<io::serialize::StringValue>(type_name)});

                  auto io_attribute_data = std::make_shared<io::serialize::DictionaryValue>();
                  io_attribute->elements().append({"data", io_attribute_data});
                  io_attribute_data->elements().append(
                      {"bdata_name",
                       std::make_shared<io::serialize::StringValue>(bdata_file_name)});
                  io_attribute_data->elements().append(
                      {"start", std::make_shared<io::serialize::IntValue>(binary_file_offset)});

                  const bke::GAttributeReader attribute = attributes.lookup(attribute_id);
                  const GVArraySpan attribute_span(attribute.varray);

                  const int64_t binary_size = attribute_span.size() * attribute_span.type().size();
                  binary_data_file.write(reinterpret_cast<const char *>(attribute_span.data()),
                                         binary_size);
                  binary_file_offset += binary_size;

                  return true;
                });
              }
            }
          }
        }
      }

      /* TODO: Escape modifier name or use different file name. */
      std::stringstream meta_file_name_ss;
      meta_file_name_ss << md->modifier.name << "_frame_" << std::setfill('0') << std::setw(5)
                        << frame << ".json";
      const std::string meta_file_name = meta_file_name_ss.str();
      char meta_file_path[FILE_MAX];
      BLI_path_join(
          meta_file_path, sizeof(meta_file_path), current_meta_directory, meta_file_name.c_str());

      io::serialize::JsonFormatter json_formatter;
      fstream meta_file{meta_file_path, std::ios::out};
      json_formatter.serialize(meta_file, io_root);
    }
  }

  scene->r.cfra = old_frame;
  DEG_time_tag_update(bmain);

  std::cout << "Hello\n";
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
