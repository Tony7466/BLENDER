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
#include "BKE_simulation_state_serialize.hh"

#include "RNA_access.h"
#include "RNA_enum_types.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"

#include "object_intern.h"

namespace blender::ed::object::bake_simulation {

class MyBDataReader : public bke::sim::BDataReader {
 private:
  std::string bdata_dir_;

 public:
  MyBDataReader(std::string bdata_dir) : bdata_dir_(std::move(bdata_dir)) {}

  [[nodiscard]] bool read(const bke::sim::BDataSlice &slice, void *r_data) const override
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

class MyBDataWriter : public bke::sim::BDataWriter {
 private:
  std::string bdata_name_;
  std::ostream &bdata_file_;
  int64_t current_offset_;
  Map<const ImplicitSharingInfo *, std::shared_ptr<io::serialize::DictionaryValue>> &shared_data_;

 public:
  MyBDataWriter(std::string bdata_name,
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

  bke::sim::BDataSlice write(const void *data, const int64_t size) override
  {
    const int64_t old_offset = current_offset_;
    bdata_file_.write(static_cast<const char *>(data), size);
    current_offset_ += size;
    return {bdata_name_, {old_offset, size}};
  }

  std::shared_ptr<io::serialize::DictionaryValue> write_shared(
      const ImplicitSharingInfo *sharing_info,
      const FunctionRef<std::shared_ptr<io::serialize::DictionaryValue>()> write_fn) override
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

void load_simulation_state(const StringRefNull meta_path,
                           const StringRefNull bdata_dir,
                           bke::sim::ModifierSimulationState &r_state);
void load_simulation_state(const StringRefNull meta_path,
                           const StringRefNull bdata_dir,
                           bke::sim::ModifierSimulationState &r_state)
{
  const MyBDataReader bdata_reader{bdata_dir};
  fstream meta_file{meta_path.c_str(), std::ios::in};
  io::serialize::JsonFormatter formatter;
  std::shared_ptr<io::serialize::Value> io_root_value = formatter.deserialize(meta_file);
  if (!io_root_value) {
    return;
  }
  const io::serialize::DictionaryValue *io_root = io_root_value->as_dictionary_value();
  if (!io_root) {
    return;
  }
  bke::sim::deserialize_modifier_simulation_state(*io_root, bdata_reader, r_state);
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
                                    bke::sim::get_meta_directory(*bmain, *object, *md),
                                    bke::sim::get_bdata_directory(*bmain, *object, *md)});
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
        ModifierSimulationCache &sim_cache = *nmd.simulation_cache;
        const ModifierSimulationState *sim_state = sim_cache.get_state_at_exact_frame(frame);
        if (sim_state == nullptr) {
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
        MyBDataWriter bdata_writer{bdata_file_name, bdata_file, 0, modifier_bake_data.shared_data};

        io::serialize::DictionaryValue io_root;
        bke::sim::serialize_modifier_simulation_state(*sim_state, bdata_writer, io_root);

        BLI_make_existing_file(meta_path);
        fstream meta_file{meta_path, std::ios::out};
        io::serialize::JsonFormatter formatter;
        formatter.serialize(meta_file, io_root);
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
        const std::string bake_directory = bke::sim::get_bake_directory(*bmain, *object, *md);
        BLI_delete(bake_directory.c_str(), true, true);
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
