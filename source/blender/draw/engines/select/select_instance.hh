/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup draw_engine
 *
 */

#pragma once

#include "DRW_gpu_wrapper.hh"

#include "draw_manager.hh"
#include "draw_pass.hh"

#include "gpu_shader_create_info.hh"

#include "select_defines.h"
#include "select_shader_shared.hh"

namespace blender::draw::select {

/**
 * Dummy implementation of the select engine types to avoid any overhead.
 * Bypass any selection logic.
 */
struct InstanceDummy {
  /* Add type safety to selection ID. Only the select engine should provide them. */
  class ID {
   public:
    constexpr uint32_t get() const
    {
      return 0;
    }
  };

  struct SelectShader {
    static void patch(gpu::shader::ShaderCreateInfo &){};
  };

  struct SelectBuf {
    void select_clear(){};
    void select_append(ID){};
    void select_bind(PassSimple &){};
  };

  struct SelectMap {
    [[nodiscard]] const ID select_id(const ObjectRef &, uint = 0)
    {
      return {};
    }

    void begin_sync(){};

    void select_bind(PassSimple &){};

    void select_bind(PassMain &){};

    void end_sync(){};

    void read_result(){};
  };
};

/**
 * This is an implementation of the Select engine specialized for selecting object.
 * Should plug seamlessly inside the overlay engine logic.
 */
struct Instance {
  /* Add type safety to selection ID. Only the select engine should provide them. */
  class ID {
   public:
    uint32_t value;

    uint32_t get() const
    {
      return value;
    }
  };

  struct SelectShader {
    static void patch(gpu::shader::ShaderCreateInfo &info)
    {
      info.define("SELECT_ENABLE");
      /* Replace additional info. */
      for (StringRefNull &str : info.additional_infos_) {
        if (str == "draw_modelmat_new") {
          str = "draw_modelmat_new_with_custom_id";
        }
      }
      info.additional_info("select_id_patch");
    };
  };

  /**
   * Add a dedicated selection id buffer to a pass.
   * To be used when not using a #PassMain which can pass the select ID via CustomID.
   */
  struct SelectBuf {
    StorageVectorBuffer<uint32_t> select_buf = {"select_buf"};

    void select_clear()
    {
      select_buf.clear();
    }

    void select_append(ID select_id)
    {
      select_buf.append(select_id.get());
    }

    void select_bind(PassSimple &pass)
    {
      select_buf.push_update();
      pass.bind_ssbo(SELECT_ID_IN, &select_buf);
    }
  };

  /**
   * Generate selection IDs from objects and keep record of the mapping between them.
   * The id's are contiguous so that we can create a destination buffer.
   */
  struct SelectMap {
    /** Mapping between internal IDs and `object->runtime.select_id`. */
    Vector<uint> select_id_map;
#ifdef DEBUG
    /** Debug map containing a copy of the object name. */
    Vector<std::string> map_names;
#endif
    /** Stores the result of the whole selection drawing. Content depends on selection mode. */
    StorageArrayBuffer<uint> select_output_buf = {"select_output_buf"};
    /** Dummy buffer. Might be better to remove, but simplify the shader create info patching. */
    StorageArrayBuffer<uint, 4, true> dummy_select_buf = {"dummy_select_buf"};
    /** Uniform buffer to bind to all passes to pass information about the selection state. */
    UniformBuffer<SelectInfoData> info_buf;

    /* TODO(fclem): The sub_object_id id should eventually become some enum or take a sub-object
     * reference directly. This would isolate the selection logic to this class. */
    [[nodiscard]] const ID select_id(const ObjectRef &ob_ref, uint sub_object_id = 0)
    {
      uint object_id = ob_ref.object->runtime.select_id;
      uint id = select_id_map.append_and_get_index(object_id | sub_object_id);
#ifdef DEBUG
      map_names.append(ob_ref.object->id.name);
#endif
      return {id};
    }

    /* Load an invalid index that will not write to the output (not selectable). */
    [[nodiscard]] const ID select_invalid_id()
    {
      return {uint32_t(-1)};
    }

    void begin_sync()
    {
      select_id_map.clear();
#ifdef DEBUG
      map_names.clear();
#endif
    }

    /** IMPORTANT: Changes the draw state. Need to be called after the pass own state. */
    void select_bind(PassSimple &pass)
    {
      pass.bind_ubo(SELECT_DATA, &info_buf);
      pass.bind_ssbo(SELECT_ID_OUT, &select_output_buf);
    }

    /** IMPORTANT: Changes the draw state. Need to be called after the pass own state. */
    void select_bind(PassMain &pass)
    {
      pass.use_custom_ids = true;
      pass.bind_ubo(SELECT_DATA, &info_buf);
      /* IMPORTANT: This binds a dummy buffer `in_select_buf` but it is not supposed to be used. */
      pass.bind_ssbo(SELECT_ID_IN, &dummy_select_buf);
      pass.bind_ssbo(SELECT_ID_OUT, &select_output_buf);
    }

    void end_sync()
    {
      info_buf.mode = SelectType::SELECT_ALL;
      /* TODO: Should be select rect center. */
      info_buf.cursor = int2(512, 512);
      info_buf.push_update();

      select_output_buf.resize(ceil_to_multiple_u(select_id_map.size(), 4));
      select_output_buf.push_update();
      select_output_buf.clear_to_zero();
    }

    void read_result()
    {
      /* TODO right barrier. */
      GPU_finish();
      select_output_buf.read();

#ifdef DEBUG
      for (auto i : IndexRange(select_output_buf.size())) {
        uint32_t word = select_output_buf[i];
        for (auto bit : IndexRange(32)) {
          if ((word & 1) != 0) {
            uint index = i * 32 + bit;
            std::cout << index << map_names[index] << std::endl;
          }
          word >>= 1;
        }
      }
#endif
    }
  };
};

}  // namespace blender::draw::select