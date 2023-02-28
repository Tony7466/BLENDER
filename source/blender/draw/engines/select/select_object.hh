/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup draw_engine
 *
 * This is an implementation of the Select engine specialized for selecting object.
 * Should plug seamlessly inside the overlay engine logic.
 */

#pragma once

#include "DRW_gpu_wrapper.hh"
#include "draw_manager.hh"

namespace blender::draw::select {

struct EngineObject {
  /* Add type safety to selection ID. Only the select engine should provide them. */
  struct ID {
    uint32_t value;
  };

  /**
   * Add a dedicated selection id buffer to a pass.
   * Use this when not using a #PassMain which can pass the select ID via CustomID.
   */
  struct SelectBuf {
    StorageVectorBuffer<uint32_t> select_buf = {"select_buf"};

    void select_clear()
    {
      select_buf.clear();
    }

    void select_append(ID select_id)
    {
      select_buf.append(select_id.value);
    }

    void select_bind(PassSimple &pass)
    {
      select_buf.push_update();
      pass.bind_ssbo("select_buf", &select_buf);
    }
  };

  /**
   * Generate selection IDs from objects and keep record of the mapping between them.
   */
  struct SelectMap {
    uint next_id = 0;
    Map<uint, ObjectRef> map;

    /* TODO(fclem): The sub_object_id id should eventually become some enum or take a sub-object
     * reference directly. */
    [[nodiscard]] const ID select_id(const ObjectRef &, uint sub_object_id = 0)
    {
      /* TODO Insert Ref into the map. */
      return {sub_object_id | next_id++};
    }
  };
};

}  // namespace blender::draw::select