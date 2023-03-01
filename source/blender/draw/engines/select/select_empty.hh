/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup draw_engine
 *
 * Dummy implementation of the select engine types to avoid any overhead.
 */

#pragma once

#include "draw_manager.hh"

#include "gpu_shader_create_info.hh"

namespace blender::draw::select {

struct EngineEmpty {
  /* Add type safety to selection ID. Only the select engine should provide them. */
  struct ID {};

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

}  // namespace blender::draw::select