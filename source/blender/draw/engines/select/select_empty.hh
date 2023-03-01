/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

/** \file
 * \ingroup draw_engine
 *
 * Dummy implementation of the select engine types to avoid any overhead.
 */

#pragma once

#include "draw_manager.hh"

namespace blender::draw::select {

struct EngineEmpty {
  /* Add type safety to selection ID. Only the select engine should provide them. */
  struct ID {};

  static constexpr const char *shader_define = "NO_SELECT";

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
  };
};

}  // namespace blender::draw::select