/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Particles {
 private:
  PassSimple particle_ps_ = {"particle_ps_"};
  PassSimple edit_particle_ps_ = {"edit_particle_ps_"};

 public:
  void begin_sync(Resources &res, const State &state)
  {
    auto &pass = particle_ps_;
    pass.init();
    pass.state_set(DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
    pass.shader_set(res.shaders.depth_mesh.get());
    res.select_bind(pass);
  }

  void object_sync(const ObjectRef & /*ob_ref*/, Resources & /*res*/, const State & /*state*/) {}

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(framebuffer);
    manager.submit(particle_ps_, view);
  }
};
}  // namespace blender::draw::overlay
