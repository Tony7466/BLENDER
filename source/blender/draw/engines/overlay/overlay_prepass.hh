/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 *
 * A depth pass that write surface depth when it is needed.
 * It is also used for selecting non overlay-only objects.
 */

#pragma once

#include "draw_cache.h"

#include "overlay_private.hh"

namespace blender::draw::overlay {

template<typename SelectEngineT> class Prepass {
  using SelectID = typename SelectEngineT::ID;
  using ResourcesT = Resources<SelectEngineT>;

 private:
  PassMain prepass_ps_ = {"prepass"};
  PassMain prepass_in_front_ps_ = {"prepass_in_front"};

 public:
  void begin_sync(const State &state)
  {
    auto init_pass = [&](PassMain &pass) {
      pass.init();
      pass.state_set(DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL | state.clipping_state);
      pass.shader_set(OVERLAY_shader_depth_only());
    };
    init_pass(prepass_ps_);
    init_pass(prepass_in_front_ps_);
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, ResourcesT & /*res*/)
  {
    PassMain &pass = (ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0 ? prepass_in_front_ps_ :
                                                                    prepass_ps_;

    /* TODO(fclem) This function should contain what `basic_cache_populate` contained. */

    GPUBatch *geom = DRW_cache_object_surface_get(ob_ref.object);
    if (geom) {
      ResourceHandle res_handle = manager.resource_handle(ob_ref);
      pass.draw(geom, res_handle);

      /* TODO */
      // const SelectID radius_id = res.select_id(ob_ref);
      // pass.draw(geom, res_handle, radius_id.value);
    }
  }

  void draw(ResourcesT &res, Manager &manager, View &view)
  {
    /* Should be fine to use the line buffer since the prepass only writes to the depth buffer. */
    GPU_framebuffer_bind(res.overlay_line_fb);
    manager.submit(prepass_ps_, view);
  }

  void draw_in_front(ResourcesT &res, Manager &manager, View &view)
  {
    /* Should be fine to use the line buffer since the prepass only writes to the depth buffer. */
    GPU_framebuffer_bind(res.overlay_line_in_front_fb);
    manager.submit(prepass_in_front_ps_, view);
  }
};

}  // namespace blender::draw::overlay
