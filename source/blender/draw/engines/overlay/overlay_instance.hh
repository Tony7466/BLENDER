/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "draw_manager.hh"

#include "overlay_background.hh"
#include "overlay_empty.hh"
#include "overlay_grid.hh"
#include "overlay_metaball.hh"
#include "overlay_shape.hh"

#include "../select/select_empty.hh"
#include "../select/select_object.hh"

namespace blender::draw::overlay {

class ShaderCache {
  Map<StringRefNull, std::array<GPUShader *, 2>> cache;

  int clipping_enabled = 0;
};

template<
    /* Selection engine reuse most of the Overlay engine by creating selection IDs for each
     * selectable component and using a special shaders for drawing.
     * Making the select engine templated makes it easier to phase out any overhead of the
     * selection for the regular non-selection case.*/
    typename SelectEngineT = select::EngineEmpty>
class Instance {
 public:
  ShaderCache shaders;
  ShapeCache shapes;

  /* WORKAROUND: Legacy. Move to grid pass. */
  GPUUniformBuf *grid_ubo = nullptr;

  /** Global types. */
  Resources<SelectEngineT> resources;
  State state;

  /** Overlay types. */
  Background<SelectEngineT> background;
  Metaballs<SelectEngineT> metaballs;
  Empties<SelectEngineT> empties;
  Grid<SelectEngineT> grid;

  ~Instance()
  {
    DRW_UBO_FREE_SAFE(grid_ubo);
  }

  void init();
  void begin_sync();
  void object_sync(ObjectRef &ob_ref);
  void end_sync();
  void draw(Manager &manager);

 private:
  bool object_is_edit_mode(const Object *ob)
  {
    if (DRW_object_is_in_edit_mode(ob)) {
      /* Also check for context mode as the object mode is not 100% reliable. (see T72490) */
      switch (ob->type) {
        case OB_MESH:
          return state.ctx_mode == CTX_MODE_EDIT_MESH;
        case OB_ARMATURE:
          return state.ctx_mode == CTX_MODE_EDIT_ARMATURE;
        case OB_CURVES_LEGACY:
          return state.ctx_mode == CTX_MODE_EDIT_CURVE;
        case OB_SURF:
          return state.ctx_mode == CTX_MODE_EDIT_SURFACE;
        case OB_LATTICE:
          return state.ctx_mode == CTX_MODE_EDIT_LATTICE;
        case OB_MBALL:
          return state.ctx_mode == CTX_MODE_EDIT_METABALL;
        case OB_FONT:
          return state.ctx_mode == CTX_MODE_EDIT_TEXT;
        case OB_CURVES:
          return state.ctx_mode == CTX_MODE_EDIT_CURVES;
        case OB_POINTCLOUD:
        case OB_VOLUME:
          /* No edit mode yet. */
          return false;
      }
    }
    return false;
  }
};

/* Instantiation. */
extern template void Instance<>::init();
extern template void Instance<>::begin_sync();
extern template void Instance<>::object_sync(ObjectRef &ob_ref);
extern template void Instance<>::end_sync();
extern template void Instance<>::draw(Manager &manager);

}  // namespace blender::draw::overlay
