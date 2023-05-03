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
#include "overlay_prepass.hh"
#include "overlay_shape.hh"

#include "../select/select_instance.hh"

namespace blender::draw::overlay {

using eSelectionType = select::eSelectionType;

/* Selection engine reuse most of the Overlay engine by creating selection IDs for each
 * selectable component and using a special shaders for drawing.*/
class Instance {
  const eSelectionType selection_type_;

 public:
  /* WORKAROUND: Legacy. Move to grid pass. */
  GPUUniformBuf *grid_ubo = nullptr;

  ShapeCache shapes;

  /** Global types. */
  Resources resources = {selection_type_,
                         overlay::ShaderModule::module_get(selection_type, false /*TODO*/)};
  State state;

  /** Overlay types. */
  Background background = {selection_type_};
  Prepass prepass = {selection_type_};
  Metaballs metaballs = {selection_type_};
  Empties empties = {selection_type_};
  Grid grid = {selection_type_};

  Instance(const eSelectionType selection_type) : selection_type_(selection_type){};

  ~Instance()
  {
    DRW_UBO_FREE_SAFE(grid_ubo);
  }

  void init();
  void begin_sync();
  void object_sync(ObjectRef &ob_ref, Manager &manager);
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

}  // namespace blender::draw::overlay
