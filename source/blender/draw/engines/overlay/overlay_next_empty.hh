/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"
namespace blender::draw::overlay {

class EmptyPassesBase : public OverlayPasses {

  ExtraInstanceBuf plain_axes = extra_buf("plain_axes", shapes.plain_axes);
  ExtraInstanceBuf single_arrow = extra_buf("single_arrow", shapes.single_arrow);
  ExtraInstanceBuf arrows = extra_buf("arrows", shapes.arrows);
  ExtraInstanceBuf image = extra_buf("image", shapes.quad_wire);
  ExtraInstanceBuf circle = extra_buf("circle", shapes.circle);
  ExtraInstanceBuf cube = extra_buf("cube", shapes.empty_cube);
  ExtraInstanceBuf sphere = extra_buf("sphere", shapes.empty_sphere);
  ExtraInstanceBuf cone = extra_buf("cone", shapes.empty_cone);

 public:
  EmptyPassesBase(const char *name,
                  SelectionType selection_type,
                  const ShapeCache &shapes,
                  const GlobalsUboStorage &theme_colors,
                  bool in_front)
      : OverlayPasses(name, selection_type, shapes, theme_colors, in_front){};

 protected:
  ExtraInstanceBuf &empty_buf(int empty_drawtype)
  {
    switch (empty_drawtype) {
      case OB_PLAINAXES:
        return plain_axes;
      case OB_SINGLE_ARROW:
        return single_arrow;
      case OB_CUBE:
        return cube;
      case OB_CIRCLE:
        return circle;
      case OB_EMPTY_SPHERE:
        return sphere;
      case OB_EMPTY_CONE:
        return cone;
      case OB_ARROWS:
        return arrows;
      case OB_EMPTY_IMAGE:
        return image;
      default:
        BLI_assert_unreachable();
        return plain_axes;
    }
  }
};

class EmptyPasses : public EmptyPassesBase {

 public:
  EmptyPasses(SelectionType selection_type,
              const ShapeCache &shapes,
              const GlobalsUboStorage &theme_colors,
              bool in_front)
      : EmptyPassesBase("Empties", selection_type, shapes, theme_colors, in_front){};

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources &res,
                           const State &state) final override
  {
    Object *ob = ob_ref.object;
    BLI_assert(ob->type == OB_EMPTY);

    ExtraInstanceData data(ob, res.object_wire_color(ob_ref, state));
    empty_buf(ob->empty_drawtype).append(data, select_id);
    if (ob->empty_drawtype == OB_EMPTY_IMAGE) {
      /** TODO:
       * See OVERLAY_image_empty_cache_populate() for the image. */
    }
  }
};

using Empty = OverlayType<EmptyPasses>;

}  // namespace blender::draw::overlay
