/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

#include "overlay_next_background.hh"
#include "overlay_next_grid.hh"
#include "overlay_next_metaball.hh"
#include "overlay_next_object_center.hh"
#include "overlay_next_object_relation.hh"
#include "overlay_next_prepass.hh"

#include "overlay_next_bounds.hh"
#include "overlay_next_camera.hh"
#include "overlay_next_collision.hh"
#include "overlay_next_empty.hh"
#include "overlay_next_force_field.hh"
#include "overlay_next_light.hh"
#include "overlay_next_light_probe.hh"
#include "overlay_next_object_center.hh"
#include "overlay_next_object_relation.hh"
#include "overlay_next_speaker.hh"

namespace blender::draw::overlay {

/**
 * Selection engine reuse most of the Overlay engine by creating selection IDs for each
 * selectable component and using a special shaders for drawing.
 */
class Instance {
  const SelectionType selection_type_;

 public:
  /* WORKAROUND: Legacy. Move to grid pass. */
  GPUUniformBuf *grid_ubo = nullptr;

  ShapeCache shapes;

  /** Global types. */
  Resources resources = {selection_type_,
                         overlay::ShaderModule::module_get(selection_type_, false /*TODO*/)};
  State state;

  /** Overlay types. */
  Background background;
  Prepass prepass;
  Metaballs metaballs = {selection_type_};
  Grid grid;
  Bound bounds = {selection_type_, shapes, G_draw.block};
  Camera cameras = {selection_type_, shapes, G_draw.block};
  Collision collisions = {selection_type_, shapes, G_draw.block};
  Empty empties = {selection_type_, shapes, G_draw.block};
  ForceField force_fields = {selection_type_, shapes, G_draw.block};
  Light lights = {selection_type_, shapes, G_draw.block};
  LightProbe light_probes = {selection_type_, shapes, G_draw.block};
  ObjectCenter object_centers = {selection_type_, shapes, G_draw.block};
  ObjectRelation object_relations = {selection_type_, shapes, G_draw.block};
  Speaker speakers = {selection_type_, shapes, G_draw.block};

  Instance(const SelectionType selection_type) : selection_type_(selection_type){};

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
  bool object_is_edit_mode(const Object *ob);
};

}  // namespace blender::draw::overlay
