/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

#include "BKE_mball.h"
#include "DNA_lightprobe_types.h"
#include "DNA_rigidbody_types.h"

namespace blender::draw::overlay {

class Extras {
  struct InstanceBuf : public ShapeInstanceBuf<ExtraInstanceData> {
    GPUBatch *shape;

    InstanceBuf(const char *name,
                GPUBatch *shape,
                const SelectionType selection_type,
                Vector<InstanceBuf *> &vector)
        : ShapeInstanceBuf<ExtraInstanceData>(selection_type, name), shape(shape)
    {
      vector.append(this);
    };
  };

  struct InstanceBuffers {
    const SelectionType selection_type;
    const ShapeCache &shapes;

    Vector<InstanceBuf *> vector;

    InstanceBuffers(SelectionType selection_type, const ShapeCache &shapes)
        : selection_type(selection_type), shapes(shapes){};

    InstanceBuf plain_axes = make_buf("plain_axes", shapes.plain_axes);
    InstanceBuf single_arrow = make_buf("single_arrow", shapes.single_arrow);
    InstanceBuf arrows = make_buf("arrows", shapes.arrows);
    InstanceBuf image = make_buf("image", shapes.quad_wire);
    InstanceBuf circle = make_buf("circle", shapes.circle);
    InstanceBuf cube = make_buf("cube", shapes.empty_cube);
    InstanceBuf sphere = make_buf("sphere", shapes.empty_sphere);
    InstanceBuf cone = make_buf("cone", shapes.empty_cone);
    InstanceBuf cylinder = make_buf("cylinder", shapes.empty_cylinder);
    InstanceBuf capsule_body = make_buf("capsule_body", shapes.empty_capsule_body);
    InstanceBuf capsule_cap = make_buf("capsule_cap", shapes.empty_capsule_cap);
    InstanceBuf quad = make_buf("quad", shapes.quad);
    InstanceBuf speaker = make_buf("speaker", shapes.speaker);
    InstanceBuf probe_cube = make_buf("probe_cube", shapes.probe_cube);
    InstanceBuf probe_grid = make_buf("probe_grid", shapes.probe_grid);
    InstanceBuf probe_planar = make_buf("probe_planar", shapes.probe_planar);

   private:
    InstanceBuf make_buf(const char *name, const ShapeCache::BatchPtr &shape_ptr)
    {
      return {name, shape_ptr.get(), selection_type, vector};
    };
  };

 private:
  const SelectionType selection_type_;

  InstanceBuffers buffers_[2];

  PassSimple empty_ps_ = {"Extras"};
  PassSimple empty_in_front_ps_ = {"Extras_In_front"};

 public:
  Extras(const SelectionType selection_type, const ShapeCache &shapes)
      : selection_type_(selection_type),
        buffers_{{selection_type, shapes}, {selection_type, shapes}} {};

  void begin_sync()
  {
    for (InstanceBuffers &bufs : buffers_) {
      for (InstanceBuf *buf : bufs.vector) {
        buf->clear();
      }
    }
  }

  void object_sync(const ObjectRef &ob_ref, Resources &res, const State &state)
  {
    InstanceBuffers &bufs = buffers_[int((ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0)];

    float4 color = res.object_wire_color(ob_ref, state);
    float size = ob_ref.object->type == OB_EMPTY ? ob_ref.object->empty_drawsize : 1.0f;
    ExtraInstanceData data(float4x4(ob_ref.object->object_to_world), color, size);

    const select::ID select_id = res.select_id(ob_ref);

    switch (ob_ref.object->type) {
      case OB_EMPTY:
        empty_sync(bufs, ob_ref, data, select_id);
        break;
      case OB_LIGHTPROBE:
        probe_sync(bufs, ob_ref, data, select_id);
        break;
      case OB_SPEAKER:
        bufs.speaker.append(data, select_id);
        break;
    }

    const Scene *scene = state.scene;
    Object *ob = ob_ref.object;
    ModifierData *md = nullptr;

    const bool is_select_mode = selection_type_ != SelectionType::DISABLED;
    const bool is_paint_mode = (state.object_mode &
                                (OB_MODE_ALL_PAINT | OB_MODE_ALL_PAINT_GPENCIL |
                                 OB_MODE_SCULPT_CURVES)) != 0;
    const bool from_dupli = (ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI)) != 0;
    const bool has_bounds = !ELEM(
        ob->type, OB_LAMP, OB_CAMERA, OB_EMPTY, OB_SPEAKER, OB_LIGHTPROBE);
    const bool has_texspace = has_bounds &&
                              !ELEM(
                                  ob->type, OB_EMPTY, OB_LATTICE, OB_ARMATURE, OB_GPENCIL_LEGACY);
    const bool draw_relations = ((state.v3d_flag & V3D_HIDE_HELPLINES) == 0) && !is_select_mode;
    const bool draw_obcenters = !is_paint_mode &&
                                (state.overlay.flag & V3D_OVERLAY_HIDE_OBJECT_ORIGINS) == 0;
    const bool draw_texspace = (ob->dtx & OB_TEXSPACE) && has_texspace;
    const bool draw_obname = (ob->dtx & OB_DRAWNAME) && DRW_state_show_text();
    const bool draw_bounds = has_bounds && ((ob->dt == OB_BOUNDBOX) ||
                                            ((ob->dtx & OB_DRAWBOUNDOX) && !from_dupli));
    const bool draw_xform = state.object_mode == OB_MODE_OBJECT &&
                            (scene->toolsettings->transform_flag & SCE_XFORM_DATA_ORIGIN) &&
                            (ob->base_flag & BASE_SELECTED) && !is_select_mode;
#if 0
    /* TODO */
    /* Don't show fluid domain overlay extras outside of cache range. */
    const bool draw_volume =
        !from_dupli && (md = BKE_modifiers_findby_type(ob, eModifierType_Fluid)) &&
        BKE_modifier_is_enabled(scene, md, eModifierMode_Realtime) &&
        (((FluidModifierData *)md)->domain != nullptr) &&
        (scene->r.cfra >= (((FluidModifierData *)md)->domain->cache_frame_start)) &&
        (scene->r.cfra <= (((FluidModifierData *)md)->domain->cache_frame_end));
#endif

    if (draw_bounds) {
      bounds_sync(bufs, ob_ref, data, select_id, ob->boundtype, false);
    }

    /* Helpers for when we're transforming origins. */
    if (draw_xform) {
      /* TODO
      const float color_xform[4] = {0.15f, 0.15f, 0.15f, 0.7f};
      DRW_buffer_add_entry(cb->origin_xform, color_xform, ob->object_to_world);
      */
    }
    if (ob->pd && ob->pd->forcefield) {
      /* TODO
      OVERLAY_forcefield(cb, ob, view_layer);
      */
    }

    /* don't show object extras in set's */
    if (!from_dupli) {
      if (draw_obcenters) {
        /* TODO
        OVERLAY_object_center(cb, ob, pd, scene, view_layer);
        */
      }
      if (draw_relations) {
        /* TODO
        OVERLAY_relationship_lines(cb, draw_ctx->depsgraph, draw_ctx->scene, ob);
        */
      }
      if (draw_obname) {
        /* TODO
        OVERLAY_object_name(ob, theme_id);
        */
      }
      if (draw_texspace) {
        /* TODO
        OVERLAY_texture_space(cb, ob, color);
        */
      }
      if (ob->rigidbody_object != nullptr) {
        collision_sync(bufs, ob_ref, data, select_id);
      }
      if (ob->dtx & OB_AXIS) {
        /* TODO
        DRW_buffer_add_entry(cb->empty_axes, color, ob->object_to_world);
        */
      }
      /* TODO
      if (draw_volume) {
        OVERLAY_volume_extra(cb, vedata, ob, md, scene, color);
      }
      */
    }
  }

  void end_sync(Resources &res, const State &state)
  {
    auto init_pass = [&](PassSimple &pass, InstanceBuffers &bufs) {
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     state.clipping_state);
      pass.shader_set(res.shaders.extra_shape.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      res.select_bind(pass);

      for (InstanceBuf *buf : bufs.vector) {
        buf->end_sync(pass, buf->shape);
      }
    };
    init_pass(empty_ps_, buffers_[0]);
    init_pass(empty_in_front_ps_, buffers_[1]);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_fb);
    manager.submit(empty_ps_, view);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    GPU_framebuffer_bind(res.overlay_line_in_front_fb);
    manager.submit(empty_in_front_ps_, view);
  }

 private:
  void empty_sync(InstanceBuffers &bufs,
                  const ObjectRef &ob_ref,
                  const ExtraInstanceData data,
                  const select::ID select_id)
  {
    switch (ob_ref.object->empty_drawtype) {
      case OB_PLAINAXES:
        bufs.plain_axes.append(data, select_id);
        break;
      case OB_SINGLE_ARROW:
        bufs.single_arrow.append(data, select_id);
        break;
      case OB_CUBE:
        bufs.cube.append(data, select_id);
        break;
      case OB_CIRCLE:
        bufs.circle.append(data, select_id);
        break;
      case OB_EMPTY_SPHERE:
        bufs.sphere.append(data, select_id);
        break;
      case OB_EMPTY_CONE:
        bufs.cone.append(data, select_id);
        break;
      case OB_ARROWS:
        bufs.arrows.append(data, select_id);
        break;
      case OB_EMPTY_IMAGE:
        /* This only show the frame. See OVERLAY_image_empty_cache_populate() for the image. */
        bufs.image.append(data, select_id);
        break;
    }
  }

  void bounds_sync(InstanceBuffers &bufs,
                   const ObjectRef &ob_ref,
                   const ExtraInstanceData data,
                   const select::ID select_id,
                   char boundtype,
                   bool around_origin)
  {
    Object *ob = ob_ref.object;

    if (ObjectType(ob->type) == OB_MBALL && !BKE_mball_is_basis(ob)) {
      return;
    }

    const BoundBox *bb = BKE_object_boundbox_get(ob);
    BoundBox bb_local;
    if (bb == nullptr) {
      const float3 min = float3(-1.0f);
      const float3 max = float3(1.0f);
      BKE_boundbox_init_from_minmax(&bb_local, min, max);
      bb = &bb_local;
    }

    float3 center = float3(0);
    if (!around_origin) {
      BKE_boundbox_calc_center_aabb(bb, center);
    }

    float3 size;
    BKE_boundbox_calc_size_aabb(bb, size);

    ExtraInstanceData _data = data;

    if (boundtype == OB_BOUND_BOX) {
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.cube.append(_data, select_id);
    }
    else if (boundtype == OB_BOUND_SPHERE) {
      size = float3(std::max({size.x, size.y, size.z}));
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.sphere.append(_data, select_id);
    }
    else if (boundtype == OB_BOUND_CYLINDER) {
      size.x = size.y = std::max(size.x, size.y);
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.cylinder.append(_data, select_id);
    }
    else if (boundtype == OB_BOUND_CONE) {
      size.x = size.y = std::max(size.x, size.y);
      float4x4 mat = math::from_scale<float4x4>(size);
      mat.location() = center;
      /* Cone batch has base at 0 and is pointing towards +Y. */
      std::swap(mat[1], mat[2]);
      mat.location().z -= size.z;
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.cone.append(_data, select_id);
    }
    else if (boundtype == OB_BOUND_CAPSULE) {
      size.x = size.y = std::max(size.x, size.y);
      float4x4 mat = math::from_scale<float4x4>(float3(size.x));
      mat.location() = center;

      mat.location().z = center.z + std::max(0.0f, size.z - size.x);
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.capsule_cap.append(_data, select_id);

      mat.location().z = center.z - std::max(0.0f, size.z - size.x);
      mat.z_axis() *= -1.0f;
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.capsule_cap.append(_data, select_id);

      mat.z_axis().z = std::max(0.0f, (size.z - size.x) * 2.0f);
      _data.object_to_world_ = data.object_to_world_ * mat;
      bufs.capsule_body.append(_data, select_id);
    }
  }

  void collision_sync(InstanceBuffers &bufs,
                      const ObjectRef &ob_ref,
                      const ExtraInstanceData data,
                      const select::ID select_id)
  {
    Object *ob = ob_ref.object;

    switch (ob->rigidbody_object->shape) {
      case RB_SHAPE_BOX:
        bounds_sync(bufs, ob_ref, data, select_id, OB_BOUND_BOX, true);
        break;
      case RB_SHAPE_SPHERE:
        bounds_sync(bufs, ob_ref, data, select_id, OB_BOUND_SPHERE, true);
        break;
      case RB_SHAPE_CONE:
        bounds_sync(bufs, ob_ref, data, select_id, OB_BOUND_CONE, true);
        break;
      case RB_SHAPE_CYLINDER:
        bounds_sync(bufs, ob_ref, data, select_id, OB_BOUND_CYLINDER, true);
        break;
      case RB_SHAPE_CAPSULE:
        bounds_sync(bufs, ob_ref, data, select_id, OB_BOUND_CAPSULE, true);
        break;
    }
  }

  void probe_sync(InstanceBuffers &bufs,
                  const ObjectRef &ob_ref,
                  const ExtraInstanceData data,
                  const select::ID select_id)
  {
    const LightProbe *probe = (LightProbe *)ob_ref.object->data;
    const bool show_clipping = (probe->flag & LIGHTPROBE_FLAG_SHOW_CLIP_DIST) != 0;
    const bool show_parallax = (probe->flag & LIGHTPROBE_FLAG_SHOW_PARALLAX) != 0;
    const bool show_influence = (probe->flag & LIGHTPROBE_FLAG_SHOW_INFLUENCE) != 0;
    const bool show_data = (ob_ref.object->base_flag & BASE_SELECTED) ||
                           selection_type_ != SelectionType::DISABLED;

    if (probe->type == LIGHTPROBE_TYPE_CUBE) {
      ExtraInstanceData _data = data;
      _data.object_to_world_[2][3] = show_clipping ? probe->clipsta : -1.0;
      _data.object_to_world_[3][3] = show_clipping ? probe->clipend : -1.0;

      bufs.probe_cube.append(_data, select_id);

#if 0
      /* TODO: This requires a different shader.  */
      _data.object_to_world_ = math::translate(float4x4::identity(),
                                               _data.object_to_world_.location());
      bufs.groundline.append(_data, select_id);
#endif

      if (show_influence) {
        float influence_start = probe->distinf * (1.0f - probe->falloff);
        float influence_end = probe->distinf;

        InstanceBuf &buf = (probe->attenuation_type == LIGHTPROBE_SHAPE_BOX) ? bufs.cube :
                                                                               bufs.sphere;

        buf.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_start),
                   select_id);
        buf.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_end),
                   select_id);
      }

      if (show_parallax) {
        float radius = (probe->flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) ? probe->distpar :
                                                                         probe->distinf;
        InstanceBuf &buf = (probe->parallax_type == LIGHTPROBE_SHAPE_BOX) ? bufs.cube :
                                                                            bufs.sphere;
        buf.append(ExtraInstanceData(data.object_to_world_, data.color_, radius), select_id);
      }
    }
    else if (probe->type == LIGHTPROBE_TYPE_GRID) {
      ExtraInstanceData _data = data;
      _data.object_to_world_[2][3] = show_clipping ? probe->clipsta : -1.0;
      _data.object_to_world_[3][3] = show_clipping ? probe->clipend : -1.0;
      bufs.probe_grid.append(_data, select_id);

      if (show_influence) {
        float influence_start = 1.0f + probe->distinf * (1.0f - probe->falloff);
        float influence_end = 1.0f + probe->distinf;

        bufs.cube.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_start),
                         select_id);
        bufs.cube.append(ExtraInstanceData(data.object_to_world_, data.color_, influence_end),
                         select_id);
      }

      /* TODO(Miguel Pozo) */
#if 0
    /* Data dots */
    if (show_data) {
      _data.object_to_world_[0][3] = probe->grid_resolution_x;
      _data.object_to_world_[1][3] = probe->grid_resolution_y;
      _data.object_to_world_[2][3] = probe->grid_resolution_z;
      /* Put theme id in matrix. */
      if (theme_id == TH_ACTIVE) {
        _data.object_to_world_[3][3] = 1.0;
      }
      else /* TH_SELECT */ {
        _data.object_to_world_[3][3] = 2.0;
      }

      uint cell_count = probe->grid_resolution_x * probe->grid_resolution_y *
                        probe->grid_resolution_z;
      DRWShadingGroup *grp = DRW_shgroup_create_sub(vedata->stl->pd->extra_grid_grp);
      DRW_shgroup_uniform_mat4_copy(grp, "gridModelMatrix", instdata.mat);
      DRW_shgroup_call_procedural_points(grp, nullptr, cell_count);
    }
#endif
    }
    else if (probe->type == LIGHTPROBE_TYPE_PLANAR) {
      bufs.probe_planar.append(data, select_id);

      if (selection_type_ != SelectionType::DISABLED && (probe->flag & LIGHTPROBE_FLAG_SHOW_DATA))
      {
        bufs.quad.append(data, select_id);
      }

      ExtraInstanceData _data = data;
      float3 &z = _data.object_to_world_.z_axis();

      if (show_influence) {
        z = math::normalize(z) * probe->distinf;
        bufs.cube.append(_data, select_id);
        z *= 1.0f - probe->falloff;
        bufs.cube.append(_data, select_id);
      }

      z = float3(0);
      bufs.cube.append(_data, select_id);

      _data = data;
      _data.object_to_world_ = math::normalize(_data.object_to_world_);
      _data.object_to_world_[3] = data.object_to_world_[3];
      bufs.single_arrow.append(
          ExtraInstanceData(_data.object_to_world_, data.color_, ob_ref.object->empty_drawsize),
          select_id);
    }
  }
};

}  // namespace blender::draw::overlay
