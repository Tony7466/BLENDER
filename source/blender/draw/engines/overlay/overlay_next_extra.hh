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
                Vector<InstanceBuf *> &pass_vector)
        : ShapeInstanceBuf<ExtraInstanceData>(selection_type, name), shape(shape)
    {
      pass_vector.append(this);
    };
  };

  class InstanceBuffers {
    enum PassType {
      DEFAULT = 0,
      DEFAULT_ALWAYS,
      BLEND_CULL_FRONT,
      BLEND_CULL_BACK,
      GROUNDLINE,
      WIRE,
      WIRE_OB,
      POINT,
      LOOSE_POINT,
      CENTER,
      MAX
    };

   private:
    const SelectionType selection_type;
    const ShapeCache &shapes;

    Vector<InstanceBuf *> pass_buffers[PassType::MAX] = {{}};

   public:
    InstanceBuffers(SelectionType selection_type, const ShapeCache &shapes)
        : selection_type(selection_type), shapes(shapes){};

    InstanceBuf plain_axes = make_buf("plain_axes", shapes.plain_axes, DEFAULT);
    InstanceBuf single_arrow = make_buf("single_arrow", shapes.single_arrow, DEFAULT);
    InstanceBuf arrows = make_buf("arrows", shapes.arrows, DEFAULT);
    InstanceBuf image = make_buf("image", shapes.quad_wire, DEFAULT);
    InstanceBuf circle = make_buf("circle", shapes.circle, DEFAULT);
    InstanceBuf cube = make_buf("cube", shapes.empty_cube, DEFAULT);
    InstanceBuf sphere = make_buf("sphere", shapes.empty_sphere, DEFAULT);
    InstanceBuf cone = make_buf("cone", shapes.empty_cone, DEFAULT);
    InstanceBuf cylinder = make_buf("cylinder", shapes.empty_cylinder, DEFAULT);
    InstanceBuf capsule_body = make_buf("capsule_body", shapes.empty_capsule_body, DEFAULT);
    InstanceBuf capsule_cap = make_buf("capsule_cap", shapes.empty_capsule_cap, DEFAULT);

    InstanceBuf quad = make_buf("quad", shapes.quad, DEFAULT);

    InstanceBuf speaker = make_buf("speaker", shapes.speaker, DEFAULT);

    InstanceBuf groundline = make_buf("groundline", shapes.groundline, GROUNDLINE);
    InstanceBuf light_icon_inner = make_buf("light_icon_inner", shapes.light_icon_inner, DEFAULT);
    InstanceBuf light_icon_outer = make_buf("light_icon_outer", shapes.light_icon_outer, DEFAULT);
    InstanceBuf light_icon_sun_rays = make_buf(
        "light_icon_sun_rays", shapes.light_icon_sun_rays, DEFAULT);
    InstanceBuf light_point = make_buf("light_point", shapes.light_point, DEFAULT);
    InstanceBuf light_sun = make_buf("light_sun", shapes.light_sun, DEFAULT);
    InstanceBuf light_spot = make_buf("light_spot", shapes.light_spot, DEFAULT);
    InstanceBuf light_spot_cone_back = make_buf(
        "light_spot_cone_back", shapes.light_spot_cone, BLEND_CULL_BACK);
    InstanceBuf light_spot_cone_front = make_buf(
        "light_spot_cone_front", shapes.light_spot_cone, BLEND_CULL_FRONT);
    InstanceBuf light_area_disk = make_buf("light_area_disk", shapes.light_area_disk, DEFAULT);
    InstanceBuf light_area_square = make_buf(
        "light_area_square", shapes.light_area_square, DEFAULT);

    InstanceBuf probe_cube = make_buf("probe_cube", shapes.probe_cube, DEFAULT);
    InstanceBuf probe_grid = make_buf("probe_grid", shapes.probe_grid, DEFAULT);
    InstanceBuf probe_planar = make_buf("probe_planar", shapes.probe_planar, DEFAULT);

    InstanceBuf camera_volume = make_buf("camera_volume", shapes.camera_volume, BLEND_CULL_BACK);
    InstanceBuf camera_volume_wire = make_buf(
        "camera_volume_wire", shapes.camera_volume_wire, BLEND_CULL_BACK);
    InstanceBuf camera_frame = make_buf("camera_frame", shapes.camera_frame, DEFAULT);
    InstanceBuf camera_distances = make_buf("camera_distances", shapes.camera_distances, DEFAULT);
    InstanceBuf camera_tria_wire = make_buf("camera_tria_wire", shapes.camera_tria_wire, DEFAULT);
    InstanceBuf camera_tria = make_buf("camera_tria", shapes.camera_tria, DEFAULT);

    InstanceBuf field_wind = make_buf("field_wind", shapes.field_wind, DEFAULT);
    InstanceBuf field_force = make_buf("field_force", shapes.field_force, DEFAULT);
    InstanceBuf field_vortex = make_buf("field_vortex", shapes.field_vortex, DEFAULT);
    InstanceBuf field_curve = make_buf("field_curve", shapes.field_curve, DEFAULT);
    InstanceBuf field_tube_limit = make_buf("field_tube_limit", shapes.field_tube_limit, DEFAULT);
    InstanceBuf field_cone_limit = make_buf("field_cone_limit", shapes.field_cone_limit, DEFAULT);
    InstanceBuf field_sphere_limit = make_buf(
        "field_sphere_limit", shapes.field_sphere_limit, DEFAULT);

    InstanceBuf origin_xform = make_buf("origin_xform", shapes.plain_axes, DEFAULT_ALWAYS);

    /* TODO
    InstanceBuf dashed_lines = make_buf("dashed_lines", nullptr, WIRE);
    InstanceBuf lines = make_buf("lines", nullptr, WIRE);

    InstanceBuf wires = make_buf("wires", nullptr, WIRE_OB);

    InstanceBuf loose_points = make_buf("loose_points", nullptr, LOOSE_POINTS);

    InstanceBuf points = make_buf("points", nullptr, POINTS);

    InstanceBuf center_active = make_buf("center_active", nullptr, CENTER);
    InstanceBuf center_selected = make_buf("center_selected", nullptr, CENTER);
    InstanceBuf center_deselected = make_buf("center_deselected", nullptr, CENTER);
    InstanceBuf center_selected_lib = make_buf("center_selected_lib", nullptr, CENTER);
    InstanceBuf center_deselected_lib = make_buf("center_deselected_lib", nullptr, CENTER);
    */

    void begin_sync()
    {
      for (Vector<InstanceBuf *> &vector : pass_buffers) {
        for (InstanceBuf *buf : vector) {
          buf->clear();
        }
      }
    }

    void end_sync(PassSimple &pass, Resources &res, const State &state)
    {
      pass.init();
      res.select_bind(pass);

      auto sub_pass = [&](const char *name, PassType type, ShaderPtr &shader, DRWState drw_state) {
        if (shader == nullptr) {
          /*TODO*/
          return;
        }
        PassSimple::Sub &ps = pass.sub(name);
        ps.state_set(drw_state);
        ps.shader_set(shader.get());
        /* TODO: Fixed index. */
        ps.bind_ubo("globalsBlock", &res.globals_buf);
        for (InstanceBuf *buf : pass_buffers[type]) {
          buf->end_sync(ps, buf->shape);
        }
      };

      DRWState state_base = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | state.clipping_state;
      DRWState state_default = state_base | DRW_STATE_DEPTH_LESS_EQUAL;
      DRWState state_blend = state_default | DRW_STATE_BLEND_ALPHA;

      sub_pass("Default", DEFAULT, res.shaders.extra_shape, state_default);
      sub_pass("Default Always",
               DEFAULT_ALWAYS,
               res.shaders.extra_shape,
               state_base | DRW_STATE_DEPTH_ALWAYS);
      sub_pass("Blend Cull Back",
               BLEND_CULL_BACK,
               res.shaders.extra_shape,
               state_blend | DRW_STATE_CULL_BACK);
      sub_pass("Blend Cull Front",
               BLEND_CULL_FRONT,
               res.shaders.extra_shape,
               state_blend | DRW_STATE_CULL_FRONT);
      sub_pass("GroundLine", GROUNDLINE, res.shaders.extra_groundline, state_blend);
      sub_pass("Wire", WIRE, res.shaders.extra_wire, state_default);
      sub_pass("Wire Object", WIRE_OB, res.shaders.extra_wire_object, state_default);
      sub_pass("Point", POINT, res.shaders.extra_point, state_default);
      sub_pass("Loose Point", LOOSE_POINT, res.shaders.extra_loose_point, state_default);
      sub_pass("Center", CENTER, res.shaders.extra_point, state_blend);
    }

   private:
    InstanceBuf make_buf(const char *name, const BatchPtr &shape_ptr, PassType pass_type = DEFAULT)
    {
      Vector<InstanceBuf *> &vector = pass_buffers[pass_type];
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
    buffers_[0].begin_sync();
    buffers_[1].begin_sync();
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
      case OB_LAMP:
        light_sync(state, bufs, ob_ref, data, select_id);
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
        bufs.arrows.append(data, select_id);
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
    buffers_[0].end_sync(empty_ps_, res, state);
    buffers_[1].end_sync(empty_in_front_ps_, res, state);
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

  void light_sync(const State &state,
                  InstanceBuffers &bufs,
                  const ObjectRef &ob_ref,
                  const ExtraInstanceData data,
                  const select::ID select_id)
  {
    Object *ob = ob_ref.object;
    Light *la = static_cast<Light *>(ob->data);

    ExtraInstanceData _data = data;

    /* Pack render data into object matrix. */
    float4x4 &matrix = _data.object_to_world_;
    float &area_size_x = matrix[0].w;
    float &area_size_y = matrix[1].w;
    float &spot_cosine = matrix[0].w;
    float &spot_blend = matrix[1].w;
    float &clip_sta = matrix[2].w;
    float &clip_end = matrix[3].w;

    /* FIXME / TODO: clip_end has no meaning nowadays.
     * In EEVEE, Only clip_sta is used shadow-mapping.
     * Clip end is computed automatically based on light power.
     * For now, always use the custom distance as clip_end. */
    clip_end = la->att_dist;
    clip_sta = la->clipsta;

    /* Remove the alpha. */
    float4 theme_color = float4(data.color_.xyz(), 1.0f);

    float4 light_color = theme_color;
    if (state.overlay.flag & V3D_OVERLAY_SHOW_LIGHT_COLORS) {
      light_color = float4(la->r, la->g, la->b, 1.0f);
    }

    _data.color_ = theme_color;

    /* TODO
    DRW_buffer_add_entry(cb->groundline, matrix.pos);
    */

    bufs.light_icon_inner.append(_data.with_color(light_color), select_id);
    bufs.light_icon_outer.append(_data, select_id);

    if (la->type == LA_LOCAL) {
      area_size_x = area_size_y = la->radius;
      bufs.light_point.append(_data, select_id);
    }
    else if (la->type == LA_SUN) {
      bufs.light_sun.append(_data, select_id);
      bufs.light_icon_sun_rays.append(_data.with_color(light_color), select_id);
    }
    else if (la->type == LA_SPOT) {
      /* Previous implementation was using the clip-end distance as cone size.
       * We cannot do this anymore so we use a fixed size of 10. (see #72871) */
      matrix = math::scale(matrix, float3(10.0f));
      /* For cycles and EEVEE the spot attenuation is:
       * `y = (1/sqrt(1 + x^2) - a)/((1 - a) b)`
       * x being the tangent of the angle between the light direction and the
       * generatrix of the cone. We solve the case where spot attenuation y = 1
       * and y = 0 root for y = 1 is sqrt(1/c^2 - 1) root for y = 0 is
       * sqrt(1/a^2 - 1) and use that to position the blend circle. */
      float a = cosf(la->spotsize * 0.5f);
      float b = la->spotblend;
      float c = a * b - a - b;
      float a2 = a * a;
      float c2 = c * c;
      /* Optimized version or root1 / root0 */
      spot_blend = sqrtf((a2 - a2 * c2) / (c2 - a2 * c2));
      spot_cosine = a;
      /* HACK: We pack the area size in alpha color. This is decoded by the shader. */
      _data.color_.w = -max_ff(la->radius, FLT_MIN);
      bufs.light_spot.append(_data, select_id);
      if ((la->mode & LA_SHOW_CONE) && !DRW_state_is_select()) {
        bufs.light_spot_cone_front.append(_data.with_color({0.0f, 0.0f, 0.0f, 0.5f}), select_id);
        bufs.light_spot_cone_back.append(_data.with_color({1.0f, 1.0f, 1.0f, 0.3f}), select_id);
      }
    }
    else if (la->type == LA_AREA) {
      InstanceBuf &buf = ELEM(la->area_shape, LA_AREA_SQUARE, LA_AREA_RECT) ?
                             bufs.light_area_square :
                             bufs.light_area_disk;
      bool uniform_scale = !ELEM(la->area_shape, LA_AREA_RECT, LA_AREA_ELLIPSE);
      area_size_x = la->area_size;
      area_size_y = uniform_scale ? la->area_size : la->area_sizey;
      buf.append(_data, select_id);
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
