/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_mesh_types.h"

#include "BKE_customdata.hh"
#include "BKE_editmesh.hh"
#include "BKE_global.hh"
#include "BKE_mesh_types.hh"
#include "BKE_subdiv_modifier.hh"

#include "ED_image.hh"

#include "GPU_capabilities.hh"

#include "draw_cache_impl.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

constexpr int overlay_edit_text = V3D_OVERLAY_EDIT_EDGE_LEN | V3D_OVERLAY_EDIT_FACE_AREA |
                                  V3D_OVERLAY_EDIT_FACE_ANG | V3D_OVERLAY_EDIT_EDGE_ANG |
                                  V3D_OVERLAY_EDIT_INDICES;

class Meshes {
 private:
  PassSimple edit_mesh_normals_ps_ = {"Normals"};
  PassSimple::Sub *face_normals_ = nullptr;
  PassSimple::Sub *face_normals_subdiv_ = nullptr;
  PassSimple::Sub *loop_normals_ = nullptr;
  PassSimple::Sub *loop_normals_subdiv_ = nullptr;
  PassSimple::Sub *vert_normals_ = nullptr;

  PassSimple edit_mesh_analysis_ps_ = {"Mesh Analysis"};

  PassSimple edit_mesh_edges_ps_ = {"Edges"};
  PassSimple edit_mesh_faces_ps_ = {"Faces"};
  PassSimple edit_mesh_cages_ps_ = {"Cages"}; /* Same as faces but with a different offset. */
  PassSimple edit_mesh_verts_ps_ = {"Verts"};
  PassSimple edit_mesh_facedots_ps_ = {"FaceDots"};
  PassSimple edit_mesh_skin_roots_ps_ = {"SkinRoots"};

  /* Depth pre-pass to cull edit cage in case the object is not opaque. */
  PassSimple edit_mesh_prepass_ps_ = {"Prepass"};

  bool xray_enabled = false;

  bool show_retopology = false;
  bool show_mesh_analysis = false;
  bool show_face = false;
  bool show_face_dots = false;

  bool select_edge = false;
  bool select_face = false;
  bool select_vert = false;

  /* TODO(fclem): This is quite wasteful and expensive, prefer in shader Z modification like the
   * retopology offset. */
  View view_edit_cage = {"view_edit_cage"};
  View view_edit_edge = {"view_edit_edge"};
  View view_edit_vert = {"view_edit_vert"};
  float view_dist = 0.0f;

  bool enabled_ = false;

 public:
  void begin_sync(Resources &res, const State &state, const View &view)
  {
    enabled_ = state.space_type == SPACE_VIEW3D;

    if (!enabled_) {
      return;
    }

    view_dist = state.view_dist_get(view.winmat());
    xray_enabled = state.xray_enabled;

    ToolSettings *tsettings = state.scene->toolsettings;
    select_edge = (tsettings->selectmode & SCE_SELECT_EDGE);
    select_face = (tsettings->selectmode & SCE_SELECT_FACE);
    select_vert = (tsettings->selectmode & SCE_SELECT_VERTEX);

    int edit_flag = state.v3d->overlay.edit_flag;
    show_retopology = (edit_flag & V3D_OVERLAY_EDIT_RETOPOLOGY) && !state.xray_enabled;
    show_mesh_analysis = (edit_flag & V3D_OVERLAY_EDIT_STATVIS);
    show_face = (edit_flag & V3D_OVERLAY_EDIT_FACES);
    show_face_dots = ((edit_flag & V3D_OVERLAY_EDIT_FACE_DOT) || state.xray_enabled) & select_face;

    const bool show_face_nor = (edit_flag & V3D_OVERLAY_EDIT_FACE_NORMALS);
    const bool show_loop_nor = (edit_flag & V3D_OVERLAY_EDIT_LOOP_NORMALS);
    const bool show_vert_nor = (edit_flag & V3D_OVERLAY_EDIT_VERT_NORMALS);

    const bool do_smooth_wire = (U.gpu_flag & USER_GPU_FLAG_NO_EDIT_MODE_SMOOTH_WIRE) == 0;
    const bool is_wire_shading_mode = (state.v3d->shading.type == OB_WIRE);

    uint4 data_mask = data_mask_get(edit_flag);

    float backwire_opacity = (state.xray_enabled) ? 0.5f : 1.0f;
    float face_alpha = (show_face) ? 1.0f : 0.0f;
    float retopology_offset = RETOPOLOGY_OFFSET(state.v3d);
    /* Cull back-faces for retopology face pass. This makes it so back-faces are not drawn.
     * Doing so lets us distinguish back-faces from front-faces. */
    DRWState face_culling = (show_retopology) ? DRW_STATE_CULL_BACK : DRWState(0);

    GPUTexture **depth_tex = (state.xray_enabled) ? &res.depth_tx : &res.dummy_depth_tx;

    {
      auto &pass = edit_mesh_prepass_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL | face_culling,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_depth.get());
      pass.push_constant("retopologyOffset", retopology_offset);
    }
    {
      /* Normals */
      const bool use_screen_size = (edit_flag & V3D_OVERLAY_EDIT_CONSTANT_SCREEN_SIZE_NORMALS);
      const bool use_hq_normals = (state.scene->r.perf_flag & SCE_PERF_HQ_NORMALS) ||
                                  GPU_use_hq_normals_workaround();

      DRWState pass_state = DRW_STATE_WRITE_DEPTH | DRW_STATE_WRITE_COLOR |
                            DRW_STATE_DEPTH_LESS_EQUAL;
      if (state.xray_enabled) {
        pass_state |= DRW_STATE_BLEND_ALPHA;
      }

      auto &pass = edit_mesh_normals_ps_;
      pass.init();
      pass.state_set(pass_state, state.clipping_plane_count);

      auto shader_pass = [&](GPUShader *shader, const char *name) {
        auto &sub = pass.sub(name);
        sub.shader_set(shader);
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.bind_texture("depthTex", depth_tex);
        sub.push_constant("alpha", backwire_opacity);
        sub.push_constant("isConstantScreenSizeNormals", use_screen_size);
        sub.push_constant("normalSize", state.overlay.normals_length);
        sub.push_constant("normalScreenSize", state.overlay.normals_constant_screen_size);
        sub.push_constant("retopologyOffset", retopology_offset);
        sub.push_constant("hq_normals", use_hq_normals);
        return &sub;
      };

      face_normals_ = loop_normals_ = vert_normals_ = nullptr;

      if (show_face_nor) {
        face_normals_subdiv_ = shader_pass(res.shaders.mesh_face_normal_subdiv.get(), "SubdFNor");
        face_normals_ = shader_pass(res.shaders.mesh_face_normal.get(), "FaceNor");
      }
      if (show_loop_nor) {
        loop_normals_subdiv_ = shader_pass(res.shaders.mesh_loop_normal_subdiv.get(), "SubdLNor");
        loop_normals_ = shader_pass(res.shaders.mesh_loop_normal.get(), "LoopNor");
      }
      if (show_vert_nor) {
        vert_normals_ = shader_pass(res.shaders.mesh_vert_normal.get(), "VertexNor");
      }
    }
    {
      auto &pass = edit_mesh_analysis_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_analysis.get());
      pass.bind_texture("weightTex", res.weight_ramp_tx);
    }

    auto mesh_edit_common_resource_bind = [&](PassSimple &pass, float alpha) {
      pass.bind_texture("depthTex", depth_tex);
      /* TODO(fclem): UBO. */
      pass.push_constant("wireShading", is_wire_shading_mode);
      pass.push_constant("selectFace", select_face);
      pass.push_constant("selectEdge", select_edge);
      pass.push_constant("alpha", alpha);
      pass.push_constant("retopologyOffset", retopology_offset);
      pass.push_constant("dataMask", int4(data_mask));
      pass.bind_ubo("globalsBlock", &res.globals_buf);
    };

    {
      auto &pass = edit_mesh_edges_ps_;
      pass.init();
      /* Change first vertex convention to match blender loop structure. */
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                         DRW_STATE_FIRST_VERTEX_CONVENTION,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_edge.get());
      pass.push_constant("do_smooth_wire", do_smooth_wire);
      pass.push_constant("use_vertex_selection", select_vert);
      mesh_edit_common_resource_bind(pass, backwire_opacity);
    }
    {
      auto &pass = edit_mesh_faces_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                         face_culling,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_face.get());
      mesh_edit_common_resource_bind(pass, face_alpha);
    }
    {
      auto &pass = edit_mesh_cages_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_face.get());
      mesh_edit_common_resource_bind(pass, face_alpha);
    }
    {
      auto &pass = edit_mesh_verts_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                         DRW_STATE_WRITE_DEPTH,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_vert.get());
      mesh_edit_common_resource_bind(pass, backwire_opacity);
    }
    {
      auto &pass = edit_mesh_facedots_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                         DRW_STATE_WRITE_DEPTH,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_facedot.get());
      mesh_edit_common_resource_bind(pass, backwire_opacity);
    }
    {
      auto &pass = edit_mesh_skin_roots_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA |
                         DRW_STATE_WRITE_DEPTH,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders.mesh_edit_skin_root.get());
      pass.push_constant("retopologyOffset", retopology_offset);
      pass.bind_ubo("globalsBlock", &res.globals_buf);
    }
  }

  void edit_object_sync(Manager &manager,
                        const ObjectRef &ob_ref,
                        const State &state,
                        Resources & /*res*/)
  {
    if (!enabled_) {
      return;
    }

    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    Object *ob = ob_ref.object;
    Mesh &mesh = *static_cast<Mesh *>(ob->data);
    /* WORKAROUND: GPU subdiv uses a different normal format. Remove this once GPU subdiv is
     * refactored. */
    const bool use_gpu_subdiv = BKE_subsurf_modifier_has_gpu_subdiv(static_cast<Mesh *>(ob->data));
    const bool draw_as_solid = (ob->dt > OB_WIRE);

    if (show_retopology) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_triangles(mesh);
      edit_mesh_prepass_ps_.draw(geom, res_handle);
    }
    if (draw_as_solid) {
      gpu::Batch *geom = DRW_cache_mesh_surface_get(ob);
      edit_mesh_prepass_ps_.draw(geom, res_handle);
    }

    if (show_mesh_analysis) {
      gpu::Batch *geom = DRW_cache_mesh_surface_mesh_analysis_get(ob);
      edit_mesh_analysis_ps_.draw(geom, res_handle);
    }

    if (face_normals_) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_facedots(mesh);
      (use_gpu_subdiv ? face_normals_subdiv_ : face_normals_)
          ->draw_expand(geom, GPU_PRIM_LINES, 1, 1, res_handle);
    }
    if (loop_normals_) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_loop_normals(mesh);
      (use_gpu_subdiv ? loop_normals_subdiv_ : loop_normals_)
          ->draw_expand(geom, GPU_PRIM_LINES, 1, 1, res_handle);
    }
    if (vert_normals_) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_vert_normals(mesh);
      vert_normals_->draw_expand(geom, GPU_PRIM_LINES, 1, 1, res_handle);
    }

    {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_edges(mesh);
      edit_mesh_edges_ps_.draw_expand(geom, GPU_PRIM_TRIS, 2, 1, res_handle);
    }
    {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_triangles(mesh);
      (mesh_has_edit_cage(ob) ? &edit_mesh_cages_ps_ : &edit_mesh_faces_ps_)
          ->draw(geom, res_handle);
    }
    if (select_vert) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_vertices(mesh);
      edit_mesh_verts_ps_.draw(geom, res_handle);
    }
    if (show_face_dots) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_facedots(mesh);
      edit_mesh_facedots_ps_.draw(geom, res_handle);
    }

    if (mesh_has_skin_roots(ob)) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edit_skin_roots(mesh);
      edit_mesh_skin_roots_ps_.draw_expand(geom, GPU_PRIM_LINES, 32, 1, res_handle);
    }
    if (DRW_state_show_text() && (state.overlay.edit_flag & overlay_edit_text)) {
      DRW_text_edit_mesh_measure_stats(state.region, state.v3d, ob, &state.scene->unit, state.dt);
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }

    GPU_debug_group_begin("Mesh Edit");

    GPU_framebuffer_bind(framebuffer);
    manager.submit(edit_mesh_prepass_ps_, view);
    manager.submit(edit_mesh_analysis_ps_, view);

    if (xray_enabled) {
      GPU_debug_group_end();
      return;
    }

    view_edit_cage.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 0.5f));
    view_edit_edge.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 1.0f));
    view_edit_vert.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 1.5f));

    manager.submit(edit_mesh_normals_ps_, view);
    manager.submit(edit_mesh_faces_ps_, view);
    manager.submit(edit_mesh_cages_ps_, view_edit_cage);
    manager.submit(edit_mesh_edges_ps_, view_edit_edge);
    manager.submit(edit_mesh_verts_ps_, view_edit_vert);
    manager.submit(edit_mesh_skin_roots_ps_, view_edit_vert);
    manager.submit(edit_mesh_facedots_ps_, view_edit_vert);

    GPU_debug_group_end();
  }

  void draw_color_only(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }

    if (!xray_enabled) {
      return;
    }

    GPU_debug_group_begin("Mesh Edit Color Only");

    view_edit_cage.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 0.5f));
    view_edit_edge.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 1.0f));
    view_edit_vert.sync(view.viewmat(), winmat_polygon_offset(view.winmat(), view_dist, 1.5f));

    GPU_framebuffer_bind(framebuffer);
    manager.submit(edit_mesh_normals_ps_, view);
    manager.submit(edit_mesh_faces_ps_, view);
    manager.submit(edit_mesh_cages_ps_, view_edit_cage);
    manager.submit(edit_mesh_edges_ps_, view_edit_edge);
    manager.submit(edit_mesh_verts_ps_, view_edit_vert);
    manager.submit(edit_mesh_skin_roots_ps_, view_edit_vert);
    manager.submit(edit_mesh_facedots_ps_, view_edit_vert);

    GPU_debug_group_end();
  }

  static bool mesh_has_edit_cage(const Object *ob)
  {
    const Mesh &mesh = *static_cast<const Mesh *>(ob->data);
    if (mesh.runtime->edit_mesh.get() != nullptr) {
      const Mesh *editmesh_eval_final = BKE_object_get_editmesh_eval_final(ob);
      const Mesh *editmesh_eval_cage = BKE_object_get_editmesh_eval_cage(ob);

      return (editmesh_eval_cage != nullptr) && (editmesh_eval_cage != editmesh_eval_final);
    }
    return false;
  }

 private:
  uint4 data_mask_get(const int flag)
  {
    uint4 mask = {0xFF, 0xFF, 0x00, 0x00};
    SET_FLAG_FROM_TEST(mask[0], flag & V3D_OVERLAY_EDIT_FACES, VFLAG_FACE_SELECTED);
    SET_FLAG_FROM_TEST(mask[0], flag & V3D_OVERLAY_EDIT_FREESTYLE_FACE, VFLAG_FACE_FREESTYLE);
    SET_FLAG_FROM_TEST(mask[1], flag & V3D_OVERLAY_EDIT_FREESTYLE_EDGE, VFLAG_EDGE_FREESTYLE);
    SET_FLAG_FROM_TEST(mask[1], flag & V3D_OVERLAY_EDIT_SEAMS, VFLAG_EDGE_SEAM);
    SET_FLAG_FROM_TEST(mask[1], flag & V3D_OVERLAY_EDIT_SHARP, VFLAG_EDGE_SHARP);
    SET_FLAG_FROM_TEST(mask[2], flag & V3D_OVERLAY_EDIT_CREASES, 0xFF);
    SET_FLAG_FROM_TEST(mask[3], flag & V3D_OVERLAY_EDIT_BWEIGHTS, 0xFF);
    return mask;
  }

  static bool mesh_has_skin_roots(const Object *ob)
  {
    const Mesh &mesh = *static_cast<const Mesh *>(ob->data);
    if (BMEditMesh *em = mesh.runtime->edit_mesh.get()) {
      return CustomData_get_offset(&em->bm->vdata, CD_MVERT_SKIN) != -1;
    }
    return false;
  }
};

class MeshUVs {
 private:
  PassSimple analysis_ps_ = {"Mesh Analysis"};

  PassSimple wireframe_ps_ = {"Wireframe"};

  PassSimple edges_ps_ = {"Edges"};
  PassSimple faces_ps_ = {"Faces"};
  PassSimple verts_ps_ = {"Verts"};
  PassSimple facedots_ps_ = {"FaceDots"};

  bool show_vert = false;
  bool show_face = false;
  bool show_face_dots = false;
  bool show_uv_edit = false;

  /** Wireframe Overlay */
  /* Draw final evaluated UVs (modifier stack applied) as greyed out wire-frame. */
  /* TODO(fclem): Maybe should be its own Overlay?. */
  bool show_wireframe = false;

  /** Brush stencil. */
  /* TODO(fclem): Maybe should be its own Overlay?. */
  bool show_stencil = false;

  /** Paint Mask overlay. */
  /* TODO(fclem): Maybe should be its own Overlay?. */
  bool show_mask = false;
  eMaskOverlayMode mask_mode = MASK_OVERLAY_ALPHACHANNEL;
  Mask *mask_id = nullptr;

  /** Stretching Overlay. */
  bool show_mesh_analysis = false;
  eSpaceImage_UVDT_Stretch mesh_analysis_type;
  /**
   * In order to display the stretching relative to all objects in edit mode, we have to sum the
   * area ***AFTER*** extraction and before drawing. To that end, we get a pointer to the resulting
   * total per mesh area location to dereference after extraction.
   */
  Vector<float *> per_mesh_area_3d;
  Vector<float *> per_mesh_area_2d;

  /** UDIM border overlay. */
  bool show_tiled_image_active = false;
  bool show_tiled_image_border = false;

  bool enabled_ = false;

 public:
  void begin_sync(Resources &res, const State &state)
  {
    enabled_ = state.space_type == SPACE_IMAGE;

    if (!enabled_) {
      return;
    }

    const ToolSettings *tool_setting = state.scene->toolsettings;
    const SpaceImage *space_image = reinterpret_cast<const SpaceImage *>(state.space_data);
    ::Image *image = space_image->image;
    const bool is_tiled_image = image && (image->source == IMA_SRC_TILED);
    const bool is_viewer = image && ELEM(image->type, IMA_TYPE_R_RESULT, IMA_TYPE_COMPOSITE);
    /* Only disable UV drawing on top of render results.
     * Otherwise, show UVs even in the absence of active image. */
    enabled_ = !is_viewer;

    if (!enabled_) {
      return;
    }

    const bool space_mode_is_paint = space_image->mode == SI_MODE_PAINT;
    const bool space_mode_is_view = space_image->mode == SI_MODE_VIEW;
    const bool space_mode_is_mask = space_image->mode == SI_MODE_MASK;
    const bool space_mode_is_uv = space_image->mode == SI_MODE_UV;

    const bool object_mode_is_edit = state.object_mode & OB_MODE_EDIT;
    const bool object_mode_is_paint = state.object_mode & OB_MODE_TEXTURE_PAINT;

    {
      /* Edit UV Overlay. */
      show_uv_edit = space_mode_is_uv && object_mode_is_edit;
      show_mesh_analysis = show_uv_edit && (space_image->flag & SI_DRAW_STRETCH);

      const bool hide_faces = space_image->flag & SI_NO_DRAWFACES;

      int sel_mode_2d = tool_setting->uv_selectmode;
      show_vert = (sel_mode_2d != UV_SELECT_EDGE);
      show_face = !show_mesh_analysis && !hide_faces;
      show_face_dots = (sel_mode_2d & UV_SELECT_FACE) && !hide_faces;

      if (tool_setting->uv_flag & UV_SYNC_SELECTION) {
        int sel_mode_3d = tool_setting->selectmode;
        /* NOTE: Ignore #SCE_SELECT_VERTEX because a single selected edge
         * on the mesh may cause single UV vertices to be selected. */
        show_vert = true /* (sel_mode_3d & SCE_SELECT_VERTEX) */;
        show_face_dots = (sel_mode_3d & SCE_SELECT_FACE) && !hide_faces;
      }

      if (show_mesh_analysis) {
        mesh_analysis_type = eSpaceImage_UVDT_Stretch(space_image->dt_uvstretch);
      }
    }
    {
      /* Wireframe UV Overlay. */
      const bool show_wireframe_uv_edit = space_image->flag & SI_DRAWSHADOW;
      const bool show_wireframe_tex_paint = !(space_image->flag & SI_NO_DRAW_TEXPAINT);

      if (space_mode_is_uv && object_mode_is_edit) {
        show_wireframe = show_wireframe_uv_edit;
      }
      else if (space_mode_is_uv && object_mode_is_paint) {
        show_wireframe = show_wireframe_tex_paint;
      }
      else if (space_mode_is_paint && (object_mode_is_paint || object_mode_is_edit)) {
        show_wireframe = show_wireframe_tex_paint;
      }
      else if (space_mode_is_view && object_mode_is_paint) {
        show_wireframe = show_wireframe_tex_paint;
      }
      else {
        show_wireframe = false;
      }
    }
    {
      /* Brush Stencil Overlay. */
      const Brush *brush = BKE_paint_brush_for_read(&tool_setting->imapaint.paint);
      show_stencil = space_mode_is_paint && brush &&
                     (brush->image_brush_type == IMAGE_PAINT_BRUSH_TYPE_CLONE) &&
                     brush->clone.image;
    }
    {
      /* Mask Overlay. */
      show_mask = space_mode_is_mask && space_image->mask_info.mask &&
                  space_image->mask_info.draw_flag & MASK_DRAWFLAG_OVERLAY;
      if (show_mask) {
        mask_mode = eMaskOverlayMode(space_image->mask_info.overlay_mode);
        mask_id = (Mask *)DEG_get_evaluated_id(state.depsgraph, &space_image->mask_info.mask->id);
      }
      else {
        mask_id = nullptr;
      }
    }
    {
      /* UDIM Overlay. */
      /* TODO: Always enable this overlay even if overlays are disabled. */
      show_tiled_image_active = is_tiled_image; /* TODO: Only disable this if overlays are off. */
      show_tiled_image_border = is_tiled_image;
    }

    const bool do_smooth_wire = (U.gpu_flag & USER_GPU_FLAG_OVERLAY_SMOOTH_WIRE) == 0;
    const float dash_length = 4.0f * UI_SCALE_FAC;

    if (show_wireframe) {
      auto &pass = wireframe_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     DRW_STATE_BLEND_ALPHA);
      pass.shader_set(show_vert ? res.shaders.uv_edit_edges_flat.get() :
                                  res.shaders.uv_edit_edges.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      pass.push_constant("lineStyle", OVERLAY_UV_LINE_STYLE_SHADOW);
      pass.push_constant("alpha", space_image->uv_opacity);
      pass.push_constant("dashLength", dash_length);
      pass.push_constant("doSmoothWire", do_smooth_wire);
    }

    if (show_uv_edit) {
      auto &pass = edges_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                     DRW_STATE_BLEND_ALPHA);
      pass.shader_set(show_vert ? res.shaders.uv_edit_edges_flat.get() :
                                  res.shaders.uv_edit_edges.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      pass.push_constant("lineStyle", int(edit_uv_line_style_from_space_image(space_image)));
      pass.push_constant("alpha", space_image->uv_opacity);
      pass.push_constant("dashLength", dash_length);
      pass.push_constant("doSmoothWire", do_smooth_wire);
    }

#if 0
    if (pd->edit_uv.do_uv_overlay) {
      if (pd->edit_uv.do_verts || pd->edit_uv.do_face_dots) {
        DRW_PASS_CREATE(psl->edit_uv_verts_ps,
                        DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                            DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA);
      }

      /* uv verts */
      if (pd->edit_uv.do_verts) {
        GPUShader *sh = OVERLAY_shader_edit_uv_verts_get();
        pd->edit_uv_verts_grp = DRW_shgroup_create(sh, psl->edit_uv_verts_ps);

        const float point_size = UI_GetThemeValuef(TH_VERTEX_SIZE) * UI_SCALE_FAC;

        DRW_shgroup_uniform_block(pd->edit_uv_verts_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_float_copy(
            pd->edit_uv_verts_grp, "pointSize", (point_size + 1.5f) * M_SQRT2);
        DRW_shgroup_uniform_float_copy(pd->edit_uv_verts_grp, "outlineWidth", 0.75f);
        float theme_color[4];
        UI_GetThemeColor4fv(TH_VERTEX, theme_color);
        srgb_to_linearrgb_v4(theme_color, theme_color);
        DRW_shgroup_uniform_vec4_copy(pd->edit_uv_verts_grp, "color", theme_color);
      }

      /* uv faces */
      if (pd->edit_uv.do_faces) {
        DRW_PASS_CREATE(psl->edit_uv_faces_ps,
                        DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS | DRW_STATE_BLEND_ALPHA);
        GPUShader *sh = OVERLAY_shader_edit_uv_face_get();
        pd->edit_uv_faces_grp = DRW_shgroup_create(sh, psl->edit_uv_faces_ps);
        DRW_shgroup_uniform_block(pd->edit_uv_faces_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_float(pd->edit_uv_faces_grp, "uvOpacity", &pd->edit_uv.uv_opacity, 1);
      }

      /* uv face dots */
      if (pd->edit_uv.do_face_dots) {
        const float point_size = UI_GetThemeValuef(TH_FACEDOT_SIZE) * UI_SCALE_FAC;
        GPUShader *sh = OVERLAY_shader_edit_uv_face_dots_get();
        pd->edit_uv_face_dots_grp = DRW_shgroup_create(sh, psl->edit_uv_verts_ps);
        DRW_shgroup_uniform_block(pd->edit_uv_face_dots_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_float_copy(pd->edit_uv_face_dots_grp, "pointSize", point_size);
      }
    }

    /* uv stretching */
    if (pd->edit_uv.do_uv_stretching_overlay) {
      DRW_PASS_CREATE(psl->edit_uv_stretching_ps,
                      DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS | DRW_STATE_BLEND_ALPHA);
      if (pd->edit_uv.draw_type == SI_UVDT_STRETCH_ANGLE) {
        GPUShader *sh = OVERLAY_shader_edit_uv_stretching_angle_get();
        pd->edit_uv_stretching_grp = DRW_shgroup_create(sh, psl->edit_uv_stretching_ps);
        DRW_shgroup_uniform_block(pd->edit_uv_stretching_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_vec2_copy(pd->edit_uv_stretching_grp, "aspect", pd->edit_uv.uv_aspect);
        DRW_shgroup_uniform_float_copy(
            pd->edit_uv_stretching_grp, "stretch_opacity", pd->edit_uv.stretch_opacity);
      }
      else /* SI_UVDT_STRETCH_AREA */ {
        GPUShader *sh = OVERLAY_shader_edit_uv_stretching_area_get();
        pd->edit_uv_stretching_grp = DRW_shgroup_create(sh, psl->edit_uv_stretching_ps);
        DRW_shgroup_uniform_block(pd->edit_uv_stretching_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_float(
            pd->edit_uv_stretching_grp, "totalAreaRatio", &pd->edit_uv.total_area_ratio, 1);
        DRW_shgroup_uniform_float_copy(
            pd->edit_uv_stretching_grp, "stretch_opacity", pd->edit_uv.stretch_opacity);
      }
    }

    if (pd->edit_uv.do_tiled_image_border_overlay) {
      blender::gpu::Batch *geom = DRW_cache_quad_wires_get();
      float obmat[4][4];
      unit_m4(obmat);

      DRW_PASS_CREATE(psl->edit_uv_tiled_image_borders_ps,
                      DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS);
      GPUShader *sh = OVERLAY_shader_edit_uv_tiled_image_borders_get();

      float theme_color[4], selected_color[4];
      UI_GetThemeColorShade4fv(TH_BACK, 60, theme_color);
      UI_GetThemeColor4fv(TH_FACE_SELECT, selected_color);
      srgb_to_linearrgb_v4(theme_color, theme_color);
      srgb_to_linearrgb_v4(selected_color, selected_color);

      DRWShadingGroup *grp = DRW_shgroup_create(sh, psl->edit_uv_tiled_image_borders_ps);
      DRW_shgroup_uniform_vec4_copy(grp, "ucolor", theme_color);
      const float3 offset = {0.0f, 0.0f, 0.0f};
      DRW_shgroup_uniform_vec3_copy(grp, "offset", offset);

      LISTBASE_FOREACH (ImageTile *, tile, &image->tiles) {
        const int tile_x = ((tile->tile_number - 1001) % 10);
        const int tile_y = ((tile->tile_number - 1001) / 10);
        obmat[3][1] = float(tile_y);
        obmat[3][0] = float(tile_x);
        DRW_shgroup_call_obmat(grp, geom, obmat);
      }
      /* Only mark active border when overlays are enabled. */
      if (pd->edit_uv.do_tiled_image_overlay) {
        /* Active tile border */
        ImageTile *active_tile = static_cast<ImageTile *>(
            BLI_findlink(&image->tiles, image->active_tile_index));
        if (active_tile) {
          obmat[3][0] = float((active_tile->tile_number - 1001) % 10);
          obmat[3][1] = float((active_tile->tile_number - 1001) / 10);
          grp = DRW_shgroup_create(sh, psl->edit_uv_tiled_image_borders_ps);
          DRW_shgroup_uniform_vec4_copy(grp, "ucolor", selected_color);
          DRW_shgroup_call_obmat(grp, geom, obmat);
        }
      }
    }

    if (pd->edit_uv.do_tiled_image_overlay) {
      DRWTextStore *dt = DRW_text_cache_ensure();
      uchar color[4];
      /* Color Management: Exception here as texts are drawn in sRGB space directly. */
      UI_GetThemeColorShade4ubv(TH_BACK, 60, color);
      char text[16];
      LISTBASE_FOREACH (ImageTile *, tile, &image->tiles) {
        BLI_snprintf(text, 5, "%d", tile->tile_number);
        float tile_location[3] = {
            float((tile->tile_number - 1001) % 10), float((tile->tile_number - 1001) / 10), 0.0f};
        DRW_text_cache_add(
            dt, tile_location, text, strlen(text), 10, 10, DRW_TEXT_CACHE_GLOBALSPACE, color);
      }
    }

    if (pd->edit_uv.do_stencil_overlay) {
      const Brush *brush = BKE_paint_brush(&ts->imapaint.paint);
      ::Image *stencil_image = brush->clone.image;
      GPUTexture *stencil_texture = BKE_image_get_gpu_texture(stencil_image, nullptr);

      if (stencil_texture != nullptr) {
      DRW_PASS_CREATE(psl->edit_uv_stencil_ps,
                      DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS |
                          DRW_STATE_BLEND_ALPHA_PREMUL);
      GPUShader *sh = OVERLAY_shader_edit_uv_stencil_image();
      blender::gpu::Batch *geom = DRW_cache_quad_get();
      DRWShadingGroup *grp = DRW_shgroup_create(sh, psl->edit_uv_stencil_ps);
      DRW_shgroup_uniform_texture(grp, "imgTexture", stencil_texture);
      DRW_shgroup_uniform_bool_copy(grp, "imgPremultiplied", true);
      DRW_shgroup_uniform_bool_copy(grp, "imgAlphaBlend", true);
      const float4 color = {1.0f, 1.0f, 1.0f, brush->clone.alpha};
      DRW_shgroup_uniform_vec4_copy(grp, "ucolor", color);

      float size_image[2];
      BKE_image_get_size_fl(image, nullptr, size_image);
      float size_stencil_image[2] = {float(GPU_texture_original_width(stencil_texture)),
                                     float(GPU_texture_original_height(stencil_texture))};

      float obmat[4][4];
      unit_m4(obmat);
      obmat[3][1] = brush->clone.offset[1];
      obmat[3][0] = brush->clone.offset[0];
      obmat[0][0] = size_stencil_image[0] / size_image[0];
      obmat[1][1] = size_stencil_image[1] / size_image[1];

      DRW_shgroup_call_obmat(grp, geom, obmat);
      }
    }

    if (pd->edit_uv.do_mask_overlay) {
    const bool is_combined_overlay = pd->edit_uv.mask_overlay_mode == MASK_OVERLAY_COMBINED;
    DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS;
    state |= is_combined_overlay ? DRW_STATE_BLEND_MUL : DRW_STATE_BLEND_ALPHA;
    DRW_PASS_CREATE(psl->edit_uv_mask_ps, state);

    GPUShader *sh = OVERLAY_shader_edit_uv_mask_image();
    blender::gpu::Batch *geom = DRW_cache_quad_get();
    DRWShadingGroup *grp = DRW_shgroup_create(sh, psl->edit_uv_mask_ps);
    GPUTexture *mask_texture = edit_uv_mask_texture(pd->edit_uv.mask,
                                                    pd->edit_uv.image_size[0],
                                                    pd->edit_uv.image_size[1],
                                                    pd->edit_uv.image_aspect[1],
                                                    pd->edit_uv.image_aspect[1]);
    pd->edit_uv.mask_texture = mask_texture;
    DRW_shgroup_uniform_texture(grp, "imgTexture", mask_texture);
    const float4 color = {1.0f, 1.0f, 1.0f, 1.0f};
    DRW_shgroup_uniform_vec4_copy(grp, "color", color);
    DRW_shgroup_call_obmat(grp, geom, nullptr);
    }

  /* HACK: When editing objects that share the same mesh we should only draw the
   * first one in the order that is used during uv editing. We can only trust that the first object
   * has the correct batches with the correct selection state. See #83187. */
  if ((pd->edit_uv.do_uv_overlay || pd->edit_uv.do_uv_shadow_overlay) &&
      draw_ctx->obact->type == OB_MESH)
  {
    Vector<Object *> objects = BKE_view_layer_array_from_objects_in_mode_unique_data(
        draw_ctx->scene, draw_ctx->view_layer, nullptr, draw_ctx->object_mode);
    for (Object *object : objects) {
      Object *object_eval = DEG_get_evaluated_object(draw_ctx->depsgraph, object);
      DRW_mesh_batch_cache_validate(*object_eval, *(Mesh *)object_eval->data);
      overlay_edit_uv_cache_populate(vedata, *object_eval);
    }
  }
#endif

    per_mesh_area_3d.clear();
    per_mesh_area_2d.clear();
  }

  void edit_object_sync(Manager &manager, const ObjectRef &ob_ref)
  {
    if (!enabled_ || ob_ref.object->type != OB_MESH) {
      return;
    }

    ResourceHandle res_handle = manager.resource_handle(ob_ref);

    Object &ob = *ob_ref.object;
    Mesh &mesh = *static_cast<Mesh *>(ob.data);

    if (show_uv_edit) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_edituv_edges(ob, mesh);
      edges_ps_.draw(geom, res_handle);
    }

    if (show_wireframe) {
      gpu::Batch *geom = DRW_mesh_batch_cache_get_uv_edges(ob, mesh);
      wireframe_ps_.draw(geom, res_handle);
    }
  }

  void draw(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }

    GPU_debug_group_begin("Mesh Edit UVs");

    GPU_framebuffer_bind(framebuffer);
    if (show_wireframe) {
      manager.submit(wireframe_ps_, view);
    }
    // manager.submit(analysis_ps_, view);
    // manager.submit(faces_ps_, view);
    if (show_uv_edit) {
      manager.submit(edges_ps_, view);
    }
    // manager.submit(facedots_ps_, view);
    // manager.submit(verts_ps_, view);

    GPU_debug_group_end();
  }

 private:
  static OVERLAY_UVLineStyle edit_uv_line_style_from_space_image(const SpaceImage *sima)
  {
    const bool is_uv_editor = sima->mode == SI_MODE_UV;
    if (is_uv_editor) {
      switch (sima->dt_uv) {
        case SI_UVDT_OUTLINE:
          return OVERLAY_UV_LINE_STYLE_OUTLINE;
        case SI_UVDT_BLACK:
          return OVERLAY_UV_LINE_STYLE_BLACK;
        case SI_UVDT_WHITE:
          return OVERLAY_UV_LINE_STYLE_WHITE;
        case SI_UVDT_DASH:
          return OVERLAY_UV_LINE_STYLE_DASH;
        default:
          return OVERLAY_UV_LINE_STYLE_BLACK;
      }
    }
    else {
      return OVERLAY_UV_LINE_STYLE_SHADOW;
    }
  }
};

}  // namespace blender::draw::overlay
