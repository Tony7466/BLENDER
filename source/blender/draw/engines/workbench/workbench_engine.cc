/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_editmesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_paint.h"
#include "BKE_particle.h"
#include "BKE_pbvh.h"
#include "BKE_report.h"
#include "DEG_depsgraph_query.h"
#include "DNA_fluid_types.h"
#include "ED_paint.h"
#include "ED_view3d.h"
#include "GPU_capabilities.h"

#include "draw_common.hh"

#include "workbench_private.hh"

#include "BKE_volume.h"
#include "BKE_volume_render.h"
#include "BLI_rand.h"

#include "workbench_engine.h" /* Own include. */

namespace blender::workbench {

using namespace draw;

class VolumePass {
  bool active_ = true;

  PassMain ps_ = {"Volume Ps"};
  Framebuffer fb_ = {"Volume Fb"};

  Texture dummy_shadow_tx_ = {"Dummy Shadow"};
  Texture dummy_volume_tx_ = {"Dummy Volume"};
  Texture dummy_coba_tx_ = {"Dummy Coba"};

  GPUShader *shaders_[2][2][3][2];

  GPUShader *get_shader(bool slice, bool coba, int interpolation, bool smoke)
  {
    GPUShader *&shader = shaders_[slice][coba][interpolation][smoke];

    if (shader == nullptr) {
      std::string create_info_name = "workbench_next_volume";
      create_info_name += (smoke) ? "_smoke" : "_object";
      switch (interpolation) {
        case VOLUME_DISPLAY_INTERP_LINEAR:
          create_info_name += "_linear";
          break;
        case VOLUME_DISPLAY_INTERP_CUBIC:
          create_info_name += "_linear";
          break;
        case VOLUME_DISPLAY_INTERP_CLOSEST:
          create_info_name += "_linear";
          break;
        default:
          BLI_assert_unreachable();
      }
      create_info_name += (coba) ? "_coba" : "_no_coba";
      create_info_name += (slice) ? "_slice" : "_no_slice";
      shader = GPU_shader_create_from_info_name(create_info_name.c_str());
    }
    return shader;
  }

  void setup_slice_ps(PassMain::Sub &ps, Object *ob, int slice_axis_enum, float slice_depth)
  {
    float4x4 view_mat_inv;
    DRW_view_viewmat_get(nullptr, view_mat_inv.ptr(), true);

    const int axis = (slice_axis_enum == SLICE_AXIS_AUTO) ?
                         axis_dominant_v3_single(view_mat_inv[2]) :
                         slice_axis_enum - 1;

    float3 dimensions;
    BKE_object_dimensions_get(ob, dimensions);
    /* 0.05f to achieve somewhat the same opacity as the full view. */
    float step_length = max_ff(1e-16f, dimensions[axis] * 0.05f);

    /*TODO (Miguel Pozo): Does this override or replace the parent pass state ? */
    ps.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL | ~DRW_STATE_CULL_FRONT);
    ps.push_constant("slicePosition", slice_depth);
    ps.push_constant("sliceAxis", axis);
    ps.push_constant("stepLength", step_length);
  }

  void setup_non_slice_ps(
      PassMain::Sub &ps, Object *ob, int taa_sample, float3 slice_count, float3 world_size)
  {
    double noise_offset;
    BLI_halton_1d(3, 0.0, taa_sample, &noise_offset);

    int max_slice = std::max({UNPACK3(slice_count)});
    float step_length = math::length((1.0f / slice_count) * world_size);

    /*TODO (Miguel Pozo): Does this override or replace the parent pass state ? */
    ps.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL | DRW_STATE_CULL_FRONT);
    ps.push_constant("samplesLen", max_slice);
    ps.push_constant("stepLength", step_length);
    ps.push_constant("noiseOfs", float(noise_offset));
  }

 public:
  void sync(SceneResources &resources)
  {
    active_ = false;
    ps_.init();
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL | DRW_STATE_CULL_FRONT);
    ps_.bind_ubo(WB_WORLD_SLOT, resources.world_buf);

    dummy_shadow_tx_.ensure_3d(GPU_RGBA8, int3(1), GPU_TEXTURE_USAGE_SHADER_READ, float4(1));
    dummy_volume_tx_.ensure_3d(GPU_RGBA8, int3(1), GPU_TEXTURE_USAGE_SHADER_READ, float4(0));
    dummy_coba_tx_.ensure_1d(GPU_RGBA8, 1, GPU_TEXTURE_USAGE_SHADER_READ, float4(0));
  }

  void object_sync_volume(Manager &manager,
                          SceneResources &resources,
                          const SceneState &scene_state,
                          ObjectRef &ob_ref,
                          float3 color)
  {
    Object *ob = ob_ref.object;
    /* Create 3D textures. */
    Volume *volume = static_cast<Volume *>(ob->data);
    BKE_volume_load(volume, G.main);
    const VolumeGrid *volume_grid = BKE_volume_grid_active_get_for_read(volume);
    if (volume_grid == nullptr) {
      return;
    }

#if 0
    /* TODO (Miguel Pozo): Use common API? */
    Vector<VolumeAttribute> attrs = {
        {"densityTexture", BKE_volume_grid_name(volume_grid), eGPUDefaultValue::GPU_DEFAULT_1}};
    PassMain::Sub &sub_ps = ps_.sub(ob->id.name);
    if (!volume_sub_pass(ps_, nullptr, nullptr, attrs)) {
      return;
    }
#endif

    DRWVolumeGrid *grid = DRW_volume_batch_cache_get_grid(volume, volume_grid);
    if (grid == nullptr) {
      return;
    }

    active_ = true;

    PassMain::Sub &sub_ps = ps_.sub(ob->id.name);

    const bool use_slice = (volume->display.axis_slice_method == AXIS_SLICE_SINGLE);

    sub_ps.shader_set(get_shader(use_slice, false, volume->display.interpolation_method, false));

    const float density_scale = volume->display.density *
                                BKE_volume_density_scale(volume, ob->object_to_world);

    sub_ps.bind_texture("depthBuffer", &resources.depth_tx);
    sub_ps.bind_texture("densityTexture", grid->texture);
    /* TODO: implement shadow texture, see manta_smoke_calc_transparency. */
    sub_ps.bind_texture("shadowTexture", dummy_shadow_tx_);
    sub_ps.push_constant("activeColor", color);
    sub_ps.push_constant("densityScale", density_scale);
    sub_ps.push_constant("volumeObjectToTexture", float4x4(grid->object_to_texture));
    sub_ps.push_constant("volumeTextureToObject", float4x4(grid->texture_to_object));

    if (use_slice) {
      setup_slice_ps(sub_ps, ob, volume->display.slice_axis, volume->display.slice_depth);
    }
    else {
      float3 world_size;
      float4x4 texture_to_world = float4x4(ob->object_to_world) *
                                  float4x4(grid->texture_to_object);
      math::normalize_and_get_size(float3x3(texture_to_world), world_size);

      int3 resolution;
      GPU_texture_get_mipmap_size(grid->texture, 0, resolution);
      float3 slice_count = float3(resolution) * 5.0f;

      setup_non_slice_ps(sub_ps, ob, scene_state.sample, slice_count, world_size);
    }

    sub_ps.draw(DRW_cache_cube_get(), manager.resource_handle(ob_ref));
  }

  void object_sync_modifier(Manager &manager,
                            SceneResources &resources,
                            const SceneState &scene_state,
                            ObjectRef &ob_ref,
                            ModifierData *md)
  {
    Object *ob = ob_ref.object;

    FluidModifierData *modifier = reinterpret_cast<FluidModifierData *>(md);
    FluidDomainSettings &settings = *modifier->domain;

    if (!settings.fluid) {
      return;
    }

    bool can_load = false;
    if (settings.use_coba) {
      DRW_smoke_ensure_coba_field(modifier);
      can_load = settings.tex_field != nullptr;
    }
    else if (settings.type == FLUID_DOMAIN_TYPE_GAS) {
      DRW_smoke_ensure(modifier, settings.flags & FLUID_DOMAIN_USE_NOISE);
      can_load = settings.tex_density != nullptr || settings.tex_color != nullptr;
    }

    if (!can_load) {
      return;
    }

    active_ = true;

    PassMain::Sub &sub_ps = ps_.sub(ob->id.name);

    const bool use_slice = settings.axis_slice_method == AXIS_SLICE_SINGLE;

    sub_ps.shader_set(get_shader(use_slice, settings.use_coba, settings.interp_method, true));

    if (settings.use_coba) {
      const bool show_flags = settings.coba_field == FLUID_DOMAIN_FIELD_FLAGS;
      const bool show_pressure = settings.coba_field == FLUID_DOMAIN_FIELD_PRESSURE;
      const bool show_phi = ELEM(settings.coba_field,
                                 FLUID_DOMAIN_FIELD_PHI,
                                 FLUID_DOMAIN_FIELD_PHI_IN,
                                 FLUID_DOMAIN_FIELD_PHI_OUT,
                                 FLUID_DOMAIN_FIELD_PHI_OBSTACLE);

      sub_ps.push_constant("showFlags", show_flags);
      sub_ps.push_constant("showPressure", show_pressure);
      sub_ps.push_constant("showPhi", show_phi);
      sub_ps.push_constant("gridScale", settings.grid_scale);

      if (show_flags) {
        sub_ps.bind_texture("flagTexture", settings.tex_field);
      }
      else {
        sub_ps.bind_texture("densityTexture", settings.tex_field);
      }

      if (!show_flags && !show_pressure && !show_phi) {
        sub_ps.bind_texture("transferTexture", settings.tex_coba);
      }
    }
    else {
      bool use_constant_color = ((settings.active_fields & FLUID_DOMAIN_ACTIVE_COLORS) == 0 &&
                                 (settings.active_fields & FLUID_DOMAIN_ACTIVE_COLOR_SET) != 0);

      sub_ps.push_constant("activeColor",
                           use_constant_color ? float3(settings.active_color) : float3(1));

      sub_ps.bind_texture("densityTexture",
                          settings.tex_color ? settings.tex_color : settings.tex_density);
      sub_ps.bind_texture("flameTexture",
                          settings.tex_flame ? settings.tex_flame : dummy_volume_tx_);
      sub_ps.bind_texture("flameColorTexture",
                          settings.tex_flame ? settings.tex_flame_coba : dummy_coba_tx_);
      sub_ps.bind_texture("shadowTexture", settings.tex_shadow);
    }

    sub_ps.push_constant("densityScale", 10.0f * settings.display_thickness);
    sub_ps.bind_texture("depthBuffer", &resources.depth_tx);

    if (use_slice) {
      setup_slice_ps(sub_ps, ob, settings.slice_axis, settings.slice_depth);
      /* TODO (Miguel Pozo): Why is a quad used here, but not in volume? */
      sub_ps.draw(DRW_cache_quad_get(), manager.resource_handle(ob_ref));
    }
    else {
      float3 world_size;
      BKE_object_dimensions_get(ob, world_size);

      float3 slice_count = float3(settings.res) * std::max(0.001f, settings.slice_per_voxel);

      setup_non_slice_ps(sub_ps, ob, scene_state.sample, slice_count, world_size);
      sub_ps.draw(DRW_cache_cube_get(), manager.resource_handle(ob_ref));
    }
  }

  void draw(Manager &manager, View &view, SceneResources &resources)
  {
    if (!active_) {
      return;
    }
    fb_.ensure(GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(resources.color_tx));
    fb_.bind();
    manager.submit(ps_, view);
  }
};

class Instance {
 public:
  View view = {"DefaultView"};

  SceneState scene_state;

  SceneResources resources;

  OpaquePass opaque_ps;
  TransparentPass transparent_ps;
  TransparentDepthPass transparent_depth_ps;

  ShadowPass shadow_ps;
  VolumePass volume_ps;
  OutlinePass outline_ps;
  DofPass dof_ps;
  AntiAliasingPass anti_aliasing_ps;

  /* An array of nullptr GPUMaterial pointers so we can call DRW_cache_object_surface_material_get.
   * They never get actually used. */
  Vector<GPUMaterial *> dummy_gpu_materials = {1, nullptr, {}};
  GPUMaterial **get_dummy_gpu_materials(int material_count)
  {
    if (material_count > dummy_gpu_materials.size()) {
      dummy_gpu_materials.resize(material_count, nullptr);
    }
    return dummy_gpu_materials.begin();
  };

  void init(Object *camera_ob = nullptr)
  {
    scene_state.init(camera_ob);
    shadow_ps.init(scene_state, resources);
    resources.init(scene_state);

    outline_ps.init(scene_state);
    dof_ps.init(scene_state);
    anti_aliasing_ps.init(scene_state);
  }

  void begin_sync()
  {
    const float2 viewport_size = DRW_viewport_size_get();
    const int2 resolution = {int(viewport_size.x), int(viewport_size.y)};
    resources.depth_tx.ensure_2d(GPU_DEPTH24_STENCIL8,
                                 resolution,
                                 GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT |
                                     GPU_TEXTURE_USAGE_MIP_SWIZZLE_VIEW);
    resources.material_buf.clear();

    opaque_ps.sync(scene_state, resources);
    transparent_ps.sync(scene_state, resources);
    transparent_depth_ps.sync(scene_state, resources);

    shadow_ps.sync();
    volume_ps.sync(resources);
    outline_ps.sync(resources);
    dof_ps.sync(resources);
    anti_aliasing_ps.sync(resources, scene_state.resolution);
  }

  void end_sync()
  {
    resources.material_buf.push_update();
  }

  Material get_material(ObjectRef ob_ref, eV3DShadingColorType color_type, int slot = 0)
  {
    switch (color_type) {
      case V3D_SHADING_OBJECT_COLOR:
        return Material(*ob_ref.object);
      case V3D_SHADING_RANDOM_COLOR:
        return Material(*ob_ref.object, true);
      case V3D_SHADING_SINGLE_COLOR:
        return scene_state.material_override;
      case V3D_SHADING_VERTEX_COLOR:
        return scene_state.material_attribute_color;
      case V3D_SHADING_MATERIAL_COLOR:
        if (::Material *_mat = BKE_object_material_get_eval(ob_ref.object, slot + 1)) {
          return Material(*_mat);
        }
      default:
        return Material(*BKE_material_default_empty());
    }
  }

  void object_sync(Manager &manager, ObjectRef &ob_ref)
  {
    if (scene_state.render_finished) {
      return;
    }

    Object *ob = ob_ref.object;
    if (!DRW_object_is_renderable(ob)) {
      return;
    }

    const ObjectState object_state = ObjectState(scene_state, ob);

    /* Needed for mesh cache validation, to prevent two copies of
     * of vertex color arrays from being sent to the GPU (e.g.
     * when switching from eevee to workbench).
     */
    if (ob_ref.object->sculpt && ob_ref.object->sculpt->pbvh) {
      BKE_pbvh_is_drawing_set(ob_ref.object->sculpt->pbvh, object_state.sculpt_pbvh);
    }

    if (ob->type == OB_MESH && ob->modifiers.first != nullptr) {

      LISTBASE_FOREACH (ModifierData *, md, &ob->modifiers) {
        if (md->type != eModifierType_ParticleSystem) {
          continue;
        }
        ParticleSystem *psys = ((ParticleSystemModifierData *)md)->psys;
        if (!DRW_object_is_visible_psys_in_active_context(ob, psys)) {
          continue;
        }
        ParticleSettings *part = psys->part;
        const int draw_as = (part->draw_as == PART_DRAW_REND) ? part->ren_as : part->draw_as;

        if (draw_as == PART_DRAW_PATH) {
#if 0 /* TODO(@pragma37): */
          workbench_cache_hair_populate(wpd,
                                        ob,
                                        psys,
                                        md,
                                        object_state.color_type,
                                        object_state.texture_paint_mode,
                                        part->omat);
#endif
        }
      }
    }

    if (!(ob->base_flag & BASE_FROM_DUPLI)) {
      ModifierData *md = BKE_modifiers_findby_type(ob, eModifierType_Fluid);
      if (md && BKE_modifier_is_enabled(scene_state.scene, md, eModifierMode_Realtime)) {
        FluidModifierData *fmd = (FluidModifierData *)md;
        if (fmd->domain) {
          volume_ps.object_sync_modifier(manager, resources, scene_state, ob_ref, md);

          if (fmd->domain->type == FLUID_DOMAIN_TYPE_GAS) {
            return; /* Do not draw solid in this case. */
          }
        }
      }
    }

    if (!(DRW_object_visibility_in_active_context(ob) & OB_VISIBLE_SELF)) {
      return;
    }

    if ((ob->dt < OB_SOLID) && !DRW_state_is_scene_render()) {
      return;
    }

    if (ELEM(ob->type, OB_MESH, OB_POINTCLOUD)) {
      mesh_sync(manager, ob_ref, object_state);
    }
    else if (ob->type == OB_CURVES) {
#if 0 /* TODO(@pragma37): */
      DRWShadingGroup *grp = workbench_material_hair_setup(
          wpd, ob, CURVES_MATERIAL_NR, object_state.color_type);
      DRW_shgroup_curves_create_sub(ob, grp, nullptr);
#endif
    }
    else if (ob->type == OB_VOLUME) {
      if (scene_state.shading.type != OB_WIRE) {
        volume_ps.object_sync_volume(manager,
                                     resources,
                                     scene_state,
                                     ob_ref,
                                     get_material(ob_ref, object_state.color_type).base_color);
      }
    }
  }

  void mesh_sync(Manager &manager, ObjectRef &ob_ref, const ObjectState &object_state)
  {
    ResourceHandle handle = manager.resource_handle(ob_ref);
    bool has_transparent_material = false;

    if (object_state.sculpt_pbvh) {
#if 0 /* TODO(@pragma37): */
      workbench_cache_sculpt_populate(wpd, ob, object_state.color_type);
#endif
    }
    else {
      if (object_state.use_per_material_batches) {
        const int material_count = DRW_cache_object_material_count_get(ob_ref.object);

        struct GPUBatch **batches;
        if (object_state.color_type == V3D_SHADING_TEXTURE_COLOR) {
          batches = DRW_cache_mesh_surface_texpaint_get(ob_ref.object);
        }
        else {
          batches = DRW_cache_object_surface_material_get(
              ob_ref.object, get_dummy_gpu_materials(material_count), material_count);
        }

        if (batches) {
          for (auto i : IndexRange(material_count)) {
            if (batches[i] == nullptr) {
              continue;
            }

            Material mat = get_material(ob_ref, object_state.color_type, i);
            has_transparent_material = has_transparent_material || mat.is_transparent();

            ::Image *image = nullptr;
            ImageUser *iuser = nullptr;
            GPUSamplerState sampler_state = GPUSamplerState::default_sampler();
            if (object_state.color_type == V3D_SHADING_TEXTURE_COLOR) {
              get_material_image(ob_ref.object, i + 1, image, iuser, sampler_state);
            }

            draw_mesh(ob_ref, mat, batches[i], handle, image, sampler_state, iuser);
          }
        }
      }
      else {
        struct GPUBatch *batch;
        if (object_state.color_type == V3D_SHADING_TEXTURE_COLOR) {
          batch = DRW_cache_mesh_surface_texpaint_single_get(ob_ref.object);
        }
        else if (object_state.color_type == V3D_SHADING_VERTEX_COLOR) {
          if (ob_ref.object->mode & OB_MODE_VERTEX_PAINT) {
            batch = DRW_cache_mesh_surface_vertpaint_get(ob_ref.object);
          }
          else {
            batch = DRW_cache_mesh_surface_sculptcolors_get(ob_ref.object);
          }
        }
        else {
          batch = DRW_cache_object_surface_get(ob_ref.object);
        }

        if (batch) {
          Material mat = get_material(ob_ref, object_state.color_type);
          has_transparent_material = has_transparent_material || mat.is_transparent();

          draw_mesh(ob_ref,
                    mat,
                    batch,
                    handle,
                    object_state.image_paint_override,
                    object_state.override_sampler_state);
        }
      }
    }

    if (object_state.draw_shadow) {
      shadow_ps.object_sync(scene_state, ob_ref, handle, has_transparent_material);
    }
  }

  void draw_mesh(ObjectRef &ob_ref,
                 Material &material,
                 GPUBatch *batch,
                 ResourceHandle handle,
                 ::Image *image = nullptr,
                 GPUSamplerState sampler_state = GPUSamplerState::default_sampler(),
                 ImageUser *iuser = nullptr)
  {
    const bool in_front = (ob_ref.object->dtx & OB_DRAW_IN_FRONT) != 0;
    resources.material_buf.append(material);
    int material_index = resources.material_buf.size() - 1;

    auto draw = [&](MeshPass &pass) {
      pass.draw(ob_ref, batch, handle, material_index, image, sampler_state, iuser);
    };

    if (scene_state.xray_mode || material.is_transparent()) {
      if (in_front) {
        draw(transparent_ps.accumulation_in_front_ps_);
        if (scene_state.draw_transparent_depth) {
          draw(transparent_depth_ps.in_front_ps_);
        }
      }
      else {
        draw(transparent_ps.accumulation_ps_);
        if (scene_state.draw_transparent_depth) {
          draw(transparent_depth_ps.main_ps_);
        }
      }
    }
    else {
      if (in_front) {
        draw(opaque_ps.gbuffer_in_front_ps_);
      }
      else {
        draw(opaque_ps.gbuffer_ps_);
      }
    }
  }

  void draw(Manager &manager, GPUTexture *depth_tx, GPUTexture *color_tx)
  {
    view.sync(DRW_view_default_get());

    int2 resolution = scene_state.resolution;

    if (scene_state.render_finished) {
      /* Just copy back the already rendered result */
      anti_aliasing_ps.draw(manager, view, resources, resolution, depth_tx, color_tx);
      return;
    }

    anti_aliasing_ps.setup_view(view, resolution);

    resources.color_tx.acquire(
        resolution, GPU_RGBA16F, GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT);
    resources.color_tx.clear(resources.world_buf.background_color);
    if (scene_state.draw_object_id) {
      resources.object_id_tx.acquire(
          resolution, GPU_R16UI, GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT);
      resources.object_id_tx.clear(uint4(0));
    }

    Framebuffer fb = Framebuffer("Workbench.Clear");
    fb.ensure(GPU_ATTACHMENT_TEXTURE(resources.depth_tx));
    fb.bind();
    GPU_framebuffer_clear_depth_stencil(fb, 1.0f, 0x00);

    if (!transparent_ps.accumulation_in_front_ps_.is_empty()) {
      resources.depth_in_front_tx.acquire(resolution,
                                          GPU_DEPTH24_STENCIL8,
                                          GPU_TEXTURE_USAGE_SHADER_READ |
                                              GPU_TEXTURE_USAGE_ATTACHMENT);
      if (opaque_ps.gbuffer_in_front_ps_.is_empty()) {
        /* Clear only if it wont be overwritten by `opaque_ps`. */
        Framebuffer fb = Framebuffer("Workbench.Clear");
        fb.ensure(GPU_ATTACHMENT_TEXTURE(resources.depth_in_front_tx));
        fb.bind();
        GPU_framebuffer_clear_depth_stencil(fb, 1.0f, 0x00);
      }
    }

    opaque_ps.draw(manager,
                   view,
                   resources,
                   resolution,
                   &shadow_ps,
                   transparent_ps.accumulation_ps_.is_empty());
    transparent_ps.draw(manager, view, resources, resolution);
    transparent_depth_ps.draw(manager, view, resources);

    volume_ps.draw(manager, view, resources);
    outline_ps.draw(manager, resources);
    dof_ps.draw(manager, view, resources, resolution);
    anti_aliasing_ps.draw(manager, view, resources, resolution, depth_tx, color_tx);

    resources.color_tx.release();
    resources.object_id_tx.release();
    resources.depth_in_front_tx.release();
  }

  void draw_viewport(Manager &manager, GPUTexture *depth_tx, GPUTexture *color_tx)
  {
    this->draw(manager, depth_tx, color_tx);

    if (scene_state.sample + 1 < scene_state.samples_len) {
      DRW_viewport_request_redraw();
    }
  }
};

}  // namespace blender::workbench

/* -------------------------------------------------------------------- */
/** \name Interface with legacy C DRW manager
 * \{ */

using namespace blender;

struct WORKBENCH_Data {
  DrawEngineType *engine_type;
  DRWViewportEmptyList *fbl;
  DRWViewportEmptyList *txl;
  DRWViewportEmptyList *psl;
  DRWViewportEmptyList *stl;
  workbench::Instance *instance;

  char info[GPU_INFO_SIZE];
};

static void workbench_engine_init(void *vedata)
{
  /* TODO(fclem): Remove once it is minimum required. */
  if (!GPU_shader_storage_buffer_objects_support()) {
    return;
  }

  WORKBENCH_Data *ved = reinterpret_cast<WORKBENCH_Data *>(vedata);
  if (ved->instance == nullptr) {
    ved->instance = new workbench::Instance();
  }

  ved->instance->init();
}

static void workbench_cache_init(void *vedata)
{
  if (!GPU_shader_storage_buffer_objects_support()) {
    return;
  }
  reinterpret_cast<WORKBENCH_Data *>(vedata)->instance->begin_sync();
}

static void workbench_cache_populate(void *vedata, Object *object)
{
  if (!GPU_shader_storage_buffer_objects_support()) {
    return;
  }
  draw::Manager *manager = DRW_manager_get();

  draw::ObjectRef ref;
  ref.object = object;
  ref.dupli_object = DRW_object_get_dupli(object);
  ref.dupli_parent = DRW_object_get_dupli_parent(object);

  reinterpret_cast<WORKBENCH_Data *>(vedata)->instance->object_sync(*manager, ref);
}

static void workbench_cache_finish(void *vedata)
{
  if (!GPU_shader_storage_buffer_objects_support()) {
    return;
  }
  reinterpret_cast<WORKBENCH_Data *>(vedata)->instance->end_sync();
}

static void workbench_draw_scene(void *vedata)
{
  WORKBENCH_Data *ved = reinterpret_cast<WORKBENCH_Data *>(vedata);
  if (!GPU_shader_storage_buffer_objects_support()) {
    STRNCPY(ved->info, "Error: No shader storage buffer support");
    return;
  }
  DefaultTextureList *dtxl = DRW_viewport_texture_list_get();
  draw::Manager *manager = DRW_manager_get();
  ved->instance->draw_viewport(*manager, dtxl->depth, dtxl->color);
}

static void workbench_instance_free(void *instance)
{
  if (!GPU_shader_storage_buffer_objects_support()) {
    return;
  }
  delete reinterpret_cast<workbench::Instance *>(instance);
}

static void workbench_view_update(void *vedata)
{
  WORKBENCH_Data *ved = reinterpret_cast<WORKBENCH_Data *>(vedata);
  if (ved->instance) {
    ved->instance->scene_state.reset_taa_next_sample = true;
  }
}

static void workbench_id_update(void *vedata, struct ID *id)
{
  UNUSED_VARS(vedata, id);
}

/* RENDER */

static bool workbench_render_framebuffers_init(void)
{
  /* For image render, allocate own buffers because we don't have a viewport. */
  const float2 viewport_size = DRW_viewport_size_get();
  const int2 size = {int(viewport_size.x), int(viewport_size.y)};

  DefaultTextureList *dtxl = DRW_viewport_texture_list_get();

  /* When doing a multi view rendering the first view will allocate the buffers
   * the other views will reuse these buffers */
  if (dtxl->color == nullptr) {
    BLI_assert(dtxl->depth == nullptr);
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_GENERAL;
    dtxl->color = GPU_texture_create_2d(
        "txl.color", size.x, size.y, 1, GPU_RGBA16F, usage, nullptr);
    dtxl->depth = GPU_texture_create_2d(
        "txl.depth", size.x, size.y, 1, GPU_DEPTH24_STENCIL8, usage, nullptr);
  }

  if (!(dtxl->depth && dtxl->color)) {
    return false;
  }

  DefaultFramebufferList *dfbl = DRW_viewport_framebuffer_list_get();

  GPU_framebuffer_ensure_config(
      &dfbl->default_fb,
      {GPU_ATTACHMENT_TEXTURE(dtxl->depth), GPU_ATTACHMENT_TEXTURE(dtxl->color)});

  GPU_framebuffer_ensure_config(&dfbl->depth_only_fb,
                                {GPU_ATTACHMENT_TEXTURE(dtxl->depth), GPU_ATTACHMENT_NONE});

  GPU_framebuffer_ensure_config(&dfbl->color_only_fb,
                                {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(dtxl->color)});

  return GPU_framebuffer_check_valid(dfbl->default_fb, nullptr) &&
         GPU_framebuffer_check_valid(dfbl->color_only_fb, nullptr) &&
         GPU_framebuffer_check_valid(dfbl->depth_only_fb, nullptr);
}

#ifdef DEBUG
/* This is just to ease GPU debugging when the frame delimiter is set to Finish */
#  define GPU_FINISH_DELIMITER() GPU_finish()
#else
#  define GPU_FINISH_DELIMITER()
#endif

static void write_render_color_output(struct RenderLayer *layer,
                                      const char *viewname,
                                      GPUFrameBuffer *fb,
                                      const struct rcti *rect)
{
  RenderPass *rp = RE_pass_find_by_name(layer, RE_PASSNAME_COMBINED, viewname);
  if (rp) {
    GPU_framebuffer_bind(fb);
    GPU_framebuffer_read_color(fb,
                               rect->xmin,
                               rect->ymin,
                               BLI_rcti_size_x(rect),
                               BLI_rcti_size_y(rect),
                               4,
                               0,
                               GPU_DATA_FLOAT,
                               rp->rect);
  }
}

static void write_render_z_output(struct RenderLayer *layer,
                                  const char *viewname,
                                  GPUFrameBuffer *fb,
                                  const struct rcti *rect,
                                  float4x4 winmat)
{
  RenderPass *rp = RE_pass_find_by_name(layer, RE_PASSNAME_Z, viewname);
  if (rp) {
    GPU_framebuffer_bind(fb);
    GPU_framebuffer_read_depth(fb,
                               rect->xmin,
                               rect->ymin,
                               BLI_rcti_size_x(rect),
                               BLI_rcti_size_y(rect),
                               GPU_DATA_FLOAT,
                               rp->rect);

    int pix_num = BLI_rcti_size_x(rect) * BLI_rcti_size_y(rect);

    /* Convert GPU depth [0..1] to view Z [near..far] */
    if (DRW_view_is_persp_get(nullptr)) {
      for (float &z : MutableSpan(rp->rect, pix_num)) {
        if (z == 1.0f) {
          z = 1e10f; /* Background */
        }
        else {
          z = z * 2.0f - 1.0f;
          z = winmat[3][2] / (z + winmat[2][2]);
        }
      }
    }
    else {
      /* Keep in mind, near and far distance are negatives. */
      float near = DRW_view_near_distance_get(nullptr);
      float far = DRW_view_far_distance_get(nullptr);
      float range = fabsf(far - near);

      for (float &z : MutableSpan(rp->rect, pix_num)) {
        if (z == 1.0f) {
          z = 1e10f; /* Background */
        }
        else {
          z = z * range - near;
        }
      }
    }
  }
}

static void workbench_render_to_image(void *vedata,
                                      struct RenderEngine *engine,
                                      struct RenderLayer *layer,
                                      const struct rcti *rect)
{
  /* TODO(fclem): Remove once it is minimum required. */
  if (!GPU_shader_storage_buffer_objects_support()) {
    return;
  }

  if (!workbench_render_framebuffers_init()) {
    RE_engine_report(engine, RPT_ERROR, "Failed to allocate GPU buffers");
    return;
  }

  GPU_FINISH_DELIMITER();

  /* Setup */

  DefaultFramebufferList *dfbl = DRW_viewport_framebuffer_list_get();
  const DRWContextState *draw_ctx = DRW_context_state_get();
  Depsgraph *depsgraph = draw_ctx->depsgraph;

  WORKBENCH_Data *ved = reinterpret_cast<WORKBENCH_Data *>(vedata);
  if (ved->instance == nullptr) {
    ved->instance = new workbench::Instance();
  }

  /* TODO(sergey): Shall render hold pointer to an evaluated camera instead? */
  Object *camera_ob = DEG_get_evaluated_object(depsgraph, RE_GetCamera(engine->re));

  /* Set the perspective, view and window matrix. */
  float4x4 winmat, viewmat, viewinv;
  RE_GetCameraWindow(engine->re, camera_ob, winmat.ptr());
  RE_GetCameraModelMatrix(engine->re, camera_ob, viewinv.ptr());
  viewmat = math::invert(viewinv);

  DRWView *view = DRW_view_create(viewmat.ptr(), winmat.ptr(), nullptr, nullptr, nullptr);
  DRW_view_default_set(view);
  DRW_view_set_active(view);

  /* Render */
  do {
    if (RE_engine_test_break(engine)) {
      break;
    }

    ved->instance->init(camera_ob);

    DRW_manager_get()->begin_sync();

    workbench_cache_init(vedata);
    auto workbench_render_cache = [](void *vedata,
                                     struct Object *ob,
                                     struct RenderEngine * /*engine*/,
                                     struct Depsgraph * /*depsgraph*/) {
      workbench_cache_populate(vedata, ob);
    };
    DRW_render_object_iter(vedata, engine, depsgraph, workbench_render_cache);
    workbench_cache_finish(vedata);

    DRW_manager_get()->end_sync();

    /* Also we weed to have a correct FBO bound for #DRW_curves_update */
    // GPU_framebuffer_bind(dfbl->default_fb);
    // DRW_curves_update(); /* TODO(@pragma37): Check this once curves are implemented */

    workbench_draw_scene(vedata);

    /* Perform render step between samples to allow
     * flushing of freed GPUBackend resources. */
    GPU_render_step();
    GPU_FINISH_DELIMITER();
  } while (ved->instance->scene_state.sample + 1 < ved->instance->scene_state.samples_len);

  const char *viewname = RE_GetActiveRenderView(engine->re);
  write_render_color_output(layer, viewname, dfbl->default_fb, rect);
  write_render_z_output(layer, viewname, dfbl->default_fb, rect, winmat);
}

static void workbench_render_update_passes(RenderEngine *engine,
                                           Scene *scene,
                                           ViewLayer *view_layer)
{
  if (view_layer->passflag & SCE_PASS_COMBINED) {
    RE_engine_register_pass(engine, scene, view_layer, RE_PASSNAME_COMBINED, 4, "RGBA", SOCK_RGBA);
  }
  if (view_layer->passflag & SCE_PASS_Z) {
    RE_engine_register_pass(engine, scene, view_layer, RE_PASSNAME_Z, 1, "Z", SOCK_FLOAT);
  }
}

extern "C" {

static const DrawEngineDataSize workbench_data_size = DRW_VIEWPORT_DATA_SIZE(WORKBENCH_Data);

DrawEngineType draw_engine_workbench_next = {
    nullptr,
    nullptr,
    N_("Workbench"),
    &workbench_data_size,
    &workbench_engine_init,
    nullptr,
    &workbench_instance_free,
    &workbench_cache_init,
    &workbench_cache_populate,
    &workbench_cache_finish,
    &workbench_draw_scene,
    &workbench_view_update,
    &workbench_id_update,
    &workbench_render_to_image,
    nullptr,
};

RenderEngineType DRW_engine_viewport_workbench_next_type = {
    nullptr,
    nullptr,
    "BLENDER_WORKBENCH_NEXT",
    N_("Workbench Next"),
    RE_INTERNAL | RE_USE_STEREO_VIEWPORT | RE_USE_GPU_CONTEXT,
    nullptr,
    &DRW_render_to_image,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &workbench_render_update_passes,
    &draw_engine_workbench_next,
    {nullptr, nullptr, nullptr},
};
}

/** \} */
