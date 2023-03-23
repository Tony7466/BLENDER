/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Contains everything about light baking.
 */

#include "DRW_render.h"

#include "BKE_global.h"

#include "BLI_endian_switch.h"
#include "BLI_threads.h"

#include "DEG_depsgraph_build.h"
#include "DEG_depsgraph_query.h"

#include "PIL_time.h"

#include "GPU_capabilities.h"
#include "GPU_context.h"

#include "WM_api.h"
#include "WM_types.h"

#include "BLO_read_write.h"

#include "wm_window.h"

#include "eevee_engine.h"
#include "eevee_instance.hh"

#include "eevee_lightcache.h"

/* -------------------------------------------------------------------- */
/** \name Light Probe Baking
 * \{ */

namespace blender::eevee {

class LightBake {
 private:
  Depsgraph *depsgraph_;

  /** Scene frame to evaluate the depsgraph at. */
  int frame_;
  /** Milliseconds. Delay the start of the baking to not slowdown interactions (TODO: remove). */
  int delay_ms_;

  /**
   * If running in parallel (in a separate thread), use this context.
   * Created on main thread but first bound in worker thread.
   */
  void *gl_context_ = nullptr;
  /** Context associated to `gl_context_`. Created in the worker thread. */
  GPUContext *gpu_context_ = nullptr;

  /** Baking instance. Created and freed in the worker thread. */
  Instance *instance_ = nullptr;
  /** Light Cache being baked. Create in worker thread and pass ownership to original scene on
   * first `update()` call. */
  ::LightCache *light_cache_ = nullptr;
  /** Manager used for command submission. Created and freed in the worker thread. */
  draw::Manager *manager_ = nullptr;

 public:
  LightBake(struct Main *bmain,
            struct ViewLayer *view_layer,
            struct Scene *scene,
            bool run_as_job,
            int frame,
            int delay_ms = 0)
      : depsgraph_(DEG_graph_new(bmain, scene, view_layer, DAG_EVAL_RENDER)),
        frame_(frame),
        delay_ms_(delay_ms)
  {
    BLI_assert(BLI_thread_is_main());
    if (run_as_job && !GPU_use_main_context_workaround()) {
      /* This needs to happen in main thread. */
      gl_context_ = WM_opengl_context_create();
      wm_window_reset_drawable();
    }
  }

  ~LightBake()
  {
    BLI_assert(BLI_thread_is_main());
    DEG_graph_free(depsgraph_);
  }

  /**
   * Called from main thread.
   * Copy result to original scene data.
   * Note that since this is in the main thread, the viewport cannot be using the light cache.
   * So there is no race condition here.
   */
  void update()
  {
    BLI_assert(BLI_thread_is_main());
    Scene *original_scene = DEG_get_input_scene(depsgraph_);
    LightCache *&scene_light_cache = original_scene->eevee.light_cache_data;

    if (scene_light_cache != light_cache_) {
      if (scene_light_cache != nullptr) {
        /* Delete old data if existing. */
        EEVEE_NEXT_lightcache_free(scene_light_cache);
      }
      scene_light_cache = light_cache_;
    }

    EEVEE_NEXT_lightcache_info_update(&original_scene->eevee);
  }

  /**
   * Called from worker thread.
   */
  void run(bool *stop = nullptr, bool *do_update = nullptr, float *progress = nullptr)
  {
    UNUSED_VARS(stop, do_update, progress);

    DEG_graph_relations_update(depsgraph_);
    DEG_evaluate_on_framechange(depsgraph_, frame_);

    context_enable();
    manager_ = new draw::Manager();
    instance_ = new eevee::Instance();
    instance_->init_light_bake(depsgraph_, manager_);
    context_disable();

    if (delay_ms_ > 0) {
      PIL_sleep_ms(delay_ms_);
    }

    instance_->light_bake_irradiance(
        light_cache_, [this]() { context_enable(); }, [this]() { context_disable(); });

    delete_resources();
  }

 private:
  void context_enable(bool render_begin = true)
  {
    if (GPU_use_main_context_workaround() && !BLI_thread_is_main()) {
      /* Reuse main draw context. */
      GPU_context_main_lock();
      DRW_opengl_context_enable();
    }
    else if (gl_context_ == nullptr) {
      /* Main thread case. */
      DRW_opengl_context_enable();
    }
    else {
      /* Worker thread case. */
      DRW_opengl_render_context_enable(gl_context_);
      if (gpu_context_ == nullptr) {
        /* Create GPUContext in worker thread as it needs the correct gl context bound (which can
         * only be bound in worker thread because of some GL driver requirements). */
        gpu_context_ = GPU_context_create(nullptr, gl_context_);
      }
      DRW_gpu_render_context_enable(gpu_context_);
    }

    if (render_begin) {
      GPU_render_begin();
    }
  }

  void context_disable()
  {
    GPU_render_end();

    if (GPU_use_main_context_workaround() && !BLI_thread_is_main()) {
      /* Reuse main draw context. */
      DRW_opengl_context_disable();
      GPU_context_main_unlock();
    }
    else if (gl_context_ == nullptr) {
      /* Main thread case. */
      DRW_opengl_context_disable();
    }
    else {
      /* Worker thread case. */
      DRW_gpu_render_context_disable(gpu_context_);
      DRW_opengl_render_context_disable(gl_context_);
    }
  }

  /**
   * Delete the engine instance and the optional contexts.
   * This needs to run on the worker thread because the OpenGL context can only be ever bound to a
   * single thread (because of some driver implementation), and the resources (textures,
   * buffers,...) need to be freed with the right context bound.
   */
  void delete_resources()
  {
    /* Bind context without GPU_render_begin(). */
    context_enable(false);

    /* Free GPU data (Textures, Framebuffers, etc...).  */
    delete instance_;
    delete manager_;

    /* Delete / unbind the GL & GPU context. Assumes it is currently bound. */
    if (GPU_use_main_context_workaround() && !BLI_thread_is_main()) {
      /* Reuse main draw context. */
      DRW_opengl_context_disable();
      GPU_context_main_unlock();
    }
    else if (gl_context_ == nullptr) {
      /* Main thread case. */
      DRW_opengl_context_disable();
    }
    else {
      /* Worker thread case. */
      if (gpu_context_ != nullptr) {
        GPU_context_discard(gpu_context_);
      }
      DRW_opengl_render_context_disable(gl_context_);
      WM_opengl_context_dispose(gl_context_);
    }
  }
};

}  // namespace blender::eevee

extern "C" {

using namespace blender::eevee;

/* -------------------------------------------------------------------- */
/** \name Light Bake Job
 * \{ */

wmJob *EEVEE_NEXT_lightbake_job_create(struct wmWindowManager *wm,
                                       struct wmWindow *win,
                                       struct Main *bmain,
                                       struct ViewLayer *view_layer,
                                       struct Scene *scene,
                                       int delay_ms,
                                       int frame)
{
  /* Do not bake if there is a render going on. */
  if (WM_jobs_test(wm, scene, WM_JOB_TYPE_RENDER)) {
    return nullptr;
  }

  /* Stop existing baking job. */
  WM_jobs_stop(wm, nullptr, (void *)EEVEE_NEXT_lightbake_job);

  wmJob *wm_job = WM_jobs_get(wm,
                              win,
                              scene,
                              "Bake Lighting",
                              WM_JOB_EXCL_RENDER | WM_JOB_PRIORITY | WM_JOB_PROGRESS,
                              WM_JOB_TYPE_LIGHT_BAKE);

  LightBake *bake = new LightBake(bmain, view_layer, scene, true, frame, delay_ms);

  WM_jobs_customdata_set(wm_job, bake, EEVEE_NEXT_lightbake_job_data_free);
  WM_jobs_timer(wm_job, 0.4, NC_SCENE | NA_EDITED, 0);
  WM_jobs_callbacks(wm_job,
                    EEVEE_NEXT_lightbake_job,
                    nullptr,
                    EEVEE_NEXT_lightbake_update,
                    EEVEE_NEXT_lightbake_update);

  G.is_break = false;

  return wm_job;
}

void *EEVEE_NEXT_lightbake_job_data_alloc(struct Main *bmain,
                                          struct ViewLayer *view_layer,
                                          struct Scene *scene,
                                          bool run_as_job,
                                          int frame)
{
  /* This should only be used for exec job. Eventually, remove `run_as_job` parameter later. */
  BLI_assert(run_as_job == false);
  LightBake *bake = new LightBake(bmain, view_layer, scene, run_as_job, frame);
  /* TODO(fclem): Can remove this cast once we remove the previous EEVEE light cache. */
  return reinterpret_cast<void *>(bake);
}

void EEVEE_NEXT_lightbake_job_data_free(void *job_data)
{
  delete reinterpret_cast<LightBake *>(job_data);
}

void EEVEE_NEXT_lightbake_update(void *job_data)
{
  reinterpret_cast<LightBake *>(job_data)->update();
}

void EEVEE_NEXT_lightbake_job(void *job_data, bool *stop, bool *do_update, float *progress)
{
  reinterpret_cast<LightBake *>(job_data)->run(stop, do_update, progress);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Light Cache Create / Delete
 * \{ */

LightCache *EEVEE_NEXT_lightcache_create()
{
  LightCache *light_cache = (LightCache *)MEM_callocN(sizeof(LightCache), "LightCache");

  light_cache->version = LIGHTCACHE_NEXT_STATIC_VERSION;
  light_cache->type = LIGHTCACHE_TYPE_STATIC;

  return light_cache;
}

void EEVEE_NEXT_lightcache_free(LightCache *light_cache)
{
  if (light_cache->version < LIGHTCACHE_NEXT_STATIC_VERSION) {
    /* Allow deleting old EEVEE cache. */
    DRW_TEXTURE_FREE_SAFE(light_cache->cube_tx.tex);
    MEM_SAFE_FREE(light_cache->cube_tx.data);
    DRW_TEXTURE_FREE_SAFE(light_cache->grid_tx.tex);
    MEM_SAFE_FREE(light_cache->grid_tx.data);
    if (light_cache->cube_mips) {
      for (int i = 0; i < light_cache->mips_len; i++) {
        MEM_SAFE_FREE(light_cache->cube_mips[i].data);
      }
      MEM_SAFE_FREE(light_cache->cube_mips);
    }
    MEM_SAFE_FREE(light_cache->cube_data);
    MEM_SAFE_FREE(light_cache->grid_data);
  }
  else {
    for (int i = 0; i < light_cache->grid_len; i++) {
      MEM_SAFE_FREE(light_cache->grids[i].surfels);
      MEM_SAFE_FREE(light_cache->grids[i].irradiance_L0_L1_a.data);
      MEM_SAFE_FREE(light_cache->grids[i].irradiance_L0_L1_b.data);
      MEM_SAFE_FREE(light_cache->grids[i].irradiance_L0_L1_c.data);
    }
    MEM_SAFE_FREE(light_cache->grids);
  }
  MEM_freeN(light_cache);
}

void EEVEE_NEXT_lightcache_info_update(SceneEEVEE *eevee)
{
  LightCache *light_cache = eevee->light_cache_data;

  if (light_cache == nullptr) {
    BLI_strncpy(eevee->light_cache_info,
                TIP_("No light cache in this scene"),
                sizeof(eevee->light_cache_info));
    return;
  }

  if (light_cache->version != LIGHTCACHE_NEXT_STATIC_VERSION) {
    BLI_strncpy(eevee->light_cache_info,
                TIP_("Incompatible Light cache version, please bake again"),
                sizeof(eevee->light_cache_info));
    return;
  }

  if (light_cache->flag & LIGHTCACHE_INVALID) {
    BLI_strncpy(eevee->light_cache_info,
                TIP_("Error: Light cache dimensions not supported by the GPU"),
                sizeof(eevee->light_cache_info));
    return;
  }

  if (light_cache->flag & LIGHTCACHE_BAKING) {
    BLI_strncpy(
        eevee->light_cache_info, TIP_("Baking light cache"), sizeof(eevee->light_cache_info));
    return;
  }

  int irradiance_sample_len = 0;
  for (const LightCacheIrradianceGrid &grid :
       blender::Span<LightCacheIrradianceGrid>(light_cache->grids, light_cache->grid_len)) {
    irradiance_sample_len += grid.resolution[0] * grid.resolution[1] * grid.resolution[2];
  }

  size_t size_in_bytes = irradiance_sample_len * sizeof(uint16_t) * 4 * 3;

  char formatted_mem[BLI_STR_FORMAT_INT64_BYTE_UNIT_SIZE];
  BLI_str_format_byte_unit(formatted_mem, size_in_bytes, false);

  BLI_snprintf(eevee->light_cache_info,
               sizeof(eevee->light_cache_info),
               TIP_("%d Ref. Cubemaps, %d Irr. Samples (%s in memory)"),
               light_cache->cube_len,
               irradiance_sample_len,
               formatted_mem);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Light Cache Read/Write to file
 * \{ */

void EEVEE_NEXT_lightcache_blend_write(BlendWriter *writer, LightCache *light_cache)
{
  auto write_lightcache_texture = [&](LightCacheTexture &tex) {
    if (tex.data) {
      size_t data_size = tex.components * tex.tex_size[0] * tex.tex_size[1] * tex.tex_size[2];
      if (tex.data_type == LIGHTCACHETEX_FLOAT) {
        data_size *= sizeof(float);
      }
      else if (tex.data_type == LIGHTCACHETEX_UINT) {
        data_size *= sizeof(uint);
      }

      /* FIXME: We can't save more than what 32bit systems can handle.
       * The solution would be to split the texture but it is too late for 2.90. (see #78529) */
      if (data_size < INT_MAX) {
        BLO_write_raw(writer, data_size, tex.data);
      }
    }
  };

  BLO_write_struct_array(
      writer, LightCacheIrradianceGrid, light_cache->grid_len, light_cache->grids);

  for (int i = 0; i < light_cache->grid_len; i++) {
    LightCacheIrradianceGrid &grid = light_cache->grids[i];
    /* Surfels are runtime data. Not stored in the blend file. */
    write_lightcache_texture(grid.irradiance_L0_L1_a);
    write_lightcache_texture(grid.irradiance_L0_L1_b);
    write_lightcache_texture(grid.irradiance_L0_L1_c);
  }
}

void EEVEE_NEXT_lightcache_blend_read_data(BlendDataReader *reader, LightCache *light_cache)
{

  if (light_cache->grids) {
    BLO_read_data_address(reader, &light_cache->grids);

    auto direct_link_lightcache_texture = [&](LightCacheTexture &lctex) {
      /* Runtime data. Not stored in the blend file. */
      lctex.tex = nullptr;

      if (lctex.data) {
        BLO_read_data_address(reader, &lctex.data);
        if (lctex.data && BLO_read_requires_endian_switch(reader)) {
          int data_size = lctex.components * lctex.tex_size[0] * lctex.tex_size[1] *
                          lctex.tex_size[2];

          if (lctex.data_type == LIGHTCACHETEX_FLOAT) {
            BLI_endian_switch_float_array((float *)lctex.data, data_size * sizeof(float));
          }
          else if (lctex.data_type == LIGHTCACHETEX_UINT) {
            BLI_endian_switch_uint32_array((uint *)lctex.data, data_size * sizeof(uint));
          }
        }
      }

      if (lctex.data == nullptr) {
        zero_v3_int(lctex.tex_size);
      }
    };

    for (int i = 0; i < light_cache->grid_len; i++) {
      LightCacheIrradianceGrid &grid = light_cache->grids[i];
      /* Runtime data. Not stored in the blend file. */
      grid.surfels_len = 0;
      grid.surfels = nullptr;
      direct_link_lightcache_texture(grid.irradiance_L0_L1_a);
      direct_link_lightcache_texture(grid.irradiance_L0_L1_b);
      direct_link_lightcache_texture(grid.irradiance_L0_L1_c);
    }
  }
}

/** \} */
}

/** \} */
