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

#include "BKE_object.h"

#include "DNA_collection_types.h"
#include "DNA_lightprobe_types.h"

#include "PIL_time.h"

#include "eevee_lightcache.h"

#include "GPU_capabilities.h"
#include "GPU_context.h"

#include "WM_api.h"
#include "WM_types.h"

#include "BLO_read_write.h"

#include "wm_window.h"

/* -------------------------------------------------------------------- */
/** \name Light Probe Baking
 * \{ */

/* TODO: should be replace by a more elegant alternative. */
extern void DRW_opengl_context_enable(void);
extern void DRW_opengl_context_disable(void);

extern void DRW_opengl_render_context_enable(void *re_gl_context);
extern void DRW_opengl_render_context_disable(void *re_gl_context);
extern void DRW_gpu_render_context_enable(void *re_gpu_context);
extern void DRW_gpu_render_context_disable(void *re_gpu_context);

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
  /** GPUContext associated to `gl_context_`. Created in the worker thread. */
  void *gpu_context_ = nullptr;

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
    std::cout << "Create" << std::endl;
  }

  ~LightBake()
  {
    BLI_assert(BLI_thread_is_main());
    std::cout << "Delete" << std::endl;
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
    std::cout << "update" << std::endl;
  }

  /**
   * Called from worker thread.
   */
  void run(bool *stop = nullptr, bool *do_update = nullptr, float *progress = nullptr)
  {
    UNUSED_VARS(stop, do_update, progress);

    DEG_graph_relations_update(depsgraph_);
    DEG_evaluate_on_framechange(depsgraph_, frame_);

    PIL_sleep_ms(1000);
    std::cout << "run" << std::endl;

    delete_resources();
  }

 private:
  void context_enable()
  {
  }

  void context_disable()
  {
  }

  /**
   * Delete the engine instance and the optional contexts.
   * This needs to run on the worker thread because the OpenGL context can only be ever bound to a
   * single thread (because of some driver implementation), and the resources (textures,
   * buffers,...) need to be freed with the right context bound.
   */
  void delete_resources()
  {
  }
};

}  // namespace blender::eevee

extern "C" {

using namespace blender::eevee;

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
}

/** \} */
