/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque type hiding eevee::LightBake. */
typedef struct EEVEE_NEXT_LightBake EEVEE_NEXT_LightBake;

/* -------------------------------------------------------------------- */
/** \name Light Bake Job
 * \{ */

/**
 * Create the job description.
 * This is called for async (modal) bake operator.
 * The actual work will be done by `EEVEE_NEXT_lightbake_job()`.
 * Will internally call `EEVEE_NEXT_lightbake_job_data_alloc()` or reuse data from an already
 * existing baking job.
 * IMPORTANT: Must run on the main thread because of potential GPUContext creation.
 */
struct wmJob *EEVEE_NEXT_lightbake_job_create(struct wmWindowManager *wm,
                                              struct wmWindow *win,
                                              struct Main *bmain,
                                              struct ViewLayer *view_layer,
                                              struct Scene *scene,
                                              int delay_ms,
                                              int frame);

/**
 * Allocate dependency graph and job description (EEVEE_NEXT_LightBake).
 * Dependency graph evaluation does *not* happen here. It is delayed until
 * `EEVEE_NEXT_lightbake_job` runs.
 * IMPORTANT: Must run on the main thread because of potential GPUContext creation.
 * Return `EEVEE_NEXT_LightBake *` but cast to `void *` because of compatibility with existing
 * EEVEE function.
 */
void *EEVEE_NEXT_lightbake_job_data_alloc(struct Main *bmain,
                                          struct ViewLayer *view_layer,
                                          struct Scene *scene,
                                          bool run_as_job,
                                          int frame);

/**
 * Free the job data.
 * NOTE: Does not free the GPUContext. This is the responsibility of `EEVEE_NEXT_lightbake_job()`
 */
void EEVEE_NEXT_lightbake_job_data_free(void *job_data /* EEVEE_NEXT_LightBake */);

/**
 * Callback for updating original scene light cache with bake result.
 * Run by the job system for each update step and the finish step.
 * This is called manually by `EEVEE_NEXT_lightbake_job()` if not run from a job.
 */
void EEVEE_NEXT_lightbake_update(void *job_data /* EEVEE_NEXT_LightBake */);

/**
 * Do the full light baking for all samples.
 * Will call `EEVEE_NEXT_lightbake_update()` on finish.
 */
void EEVEE_NEXT_lightbake_job(void *job_data /* EEVEE_NEXT_LightBake */,
                              bool *stop,
                              bool *do_update,
                              float *progress);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Light Cache Create / Delete
 * \{ */

/**
 * Create an empty light-cache.
 */
struct LightCache *EEVEE_NEXT_lightcache_create(void);

/**
 * Free a light-cache and its associated data.
 */
void EEVEE_NEXT_lightcache_free(struct LightCache *lcache);

/**
 * Update the UI message in the render panel about the state of the cache.
 */
void EEVEE_NEXT_lightcache_info_update(struct SceneEEVEE *eevee);

/** \} */

/* -------------------------------------------------------------------- */
/** \name Light Cache Read/Write to file
 * \{ */

void EEVEE_NEXT_lightcache_blend_write(struct BlendWriter *writer, struct LightCache *light_cache);
void EEVEE_NEXT_lightcache_blend_read_data(struct BlendDataReader *reader,
                                           struct LightCache *light_cache);

/** \} */

#ifdef __cplusplus
}
#endif
