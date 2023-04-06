/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright Blender Foundation */

#pragma once

/** \file
 * \ingroup bke
 * \brief General operations for probes.
 */

#ifdef __cplusplus
extern "C" {
#endif

struct LightProbe;
struct Main;
struct BlendWriter;
struct BlendDataReader;
struct LightProbeGridCache;
struct Object;

void BKE_lightprobe_type_set(struct LightProbe *probe, short lightprobe_type);
void *BKE_lightprobe_add(struct Main *bmain, const char *name);

void BKE_lightprobe_grid_cache_blend_write(struct BlendWriter *writer,
                                           struct LightProbeGridCache **grid_caches,
                                           int grid_caches_len);

void BKE_lightprobe_grid_cache_blend_read(struct BlendDataReader *reader,
                                          struct LightProbeGridCache **grid_caches,
                                          int grid_caches_len);

/**
 * Create a single empty irradiance grid cache.
 */
struct LightProbeGridCache *BKE_lightprobe_grid_cache_create(void);

/**
 * Free a single grid cache.
 */
void BKE_lightprobe_grid_cache_free(struct LightProbeGridCache *cache);

/**
 * Create the grid cache list depending on the lightprobe baking settings.
 * The list is left empty to be filled by the baking process.
 */
void BKE_lightprobe_grid_caches_create(struct Object *object);

/**
 * Free all irradiance grids allocated for the given object.
 */
void BKE_lightprobe_grid_caches_free(struct Object *object);

/**
 * Return the number of sample stored inside an irradiance cache.
 * This depends on the light cache type.
 */
int64_t BKE_lightprobe_grid_cache_sample_count(const struct LightProbeGridCache *cache);

#ifdef __cplusplus
}
#endif
