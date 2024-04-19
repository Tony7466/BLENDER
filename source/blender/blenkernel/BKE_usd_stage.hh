/*
* ***** BEGIN GPL LICENSE BLOCK *****
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
* Contributor(s): Brecht Van Lommel.
*
* ***** END GPL LICENSE BLOCK *****
*/

#ifndef __BKE_USD_STAGE_H__
#define __BKE_USD_STAGE_H__

/** \file BKE_usd_stage.h
*  \ingroup bke
*  \brief USD Stage datablock.
*/

//!TODO(kiki): Fix comments

#include "BLI_bounds_types.hh"
#include "BLI_math_vector_types.hh"


struct BoundBox;
struct Depsgraph;
struct Object;
struct Scene;
struct USDStage;
typedef struct Main Main;

/* Module */

void BKE_usd_stages_init(void);

/* Datablock Management */

void* BKE_usd_stage_add(Main* bmain, const char* name);
struct USDStage* BKE_usd_stage_copy(Main *bmain, const struct USDStage* usd_stage);
void BKE_usd_stage_copy_data(Main bmain, ID *id_dst, const ID *id_src, const int flag);

std::optional<blender::Bounds<blender::float3>> BKE_usd_stage_boundbox_get(const struct Object* ob);

bool BKE_usd_stage_is_y_up(const struct USDStage* usd_stage);

/* Depsgraph */
void BKE_usd_stage_eval(struct Main* bmain, struct USDStage* stage);
void BKE_usd_stage_eval_geometry(struct Depsgraph* depsgraph, struct USDStage* usd_stage);
void BKE_usd_stage_data_update(struct Depsgraph* depsgraph,
                              struct Scene* scene,
                              struct Object* object);

/* Draw Cache */

enum {
 BKE_USD_STAGE_BATCH_DIRTY_ALL = 0,
};

void BKE_usd_stage_batch_cache_dirty_tag(struct USDStage* usd_stage, int mode);
void BKE_usd_stage_batch_cache_free(struct USDStage* usd_stage);

extern void (*BKE_usd_stage_batch_cache_dirty_tag_cb)(struct USDStage* usd_stage, int mode);
extern void (*BKE_usd_stage_batch_cache_free_cb)(struct USDStage* usd_stage);

struct USDStage* BKE_usd_stage_new_for_eval(const struct USDStage* usd_stage_src);
struct USDStage* BKE_usd_stage_copy_for_eval(struct USDStage* usd_stage_src, bool reference);

void BKE_usd_stage_active_prim_set(struct USDStage* stage, const std::string path);

#endif
