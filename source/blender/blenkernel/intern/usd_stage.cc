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
* ***** END GPL LICENSE BLOCK *****
*/

/** \file blender/blenkernel/intern/usd_stage.cc
*  \ingroup bke
*/

#include "MEM_guardedalloc.h"

#include "DNA_defaults.h"
#include "DNA_ID.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_usd_stage_types.h"

#include "BLI_compiler_compat.h"
#include "BLI_fileops.h"
#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "BKE_anim_data.h"
#include "BKE_animsys.h"
#include "BKE_global.h"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
//#include "BKE_lib_query.h"
//#include "BKE_lib_remap.h"
#include "BKE_library.hh"
#include "BKE_lib_remap.hh"
#include "BKE_lib_query.h"

#include "BKE_main.hh"
#include "BKE_modifier.hh"
#include "BKE_object.hh"
#include "BKE_packedFile.h"
#include "BKE_scene.h"
#include "BKE_usd_stage.hh"

#include "BLT_translation.h"

#include "DEG_depsgraph_query.hh"

#include "CLG_log.h"

static CLG_LogRef LOG = {"bke.usd_stage"};

#include <list>
#include <unordered_set>
#include <string>

#include "usd_stage_utils.hh"

#include <pxr/pxr.h>
#include <pxr/usd/sdf/layer.h>
#include "pxr/usd/usd/prim.h"
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>


/* tokens we use */
//!TODO(kiki): Fill out as we come across more stuff
const pxr::TfToken USD_TK_XFORM("Xform");
const pxr::TfToken USD_TK_MESH("Mesh");
const pxr::TfToken USD_TK_CURVE("BasisCurves");

const float matrix_identity[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};


/* Module */

void BKE_usd_stages_init()
{
  usd_stage_clear_cache();
}

/* Stage datablock */
static void usd_stage_init_data(ID *id)
{
 USDStage* usd_stage = reinterpret_cast<USDStage*>(id);
 BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(usd_stage, id));
 //!TODO(kiki): I do NOT understand why this doesn't work so I'm punting
 // MEMCPY_STRUCT_AFTER(usd_stage, DNA_struct_default_get(USDStage), id);

 BLI_listbase_clear(&usd_stage->object_paths);
 usd_stage->is_sequence = false;
 usd_stage->override_frame = false;
 usd_stage->scale = 1.0f;
 usd_stage->frame = 0.0f;
 usd_stage->frame_offset = 0.0f;
 usd_stage->filepath[0] = '\0';
 usd_stage->root_prim_path[0] = '\0';
 usd_stage->error = USD_ERR_NO_FILE;

 // start off not cached, and not expanded
 usd_stage->flags = 0;

 // clear active
 BKE_usd_stage_active_prim_set(usd_stage, "");

 // null runtime
 usd_stage->runtime = nullptr;
}

float BKE_usd_stage_time_offset(const USDStage* stage, const float time, const float fps)
{
 const float time_offset = stage->frame_offset / fps;
 const float frame = (stage->override_frame ? stage->frame : time);
 return stage->is_sequence ? frame : frame / fps - time_offset;
}


bool BKE_usd_stage_filepath_get(const Main* bmain,
                               USDStage* stage)
{
 BLI_strncpy(stage->resolved_filepath, stage->filepath, FILE_MAX);
 BLI_path_abs(stage->resolved_filepath, ID_BLEND_PATH(bmain, &stage->id));

 return BLI_exists(stage->resolved_filepath);
}


void add_usd_prim_path(ListBase* root, pxr::UsdPrim prim, int* index, int parent_index=-1) {
 void* path_void = MEM_callocN(sizeof(USDStagePrimPath), "USDStagePrimPath");
 USDStagePrimPath* path = static_cast<USDStagePrimPath*>(path_void);
 BLI_strncpy(path->path, prim.GetPath().GetText(), sizeof(path->path));

 path->parent_index = parent_index;
 const int prim_index = *index; // store for below

 // translate type from name to enum
 // can't switch on TfTokens =/
 //!TODO(kiki): Use the function currently in tree_element_id_usd_stage.cc
 auto type_name = prim.GetTypeName();
 if (type_name == USD_TK_XFORM) {
   path->type = USD_PRIM_XFORM;
 }
 else if (type_name == USD_TK_CURVE) {
   path->type = USD_PRIM_CURVE;
 }
 else if (type_name == USD_TK_MESH) {
   path->type = USD_PRIM_MESH;
 }
 else {
   path->type = USD_PRIM;
 }

 printf("\t|(%s) %s - %i\n",
        prim.GetTypeName().GetText(),
        prim.GetPath().GetText(),
        path->parent_index);

 BLI_addtail(root, path);
 *index += 1;

 // depth-first traversal; make sure we updated the index above
 auto children = prim.GetChildren();
 for (auto child: children) {
   add_usd_prim_path(root, child, index, prim_index);
 }
}


void BKE_usd_stage_eval(Main* bmain, USDStage* stage) {
 printf("BKE_usd_stage_eval start\n");

 if (!(stage->flags & USD_ST_NEEDS_REBUILD)) {
   // don't want to be checking
   // for files on disk every update
   printf("BKE_usd_stage_eval -- early out, no rebuild needed\n");
   return;
 }

 BLI_freelistN(&stage->object_paths);

 if (!BKE_usd_stage_filepath_get(bmain, stage)) {
   printf("BKE_usd_stage_eval -- couldn't find file\n");
   return;
 }

 /* Because we're using the cache, UsdStage::Open should
    either load or pull an existing ref pointer. */
 pxr::UsdStageRefPtr usd_stage = usd_stage_get_pxr_stage(stage);

 if (!usd_stage) {
   printf("BKE_usd_stage_eval -- unable to open %s\n", stage->resolved_filepath);
   stage->error = USD_ERR_UNABLE_TO_LOAD;
   return;
 }

 int index = 0;

 auto root = usd_stage->GetPseudoRoot();
 auto children = root.GetChildren();
 for (auto child: children) {
   add_usd_prim_path(&stage->object_paths, child, &index, -1);
 }

 /* If the root prim path is empty, set it to the stage's default prim. */
  if (stage->root_prim_path[0] == '\0') {
    pxr::UsdPrim default_prim = usd_stage->GetDefaultPrim();
    if (default_prim) {
      BLI_strncpy(stage->root_prim_path, default_prim.GetPath().GetText(), 4096);
    }
  }

 stage->flags &= ~USD_ST_NEEDS_REBUILD;
 stage->error = USD_ERR_NO_ERROR;

 printf("BKE_usd_stage_eval finished\n");
}


void* BKE_usd_stage_add(Main *bmain, const char *name)
{
 USDStage* usd_stage = (USDStage*)BKE_libblock_alloc(bmain, ID_USD, name, 0);

 usd_stage_init_data(&usd_stage->id);

 return usd_stage;
}


void BKE_usd_stage_copy_data(Main *bmain,
                            ID *id_dst,
                            const ID *id_src,
                            const int flag)
{
 USDStage* usd_stage_dst = (USDStage*)id_dst;
 const USDStage* usd_stage_src = (const USDStage* )id_src;

 //TODO(kiki): packfile support
 // if (usd_stage_src->packedfile) {
 //   usd_stage_dst->packedfile = BKE_packedfile_duplicate(usd_stage_src->packedfile);
 // }

 BLI_duplicatelist(&usd_stage_dst->object_paths, &usd_stage_src->object_paths);

 usd_stage_dst->mat = (Material **)MEM_dupallocN(usd_stage_src->mat);
}

USDStage* BKE_usd_stage_copy(Main *bmain, const USDStage* usd_stage)
{
 USDStage* usd_stage_copy = reinterpret_cast<USDStage*>(BKE_id_copy(bmain, &usd_stage->id));
 return usd_stage_copy;
}

static void usd_stage_make_local(Main *bmain, ID *id, const bool lib_local)
{
 //!TODO(kiki):
 // BKE_lib_id_make_local_generic(bmain, id, flags);
// BKE_id_make_local_generic(bmain, id, true, lib_local);
}

static void usd_stage_free_data(ID *id)
{
 USDStage* usd_stage = (USDStage*)id;
 BKE_animdata_free(&usd_stage->id, false);
 BKE_usd_stage_batch_cache_free(usd_stage);
 MEM_SAFE_FREE(usd_stage->mat);
 BLI_freelistN(&usd_stage->object_paths);

 if (usd_stage->runtime) {
   MEM_delete(reinterpret_cast<USDStage*>(usd_stage->runtime));
 }
}

//!TODO(kiki): idtypes refactor

IDTypeInfo IDType_ID_USD = {
    /* id_code */ ID_USD,
    /* id_filter */ FILTER_ID_USD,
    /* main_listbase_index */ INDEX_ID_USD,
    /* struct_size */ sizeof(USDStage),
    /* name */ "USDStage",
    /* name_plural */ "usd_stages",
    /* translation_context */ BLT_I18NCONTEXT_ID_USD_STAGE,
    /* flags */ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /* init_data */ usd_stage_init_data,
    /* copy_data */ BKE_usd_stage_copy_data,
    /* free_data */ usd_stage_free_data,
    /* make_local */ nullptr,

    /*foreach_id*/ nullptr,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ nullptr,
    /*blend_read_data*/ nullptr,
    /*blend_read_after_liblink*/ nullptr,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};


/* Sequence */

static int usd_stage_sequence_frame(const Depsgraph *depsgraph, const USDStage* usd_stage)
{
 //!TODO(kiki): support USD stage sequences?
 return 0;
}

static void usd_stage_filepath_get(const Main *bmain, const USDStage* usd_stage, char r_filepath[FILE_MAX])
{
 BLI_strncpy(r_filepath, usd_stage->filepath, FILE_MAX);
 BLI_path_abs(r_filepath, ID_BLEND_PATH(bmain, &usd_stage->id));

 int path_frame, path_digits;
 if (usd_stage->is_sequence && BLI_path_frame_get(r_filepath, &path_frame, &path_digits)) {
   char ext[32];
   BLI_path_frame_strip(r_filepath, ext, 32);
   BLI_path_frame(r_filepath, usd_stage->frame, path_digits, FILE_MAX);
   BLI_path_extension_ensure(r_filepath, FILE_MAX, ext);
 }
}

/* File Load */

bool BKE_usd_stage_is_loaded(const USDStage* usd_stage)
{
#ifdef WITH_USD
 /* Test if there is a file to load, or if already loaded. */
 //!TODO(kiki): poll USD structure?
 return (usd_stage->filepath[0] == '\0');
#else
 UNUSED_VARS(usd_stage);
 return true;
#endif
}

bool BKE_usd_stage_load(USDStage* usd_stage, Main *bmain)
{
#ifdef WITH_USD

 return true;
#else
 UNUSED_VARS(bmain, usd_stage);
 return true;
#endif
}

void BKE_usd_stage_unload(USDStage* usd_stage)
{
#ifdef WITH_USD
//  VolumeGridVector &grids = *usd_stage->runtime.grids;
//  if (grids.filepath[0] != '\0') {
//    const char *usd_stage_name = usd_stage->id.name + 2;
//    CLOG_INFO(&LOG, 1, "Volume %s: unload", usd_stage_name);
//    grids.clear_all();
//  }
#else
 UNUSED_VARS(usd_stage);
#endif
}

std::optional<blender::Bounds<blender::float3>> BKE_usd_stage_boundbox_get(const Object *ob)
{
 BLI_assert(ob->type == OB_USD_STAGE);

 //!TODO(kiki): fill from extents on boundable prims
 return std::nullopt;
}

bool BKE_usd_stage_is_y_up(const USDStage* usd_stage)
{
 //!TODO(kiki): Deterimine up axis from USD file
 UNUSED_VARS(usd_stage);

 return false;
}

/* Dependency Graph */

static USDStage* usd_stage_evaluate_modifiers(struct Depsgraph *depsgraph,
                                             struct Scene *scene,
                                             Object *object,
                                             USDStage* usd_stage_input)
{
  //!TODO(kiki): Modifier support
 return usd_stage_input;
}

void BKE_usd_stage_eval_geometry(struct Depsgraph *depsgraph, USDStage* usd_stage)
{
 /* TODO: can we avoid modifier re-evaluation when frame did not change? */
 int frame = usd_stage_sequence_frame(depsgraph, usd_stage);
 if (frame != usd_stage->frame) {
   BKE_usd_stage_unload(usd_stage);
   usd_stage->frame = frame;
 }

 /* Flush back to original. */
 if (DEG_is_active(depsgraph)) {
   USDStage* usd_stage_orig = (USDStage*)DEG_get_original_id(&usd_stage->id);
   usd_stage_orig->frame = usd_stage->frame;
 }
}


void BKE_usd_stage_data_update(struct Depsgraph *depsgraph, struct Scene *scene, Object *object)
{
 /* Free any evaluated data and restore original data. */
 BKE_object_free_derived_caches(object);

 USDStage* usd_stage = (USDStage*)object->data;

 /* re-cache prim information if needed */
 // commenting this out here-- it might not be possible and
 // we may have to move this into the depsgraph eval
 // BKE_usd_stage_eval((usd_stage, depsgraph->Main);

 /* Evaluate modifiers. */
 USDStage* usd_stage_eval = usd_stage_evaluate_modifiers(depsgraph, scene, object, usd_stage);

 /* Assign evaluated object. */
 const bool is_owned = (usd_stage != usd_stage_eval);
 //!TODO(kiki): not in 2.81?
 // BKE_object_eval_assign_data(object, &usd_stage_eval->id, is_owned);
}


/* Draw Cache */
void (*BKE_usd_stage_batch_cache_dirty_tag_cb)(USDStage* usd_stage, int mode) = NULL;
void (*BKE_usd_stage_batch_cache_free_cb)(USDStage* usd_stage) = NULL;

void BKE_usd_stage_batch_cache_dirty_tag(USDStage* usd_stage, int mode)
{
 //  if (usd_stage->batch_cache) {
 //    BKE_usd_stage_batch_cache_dirty_tag_cb(usd_stage, mode);
 //  }
}

void BKE_usd_stage_batch_cache_free(USDStage* usd_stage)
{
 //  if (usd_stage->batch_cache) {
 //    BKE_usd_stage_batch_cache_free_cb(usd_stage);
 //  }
}

/* Transformation from index space to object space. */

/*void BKE_usd_stage_transform_matrix(const VolumeGrid *volume_grid, float mat[4][4])
{
#ifdef WITH_USD
 const openvdb::GridBase::Ptr &grid = volume_grid->vdb;
 const openvdb::math::Transform &transform = grid->transform();

 // Perspective not supported for now, getAffineMap() will leave out the
 // perspective part of the transform.
 openvdb::math::Mat4f matrix = transform.baseMap()->getAffineMap()->getMat4();
 // Blender column-major and OpenVDB right-multiplication conventions match.
 for (int col = 0; col < 4; col++) {
   for (int row = 0; row < 4; row++) {
     mat[col][row] = matrix(col, row);
   }
 }
#else
 unit_m4(mat);
 UNUSED_VARS(volume_grid);
#endif
}*/

/* Stage Editing */

USDStage* BKE_usd_stage_new_for_eval(const USDStage* usd_stage_src)
{
 USDStage* usd_stage_dst = (USDStage*)BKE_id_new_nomain(ID_USD, NULL);

 STRNCPY(usd_stage_dst->id.name, usd_stage_src->id.name);
 usd_stage_dst->mat = (Material**)MEM_dupallocN(usd_stage_src->mat);
 usd_stage_dst->totcol = usd_stage_src->totcol;

 return usd_stage_dst;
}

USDStage* BKE_usd_stage_copy_for_eval(USDStage* usd_stage_src, bool reference)
{
 int flags = LIB_ID_COPY_LOCALIZE;

// if (reference) {
//   flags |= LIB_ID_COPY_CD_REFERENCE;
// }

 USDStage* result;
 BKE_id_copy_ex(NULL, &usd_stage_src->id, (ID **)&result, flags);
 result->filepath[0] = '\0';

 return result;
}

void BKE_usd_stage_active_prim_set(struct USDStage* stage, const std::string path) {
  if (path.length() > 0) {
    BLI_strncpy(stage->active_prim_path, path.c_str(), 4096);

    fprintf(stderr, "Active Prim Set: %s\n", path.c_str());

    //!TODO(kiki): fill in the other info
  }
  else {
    stage->active_prim_path[0] = '\0';
    stage->active_prim_variants = { nullptr, nullptr };
    stage->active_prim_type = 0;
    stage->active_prim_variant[0] = '\0';
    stage->active_prim_purpose = 0;

    memcpy(stage->active_prim_matrix_local, matrix_identity, sizeof(matrix_identity));
    memcpy(stage->active_prim_matrix_world, matrix_identity, sizeof(matrix_identity));
  }
}
