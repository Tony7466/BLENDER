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

/* \file DNA_usd_stage_types.h
*  \ingroup DNA
*/

#ifndef __DNA_USD_STAGE_TYPES_H__
#define __DNA_USD_STAGE_TYPES_H__

#include "DNA_ID.h"
#include "DNA_customdata_types.h"


/* Not used below because makesdna can't parse defines. */
#define USD_MAX_PATH 4096


/*
 * !TODO(kiki):
 * Michael has a very similar setup for path searching available in
 * usd_capi_import.cc-- gather_objects_paths(). However, that struct only
 * carries the path itself and not the extra flags here.
 *
 * I'm sticking with this second structure for now while I sort out some
 * implementation details.  It's possible that in the end I will remove
 * this structure and use Michael's simpler one, but time will tell.
 */
typedef struct USDStagePrimPath {
 struct USDStagePrimPath *next, *prev;
 char path[4096];
 int type;
 int flags;
 int parent_index;
 int padding[1];
} USDStagePrimPath;


enum {
 USD_PATH_ACTIVE  = (1 << 0),
 USD_PATH_LOADED  = (1 << 1),
 USD_PATH_VISIBLE = (1 << 2),
};


enum {
 USD_STAGE = 50,
 USD_ROOT_PRIM,
 USD_PRIM,
 USD_PRIM_XFORM,
 USD_PRIM_MESH,
 USD_PRIM_CURVE,
 USD_PRIM_JOINT,
 USD_PRIM_UNKNOWN,
};

enum {
  USD_PURPOSE_NONE   = 0,
  USD_PURPOSE_GUIDE  = 1,
  USD_PURPOSE_PROXY  = 2,
  USD_PURPOSE_RENDER = 3
};


typedef struct USDStage {
 ID id;
 struct AnimData *adt; /* animation data (must be immediately after id) */

 /* Prim paths inside the USD file that can bet referenced by this USDStage. */
 ListBase object_paths;

 int flags;
 char is_sequence;
 char override_frame;
 char forward_axis;
 char up_axis;
 char error;

 char _pad1[3];

 /* Overall scale modifier to apply to geometry / prims from the stage file */
 float scale;
 /* The frame/time to lookup in the cache file. */
 float frame;
 /* The frame offset to subtract. */
 float frame_offset;

 /* 1024 = FILE_MAX. */
 char filepath[1024];
 char resolved_filepath[1024];

 /* Root prim to display -- same size as max path in USD. */
 char root_prim_path[4096];

 /* Material */
 struct Material **mat;
 short totcol;
 short _pad2[3];

 /* Active Prim */
 char active_prim_path[4096];
 char active_prim_variant[128];
 ListBase active_prim_variants;
 float active_prim_matrix_local[4][4];
 float active_prim_matrix_world[4][4];
 short active_prim_purpose;
 short active_prim_type;
 short _pad3[2];

 /* Runtime */
 void *runtime;

 /* Custom Data */
 //!TODO(kiki): do we need these?
 // struct CustomData pdata;
 // struct CustomData cdata;

#ifdef __cplusplus


#endif // __cplusplus

} USDStage;

/* stage flags */
enum {
 USD_ST_NEEDS_REBUILD = (1 << 0),
 USD_ST_CACHED        = (1 << 1),
 USD_ST_EXPAND        = (1 << 2),
 USD_ST_TRAVERSE_ALL  = (1 << 3),
};

/* error codes */
enum {
 USD_ERR_NO_ERROR = 0,
 USD_ERR_NO_FILE,
 USD_ERR_UNABLE_TO_LOAD,
 USD_ERR_INVALID_ROOT_PRIM,
};

/* Only one material supported currently. */
#define USD_STAGE_MATERIAL_NR 1

#endif /* __DNA_USD_STAGE_TYPES_H__ */
