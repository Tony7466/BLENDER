/*
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
*/

/** \file
* \ingroup DNA
*/

#ifndef __DNA_USD_STAGE_DEFAULTS_H__
#define __DNA_USD_STAGE_DEFAULTS_H__

/* Struct members on own line. */
/* clang-format off */

/* -------------------------------------------------------------------- */
/** \name USDStage Struct
* \{ */

#define _DNA_DEFAULT_USDStage \
 { \
   .object_paths ={NULL, NULL}, \
   .flags = 0, \
   .is_sequence = false, \
   .override_frame = 0, \
   .forward_axis = 0, \
   .up_axis = 1, \
   .error = 0, \
   .scale = 1.0f, \
   .frame = 0.0f, \
   .frame_offset = 0.0f, \
   .filepath[0] = '\0', \
   .root_prim_path[0] = '\0', \
   .mat = NULL, \
   .totcol = 0,               \
   .active_prim_path[0] = '\0', \
   .active_prim_variant[0] = '\0', \
   .active_prim_variants = {NULL, NULL}, \
   .active_prim_type = 0, \
   .active_prim_purpose = 0, \
   .runtime = NULL \
}

/** \} */

/* clang-format on */

#endif /* __DNA_USD_STAGE_DEFAULTS_H__ */
