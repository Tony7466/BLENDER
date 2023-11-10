/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 * SPDX-FileCopyrightText: 2003-2009 Blender Authors
 * SPDX-FileCopyrightText: 2005-2006 Peter Schlaile <peter [at] schlaile [dot] de>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <cstring>

#include "MEM_guardedalloc.h"

#include "DNA_anim_types.h"
#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"
#include "DNA_sound_types.h"

#include "BLI_listbase.h"
#include "BLI_string.h"

#include "BKE_fcurve.h"
#include "BKE_main.hh"
#include "BKE_movieclip.h"
#include "BKE_scene.h"
#include "BKE_sound.h"

#include "SEQ_clipboard.hh"
#include "SEQ_select.hh"

#include "sequencer.hh"

#ifdef WITH_AUDASPACE
#  include <AUD_Special.h>
#endif

/* -------------------------------------------------------------------- */
/* Manage pointers in the clipboard.
 * note that these pointers should _never_ be access in the sequencer,
 * they are only for storage while in the clipboard
 * notice 'newid' is used for temp pointer storage here, validate on access (this is safe usage,
 * since those data-blocks are fully out of Main lists).
 */

