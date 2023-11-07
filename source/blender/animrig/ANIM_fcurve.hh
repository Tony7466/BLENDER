/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 *
 * \brief Functions to modify FCurves.
 */

#include "DNA_anim_types.h"
#include "RNA_types.hh"

struct AnimData;
struct FCurve;
struct bAction;

namespace blender::animrig {

/**
 * Get (or add relevant data to be able to do so) F-Curve from the given Action,
 * for the given Animation Data block. This assumes that all the destinations are valid.
 */
FCurve *action_fcurve_ensure(Main *bmain,
                             bAction *act,
                             const char group[],
                             PointerRNA *ptr,
                             const char rna_path[],
                             int array_index);

/**
 * Find the F-Curve from the given Action. This assumes that all the destinations are valid.
 */
FCurve *action_fcurve_find(bAction *act, const char rna_path[], int array_index);

/**
 * \note The caller needs to run #BKE_nla_tweakedit_remap to get NLA relative frame.
 *       The caller should also check #BKE_fcurve_is_protected before keying.
 */
bool delete_keyframe_fcurve(AnimData *adt, FCurve *fcu, float cfra);

/**
 * \brief Lesser Key-framing API call.
 *
 * Use this when validation of necessary animation data isn't necessary as it already
 * exists, and there is a #BezTriple that can be directly copied into the array.
 *
 * This function adds a given #BezTriple to an F-Curve. It will allocate
 * memory for the array if needed, and will insert the #BezTriple into a
 * suitable place in chronological order.
 *
 * \note any recalculate of the F-Curve that needs to be done will need to be done by the caller.
 */
int insert_bezt_fcurve(FCurve *fcu, const BezTriple *bezt, eInsertKeyFlags flag);

/**
 * \brief Main Key-framing API call.
 *
 * Use this when validation of necessary animation data isn't necessary as it
 * already exists. It will insert a keyframe using the current value being keyframed.
 * Returns the index at which a keyframe was added (or -1 if failed).
 *
 * This function is a wrapper for #insert_bezt_fcurve(), and should be used when
 * adding a new keyframe to a curve, when the keyframe doesn't exist anywhere else yet.
 * It returns the index at which the keyframe was added.
 *
 * \param keyframe_type: The type of keyframe (#eBezTriple_KeyframeType).
 * \param flag: Optional flags (#eInsertKeyFlags) for controlling how keys get added
 * and/or whether updates get done.
 */
int insert_vert_fcurve(
    FCurve *fcu, float x, float y, eBezTriple_KeyframeType keyframe_type, eInsertKeyFlags flag);

}  // namespace blender::animrig
