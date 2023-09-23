/*This is a copy of the lineart header file.
I couldent get my operators to be recognized by object_ops.c
anymore, for some reason. I had no idea how it previously worked, has
probably something to do with cmake and can't figure it out.
Well, it's not that bad, it's just one more file.
I will gladly accept any suggestion on how to get rid of this file.
Thanks.*/

/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#pragma once

#include "DNA_windowmanager_types.h"

/* Operator types should be in exposed header. */

#ifdef __cplusplus
extern "C" {
#endif

void GPENCIL_OT_gpencilsurdeform_bind(struct wmOperatorType *ot);
void GPENCIL_OT_gpencilsurdeform_unbind(struct wmOperatorType *ot);
void GPENCIL_OT_gpencilsurdeform_bake(struct wmOperatorType *ot);
void GPENCIL_OT_gpencilsurdeform_fillrange(struct wmOperatorType *ot);

void WM_operatortypes_gpencilsurdeform(void);

#ifdef __cplusplus
}
#endif
