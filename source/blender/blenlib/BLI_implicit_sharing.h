/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#ifdef __cplusplus

namespace blender {
class ImplicitSharingInfo;
}
using ImplicitSharingInfoHandle = blender::ImplicitSharingInfo;

#else

typedef struct ImplicitSharingInfoHandle ImplicitSharingInfoHandle;

#endif
