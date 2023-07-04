/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include "DNA_armature_types.h"
#include "DNA_outliner_types.h"

#include "../outliner_intern.hh"

#include "tree_element_bone.hh"

namespace blender::ed::outliner {

TreeElementBone::TreeElementBone(TreeElement &legacy_te, Bone &bone)
    : AbstractTreeElement(legacy_te)
{
  BLI_assert(legacy_te.store_elem->type == TSE_BONE);
  legacy_te.name = bone.name;
  legacy_te.directdata = &bone;
}

}  // namespace blender::ed::outliner
