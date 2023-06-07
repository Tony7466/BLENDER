/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include "DNA_listBase.h"
#include "DNA_outliner_types.h"
#include "DNA_texture_types.h"

#include "../outliner_intern.hh"

#include "tree_element_id_tex.hh"

namespace blender::ed::outliner {

TreeElementIDTex::TreeElementIDTex(TreeElement &legacy_te, Tex &tex)
: TreeElementID(legacy_te, tex.id), tex_(tex)
{
}

bool TreeElementIDTex::isExpandValid() const
{
  return true;
}

void TreeElementIDTex::expand(SpaceOutliner &space_outliner) const
{
  expand_animation_data(space_outliner, tex_.adt);
  
  expandImage(space_outliner);
}

void TreeElementIDTex::expandImage(SpaceOutliner &space_outliner) const
{
  outliner_add_element(&space_outliner, &legacy_te_.subtree, tex_.ima, &legacy_te_, TSE_SOME_ID, 0);
}

}  // namespace blender::ed::outliner

