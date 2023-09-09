/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include "DNA_ID.h"
#include "DNA_space_types.h"

#include "BLI_listbase.h"
#include "BLI_utildefines.h"

#include "BKE_main.h"

#include "../outliner_intern.hh"
#include "common.hh"
#include "tree_display.hh"
#include "tree_element.hh"

namespace blender::ed::outliner {

TreeDisplayBakes::TreeDisplayBakes(SpaceOutliner &space_outliner)
    : AbstractTreeDisplay(space_outliner)
{
}

ListBase TreeDisplayBakes::build_tree(const TreeSourceData & /*source_data*/)
{
  ListBase tree{};
  this->add_element(&tree, nullptr, (void *)"Hello World", nullptr, TSE_GENERIC_LABEL, 0);
  return tree;
}

}  // namespace blender::ed::outliner
