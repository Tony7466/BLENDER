/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#pragma once

#include "tree_element_id.hh"

namespace blender::ed::outliner {

class TreeElementIDTex final : public TreeElementID {
  Tex &tex_;
  
public:
  TreeElementIDTex(TreeElement &legacy_te, Tex &tex);
  
  void expand(SpaceOutliner &) const override;
  bool isExpandValid() const override;
  
private:
  void expandImage(SpaceOutliner &) const;
  
};

}  // namespace blender::ed::outliner
