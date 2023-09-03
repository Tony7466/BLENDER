/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#pragma once

#include "tree_element.hh"

struct Object;
struct bActionGroup;

namespace blender::ed::outliner {

class TreeElementPoseGroupBase final : public AbstractTreeElement {
  Object &object_;

 public:
  TreeElementPoseGroupBase(TreeElement &legacy_te, Object &object);
  void expand(SpaceOutliner &) const override;
  std::optional<BIFIconID> get_icon() const override;
};

class TreeElementPoseGroup final : public AbstractTreeElement {
  /* Not needed right now, avoid unused member variable warning. */
  // Object &object_;
  bActionGroup &agrp_;

 public:
  TreeElementPoseGroup(TreeElement &legacy_te, Object &object, bActionGroup &agrp);
  std::optional<BIFIconID> get_icon() const override;
};

}  // namespace blender::ed::outliner
