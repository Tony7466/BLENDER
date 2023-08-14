/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#pragma once

#include "tree_element.hh"

struct GpencilModifierData;
struct ModifierData;
struct Object;

namespace blender::ed::outliner {

class TreeElementModifierBase final : public AbstractTreeElement {
  Object &object_;

 public:
  TreeElementModifierBase(TreeElement &legacy_te, Object &object);
  void expand(SpaceOutliner &) const override;
};

class TreeElementModifier final : public AbstractTreeElement {
  /* Not needed right now, avoid unused member variable warning. */
  Object &object_;
  std::variant<ModifierData *, GpencilModifierData *> md_;

 public:
  TreeElementModifier(TreeElement &legacy_te,
                      Object &object,
                      std::variant<ModifierData *, GpencilModifierData *> md);
  void expand(SpaceOutliner &) const override;
};

}  // namespace blender::ed::outliner
