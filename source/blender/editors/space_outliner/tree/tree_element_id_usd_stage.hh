/* SPDX-FileCopyrightText: 2023 Blender Authors
*
* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
* \ingroup spoutliner
*/

#pragma once

#include "tree_element_id.hh"
#include "BKE_usd_stage.hh"

namespace blender::ed::outliner {

class TreeElementUsdPrimPrivate;


/**
 * Prim path tree element.  Started copying from TreeElementLabel, but
 * there are a few extra bits needed for modifying the active prim, etc.
 */
class TreeElementUsdPrim final : public AbstractTreeElement {
 private:
  std::string label_;
  BIFIconID icon_ = ICON_NONE;
  class TreeElementUsdPrimPrivate *private_;

  void create_private_data();

 public:
  TreeElementUsdPrim(TreeElement &legacy_te);
  ~TreeElementUsdPrim();

  void set_icon(BIFIconID icon);
  std::optional<BIFIconID> get_icon() const override;

  void set_label(const std::string label);
  const std::string get_label() const;

  void set_path(const std::string path);
  const std::string get_path() const;
};


class TreeElementIDUSDStagePrivate;

/*
 * Support for viewing USDStage objects in the outliner, including
 * child prims from the chosen root prim as hierarchical data.
 *
 * The goal is not necessarily to display everything! For now,
 * I'm picking a subset that I think make sense.  We can add
 * to that as the feature grows.
 */
class TreeElementIDUSDStage final : public TreeElementID {
 USDStage* stage;
 class TreeElementIDUSDStagePrivate *private_;

public:
 TreeElementIDUSDStage(TreeElement &legacy_te, Object& ob);
 ~TreeElementIDUSDStage();

 void expand(SpaceOutliner &space_outliner) const override;

private:
 void expand_prim_hierarchy(SpaceOutliner &space_outliner) const;
};


}  // namespace blender::ed::outliner
