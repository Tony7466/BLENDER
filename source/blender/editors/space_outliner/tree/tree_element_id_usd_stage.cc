/* SPDX-FileCopyrightText: 2023 Blender Authors
*
* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
* \ingroup spoutliner
*/

#include "DNA_object_types.h"
#include "DNA_outliner_types.h"
#include "DNA_texture_types.h"
#include "DNA_usd_stage_types.h"

#include "../outliner_intern.hh"
#include "tree_display.hh"
#include "tree_element_id_usd_stage.hh"

#include <pxr/pxr.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usdGeom/mesh.h>
#include "pxr/usd/usd/prim.h"
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usd/stageCacheContext.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/usd/usdSkel/bindingAPI.h>
#include <pxr/usd/usdSkel/skeletonQuery.h>

#include "usd_stage_utils.hh"


namespace blender::ed::outliner {

/* ====================================================================== */

class TreeElementUsdPrimPrivate {
 public:
  pxr::SdfPath path;
};

TreeElementUsdPrim::TreeElementUsdPrim(TreeElement &legacy_te)
    : AbstractTreeElement(legacy_te), private_(nullptr)
{
  BLI_assert(legacy_te_.store_elem->type == TSE_USD_PRIM);

  set_label("");
  create_private_data();
}

TreeElementUsdPrim::~TreeElementUsdPrim() {
  if (private_) {
    MEM_delete(private_);
  }
}

void TreeElementUsdPrim::create_private_data() {
  if (!private_) {
    private_ = MEM_new<TreeElementUsdPrimPrivate>("TreeElementUsdPrim::private_");
  }
}

void TreeElementUsdPrim::set_icon(const BIFIconID icon)
{
  icon_ = icon;
}

std::optional<BIFIconID> TreeElementUsdPrim::get_icon() const
{
  return icon_;
}

void TreeElementUsdPrim::set_path(const std::string path) {
  private_->path = pxr::SdfPath(path);
}

const std::string TreeElementUsdPrim::get_path() const {
  return private_->path.GetString();
}

void TreeElementUsdPrim::set_label(const std::string label) {
  /* The draw string is actually accessed via #TreeElement.name, so make
   * sure this always points to our string. */
  label_ = label;
  legacy_te_.name = label_.c_str();
}

const std::string TreeElementUsdPrim::get_label() const {
  return label_;
}


/* ====================================================================== */
class TreeElementIDUSDStagePrivate {
  //!TODO(kiki): add stuff
};

/* ====================================================================== */

TreeElementIDUSDStage::TreeElementIDUSDStage(TreeElement &legacy_te, Object& ob)
   : TreeElementID(legacy_te, ob.id), stage(reinterpret_cast<USDStage*>(&ob.id)), private_(nullptr)
{
}

TreeElementIDUSDStage::~TreeElementIDUSDStage() {
  if (private_) {
    MEM_delete(private_);
  }
}

void TreeElementIDUSDStage::expand(SpaceOutliner &space_outliner) const
{
  expand_animation_data(stage->adt);
  expand_prim_hierarchy(space_outliner);
}

static short outliner_prim_type_to_enum(const pxr::UsdPrim prim) {
  pxr::UsdGeomMesh mesh(prim);
  if (bool(mesh)) {
    return TSE_USD_PRIM_MESH;
  }

  pxr::UsdGeomBasisCurves basis_curves(prim);
  pxr::UsdGeomCurves curves(prim);
  if (bool(basis_curves) || bool(curves)) {
    return TSE_USD_PRIM_CURVE;
  }

  //!TODO(kiki) Add support for the skeleton joints
  pxr::UsdSkelSkeleton skeleton(prim);
  if (bool(skeleton)) {
    return TSE_USD_PRIM_SKEL_ROOT;
  }

  /* This needs to be last in case someone added xforms to a leaf node. */
  pxr::UsdGeomXformable xformable(prim);
  if (xformable) {
    return TSE_USD_PRIM_XFORM;
  }

  return TSE_USD_PRIM;
}

static short outliner_type_to_icon(const short type) {
  switch (type) {
    case TSE_USD_PRIM:
      return ICON_X;
    case TSE_USD_PRIM_XFORM:
      return ICON_OUTLINER_DATA_EMPTY;
    case TSE_USD_PRIM_SKEL_ROOT:
      return ICON_ARMATURE_DATA;
    case TSE_USD_PRIM_MESH:
      return ICON_MESH_DATA;
    case TSE_USD_PRIM_CURVE:
      return ICON_CURVES_DATA;
    case TSE_USD_PRIM_JOINT:
      return ICON_BONE_DATA;
//    case TSE_USD_STAGE:
//    case TSE_USD_ROOT_PRIM:
//    case TSE_USD_PRIM_UNKNOWN:
    default:
      return ICON_X;
  }
}


static void outliner_add_prim(SpaceOutliner *space_outliner,
                              ListBase *tree,
                              ID *id,
                              pxr::UsdPrim prim,
                              TreeElement *parent,
                              int *a)
{
  if (!prim) {
    return;
  }

  const short outliner_type = outliner_prim_type_to_enum(prim);
  const short outliner_icon = outliner_type_to_icon(outliner_type);

  TreeElement *te = AbstractTreeDisplay::add_element(
      space_outliner, tree, id, nullptr, parent, TSE_USD_PRIM, *a);

  TreeElementUsdPrim *prim_el = tree_element_cast<TreeElementUsdPrim>(te);
  prim_el->set_label(prim.GetName());
  prim_el->set_path(prim.GetPath().GetString());
  prim_el->set_icon(outliner_icon);

  (*a)++;

  for (pxr::UsdPrim child_prim: prim.GetChildren()) {
    outliner_add_prim(space_outliner, &te->subtree, id, child_prim, te, a);
  }
}


void TreeElementIDUSDStage::expand_prim_hierarchy(SpaceOutliner &space_outliner) const
{
  /* Starting from root prim, recursive; make sure the path exists first. */
  pxr::UsdStageRefPtr usd_stage = get_pxr_stage_for_path(stage->resolved_filepath);
  if (!usd_stage) {
    return;
  }

  pxr::SdfPath root_path(stage->root_prim_path);
  pxr::UsdPrim root_prim = usd_stage->GetPrimAtPath(root_path);
  if (!root_prim) {
    stage->error = USD_ERR_INVALID_ROOT_PRIM;
    return;
  }

  int index = 0;
  outliner_add_prim(&space_outliner, &legacy_te_.subtree, &stage->id, root_prim, &legacy_te_, &index);
}

}  // namespace blender::ed::outliner
