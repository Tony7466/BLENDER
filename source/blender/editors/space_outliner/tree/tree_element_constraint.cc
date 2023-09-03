/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include "DNA_constraint_types.h"
#include "DNA_object_types.h"
#include "DNA_outliner_types.h"

#include "BLT_translation.h"

#include "../outliner_intern.hh"

#include "tree_element_constraint.hh"

namespace blender::ed::outliner {

TreeElementConstraintBase::TreeElementConstraintBase(TreeElement &legacy_te, Object & /* object */)
    : AbstractTreeElement(legacy_te) /* , object_(object) */
{
  BLI_assert(legacy_te.store_elem->type == TSE_CONSTRAINT_BASE);
  legacy_te.name = IFACE_("Constraints");
}

std::optional<BIFIconID> TreeElementConstraintBase::get_icon() const
{
  return ICON_CONSTRAINT;
}

TreeElementConstraint::TreeElementConstraint(TreeElement &legacy_te,
                                             Object & /* object */,
                                             bConstraint &con)
    : AbstractTreeElement(legacy_te), /* object_(object), */ con_(con)
{
  BLI_assert(legacy_te.store_elem->type == TSE_CONSTRAINT);
  legacy_te.name = con_.name;
  legacy_te.directdata = &con_;
}

std::optional<BIFIconID> TreeElementConstraint::get_icon() const
{
  switch ((eBConstraint_Types)con_.type) {
    case CONSTRAINT_TYPE_CAMERASOLVER:
      return ICON_CON_CAMERASOLVER;
    case CONSTRAINT_TYPE_FOLLOWTRACK:
      return ICON_CON_FOLLOWTRACK;
    case CONSTRAINT_TYPE_LOCLIKE:
      return ICON_CON_LOCLIKE;
    case CONSTRAINT_TYPE_ROTLIKE:
      return ICON_CON_ROTLIKE;
    case CONSTRAINT_TYPE_SIZELIKE:
      return ICON_CON_SIZELIKE;
    case CONSTRAINT_TYPE_TRANSLIKE:
      return ICON_CON_TRANSLIKE;
    case CONSTRAINT_TYPE_DISTLIMIT:
      return ICON_CON_DISTLIMIT;
    case CONSTRAINT_TYPE_LOCLIMIT:
      return ICON_CON_LOCLIMIT;
    case CONSTRAINT_TYPE_ROTLIMIT:
      return ICON_CON_ROTLIMIT;
    case CONSTRAINT_TYPE_SIZELIMIT:
      return ICON_CON_SIZELIMIT;
    case CONSTRAINT_TYPE_SAMEVOL:
      return ICON_CON_SAMEVOL;
    case CONSTRAINT_TYPE_TRANSFORM:
      return ICON_CON_TRANSFORM;
    case CONSTRAINT_TYPE_TRANSFORM_CACHE:
      return ICON_CON_TRANSFORM_CACHE;
    case CONSTRAINT_TYPE_CLAMPTO:
      return ICON_CON_CLAMPTO;
    case CONSTRAINT_TYPE_DAMPTRACK:
      return ICON_CON_TRACKTO;
    case CONSTRAINT_TYPE_KINEMATIC:
      return ICON_CON_KINEMATIC;
    case CONSTRAINT_TYPE_LOCKTRACK:
      return ICON_CON_LOCKTRACK;
    case CONSTRAINT_TYPE_SPLINEIK:
      return ICON_CON_SPLINEIK;
    case CONSTRAINT_TYPE_STRETCHTO:
      return ICON_CON_STRETCHTO;
    case CONSTRAINT_TYPE_TRACKTO:
      return ICON_CON_TRACKTO;
    case CONSTRAINT_TYPE_ACTION:
      return ICON_CON_ACTION;
    case CONSTRAINT_TYPE_ARMATURE:
      return ICON_CON_ARMATURE;
    case CONSTRAINT_TYPE_CHILDOF:
      return ICON_CON_CHILDOF;
    case CONSTRAINT_TYPE_MINMAX:
      return ICON_CON_FLOOR;
    case CONSTRAINT_TYPE_FOLLOWPATH:
      return ICON_CON_FOLLOWPATH;
    case CONSTRAINT_TYPE_PIVOT:
      return ICON_CON_PIVOT;
    case CONSTRAINT_TYPE_SHRINKWRAP:
      return ICON_CON_SHRINKWRAP;

    default:
      return ICON_DOT;
  }
}

}  // namespace blender::ed::outliner
