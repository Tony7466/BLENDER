/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include "DNA_gpencil_modifier_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_outliner_types.h"

#include "BLI_listbase.h"

#include "BLT_translation.h"

#include "../outliner_intern.hh"

#include "tree_element_modifier.hh"

namespace blender::ed::outliner {

TreeElementModifierBase::TreeElementModifierBase(TreeElement &legacy_te, Object &object)
    : AbstractTreeElement(legacy_te), object_(object)
{
  legacy_te.name = IFACE_("Modifiers");
}

void TreeElementModifierBase::expand(SpaceOutliner &space_outliner) const
{
  int index;
  LISTBASE_FOREACH_INDEX (ModifierData *, md, &object_.modifiers, index) {
    ModifierCreateElementData md_data = {&object_, md};

    outliner_add_element(
        &space_outliner, &legacy_te_.subtree, &md_data, &legacy_te_, TSE_MODIFIER, index);
  }
  LISTBASE_FOREACH_INDEX (GpencilModifierData *, md, &object_.greasepencil_modifiers, index) {
    ModifierCreateElementData md_data = {&object_, md};

    outliner_add_element(
        &space_outliner, &legacy_te_.subtree, &md_data, &legacy_te_, TSE_MODIFIER, index);
  }
}

TreeElementModifier::TreeElementModifier(TreeElement &legacy_te,
                                         Object &object,
                                         std::variant<ModifierData *, GpencilModifierData *> md)
    : AbstractTreeElement(legacy_te), object_(object), md_(md)
{
  if (std::holds_alternative<ModifierData *>(md_)) {
    ModifierData *md = std::get<ModifierData *>(md_);
    legacy_te.name = md->name;
    legacy_te.directdata = md;
  }
  else {
    GpencilModifierData *md = std::get<GpencilModifierData *>(md_);
    legacy_te.name = md->name;
    legacy_te.directdata = md;
  }
}

void TreeElementModifier::expand(SpaceOutliner &space_outliner) const
{
  if (std::holds_alternative<ModifierData *>(md_)) {
    ModifierData *md = std::get<ModifierData *>(md_);
    if (md->type == eModifierType_Lattice) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((LatticeModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
    else if (md->type == eModifierType_Curve) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((CurveModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
    else if (md->type == eModifierType_Armature) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((ArmatureModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
    else if (md->type == eModifierType_Hook) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((HookModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
    else if (md->type == eModifierType_ParticleSystem) {
      ParticleSystem *psys = ((ParticleSystemModifierData *)md)->psys;

      ParticleSystemElementCreateData psys_data = {&object_, psys};

      outliner_add_element(
          &space_outliner, &legacy_te_.subtree, &psys_data, &legacy_te_, TSE_LINKED_PSYS, 0);
    }
  }
  else {
    GpencilModifierData *md = std::get<GpencilModifierData *>(md_);
    if (md->type == eGpencilModifierType_Armature) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((ArmatureGpencilModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
    else if (md->type == eGpencilModifierType_Hook) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((HookGpencilModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
    else if (md->type == eGpencilModifierType_Lattice) {
      outliner_add_element(&space_outliner,
                           &legacy_te_.subtree,
                           ((LatticeGpencilModifierData *)md)->object,
                           &legacy_te_,
                           TSE_LINKED_OB,
                           0);
    }
  }
}

}  // namespace blender::ed::outliner
