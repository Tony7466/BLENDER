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

#include "BKE_modifier.h"

#include "../outliner_intern.hh"

#include "tree_element_modifier.hh"

namespace blender::ed::outliner {

TreeElementModifierBase::TreeElementModifierBase(TreeElement &legacy_te, Object &object)
    : AbstractTreeElement(legacy_te), object_(object)
{
  legacy_te.name = IFACE_("Modifiers");
}

void TreeElementModifierBase::expand(SpaceOutliner & /*space_outliner*/) const
{
  int index;
  LISTBASE_FOREACH_INDEX (ModifierData *, md, &object_.modifiers, index) {
    ModifierDataStoreElem md_store(md);

    add_element(&legacy_te_.subtree, &object_.id, &md_store, &legacy_te_, TSE_MODIFIER, index);
  }
  LISTBASE_FOREACH_INDEX (GpencilModifierData *, md, &object_.greasepencil_modifiers, index) {
    ModifierDataStoreElem md_store(md);

    add_element(&legacy_te_.subtree, &object_.id, &md_store, &legacy_te_, TSE_MODIFIER, index);
  }
}

std::optional<BIFIconID> TreeElementModifierBase::get_icon() const
{
  return ICON_MODIFIER_DATA;
}

TreeElementModifier::TreeElementModifier(TreeElement &legacy_te,
                                         Object &object,
                                         ModifierDataStoreElem &md)
    : AbstractTreeElement(legacy_te), object_(object), md_(md)
{
  if (md_.type == MODIFIER_TYPE) {
    legacy_te.name = md_.md->name;
    legacy_te.directdata = md_.md;
  }
  if (md_.type == GPENCIL_MODIFIER_TYPE) {
    legacy_te.name = md_.gp_md->name;
    legacy_te.directdata = md_.gp_md;
  }
}

void TreeElementModifier::expand(SpaceOutliner & /*space_outliner*/) const
{
  if (md_.type == MODIFIER_TYPE) {
    ModifierData *md = md_.md;
    if (md->type == eModifierType_Lattice) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((LatticeModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
    else if (md->type == eModifierType_Curve) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((CurveModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
    else if (md->type == eModifierType_Armature) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((ArmatureModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
    else if (md->type == eModifierType_Hook) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((HookModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
    else if (md->type == eModifierType_ParticleSystem) {
      ParticleSystem *psys = ((ParticleSystemModifierData *)md)->psys;

      add_element(&legacy_te_.subtree, &object_.id, psys, &legacy_te_, TSE_LINKED_PSYS, 0);
    }
  }
  if (md_.type == GPENCIL_MODIFIER_TYPE) {
    GpencilModifierData *md = md_.gp_md;
    if (md->type == eGpencilModifierType_Armature) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((ArmatureGpencilModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
    else if (md->type == eGpencilModifierType_Hook) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((HookGpencilModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
    else if (md->type == eGpencilModifierType_Lattice) {
      add_element(&legacy_te_.subtree,
                  reinterpret_cast<ID *>(((LatticeGpencilModifierData *)md)->object),
                  nullptr,
                  &legacy_te_,
                  TSE_LINKED_OB,
                  0);
    }
  }
}

std::optional<BIFIconID> TreeElementModifier::get_icon() const
{
  TreeStoreElem *tselem = TREESTORE(&legacy_te_);
  if (object_.type != OB_GPENCIL_LEGACY) {
    ModifierData *md = static_cast<ModifierData *>(BLI_findlink(&object_.modifiers, tselem->nr));
    ;
    const ModifierTypeInfo *modifier_type = static_cast<const ModifierTypeInfo *>(
        BKE_modifier_get_info((ModifierType)md->type));
    if (modifier_type != nullptr) {
      return modifier_type->icon;
    }
    else {
      return ICON_DOT;
    }
  }
  else {
    /* grease pencil modifiers */
    GpencilModifierData *md = static_cast<GpencilModifierData *>(
        BLI_findlink(&object_.greasepencil_modifiers, tselem->nr));
    switch ((GpencilModifierType)md->type) {
      case eGpencilModifierType_Noise:
        return ICON_MOD_NOISE;
      case eGpencilModifierType_Subdiv:
        return ICON_MOD_SUBSURF;
      case eGpencilModifierType_Thick:
        return ICON_MOD_THICKNESS;
      case eGpencilModifierType_Tint:
        return ICON_MOD_TINT;
      case eGpencilModifierType_Array:
        return ICON_MOD_ARRAY;
      case eGpencilModifierType_Build:
        return ICON_MOD_BUILD;
      case eGpencilModifierType_Opacity:
        return ICON_MOD_MASK;
      case eGpencilModifierType_Color:
        return ICON_MOD_HUE_SATURATION;
      case eGpencilModifierType_Lattice:
        return ICON_MOD_LATTICE;
      case eGpencilModifierType_Mirror:
        return ICON_MOD_MIRROR;
      case eGpencilModifierType_Simplify:
        return ICON_MOD_SIMPLIFY;
      case eGpencilModifierType_Smooth:
        return ICON_MOD_SMOOTH;
      case eGpencilModifierType_Hook:
        return ICON_HOOK;
      case eGpencilModifierType_Offset:
        return ICON_MOD_OFFSET;
      case eGpencilModifierType_Armature:
        return ICON_MOD_ARMATURE;
      case eGpencilModifierType_Multiply:
        return ICON_GP_MULTIFRAME_EDITING;
      case eGpencilModifierType_Time:
        return ICON_MOD_TIME;
      case eGpencilModifierType_Texture:
        return ICON_TEXTURE;
      case eGpencilModifierType_WeightProximity:
        return ICON_MOD_VERTEX_WEIGHT;
      case eGpencilModifierType_WeightAngle:
        return ICON_MOD_VERTEX_WEIGHT;
      case eGpencilModifierType_Shrinkwrap:
        return ICON_MOD_SHRINKWRAP;

        /* Default */
      default:
        return ICON_DOT;
    }
  }
}

}  // namespace blender::ed::outliner
