/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include "DNA_outliner_types.h"
#include "DNA_sequence_types.h"

#include "BLI_listbase.h"

#include "BLT_translation.h"

#include "../outliner_intern.hh"
#include "tree_element_seq.hh"

namespace blender::ed::outliner {

TreeElementSequence::TreeElementSequence(TreeElement &legacy_te, Sequence &sequence)
    : AbstractTreeElement(legacy_te), sequence_(sequence)
{
  BLI_assert(legacy_te.store_elem->type == TSE_SEQUENCE);
  legacy_te.name = sequence_.name + 2;
}

bool TreeElementSequence::expand_poll(const SpaceOutliner & /*space_outliner*/) const
{
  return !(sequence_.type & SEQ_TYPE_EFFECT);
}

void TreeElementSequence::expand(SpaceOutliner & /*space_outliner*/) const
{
  /*
   * This work like the sequence.
   * If the sequence have a name (not default name)
   * show it, in other case put the filename.
   */

  if (sequence_.type == SEQ_TYPE_META) {
    LISTBASE_FOREACH (Sequence *, child, &sequence_.seqbase) {
      add_element(&legacy_te_.subtree, nullptr, child, &legacy_te_, TSE_SEQUENCE, 0);
    }
  }
  else {
    add_element(&legacy_te_.subtree, nullptr, sequence_.strip, &legacy_te_, TSE_SEQ_STRIP, 0);
  }
}

std::optional<BIFIconID> TreeElementSequence::get_icon() const
{
  switch (sequence_.type) {
    case SEQ_TYPE_SCENE:
      return ICON_SCENE_DATA;
    case SEQ_TYPE_MOVIECLIP:
      return ICON_TRACKER;
    case SEQ_TYPE_MASK:
      return ICON_MOD_MASK;
    case SEQ_TYPE_MOVIE:
      return ICON_FILE_MOVIE;
    case SEQ_TYPE_SOUND_RAM:
      return ICON_SOUND;
    case SEQ_TYPE_IMAGE:
      return ICON_FILE_IMAGE;
    case SEQ_TYPE_COLOR:
    case SEQ_TYPE_ADJUSTMENT:
      return ICON_COLOR;
    case SEQ_TYPE_TEXT:
      return ICON_FONT_DATA;
    case SEQ_TYPE_ADD:
    case SEQ_TYPE_SUB:
    case SEQ_TYPE_MUL:
    case SEQ_TYPE_OVERDROP:
    case SEQ_TYPE_ALPHAOVER:
    case SEQ_TYPE_ALPHAUNDER:
    case SEQ_TYPE_COLORMIX:
    case SEQ_TYPE_MULTICAM:
    case SEQ_TYPE_TRANSFORM:
    case SEQ_TYPE_SPEED:
    case SEQ_TYPE_GLOW:
    case SEQ_TYPE_GAUSSIAN_BLUR:
      return ICON_SHADERFX;
    case SEQ_TYPE_GAMCROSS:
    case SEQ_TYPE_WIPE:
      return ICON_ARROW_LEFTRIGHT;
    case SEQ_TYPE_META:
      return ICON_SEQ_STRIP_META;
    default:
      return ICON_DOT;
  }
}

Sequence &TreeElementSequence::get_sequence() const
{
  return sequence_;
}

SequenceType TreeElementSequence::get_sequence_type() const
{
  return SequenceType(sequence_.type);
}

/* -------------------------------------------------------------------- */
/* Strip */

TreeElementSequenceStrip::TreeElementSequenceStrip(TreeElement &legacy_te, Strip &strip)
    : AbstractTreeElement(legacy_te)
{
  BLI_assert(legacy_te.store_elem->type == TSE_SEQ_STRIP);

  if (strip.dirpath[0] != '\0') {
    legacy_te_.name = strip.dirpath;
  }
  else {
    legacy_te_.name = IFACE_("Strip None");
  }
}

std::optional<BIFIconID> TreeElementSequenceStrip::get_icon() const
{
  return ICON_LIBRARY_DATA_DIRECT;
}

/* -------------------------------------------------------------------- */
/* Strip Duplicate */

TreeElementSequenceStripDuplicate::TreeElementSequenceStripDuplicate(TreeElement &legacy_te,
                                                                     Sequence &sequence)
    : AbstractTreeElement(legacy_te), sequence_(sequence)
{
  BLI_assert(legacy_te.store_elem->type == TSE_SEQUENCE_DUP);
  legacy_te_.name = sequence.strip->stripdata->filename;
}

std::optional<BIFIconID> TreeElementSequenceStripDuplicate::get_icon() const
{
  return ICON_SEQ_STRIP_DUPLICATE;
}

Sequence &TreeElementSequenceStripDuplicate::get_sequence() const
{
  return sequence_;
}

}  // namespace blender::ed::outliner
