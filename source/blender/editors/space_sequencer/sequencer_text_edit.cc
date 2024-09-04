/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include "MEM_guardedalloc.h"

#include "BLI_set.hh"

#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "ED_select_utils.hh"
#include "ED_sequencer.hh"

#include "SEQ_effects.hh"
#include "SEQ_iterator.hh"
#include "SEQ_relations.hh"
#include "SEQ_select.hh"

#include "WM_api.hh"

#include "RNA_define.hh"

#include "UI_view2d.hh"

/* Own include. */
#include "sequencer_intern.hh"

enum {
  LINE_BEGIN,
  LINE_END,
  FILE_TOP,
  FILE_BOTTOM,
  PREV_CHAR,
  NEXT_CHAR,
  PREV_WORD,
  NEXT_WORD,
  PREV_LINE,
  NEXT_LINE,
  PREV_PAGE,
  NEXT_PAGE
};

static const EnumPropertyItem move_type_items[] = {
    {LINE_BEGIN, "LINE_BEGIN", 0, "Line Begin", ""},
    {LINE_END, "LINE_END", 0, "Line End", ""},
    {FILE_TOP, "FILE_TOP", 0, "File Top", ""},
    {FILE_BOTTOM, "FILE_BOTTOM", 0, "File Bottom", ""},
    {PREV_CHAR, "PREVIOUS_CHARACTER", 0, "Previous Character", ""},  // ok
    {NEXT_CHAR, "NEXT_CHARACTER", 0, "Next Character", ""},          // ok
    {PREV_WORD, "PREVIOUS_WORD", 0, "Previous Word", ""},
    {NEXT_WORD, "NEXT_WORD", 0, "Next Word", ""},
    {PREV_LINE, "PREVIOUS_LINE", 0, "Previous Line", ""},  // ok
    {NEXT_LINE, "NEXT_LINE", 0, "Next Line", ""},          // ok
    {PREV_PAGE, "PREVIOUS_PAGE", 0, "Previous Page", ""},
    {NEXT_PAGE, "NEXT_PAGE", 0, "Next Page", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static void cursor_move_by_character(TextVarsRuntime *text, int offset)
{
  blender::seq::LineInfo cur_line = text->lines[text->cursor_line];
  /* Move to next line. */
  if (text->cursor_character + offset > cur_line.characters.size() - 1 &&
      text->cursor_line < text->lines.size())
  {
    text->cursor_character = 0;
    text->cursor_line++;
  }
  /* Move to previous line. */
  else if (text->cursor_character + offset < 0 && text->cursor_line > 0) {
    text->cursor_line--;
    text->cursor_character = text->lines[text->cursor_line].characters.size() - 1;
  }
  else {
    text->cursor_character += offset;
    const int position_max = text->lines[text->cursor_line].characters.size() - 1;
    text->cursor_character = std::clamp(text->cursor_character, 0, position_max);
  }
}

static void cursor_move_by_line(TextVarsRuntime *text, int offset)
{
  blender::seq::LineInfo cur_line = text->lines[text->cursor_line];
  const int cur_pos_x = cur_line.characters[text->cursor_character].position.x;

  const int line_max = text->lines.size() - 1;
  int new_line_index = std::clamp(text->cursor_line + offset, 0, line_max);
  blender::seq::LineInfo new_line = text->lines[new_line_index];

  if (text->cursor_line == new_line_index) {
    return;
  }

  /* Find character in another line closest to current position. */
  int best_distance = std::numeric_limits<int>::max();
  int best_character_index = 0;

  for (int i : new_line.characters.index_range()) {
    blender::seq::CharInfo character = new_line.characters[i];
    const int distance = std::abs(character.position.x - cur_pos_x);
    if (distance < best_distance) {
      best_distance = distance;
      best_character_index = i;
    }
  }

  text->cursor_character = best_character_index;
  text->cursor_line = new_line_index;
}

static int sequencer_text_cursor_move_exec(bContext *C, wmOperator *op)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);

  if (data == nullptr || data->runtime == nullptr) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  TextVarsRuntime *text = data->runtime;

  const int type = RNA_enum_get(op->ptr, "type");

  switch (type) {
    case PREV_CHAR:
      cursor_move_by_character(text, -1);
      break;
    case NEXT_CHAR:
      cursor_move_by_character(text, 1);
      break;
    case PREV_LINE:
      cursor_move_by_line(text, -1);
      break;
    case NEXT_LINE:
      cursor_move_by_line(text, 1);
      break;
  }

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, CTX_data_scene(C));
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_text_cursor_move(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Move cursor";
  ot->description = "Move cursor in text";
  ot->idname = "SEQUENCER_OT_text_cursor_move";

  /* api callbacks */
  ot->exec = sequencer_text_cursor_move_exec;
  ot->poll = sequencer_editing_initialized_and_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  RNA_def_enum(ot->srna,
               "type",
               move_type_items,
               LINE_BEGIN,
               "Type",
               "Where to move cursor to, to make a selection");
}
