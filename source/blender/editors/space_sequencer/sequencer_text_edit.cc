/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include "MEM_guardedalloc.h"

#include "BLI_set.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

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

using namespace blender;

int2 seq_cursor_offset_to_position(TextVarsRuntime *text, int cursor_offset)
{
  cursor_offset = std::clamp(cursor_offset, 0, text->character_count);

  int2 cursor_position{0, 0};
  for (seq::LineInfo line : text->lines) {
    if (cursor_offset < line.characters.size()) {
      cursor_position.x = cursor_offset;
      break;
    }
    cursor_offset -= line.characters.size();
    cursor_position.y += 1;
  }

  return cursor_position;
}

static int cursor_position_to_offset(TextVarsRuntime *text, int2 cursor_position)
{
  int cursor_offset = cursor_position.x;

  for (int line : IndexRange(0, cursor_position.y)) {
    cursor_offset += text->lines[line].characters.size();
  }
  return cursor_offset;
}

static int2 cursor_move_by_character(int2 cursor_position, TextVarsRuntime *text, int offset)
{
  seq::LineInfo cur_line = text->lines[cursor_position.y];
  /* Move to next line. */
  if (cursor_position.x + offset > cur_line.characters.size() - 1 &&
      cursor_position.y < text->lines.size() - 1)
  {
    cursor_position.x = 0;
    cursor_position.y++;
  }
  /* Move to previous line. */
  else if (cursor_position.x + offset < 0 && cursor_position.y > 0) {
    cursor_position.y--;
    cursor_position.x = text->lines[cursor_position.y].characters.size() - 1;
  }
  else {
    cursor_position.x += offset;
    const int position_max = text->lines[cursor_position.y].characters.size() - 1;
    cursor_position.x = std::clamp(cursor_position.x, 0, position_max);
  }
  return cursor_position;
}

static int2 cursor_move_by_line(int2 cursor_position, TextVarsRuntime *text, int offset)
{
  seq::LineInfo cur_line = text->lines[cursor_position.y];
  const int cur_pos_x = cur_line.characters[cursor_position.x].position.x;

  const int line_max = text->lines.size() - 1;
  int new_line_index = std::clamp(cursor_position.y + offset, 0, line_max);
  seq::LineInfo new_line = text->lines[new_line_index];

  if (cursor_position.y == new_line_index) {
    return cursor_position;
  }

  /* Find character in another line closest to current position. */
  int best_distance = std::numeric_limits<int>::max();
  int best_character_index = 0;

  for (int i : new_line.characters.index_range()) {
    seq::CharInfo character = new_line.characters[i];
    const int distance = std::abs(character.position.x - cur_pos_x);
    if (distance < best_distance) {
      best_distance = distance;
      best_character_index = i;
    }
  }

  cursor_position.x = best_character_index;
  cursor_position.y = new_line_index;
  return cursor_position;
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
  int2 cursor_position = seq_cursor_offset_to_position(text, data->cursor_offset);

  switch (type) {
    case PREV_CHAR:
      cursor_position = cursor_move_by_character(cursor_position, text, -1);
      break;
    case NEXT_CHAR:
      cursor_position = cursor_move_by_character(cursor_position, text, 1);
      break;
    case PREV_LINE:
      cursor_position = cursor_move_by_line(cursor_position, text, -1);
      break;
    case NEXT_LINE:
      cursor_position = cursor_move_by_line(cursor_position, text, 1);
      break;
  }

  data->cursor_offset = cursor_position_to_offset(text, cursor_position);

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

static int sequencer_text_insert_exec(bContext *C, wmOperator *op)
{
  /*
   */
  return OPERATOR_FINISHED;
}

static int sequencer_text_insert_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{

  if (RNA_struct_property_is_set(op->ptr, "text")) {
    return sequencer_text_insert_exec(C, op);
  }

  if (event->utf8_buf[0] == '\0') {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }
  int in_buf_len = BLI_str_utf8_size_safe(event->utf8_buf);

  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);

  if (data == nullptr || data->runtime == nullptr) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  TextVarsRuntime *text = data->runtime;
  int2 cursor_position = seq_cursor_offset_to_position(text, data->cursor_offset);

  /* XXX Bail, if array is full. */
  seq::CharInfo cur_char = text->lines[cursor_position.y].characters[cursor_position.x];
  char *cursor_addr = const_cast<char *>(cur_char.str_ptr);
  int move_len = BLI_strnlen(cur_char.str_ptr, 512) +
                 1; /* +1 to include '\0' XXX hardcoded size.*/

  std::memmove(cursor_addr + in_buf_len, cur_char.str_ptr, move_len);
  std::memcpy(cursor_addr, event->utf8_buf, in_buf_len);

  data->cursor_offset += 1;

  /* Invalidate text cache. */
  MEM_delete(data->runtime);
  data->runtime = nullptr;

  SEQ_relations_invalidate_cache_raw(CTX_data_scene(C), seq);
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, CTX_data_scene(C));
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_text_insert(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Insert Character";
  ot->description = "Insert text at cursor position";
  ot->idname = "SEQUENCER_OT_text_insert";

  /* api callbacks */
  ot->exec = sequencer_text_insert_exec;
  ot->invoke = sequencer_text_insert_invoke;
  ot->poll = sequencer_editing_initialized_and_active;

  /* flags */
  ot->flag = OPTYPE_UNDO;

  /* properties */
  // xxx
  /*  RNA_def_boolean(
        ot->srna,
        "accent",
        false,
        "Accent Mode",
        "Next typed character will strike through previous, for special character input");*/
}

static int sequencer_text_delete_exec(bContext *C, wmOperator *op)
{
  return OPERATOR_FINISHED
}

void SEQUENCER_OT_text_delete(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Delete Character";
  ot->description = "Delete text at cursor position";
  ot->idname = "SEQUENCER_OT_text_delete";

  /* api callbacks */
  ot->exec = sequencer_text_delete_exec;
  ot->poll = sequencer_editing_initialized_and_active;

  /* flags */
  ot->flag = OPTYPE_UNDO;

  /* properties */
  RNA_def_string(ot->srna, "text", nullptr, 0, "Text", "Text to insert at the cursor position");
  RNA_def_boolean(
      ot->srna,
      "accent",
      false,
      "Accent Mode",
      "Next typed character will strike through previous, for special character input");
}
