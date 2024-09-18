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

using namespace blender;

static bool sequencer_text_editing_poll(bContext *C)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  if (seq == nullptr) {
    return false;
  }

  TextVars *data = static_cast<TextVars *>(seq->effectdata);
  if (data == nullptr || data->runtime == nullptr) {
    return false;
  }

  return sequencer_editing_initialized_and_active(C);
}

int2 seq_text_cursor_offset_to_position(TextVarsRuntime *text, int cursor_offset)
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

  // xxx bad!
  cursor_position.y = std::clamp(cursor_position.y, 0, int(text->lines.size() - 1));
  cursor_position.x = std::clamp(
      cursor_position.x, 0, int(text->lines[cursor_position.y].characters.size() - 1));

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

// xxx not nice
static void text_selection_cancel(TextVars *data)
{
  data->selection_start_offset = -1;
  data->selection_end_offset = -1;
}

static bool text_has_selection(TextVars *data)
{
  return data->selection_start_offset != -1;
}

IndexRange seq_text_selection_range_get(TextVars *data)
{
  /* Ensure, that selection start < selection end. */
  int sel_start_offset = data->selection_start_offset;
  int sel_end_offset = data->selection_end_offset;
  if (sel_start_offset > sel_end_offset) {
    std::swap(sel_start_offset, sel_end_offset);
  }

  return IndexRange(sel_start_offset, sel_end_offset - sel_start_offset);
}

static void delete_selected_text(TextVars *data)
{
  if (!text_has_selection(data)) {
    return;
  }

  TextVarsRuntime *text = data->runtime;
  IndexRange sel_range = seq_text_selection_range_get(data);
  int2 sel_start = seq_text_cursor_offset_to_position(text, sel_range.first());
  int2 sel_end = seq_text_cursor_offset_to_position(text, sel_range.last());

  seq::CharInfo char_start = text->lines[sel_start.y].characters[sel_start.x];
  seq::CharInfo char_end = text->lines[sel_end.y].characters[sel_end.x];

  char *addr_start = const_cast<char *>(char_start.str_ptr);
  char *addr_end = const_cast<char *>(char_end.str_ptr) + char_end.byte_length;

  const int move_len = BLI_strnlen(addr_end, 512) + 1; /* +1 to include '\0' XXX hardcoded size.*/
  std::memmove(addr_start, addr_end, move_len);
  data->cursor_offset = cursor_position_to_offset(text, sel_start);
  text_selection_cancel(data);
}

static void text_editing_update(bContext *C)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);

  MEM_delete(data->runtime);
  data->runtime = nullptr;
  SEQ_relations_invalidate_cache_raw(CTX_data_scene(C), seq);
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, CTX_data_scene(C));
}

enum {
  LINE_BEGIN,
  LINE_END,
  TEXT_BEGIN,
  TEXT_END,
  PREV_CHAR,
  NEXT_CHAR,
  PREV_WORD,
  NEXT_WORD,
  PREV_LINE,
  NEXT_LINE,
};

static const EnumPropertyItem move_type_items[] = {
    {LINE_BEGIN, "LINE_BEGIN", 0, "Line Begin", ""},                 // ok
    {LINE_END, "LINE_END", 0, "Line End", ""},                       // ok
    {TEXT_BEGIN, "TEXT_BEGIN", 0, "Text Begin", ""},                 // ok
    {TEXT_END, "TEXT_END", 0, "Text End", ""},                       // ok
    {PREV_CHAR, "PREVIOUS_CHARACTER", 0, "Previous Character", ""},  // ok
    {NEXT_CHAR, "NEXT_CHARACTER", 0, "Next Character", ""},          // ok
    {PREV_WORD, "PREVIOUS_WORD", 0, "Previous Word", ""},
    {NEXT_WORD, "NEXT_WORD", 0, "Next Word", ""},
    {PREV_LINE, "PREVIOUS_LINE", 0, "Previous Line", ""},  // ok
    {NEXT_LINE, "NEXT_LINE", 0, "Next Line", ""},          // ok
    {0, nullptr, 0, nullptr, nullptr},
};

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

static int2 cursor_move_line_end(int2 cursor_position, TextVarsRuntime *text)
{
  seq::LineInfo cur_line = text->lines[cursor_position.y];
  cursor_position.x = cur_line.characters.size() - 1;
  return cursor_position;
}

/* XXX Not great, not terrible I guess */
static int2 cursor_move_prev_word(int2 cursor_position, TextVarsRuntime *text)
{
  cursor_position = cursor_move_by_character(cursor_position, text, -1);

  while (cursor_position.x > 0 || cursor_position.y > 0) {
    seq::CharInfo character = text->lines[cursor_position.y].characters[cursor_position.x];
    int2 prev_cursor_pos = cursor_move_by_character(cursor_position, text, -1);
    seq::CharInfo prev_character = text->lines[prev_cursor_pos.y].characters[prev_cursor_pos.x];

    /* Detect whitespace / non-whitespace transition. */
    if (ELEM(prev_character.str_ptr[0], ' ', '\t', '\n') &&
        !ELEM(character.str_ptr[0], ' ', '\t', '\n'))
    {
      break;
    }
    cursor_position = prev_cursor_pos;
  }
  return cursor_position;
}

/* XXX Not great, not terrible I guess */
static int2 cursor_move_next_word(int2 cursor_position, TextVarsRuntime *text)
{
  const int maxline = text->lines.size() - 1;
  const int maxchar = text->lines.last().characters.size() - 1;

  while ((cursor_position.x < maxchar) || (cursor_position.y < maxline)) {
    seq::CharInfo character = text->lines[cursor_position.y].characters[cursor_position.x];
    cursor_position = cursor_move_by_character(cursor_position, text, 1);
    seq::CharInfo next_character = text->lines[cursor_position.y].characters[cursor_position.x];

    /* Detect whitespace / non-whitespace transition. */
    if (ELEM(next_character.str_ptr[0], ' ', '\t', '\n', '\0') &&
        !ELEM(character.str_ptr[0], ' ', '\t', '\n'))
    {
      break;
    }
  }
  return cursor_position;
}

static int sequencer_text_cursor_move_exec(bContext *C, wmOperator *op)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);
  TextVarsRuntime *text = data->runtime;

  if (RNA_boolean_get(op->ptr, "select_text") && !text_has_selection(data)) {
    data->selection_start_offset = data->cursor_offset;
  }

  int2 cursor_position = seq_text_cursor_offset_to_position(text, data->cursor_offset);

  switch (RNA_enum_get(op->ptr, "type")) {
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
    case LINE_BEGIN:
      cursor_position.x = 0;
      break;
    case LINE_END:
      cursor_position = cursor_move_line_end(cursor_position, text);
      break;
    case TEXT_BEGIN:
      cursor_position = {0, 0};
      break;
    case TEXT_END:
      cursor_position.y = text->lines.size() - 1;
      cursor_position = cursor_move_line_end(cursor_position, text);
      break;
    case PREV_WORD:
      cursor_position = cursor_move_prev_word(cursor_position, text);
      break;
    case NEXT_WORD:
      cursor_position = cursor_move_next_word(cursor_position, text);
      break;
  }

  data->cursor_offset = cursor_position_to_offset(text, cursor_position);
  if (RNA_boolean_get(op->ptr, "select_text")) {
    data->selection_end_offset = data->cursor_offset;
  }

  if (!RNA_boolean_get(op->ptr, "select_text") ||
      data->cursor_offset == data->selection_start_offset)
  {
    text_selection_cancel(data);
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
  ot->poll = sequencer_text_editing_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  RNA_def_enum(ot->srna,
               "type",
               move_type_items,
               LINE_BEGIN,
               "Type",
               "Where to move cursor to, to make a selection");

  PropertyRNA *prop = RNA_def_boolean(
      ot->srna, "select_text", false, "Select Text", "Select text while moving cursor");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

static int sequencer_text_insert_exec(bContext * /*C*/, wmOperator * /*op*/)
{
  /* xxx
   */
  return OPERATOR_FINISHED;
}

static void text_insert(TextVars *data, const char *buf)
{
  TextVarsRuntime *text = data->runtime;
  delete_selected_text(data);

  int in_buf_len = BLI_str_utf8_size_safe(buf);

  /* XXX Bail, if array is full. */
  int2 cursor_position = seq_text_cursor_offset_to_position(text, data->cursor_offset);
  seq::CharInfo cur_char = text->lines[cursor_position.y].characters[cursor_position.x];
  char *cursor_addr = const_cast<char *>(cur_char.str_ptr);
  int move_len = BLI_strnlen(cur_char.str_ptr, 512) +
                 1; /* +1 to include '\0' XXX hardcoded size.*/

  std::memmove(cursor_addr + in_buf_len, cursor_addr, move_len);
  std::memcpy(cursor_addr, buf, in_buf_len);

  // The issue seems to be, that adding multiple spaces causes cursor offset to advance, but
  // this is not always true at the start of line when wrapping. Also wrapped string is stored
  // raw in buffer, will have to check if multiple spaces would cause issues. Likely they do!
  data->cursor_offset += 1;
}

static int sequencer_text_insert_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);

  int in_buf_len = BLI_str_utf8_size_safe(event->utf8_buf);
  if (in_buf_len == 0) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  text_insert(data, event->utf8_buf);

  text_editing_update(C);
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
  ot->poll = sequencer_text_editing_poll;

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

enum {
  DEL_NEXT_CHAR,
  DEL_PREV_CHAR,
  DEL_NEXT_WORD,
  DEL_PREV_WORD,
  DEL_SELECTION,
  DEL_NEXT_SEL,
  DEL_PREV_SEL
};

static const EnumPropertyItem delete_type_items[] = {
    {DEL_NEXT_WORD, "NEXT_WORD", 0, "Next Word", ""},
    {DEL_PREV_WORD, "PREVIOUS_WORD", 0, "Previous Word", ""},
    // xxx I guess these are not needed? at least selection is implied to be always deleted
    {DEL_NEXT_SEL, "NEXT_OR_SELECTION", 0, "Next or Selection", ""},
    {DEL_PREV_SEL, "PREVIOUS_OR_SELECTION", 0, "Previous or Selection", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static void delete_character(TextVarsRuntime *text, int2 cursor_position)
{
  seq::CharInfo cur_char = text->lines[cursor_position.y].characters[cursor_position.x];
  char *cursor_addr = const_cast<char *>(cur_char.str_ptr);
  char *next_char_addr = cursor_addr + cur_char.byte_length;
  const int len = BLI_strnlen(next_char_addr, 512) + 1; /* +1 to include '\0' XXX hardcoded size.*/
  std::memmove(cursor_addr, next_char_addr, len);
}

static int sequencer_text_delete_exec(bContext *C, wmOperator *op)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);
  TextVarsRuntime *text = data->runtime;
  const int type = RNA_enum_get(op->ptr, "type");

  if (text_has_selection(data)) {
    delete_selected_text(data);
    text_editing_update(C);
    return OPERATOR_FINISHED;
  }

  if (type == DEL_NEXT_SEL) {
    if (data->cursor_offset >= text->character_count) {
      return OPERATOR_CANCELLED;
    }

    int2 cursor_position = seq_text_cursor_offset_to_position(text, data->cursor_offset);
    delete_character(text, cursor_position);
  }
  if (type == DEL_PREV_SEL) {
    if (data->cursor_offset == 0) {
      return OPERATOR_CANCELLED;
    }

    int2 cursor_position = seq_text_cursor_offset_to_position(text, data->cursor_offset - 1);
    delete_character(text, cursor_position);
    data->cursor_offset -= 1;
  }

  text_editing_update(C);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_text_delete(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Delete Character";
  ot->description = "Delete text at cursor position";
  ot->idname = "SEQUENCER_OT_text_delete";

  /* api callbacks */
  ot->exec = sequencer_text_delete_exec;
  ot->poll = sequencer_text_editing_poll;

  /* flags */
  ot->flag = OPTYPE_UNDO;

  /* properties */
  RNA_def_enum(ot->srna,
               "type",
               delete_type_items,
               DEL_PREV_CHAR,
               "Type",
               "Which part of the text to delete");
}

static int sequencer_text_line_break_exec(bContext *C, wmOperator * /*op*/)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);
  text_insert(data, "\n");
  text_editing_update(C);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_text_line_break(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Insert Line Break";
  ot->description = "Insert line break at cursor position";
  ot->idname = "SEQUENCER_OT_text_line_break";

  /* api callbacks */
  ot->exec = sequencer_text_line_break_exec;
  ot->poll = sequencer_text_editing_poll;

  /* flags */
  ot->flag = OPTYPE_UNDO;
}

static int sequencer_text_select_all(bContext *C, wmOperator * /*op*/)
{
  Sequence *seq = SEQ_select_active_get(CTX_data_scene(C));
  TextVars *data = static_cast<TextVars *>(seq->effectdata);
  data->selection_start_offset = 0;
  data->selection_end_offset = data->runtime->character_count;
  text_editing_update(C);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_text_select_all(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Select All";
  ot->description = "Select all characters";
  ot->idname = "SEQUENCER_OT_text_select_all";

  /* api callbacks */
  ot->exec = sequencer_text_select_all;
  ot->poll = sequencer_text_editing_poll;

  /* flags */
  ot->flag = OPTYPE_UNDO;
}
