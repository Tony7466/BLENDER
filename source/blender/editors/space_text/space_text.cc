/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sptext
 */

#include <cstring>

#include "DNA_text_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_task.hh"

#include "BKE_context.hh"
#include "BKE_global.h"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.h"
#include "BKE_lib_remap.hh"
#include "BKE_main.hh"
#include "BKE_screen.hh"

#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_text.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "BLO_read_write.hh"

#include "RNA_access.hh"
#include "RNA_path.hh"

#include "text_format.hh"
#include "text_intern.hh" /* own include */

/* ******************** default callbacks for text space ***************** */

static SpaceLink *text_create(const ScrArea * /*area*/, const Scene * /*scene*/)
{
  ARegion *region;
  SpaceText *stext;

  stext = static_cast<SpaceText *>(MEM_callocN(sizeof(SpaceText), "inittext"));
  stext->spacetype = SPACE_TEXT;

  stext->lheight = 12;
  stext->tabnumber = 4;
  stext->margin_column = 80;
  stext->showsyntax = true;
  stext->showlinenrs = true;

  stext->runtime = MEM_new<SpaceText_Runtime>(__func__);

  /* header */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "header for text"));

  BLI_addtail(&stext->regionbase, region);
  region->regiontype = RGN_TYPE_HEADER;
  region->alignment = (U.uiflag & USER_HEADER_BOTTOM) ? RGN_ALIGN_BOTTOM : RGN_ALIGN_TOP;

  /* footer */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "footer for text"));
  BLI_addtail(&stext->regionbase, region);
  region->regiontype = RGN_TYPE_FOOTER;
  region->alignment = (U.uiflag & USER_HEADER_BOTTOM) ? RGN_ALIGN_TOP : RGN_ALIGN_BOTTOM;

  /* properties region */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "properties region for text"));

  BLI_addtail(&stext->regionbase, region);
  region->regiontype = RGN_TYPE_UI;
  region->alignment = RGN_ALIGN_RIGHT;
  region->flag = RGN_FLAG_HIDDEN;

  /* main region */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "main region for text"));

  BLI_addtail(&stext->regionbase, region);
  region->regiontype = RGN_TYPE_WINDOW;

  return (SpaceLink *)stext;
}

namespace blender::ed::text {
template<typename Predicate>
static void texts_search_remove_if(blender::Vector<TextSearch> &texts_search,
                                   Predicate &&predicate)
{
  for (TextSearch &ts : texts_search) {
    if (predicate(static_cast<const TextSearch &>(ts))) {
      MEM_delete(ts.matches);
      ts.matches = nullptr;
    }
  }
  texts_search.remove_if(predicate);
}

static void texts_search_clear(const SpaceText *st)
{
  texts_search_remove_if(st->runtime->texts_search,
                         [](const TextSearch & /*ts*/) { return true; });
}

void texts_search_tag_restore(const SpaceText *st)
{
  texts_search_clear(st);
  st->runtime->restore_texts_search = true;
}
/**
 * Update text search when the `text` data-block has been changed.
 */
static void text_search_update(SpaceText *st, Text *text);
}  // namespace blender::ed::text

/* Doesn't free the space-link itself. */
static void text_free(SpaceLink *sl)
{
  SpaceText *stext = (SpaceText *)sl;
  space_text_free_caches(stext);
  blender::ed::text::texts_search_clear(stext);
  MEM_delete(stext->runtime);
  stext->text = nullptr;
}

/* spacetype; init callback */
static void text_init(wmWindowManager * /*wm*/, ScrArea * /*area*/) {}

static SpaceLink *text_duplicate(SpaceLink *sl)
{
  SpaceText *stextn = static_cast<SpaceText *>(MEM_dupallocN(sl));

  /* Add its own runtime data. */
  stextn->runtime = MEM_new<SpaceText_Runtime>(__func__);

  return (SpaceLink *)stextn;
}

static void text_listener(const wmSpaceTypeListenerParams *params)
{
  ScrArea *area = params->area;
  const wmNotifier *wmn = params->notifier;
  SpaceText *st = static_cast<SpaceText *>(area->spacedata.first);

  /* context changes */
  switch (wmn->category) {
    case NC_TEXT:
      /**
       * Check if active text was changed, no need to redraw if text isn't active and there is no a
       * active search in all files, `reference == nullptr` means text was unlinked, should update
       * anyway for this case -- no way to know was text active before unlinking or not
       */
      if (wmn->reference && wmn->reference != st->text &&
          !(st->flags & ST_FIND_ALL && st->findstr[0] != '\0'))
      {
        break;
      }

      switch (wmn->data) {
        case ND_DISPLAY:
        case ND_CURSOR:
          ED_area_tag_redraw(area);
          break;
      }
      using namespace blender;
      switch (wmn->action) {
        case NA_EDITED:
          if (st->text) {
            space_text_drawcache_tag_update(st, true);
            text_update_edited(st->text);
          }

          ED_area_tag_redraw(area);
          ATTR_FALLTHROUGH; /* fall down to update text search */
        case NA_ADDED:
        case NA_SELECTED:
          ed::text::text_search_update(st, static_cast<Text *>(wmn->reference));
          ATTR_FALLTHROUGH; /* fall down to tag redraw */
        case NA_REMOVED:
          /** The text data-block has been deleted, search in the new selected Text if search
           * result is missing. */
          if (st->text && !ed::text::text_search_get(st, st->text)) {
            ed::text::text_search_update(st, st->text);
          }
          ED_area_tag_redraw(area);
          break;
      }

      break;
    case NC_SPACE:
      if (wmn->data == ND_SPACE_TEXT) {
        ED_area_tag_redraw(area);
      }
      break;
  }
}

static void text_operatortypes()
{
  WM_operatortype_append(TEXT_OT_new);
  WM_operatortype_append(TEXT_OT_open);
  WM_operatortype_append(TEXT_OT_reload);
  WM_operatortype_append(TEXT_OT_unlink);
  WM_operatortype_append(TEXT_OT_save);
  WM_operatortype_append(TEXT_OT_save_as);
  WM_operatortype_append(TEXT_OT_make_internal);
  WM_operatortype_append(TEXT_OT_run_script);
  WM_operatortype_append(TEXT_OT_refresh_pyconstraints);

  WM_operatortype_append(TEXT_OT_paste);
  WM_operatortype_append(TEXT_OT_copy);
  WM_operatortype_append(TEXT_OT_cut);
  WM_operatortype_append(TEXT_OT_duplicate_line);

  WM_operatortype_append(TEXT_OT_convert_whitespace);
  WM_operatortype_append(TEXT_OT_comment_toggle);
  WM_operatortype_append(TEXT_OT_unindent);
  WM_operatortype_append(TEXT_OT_indent);
  WM_operatortype_append(TEXT_OT_indent_or_autocomplete);

  WM_operatortype_append(TEXT_OT_select_line);
  WM_operatortype_append(TEXT_OT_select_all);
  WM_operatortype_append(TEXT_OT_select_word);

  WM_operatortype_append(TEXT_OT_move_lines);

  WM_operatortype_append(TEXT_OT_jump);
  WM_operatortype_append(TEXT_OT_move);
  WM_operatortype_append(TEXT_OT_move_select);
  WM_operatortype_append(TEXT_OT_delete);
  WM_operatortype_append(TEXT_OT_overwrite_toggle);

  WM_operatortype_append(TEXT_OT_selection_set);
  WM_operatortype_append(TEXT_OT_cursor_set);
  WM_operatortype_append(TEXT_OT_scroll);
  WM_operatortype_append(TEXT_OT_scroll_bar);
  WM_operatortype_append(TEXT_OT_line_number);

  WM_operatortype_append(TEXT_OT_line_break);
  WM_operatortype_append(TEXT_OT_insert);

  WM_operatortype_append(TEXT_OT_find);
  WM_operatortype_append(TEXT_OT_find_set_selected);
  WM_operatortype_append(TEXT_OT_replace);
  WM_operatortype_append(TEXT_OT_replace_set_selected);

  WM_operatortype_append(TEXT_OT_start_find);
  WM_operatortype_append(TEXT_OT_jump_to_file_at_point);

  WM_operatortype_append(TEXT_OT_to_3d_object);

  WM_operatortype_append(TEXT_OT_resolve_conflict);

  WM_operatortype_append(TEXT_OT_autocomplete);

  WM_operatortype_append(TEXT_OT_open_text_with_selection);
}

static void text_keymap(wmKeyConfig *keyconf)
{
  WM_keymap_ensure(keyconf, "Text Generic", SPACE_TEXT, RGN_TYPE_WINDOW);
  WM_keymap_ensure(keyconf, "Text", SPACE_TEXT, RGN_TYPE_WINDOW);
}

const char *text_context_dir[] = {"edit_text", nullptr};

static int /*eContextResult*/ text_context(const bContext *C,
                                           const char *member,
                                           bContextDataResult *result)
{
  SpaceText *st = CTX_wm_space_text(C);

  if (CTX_data_dir(member)) {
    CTX_data_dir_set(result, text_context_dir);
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "edit_text")) {
    if (st->text != nullptr) {
      CTX_data_id_pointer_set(result, &st->text->id);
    }
    return CTX_RESULT_OK;
  }

  return CTX_RESULT_MEMBER_NOT_FOUND;
}

/********************* main region ********************/

/* add handlers, stuff you only do once or on area/region changes */
static void text_main_region_init(wmWindowManager *wm, ARegion *region)
{
  wmKeyMap *keymap;
  ListBase *lb;

  UI_view2d_region_reinit(&region->v2d, V2D_COMMONVIEW_STANDARD, region->winx, region->winy);

  /* own keymap */
  keymap = WM_keymap_ensure(wm->defaultconf, "Text Generic", SPACE_TEXT, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler_v2d_mask(&region->handlers, keymap);
  keymap = WM_keymap_ensure(wm->defaultconf, "Text", SPACE_TEXT, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler_v2d_mask(&region->handlers, keymap);

  /* add drop boxes */
  lb = WM_dropboxmap_find("Text", SPACE_TEXT, RGN_TYPE_WINDOW);

  WM_event_add_dropbox_handler(&region->handlers, lb);
}

static void text_main_region_draw(const bContext *C, ARegion *region)
{
  /* draw entirely, view changes should be handled here */
  SpaceText *st = CTX_wm_space_text(C);

  if (st->runtime->restore_texts_search) {
    blender::ed::text::texts_search_update(C, st);
    ScrArea *area = CTX_wm_area(C);
    if (area) {
      ARegion *ui_region = BKE_area_find_region_type(area, RGN_TYPE_UI);
      if (ui_region) {
        ED_region_tag_redraw(ui_region);
      }
    }
  }
  st->runtime->restore_texts_search = false;

  // View2D *v2d = &region->v2d;

  /* clear and setup matrix */
  UI_ThemeClearColor(TH_BACK);

  // UI_view2d_view_ortho(v2d);

  /* data... */
  draw_text_main(C, st, region);

  /* reset view matrix */
  // UI_view2d_view_restore(C);

  /* scrollers? */
}

static void text_cursor(wmWindow *win, ScrArea *area, ARegion *region)
{
  SpaceText *st = static_cast<SpaceText *>(area->spacedata.first);
  int wmcursor = WM_CURSOR_TEXT_EDIT;

  if (st->text && BLI_rcti_isect_pt(&st->runtime->scroll_region_handle,
                                    win->eventstate->xy[0] - region->winrct.xmin,
                                    st->runtime->scroll_region_handle.ymin))
  {
    wmcursor = WM_CURSOR_DEFAULT;
  }

  WM_cursor_set(win, wmcursor);
}

/* ************* dropboxes ************* */

static bool text_drop_poll(bContext * /*C*/, wmDrag *drag, const wmEvent * /*event*/)
{
  if (drag->type == WM_DRAG_PATH) {
    const eFileSel_File_Types file_type = eFileSel_File_Types(WM_drag_get_path_file_type(drag));
    if (ELEM(file_type, 0, FILE_TYPE_PYSCRIPT, FILE_TYPE_TEXT)) {
      return true;
    }
  }
  return false;
}

static void text_drop_copy(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
{
  /* copy drag path to properties */
  RNA_string_set(drop->ptr, "filepath", WM_drag_get_single_path(drag));
}

static bool text_drop_paste_poll(bContext * /*C*/, wmDrag *drag, const wmEvent * /*event*/)
{
  return (drag->type == WM_DRAG_ID);
}

static void text_drop_paste(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
{
  char *text;
  ID *id = WM_drag_get_local_ID(drag, 0);

  /* copy drag path to properties */
  text = RNA_path_full_ID_py(id);
  RNA_string_set(drop->ptr, "text", text);
  MEM_freeN(text);
}

/* this region dropbox definition */
static void text_dropboxes()
{
  ListBase *lb = WM_dropboxmap_find("Text", SPACE_TEXT, RGN_TYPE_WINDOW);

  WM_dropbox_add(lb, "TEXT_OT_open", text_drop_poll, text_drop_copy, nullptr, nullptr);
  WM_dropbox_add(lb, "TEXT_OT_insert", text_drop_paste_poll, text_drop_paste, nullptr, nullptr);
}

/* ************* end drop *********** */

/****************** header region ******************/

/* add handlers, stuff you only do once or on area/region changes */
static void text_header_region_init(wmWindowManager * /*wm*/, ARegion *region)
{
  ED_region_header_init(region);
}

static void text_header_region_draw(const bContext *C, ARegion *region)
{
  ED_region_header(C, region);
}

/****************** properties region ******************/

/* add handlers, stuff you only do once or on area/region changes */
static void text_properties_region_init(wmWindowManager *wm, ARegion *region)
{
  wmKeyMap *keymap;

  region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;
  ED_region_panels_init(wm, region);

  /* own keymaps */
  keymap = WM_keymap_ensure(wm->defaultconf, "Text Generic", SPACE_TEXT, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler_v2d_mask(&region->handlers, keymap);
}

static void text_properties_region_draw(const bContext *C, ARegion *region)
{
  ED_region_panels(C, region);

  SpaceText *st = CTX_wm_space_text(C);
  /** This flag trick is make sure buttons have been added already. */
  if (st->flags & ST_FIND_ACTIVATE) {
    if (UI_textbutton_activate_rna(C, region, st, "find_text")) {
      /** Draw again so `find_text` text button is highlighted as active. */
      ED_region_panels(C, region);
    }
    st->flags &= ~ST_FIND_ACTIVATE;
  }
}

static void text_id_remap(ScrArea * /*area*/, SpaceLink *slink, const IDRemapper *mappings)
{
  SpaceText *stext = (SpaceText *)slink;
  BKE_id_remapper_apply(mappings, (ID **)&stext->text, ID_REMAP_APPLY_ENSURE_REAL);
  for (TextSearch &ts : stext->runtime->texts_search) {
    BKE_id_remapper_apply(mappings, (ID **)&ts.text, ID_REMAP_APPLY_ENSURE_REAL);
  }
  blender::ed::text::texts_search_remove_if(stext->runtime->texts_search,
                                            [](const TextSearch &ts) -> bool { return !ts.text; });
}

static void text_foreach_id(SpaceLink *space_link, LibraryForeachIDData *data)
{
  SpaceText *st = reinterpret_cast<SpaceText *>(space_link);
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, st->text, IDWALK_CB_USER_ONE);
}

static void text_space_blend_read_data(BlendDataReader * /*reader*/, SpaceLink *sl)
{
  SpaceText *st = (SpaceText *)sl;
  st->runtime = MEM_new<SpaceText_Runtime>(__func__);
}

static void text_space_blend_write(BlendWriter *writer, SpaceLink *sl)
{
  BLO_write_struct(writer, SpaceText, sl);
}

static void text_space_exit(wmWindowManager * /*wm*/, ScrArea *area)
{
  SpaceText *st = static_cast<SpaceText *>(area->spacedata.first);
  blender::ed::text::texts_search_tag_restore(st);
}
/********************* registration ********************/

void ED_spacetype_text()
{
  SpaceType *st = static_cast<SpaceType *>(MEM_callocN(sizeof(SpaceType), "spacetype text"));
  ARegionType *art;

  st->spaceid = SPACE_TEXT;
  STRNCPY(st->name, "Text");

  st->create = text_create;
  st->free = text_free;
  st->init = text_init;
  st->duplicate = text_duplicate;
  st->operatortypes = text_operatortypes;
  st->keymap = text_keymap;
  st->listener = text_listener;
  st->context = text_context;
  st->dropboxes = text_dropboxes;
  st->id_remap = text_id_remap;
  st->foreach_id = text_foreach_id;
  st->blend_read_data = text_space_blend_read_data;
  st->blend_read_after_liblink = nullptr;
  st->blend_write = text_space_blend_write;
  st->exit = text_space_exit;

  /* regions: main window */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype text region"));
  art->regionid = RGN_TYPE_WINDOW;
  art->init = text_main_region_init;
  art->draw = text_main_region_draw;
  art->cursor = text_cursor;
  art->event_cursor = true;

  BLI_addhead(&st->regiontypes, art);

  /* regions: properties */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype text region"));
  art->regionid = RGN_TYPE_UI;
  art->prefsizex = UI_COMPACT_PANEL_WIDTH;
  art->keymapflag = ED_KEYMAP_UI;

  art->init = text_properties_region_init;
  art->draw = text_properties_region_draw;
  BLI_addhead(&st->regiontypes, art);

  /* regions: header */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype text region"));
  art->regionid = RGN_TYPE_HEADER;
  art->prefsizey = HEADERY;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_VIEW2D | ED_KEYMAP_HEADER;

  art->init = text_header_region_init;
  art->draw = text_header_region_draw;
  BLI_addhead(&st->regiontypes, art);

  /* regions: footer */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype text region"));
  art->regionid = RGN_TYPE_FOOTER;
  art->prefsizey = HEADERY;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_VIEW2D | ED_KEYMAP_FOOTER;
  art->init = text_header_region_init;
  art->draw = text_header_region_draw;
  BLI_addhead(&st->regiontypes, art);

  BKE_spacetype_register(st);

  /* register formatters */
  ED_text_format_register_py();
  ED_text_format_register_osl();
  ED_text_format_register_pov();
  ED_text_format_register_pov_ini();
}

namespace blender::ed::text {

/** Initializes a text search if is not already initialized. */
static void text_search_init(const SpaceText *st, Text *text)
{
  if (text_search_get(st, text)) {
    return;
  }
  // TextSearch ts{text,-1,};
  st->runtime->texts_search.append({text, MEM_new<StringMatchContainer>(__func__), -1});
}

/**
 * Initilaizes the search for each Text data-blocks, only initializes the active text search
 * if #ST_FIND_ALL is not active.
 */
static void texts_search_init(const bContext *C, const SpaceText *st)
{
  const bool init_all = bool(st->flags & ST_FIND_ALL);
  if (!init_all) {
    /** Keep searh result only for the active Text data-block. */
    texts_search_remove_if(st->runtime->texts_search,
                           [st](const TextSearch &ts) { return ts.text != st->text; });
  }

  if (!init_all) {
    if (st->text) {
      text_search_init(st, st->text);
    }
    return;
  }

  Main *bmain = CTX_data_main(C);
  LISTBASE_FOREACH (Text *, text, &bmain->texts) {
    text_search_init(st, text);
  }
}

TextSearch *text_search_get(const SpaceText *st, const Text *text)
{
  auto &texts_search = st->runtime->texts_search;
  auto itr = std::find_if(texts_search.begin(), texts_search.end(), [text](const TextSearch &ts) {
    return ts.text == text;
  });
  if (itr != texts_search.end()) {
    return itr;
  }
  else {
    return nullptr;
  }
}

static const char *find_str(const char *src, const char *find, const bool match_case)
{
  if (match_case) {
    return strstr(src, find);
  }
  else {
    return BLI_strcasestr(src, find);
  }
}

static blender::Vector<StringMatch> text_search_matches_find(const Text *text, const SpaceText *st)
{
  const bool match_case = bool(st->flags & ST_MATCH_CASE);
  const size_t findstr_len = strlen(st->findstr);

  blender::Vector<StringMatch> matches;

  int line_index = 0;
  TextLine *text_line = static_cast<TextLine *>(text->lines.first);
  const char *text_line_str = text_line ? text_line->line : nullptr;

  while (text_line) {
    const char *s = find_str(text_line_str, st->findstr, match_case);
    if (s) {
      const int start = int(s - text_line->line);
      const int end = start + findstr_len;
      matches.append({text_line, line_index, start, end});
      text_line_str = s + findstr_len;
    }
    else {
      text_line = text_line->next;
      line_index++;
      if (text_line) {
        text_line_str = text_line->line;
      }
    }
  }
  return matches;
}

void texts_search_update(const bContext *C, const SpaceText *st)
{
  auto &texts_search = st->runtime->texts_search;
  if (st->findstr[0] == '\0') {
    texts_search_clear(st);
    return;
  }

  texts_search_init(C, st);

  using namespace blender;
  threading::parallel_for_each(texts_search, [st](TextSearch &ts) {
    auto matches = text_search_matches_find(ts.text, st);
    ts.matches->data = std::move(matches);
    ts.active_string_match = -1;
  });
}

int active_text_search_get(const SpaceText *st)
{
  auto &texts_search = st->runtime->texts_search;
  auto itr = std::find_if(texts_search.begin(), texts_search.end(), [st](const TextSearch &ts) {
    return ts.text == st->text;
  });
  if (itr == texts_search.end()) {
    return -1;
  }
  return itr - texts_search.begin();
}

static void text_search_update(SpaceText *st, Text *text)
{
  if (st->findstr[0] == '\0') {
    return;
  }
  const bool find_all = bool(st->flags & ST_FIND_ALL);
  if (find_all || st->text == text) {
    text_search_init(st, text);
  }

  auto *text_search = text_search_get(st, text);
  if (!text_search) {
    return;
  }
  text_search->active_string_match = -1;
  auto matches = text_search_matches_find(text_search->text, st);
  text_search->matches->data = std::move(matches);
}

TextSearch *texts_search_begin_get(const SpaceText *st)
{
  return st->runtime->texts_search.begin();
}

const int texts_search_size_get(const SpaceText *st)
{
  return int(st->runtime->texts_search.size());
}

StringMatch *string_matches_begin_get(const TextSearch *ts)
{
  return ts->matches->data.begin();
}

const int string_matches_size_get(const TextSearch *ts)
{
  return int(ts->matches->data.size());
}
}  // namespace blender::ed::text
