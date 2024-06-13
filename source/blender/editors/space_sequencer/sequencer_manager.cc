#include "BLI_path_util.h"
#include "BKE_context.h"
#include "BKE_report.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_ID.h"
#include "ED_screen.h"
#include "UI_interface.h"
#include "UI_resources.h"
#include "WM_api.h"
#include "WM_types.h"
#include "SEQ_add.h"
#include "DNA_sequence_types.h"
#include "BLI_string.h"

/* Define a simple cache for imported files */
#define MAX_CACHED_FILES 100
static ID cached_files[MAX_CACHED_FILES];
static int cached_file_count = 0;
static int current_file_id = 0;

/* Add a file to the cache */
static void add_to_cache(const char *filepath)
{
    if (cached_file_count < MAX_CACHED_FILES) {
        ID *file = &cached_files[cached_file_count];
        file->file_id = current_file_id++;
        BLI_strncpy(file->file_path, filepath, sizeof(file->file_path));
        cached_file_count++;
    }
}

/* Draw the cached files in the panel */
static void draw_cached_files(uiLayout *layout)
{
    for (int i = 0; i < cached_file_count; i++) {
        uiItemL(layout, cached_files[i].file_path, ICON_FILE);
    }
}

/* Search for files */
static void search_files(const char *query, uiLayout *layout)
{
    for (int i = 0; i < cached_file_count; i++) {
        if (BLI_strcasestr(cached_files[i].file_path, query)) {
            uiItemL(layout, cached_files[i].file_path, ICON_FILE);
        }
    }
}

/* Operator for selecting text files */
static int open_text_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    add_to_cache(filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected text file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_text_file(wmOperatorType *ot)
{
    ot->name = "Open Text File";
    ot->idname = "FILE_OT_open_text_file";
    ot->description = "Select a text file";

    ot->exec = open_text_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_TEXT, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

/* Operator for selecting video files */
static int open_video_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    add_to_cache(filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected video file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_video_file(wmOperatorType *ot)
{
    ot->name = "Open Video File";
    ot->idname = "FILE_OT_open_video_file";
    ot->description = "Select a video file";

    ot->exec = open_video_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_MOVIE, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

/* Operator for selecting sound files */
static int open_sound_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    add_to_cache(filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected sound file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_sound_file(wmOperatorType *ot)
{
    ot->name = "Open Sound File";
    ot->idname = "FILE_OT_open_sound_file";
    ot->description = "Select a sound file";

    ot->exec = open_sound_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_SOUND, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

/* Operator for selecting image files */
static int open_image_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    add_to_cache(filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected image file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_image_file(wmOperatorType *ot)
{
    ot->name = "Open Image File";
    ot->idname = "FILE_OT_open_image_file";
    ot->description = "Select an image file";

    ot->exec = open_image_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_IMAGE, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

static void file_selector_panel_draw(const bContext *C, Panel *panel)
{
    uiLayout *layout = panel->layout;
    uiLayoutSetPropSep(layout, true);

    /* Draw the search bar */
    uiLayout *row = uiLayoutRow(layout, false);
    uiItemL(row, "Search:", ICON_VIEWZOOM);
    uiItemR(row, NULL, 0, 0, "search", ICON_NONE);

    /* Draw the buttons for selecting files */
    uiItemO(layout, "Select Text File", ICON_FILE_BLEND, "FILE_OT_open_text_file");
    uiItemO(layout, "Select Video File", ICON_FILE_MOVIE, "FILE_OT_open_video_file");
    uiItemO(layout, "Select Sound File", ICON_SOUND, "FILE_OT_open_sound_file");
    uiItemO(layout, "Select Image File", ICON_IMAGE_DATA, "FILE_OT_open_image_file");

    /* Draw the cached files */
    draw_cached_files(layout);
}

void file_selector_panel_register(ARegionType *art)
{
    PanelType *pt;

    pt = MEM_callocN(sizeof(PanelType), "spacetype file selector panel");
    strcpy(pt->idname, "SEQUENCER_PT_file_selector");
    strcpy(pt->label, "File Selector");
    strcpy(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
    pt->draw = file_selector_panel_draw;
    pt->poll = ED_operator_areaactive;

    BLI_addtail(&art->paneltypes, pt);
}

/* Drag-and-drop handlers */
static void sequencer_drop_init(wmDrag *drag, wmDropBox *drop)
{
    RNA_string_set(drop->ptr, "filepath", drag->path);
}

static bool sequencer_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
    if (drag->type == WM_DRAG_PATH) {
        const char *extension = BLI_path_extension(drag->path);
        return (extension && (BLI_path_extension_check_array(extension, ".avi;.mov;.mp4;.wav;.mp3;.ogg;.png;.jpg;.jpeg;.bmp;.tiff;.tif")));
    }
    return false;
}

static int sequencer_drop_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);

    /* Add the dropped file to the sequencer */
    SEQ_add_movie_strip(C, filepath, NULL, 0, 0, 0, 0, 0);
    BKE_reportf(op->reports, RPT_INFO, "Dropped file added to sequencer: %s", filepath);

    return OPERATOR_FINISHED;
}

static void sequencer_drop(wmOperatorType *ot)
{
    ot->name = "Drop File to Sequencer";
    ot->idname = "SEQUENCER_OT_drop";
    ot->description = "Drop a file into the sequencer";

    ot->exec = sequencer_drop_exec;
    ot->poll = ED_operator_areaactive;

    RNA_def_string(ot->srna, "filepath", NULL, FILE_MAX, "File Path", "Path of the file to drop");
}

/* Register the dropbox */
void register_sequencer_dropbox(void)
{
    ListBase *lb = WM_dropboxmap_find("Sequencer", SPACE_SEQ, RGN_TYPE_WINDOW);

    WM_dropbox_add(lb, "SEQUENCER_OT_drop", sequencer_drop_poll, sequencer_drop_init);
}

/* Register the panel and dropbox in the appropriate space */
void register_file_selector_panel(void)
{
    SpaceType *st = BKE_spacetype_from_id(SPACE_SEQ);
    ARegionType *art = BKE_regiontype_from_id(st, RGN_TYPE_UI);
    file_selector_panel_register(art);

    /* Register the dropbox for the sequencer */
    register_sequencer_dropbox();
}

void register_file_selector_panel(void)
{
    SpaceType *st = BKE_spacetype_from_id(SPACE_SEQ);
    ARegionType *art = BKE_regiontype_from_id(st, RGN_TYPE_UI);
    file_selector_panel_register(art);
    void file_selector_panel_register(ARegionType *art);
    static void open_text_file(wmOperatorType *ot);
    static void open_sound_file(wmOperatorType *ot);
    static void open_video_file(wmOperatorType *ot);
    static void open_image_file(wmOperatorType *ot);
    static void file_selector_panel_draw(const bContext *C, Panel *panel);
    static int open_texte_file_exec(bContext *C, wmOperator *op);
    static int open_sound_file_exec(bContext *C, wmOperator *op);
    static int open_video_file_exec(bContext *C, wmOperator *op);
    static int open_image_file_exec(bContext *C, wmOperator *op);

}
