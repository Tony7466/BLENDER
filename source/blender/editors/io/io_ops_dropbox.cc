

#include "BLI_listbase.h"
#include "BLI_path_util.h"

#include "DNA_space_types.h"


#include "RNA_access.h"
#include "RNA_define.h"

#include "WM_api.h"

#include "io_ops_dropbox.h"

void files_drop_copy(bContext * /* C */, wmDrag *drag, wmDropBox *drop)
{
  RNA_string_set(drop->ptr, "filepath", WM_drag_get_path(drag));
  if (!RNA_struct_find_property(drop->ptr, "directory")) {
    return;
  }

  // TODO(@guishe): Add support for multiple drag&drop files import
  char dir[FILE_MAX], file[FILE_MAX];
  BLI_path_split_dir_file(WM_drag_get_path(drag), dir, sizeof(dir), file, sizeof(file));

  RNA_string_set(drop->ptr, "directory", dir);

  RNA_collection_clear(drop->ptr, "files");
  PointerRNA itemptr{};
  RNA_collection_add(drop->ptr, "files", &itemptr);
  RNA_string_set(&itemptr, "name", file);
}

template<eFileSel_File_Types file_t, const char *ext_t>
void add_drag_path_dropbox(ListBase *lb, const char *operator_name)
{
  auto poll = [](bContext * /*C*/, wmDrag *drag, const wmEvent * /*event*/) {
    if (drag->type == WM_DRAG_PATH) {
      const eFileSel_File_Types file_type = eFileSel_File_Types(WM_drag_get_path_file_type(drag));
      if (file_type == file_t && BLI_path_extension_check(WM_drag_get_path(drag), ext_t)) {
        return true;
      }
    }
    return false;
  };

  WM_dropbox_add(lb, operator_name, poll, files_drop_copy, NULL, NULL);
}

void ED_dropboxes_io(void)
{
  ListBase *lb = WM_dropboxmap_find("Window", 0, 0);

#ifdef WITH_IO_WAVEFRONT_OBJ
  static const char obj_ext[] = ".obj";
  add_drag_path_dropbox<FILE_TYPE_OBJECT_IO, obj_ext>(lb, "WM_OT_obj_import");
#endif
}
