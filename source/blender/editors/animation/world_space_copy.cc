#include "BKE_appdir.h"
#include "BKE_context.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_vector_types.hh"
#include "BLI_path_util.h"
#include "BLI_string.h"
#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph_query.hh"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"
#include "RNA_access.hh"
#include "RNA_define.hh"
#include "WM_types.hh"
#include "anim_intern.h"

typedef struct CopyBuffer {
  int name_length;
  char *name;      /* Object/Bone name. */
  float *matrices; /* Array of 4x4 matrices. */
} CopyBuffer;

typedef struct Foo {
  int object_count;
  int frame_count;
  CopyBuffer *buffer;
} Foo;

/* Copy matrix to flat array. */
static void copy_matrix(float source[4][4], float *target)
{
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      target[i * 4 + j] = source[i][j];
    }
  }
}

static void free_foo(Foo *foo)
{
  for (int id_index = 0; id_index < foo->object_count; id_index++) {
    CopyBuffer buffer = foo->buffer[id_index];
    if (buffer.matrices != nullptr) {
      MEM_freeN(buffer.matrices);
    }
    if (buffer.name != nullptr) {
      MEM_freeN(buffer.name);
    }
  }
  if (foo->buffer != nullptr) {
    MEM_freeN(foo->buffer);
  }
}

static bool write_foo(Foo *foo)
{
  char filepath[FILE_MAX];
  const char *cfgdir = BKE_appdir_folder_id_create(BLENDER_USER_CONFIG, nullptr);
  BLI_path_join(filepath, sizeof(filepath), cfgdir, "world_space_buffer");
  FILE *f = fopen(filepath, "wb");
  fwrite(&foo->frame_count, sizeof(int), 1, f);
  fwrite(&foo->object_count, sizeof(int), 1, f);
  for (int id_index = 0; id_index < foo->object_count; id_index++) {
    CopyBuffer *buffer = &foo->buffer[id_index];
    fwrite(&buffer->name_length, sizeof(int), 1, f);
    fwrite(buffer->name, sizeof(char), buffer->name_length + 1, f);
    fwrite(buffer->matrices, sizeof(float), foo->frame_count * 16, f);
  }
  fclose(f);
  return true;
}

static bool read_foo(Foo *foo)
{
  char filepath[FILE_MAX];
  const char *cfgdir = BKE_appdir_folder_id_create(BLENDER_USER_CONFIG, nullptr);
  BLI_path_join(filepath, sizeof(filepath), cfgdir, "world_space_buffer");

  FILE *f = fopen(filepath, "rb");
  if (!f) {
    return false;
  }
  fread(&foo->frame_count, sizeof(int), 1, f);
  fread(&foo->object_count, sizeof(int), 1, f);

  CopyBuffer *copy_buffer = static_cast<CopyBuffer *>(
      MEM_callocN(sizeof(CopyBuffer) * foo->object_count, "World Space Copy Buffer"));
  foo->buffer = copy_buffer;

  for (int id_index = 0; id_index < foo->object_count; id_index++) {
    CopyBuffer *buffer = &foo->buffer[id_index];
    size_t read_size;
    read_size = fread(&buffer->name_length, sizeof(int), 1, f);
    if (read_size != 1) {
      return false;
    }
    buffer->name = static_cast<char *>(
        MEM_callocN(sizeof(char) * buffer->name_length + 1, "World Space name"));
    read_size = fread(buffer->name, sizeof(char), buffer->name_length + 1, f);
    if (read_size != buffer->name_length + 1) {
      return false;
    }
    buffer->matrices = static_cast<float *>(
        MEM_callocN(sizeof(float) * foo->frame_count * 16, "Copy Buffer Frame Values"));
    read_size = fread(buffer->matrices, sizeof(float), foo->frame_count * 16, f);
    if (read_size != foo->frame_count * 16) {
      return false;
    }
  }
  fclose(f);
  return true;
}

static int world_space_copy_exec(bContext *C, wmOperator *op)
{

  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr) {
    return OPERATOR_CANCELLED;
  }
  /* TODO copy selection instead of hardcoding active. */
  ID *ids = &ob->id;
  const int selected_ids = 1;

  int range[2];
  RNA_int_get_array(op->ptr, "range", range);

  Scene *scene = CTX_data_scene(C);
  if (range[0] == 0 && range[1] == 0) {
    range[0] = scene->r.sfra;
    range[1] = scene->r.efra;
  }
  const int frame_count = range[1] - range[0];
  if (frame_count <= 0) {
    return OPERATOR_CANCELLED;
  }

  Foo foo;
  foo.object_count = selected_ids;
  foo.frame_count = frame_count;
  CopyBuffer *copy_buffer = static_cast<CopyBuffer *>(
      MEM_callocN(sizeof(CopyBuffer) * selected_ids, "World Space Copy Buffer"));
  foo.buffer = copy_buffer;

  for (int i = 0; i < selected_ids; i++) {
    copy_buffer[i].name_length = strlen(ids[i].name);
    copy_buffer[i].name = BLI_strdup(ids[i].name);
    copy_buffer[i].matrices = static_cast<float *>(
        MEM_callocN(sizeof(float) * frame_count * 16, "Copy Buffer Frame Values"));
  }

  Depsgraph *depsgraph = DEG_graph_new(
      CTX_data_main(C), scene, CTX_data_view_layer(C), DAG_EVAL_VIEWPORT);
  DEG_graph_build_from_ids(depsgraph, &ids, selected_ids);

  for (int frame_index = 0; frame_index < frame_count; frame_index++) {
    const int frame = range[0] + frame_index;
    DEG_evaluate_on_framechange(depsgraph, frame);

    for (int id_index = 0; id_index < selected_ids; id_index++) {
      CopyBuffer buffer = copy_buffer[id_index];
      float *matrix = &buffer.matrices[frame_index * 16];
      Object *eval_object = DEG_get_evaluated_object(depsgraph, ob);
      copy_matrix(eval_object->object_to_world, matrix);
    }
  }

  write_foo(&foo);
  free_foo(&foo);

  DEG_graph_free(depsgraph);

  return OPERATOR_FINISHED;
}

void ANIM_OT_world_space_copy(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Copy World Space";
  ot->idname = "ANIM_OT_world_space_copy";
  ot->description = "foo";

  ot->exec = world_space_copy_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int_array(
      ot->srna, "range", 2, nullptr, -INT_MAX, INT_MAX, "Frame Range", "", 0, INT_MAX);
}

static int world_space_paste_exec(bContext *C, wmOperator *op)
{
  Foo foo;
  const bool read_success = read_foo(&foo);
  if (!read_success) {
    free_foo(&foo);
    return OPERATOR_CANCELLED;
  }

  free_foo(&foo);
  return OPERATOR_FINISHED;
}

void ANIM_OT_world_space_paste(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Paste World Space";
  ot->idname = "ANIM_OT_world_space_paste";
  ot->description = "foo";

  ot->exec = world_space_paste_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}
