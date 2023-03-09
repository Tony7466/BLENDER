/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#include "BKE_mesh.h"
#include "BKE_object.h"

#include "DEG_depsgraph_query.h"

#include "DNA_layer_types.h"

#include "IO_stl.h"

#include "bmesh.h"
#include "bmesh_tools.h"

#include "stl_export.hh"
#include "stl_export_writer.hh"

namespace blender::io::stl {

void exporter_main(bContext *C, const STLExportParams &export_params)
{
  std::unique_ptr<FileWriter> writer;

  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);

  /* If not exporting in batch, create single writer for all objects */
  if (!export_params.use_batch) {
    writer = create_writer(export_params.filepath,
                           export_params.use_ascii ? FileWriter::Type::ASCII :
                                                     FileWriter::Type::BINARY);
  }

  DEGObjectIterSettings deg_iter_settings{};
  deg_iter_settings.depsgraph = depsgraph;
  deg_iter_settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY |
                            DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET | DEG_ITER_OBJECT_FLAG_VISIBLE |
                            DEG_ITER_OBJECT_FLAG_DUPLI;

  DEG_OBJECT_ITER_BEGIN (&deg_iter_settings, object) {
    if (object->type != OB_MESH) {
      continue;
    }

    if (export_params.use_selection_only && !(object->base_flag & BASE_SELECTED)) {
      continue;
    }

    /* If exporting in batch, create writer for each iteration over objects */
    if (export_params.use_batch) {
      // TODO: append object name to exported file name to match old Python STL exporter
      writer = create_writer(export_params.filepath,
                             export_params.use_ascii ? FileWriter::Type::ASCII :
                                                       FileWriter::Type::BINARY);
    }

    Object *obj_eval = DEG_get_evaluated_object(depsgraph, object);
    Object export_object_eval_ = dna::shallow_copy(*obj_eval);
    Mesh *mesh = export_params.use_apply_modifiers ?
                     BKE_object_get_evaluated_mesh(&export_object_eval_) :
                     BKE_object_get_pre_modified_mesh(&export_object_eval_);

    // Write triangles
    auto loops = mesh->loops();
    for (const auto &loop_tri : mesh->looptris()) {
      Triangle t{};
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          t.vertices[i][j] = mesh->vert_positions()[loops[loop_tri.tri[i]].v][j];
        }
      }
      writer->write_triangle(&t);
    }
  }
  DEG_OBJECT_ITER_END;
}

}  // namespace blender::io::stl
