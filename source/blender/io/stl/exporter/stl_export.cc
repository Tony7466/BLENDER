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

    BMeshCreateParams bm_create_params{};
    bm_create_params.use_toolflags = false;

    BMeshFromMeshParams bm_convert_params{};
    /* We need to calculate face normals, otherwise #BKE_mesh_from_bmesh_for_eval_nomain fails due
     * to an assertion in BMesh code */
    bm_convert_params.calc_face_normal = true;
    bm_convert_params.calc_vert_normal = false;

    BMesh *bmesh = BKE_mesh_to_bmesh_ex(mesh, &bm_create_params, &bm_convert_params);
    BM_mesh_triangulate(bmesh, 0, 3, 4, false, nullptr, nullptr, nullptr);
    Mesh *triangulated_mesh = BKE_mesh_from_bmesh_for_eval_nomain(bmesh, nullptr, mesh);
    BM_mesh_free(bmesh);

    // Write triangles
    const auto loops = triangulated_mesh->loops();
    for (const auto &poly : triangulated_mesh->polys()) {
      const Span<MLoop> poly_loops = loops.slice(poly.loopstart, poly.totloop);
      Triangle t{};
      for (int i = 0; i < poly_loops.size(); i++) {
        for (int j = 0; j < 3; j++) {
          t.vertices[i][j] = triangulated_mesh->vert_positions()[poly_loops[i].v][j];
        }
      }
      writer->write_triangle(&t);
    }
  }
  DEG_OBJECT_ITER_END;
}

}  // namespace blender::io::stl
