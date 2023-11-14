/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#include <algorithm>
#include <string>

#include "BKE_mesh.hh"
#include "BKE_object.hh"

#include "BLI_string.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_scene_types.h"

#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"

#include "IO_stl.hh"

#include "stl_export.hh"
#include "stl_export_writer.hh"

namespace blender::io::stl {

void exporter_main(bContext *C, const STLExportParams &export_params)
{
  std::unique_ptr<FileWriter> writer;

  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Scene *scene = CTX_data_scene(C);

  /* If not exporting in batch, create single writer for all objects. */
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

    /* If exporting in batch, create writer for each iteration over objects. */
    if (export_params.use_batch) {
      /* Get object name by skipping initial "OB" prefix. */
      std::string object_name = (object->id.name + 2);
      /* Replace spaces with underscores. */
      std::replace(object_name.begin(), object_name.end(), ' ', '_');

      /* Include object name in the exported file name. */
      std::string suffix = object_name + ".stl";
      char filepath[FILE_MAX];
      BLI_strncpy(filepath, export_params.filepath, FILE_MAX);
      BLI_path_extension_replace(filepath, FILE_MAX, suffix.c_str());
      writer = create_writer(
          filepath, export_params.use_ascii ? FileWriter::Type::ASCII : FileWriter::Type::BINARY);
    }

    Object *obj_eval = DEG_get_evaluated_object(depsgraph, object);
    Mesh *mesh = export_params.use_apply_modifiers ?
                     BKE_object_get_evaluated_mesh(obj_eval) :
                     BKE_object_get_pre_modified_mesh(obj_eval);

    /* Calculate transform. */
    float global_scale = export_params.global_scale;
    if ((scene->unit.system != USER_UNIT_NONE) && export_params.use_scene_unit) {
      global_scale *= scene->unit.scale_length;
    }
    float scale_vec[3] = {global_scale, global_scale, global_scale};
    float obmat3x3[3][3];
    unit_m3(obmat3x3);
    float obmat4x4[4][4];
    unit_m4(obmat4x4);
    /* +Y-forward and +Z-up are the Blender's default axis settings. */
    mat3_from_axis_conversion(
        IO_AXIS_Y, IO_AXIS_Z, export_params.forward_axis, export_params.up_axis, obmat3x3);
    copy_m4_m3(obmat4x4, obmat3x3);
    rescale_m4(obmat4x4, scale_vec);

    /* Write triangles. */
    const Span<float3> positions = mesh->vert_positions();
    const blender::Span<int> corner_verts = mesh->corner_verts();
    for (const MLoopTri &loop_tri : mesh->looptris()) {
      Triangle t{};
      for (int i = 0; i < 3; i++) {
        float3 co = positions[corner_verts[loop_tri.tri[i]]];
        mul_m4_v3(obmat4x4, co);
        for (int j = 0; j < 3; j++) {
          t.vertices[i][j] = co[j];
        }
      }
      writer->write_triangle(&t);
    }
  }
  DEG_OBJECT_ITER_END;
}

}  // namespace blender::io::stl
