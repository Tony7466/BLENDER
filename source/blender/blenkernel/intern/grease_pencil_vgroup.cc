/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "DNA_meshdata_types.h"

#include "BLI_listbase.h"
#include "BLI_set.hh"
#include "BLI_string.h"
#include "BLI_string_utils.hh"

#include "BKE_curves.hh"
#include "BKE_deform.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_grease_pencil_vgroup.hh"

#include "BLT_translation.h"

namespace blender::bke::greasepencil {

/* ------------------------------------------------------------------- */
/** \name Vertex groups in drawings
 * \{ */

void validate_drawing_vertex_groups(GreasePencil &grease_pencil)
{
  Set<const char *> valid_names;
  LISTBASE_FOREACH (const bDeformGroup *, defgroup, &grease_pencil.vertex_group_names) {
    valid_names.add_new(defgroup->name);
  }

  for (GreasePencilDrawingBase *base : grease_pencil.drawings()) {
    if (base->type != GP_DRAWING) {
      continue;
    }
    Drawing &drawing = reinterpret_cast<GreasePencilDrawing *>(base)->wrap();

    /* Remove unknown vertex groups. */
    CurvesGeometry &curves = drawing.strokes_for_write();
    int defgroup_index = 0;
    LISTBASE_FOREACH_MUTABLE (bDeformGroup *, defgroup, &curves.vertex_group_names) {
      if (!valid_names.contains(defgroup->name)) {
        remove_defgroup_index(curves.deform_verts_for_write(), defgroup_index);

        BLI_remlink(&curves.vertex_group_names, defgroup);
        MEM_SAFE_FREE(defgroup);
      }

      ++defgroup_index;
    }
  }
}

void assign_vertex_group(GreasePencil &grease_pencil, StringRef name, float weight)
{
  for (GreasePencilDrawingBase *base : grease_pencil.drawings()) {
    if (base->type != GP_DRAWING) {
      continue;
    }
    Drawing &drawing = reinterpret_cast<GreasePencilDrawing *>(base)->wrap();
    bke::CurvesGeometry &curves = drawing.strokes_for_write();
    ListBase &vertex_group_names = curves.vertex_group_names;

    /* Look for existing group, otherwise lazy-initialize if any vertex is selected. */
    int def_nr = BLI_findstringindex(
        &vertex_group_names, name.data(), offsetof(bDeformGroup, name));
    auto ensure_group_in_drawing = [&]() {
      if (def_nr >= 0) {
        /* Group already exists. */
        return;
      }

      bDeformGroup *defgroup = MEM_cnew<bDeformGroup>(__func__);
      STRNCPY(defgroup->name, name.data());
      BLI_uniquename_cb(
          [vertex_group_names](const StringRef name) {
            return BLI_findstring(
                       &vertex_group_names, name.data(), offsetof(bDeformGroup, name)) != nullptr;
          },
          '.',
          name);
      BLI_addtail(&vertex_group_names, defgroup);
      def_nr = BLI_listbase_count(&vertex_group_names) - 1;
      BLI_assert(def_nr >= 0);
    };

    const bke::AttributeAccessor attributes = curves.attributes();
    const VArray<bool> select_vert = *attributes.lookup_or_default<bool>(
        ".selection", bke::AttrDomain::Point, false);

    MutableSpan<MDeformVert> dverts = curves.deform_verts_for_write();
    for (const int i : dverts.index_range()) {
      if (select_vert[i]) {
        ensure_group_in_drawing();

        MDeformWeight *dw;
        dw = BKE_defvert_ensure_index(&dverts[i], def_nr);
        if (dw) {
          dw->weight = weight;
        }
      }
    }
  }
}

/** \} */

}  // namespace blender::bke::greasepencil