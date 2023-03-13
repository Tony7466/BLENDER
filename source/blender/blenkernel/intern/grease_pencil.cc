/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_anim_data.h"
#include "BKE_grease_pencil.hh"
#include "BKE_idtype.h"
#include "BKE_lib_query.h"

#include "BLI_span.hh"

#include "BLO_read_write.h"

#include "BLT_translation.h"

#include "DNA_ID.h"
#include "DNA_ID_enums.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_material_types.h"

#include "MEM_guardedalloc.h"

static void grease_pencil_init_data(ID *id)
{
}

static void grease_pencil_copy_data(Main * /*bmain*/, ID *id_dst, const ID *id_src, const int flag)
{
}

static void grease_pencil_free_data(ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  BKE_animdata_free(&grease_pencil->id, false);

  // TODO: free drawing array
  // TODO: free layer tree

  MEM_SAFE_FREE(grease_pencil->material_array);
}

static void grease_pencil_foreach_id(ID *id, LibraryForeachIDData *data)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, grease_pencil->material_array[i], IDWALK_CB_USER);
  }
  // TODO: walk all the referenced drawings
}

static void grease_pencil_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
}

static void grease_pencil_blend_read_data(BlendDataReader *reader, ID *id)
{
}

static void grease_pencil_blend_read_lib(BlendLibReader *reader, ID *id)
{
}

static void grease_pencil_blend_read_expand(BlendExpander *expander, ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BLO_expand(expander, grease_pencil->material_array[i]);
  }
}

IDTypeInfo IDType_ID_GP = {
    /*id_code*/ ID_GP,
    /*id_filter*/ FILTER_ID_GP,
    /*main_listbase_index*/ INDEX_ID_GP,
    /*struct_size*/ sizeof(GreasePencil),
    /*name*/ "GreasePencil",
    /*name_plural*/ "grease_pencils_new",
    /*translation_context*/ BLT_I18NCONTEXT_ID_GPENCIL,
    /*flags*/ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /*init_data*/ grease_pencil_init_data,
    /*copy_data*/ grease_pencil_copy_data,
    /*free_data*/ grease_pencil_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ grease_pencil_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ grease_pencil_blend_write,
    /*blend_read_data*/ grease_pencil_blend_read_data,
    /*blend_read_lib*/ grease_pencil_blend_read_lib,
    /*blend_read_expand*/ grease_pencil_blend_read_expand,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};

blender::Span<GreasePencilDrawingOrReference> blender::bke::GreasePencil::drawings() const
{
  return blender::Span<GreasePencilDrawingOrReference>{this->drawing_array, this->drawing_array_size};
}
