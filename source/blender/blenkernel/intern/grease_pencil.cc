/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_anim_data.h"
#include "BKE_curves.hh"
#include "BKE_customdata.h"
#include "BKE_grease_pencil.hh"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_object.h"

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "BLO_read_write.h"

#include "BLT_translation.h"

#include "DNA_ID.h"
#include "DNA_ID_enums.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_material_types.h"

#include "MEM_guardedalloc.h"

using blender::float3;
using blender::Span;
using blender::Vector;

static void grease_pencil_init_data(ID *id)
{
  using namespace blender::bke;

  printf("grease_pencil_init_data\n");
  GreasePencil *grease_pencil = (GreasePencil *)id;
  grease_pencil->runtime = MEM_new<GreasePencilRuntime>(__func__);
}

static void grease_pencil_copy_data(Main * /*bmain*/, ID *id_dst, const ID *id_src, const int flag)
{
  using namespace blender::bke;

  printf("grease_pencil_copy_data\n");
  GreasePencil *grease_pencil_dst = (GreasePencil *)id_dst;
  const GreasePencil *grease_pencil_src = (GreasePencil *)id_src;

  grease_pencil_dst->runtime = MEM_new<GreasePencilRuntime>(__func__);
  // grease_pencil_dst->runtime->root_group() = grease_pencil_src->runtime->root_group()
}

static void grease_pencil_free_data(ID *id)
{
  printf("grease_pencil_free_data\n");
  GreasePencil *grease_pencil = (GreasePencil *)id;
  BKE_animdata_free(&grease_pencil->id, false);

  // TODO: free drawing array
  // TODO: free layer tree

  MEM_SAFE_FREE(grease_pencil->material_array);

  MEM_delete(grease_pencil->runtime);
  grease_pencil->runtime = nullptr;
}

static void grease_pencil_foreach_id(ID *id, LibraryForeachIDData *data)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, grease_pencil->material_array[i], IDWALK_CB_USER);
  }
  /* TODO: walk all the referenced drawings */
}

static void grease_pencil_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;

  /* TODO: Flush runtime data to storage. */

  /* Write animation data. */
  if (grease_pencil->adt) {
    BKE_animdata_blend_write(writer, grease_pencil->adt);
  }

  /* Write LibData */
  BLO_write_id_struct(writer, GreasePencil, id_address, &grease_pencil->id);
  BKE_id_blend_write(writer, &grease_pencil->id);

  /* Write drawings. */
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = grease_pencil->drawing_array[i];
    switch (drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
        BLO_write_struct(writer, GreasePencilDrawing, drawing);
        drawing->geometry.wrap().blend_write(*writer, grease_pencil->id);
        break;
      }
      case GREASE_PENCIL_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
        BLO_write_struct(writer, GreasePencilDrawingReference, drawing_reference);
        break;
      }
    }
  }

  /* Write layer tree. */
  for (int i = 0; i < grease_pencil->layer_tree_storage.nodes_num; i++) {
    GreasePencilLayerTreeNode *node = grease_pencil->layer_tree_storage.nodes[i];
    switch (node->type) {
      case GREASE_PENCIL_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf
        *>(node); BLO_write_struct(writer, GreasePencilLayerTreeLeaf, node_leaf);
        /* Write layer data. */
        BLO_write_int32_array(
            writer, node_leaf->layer.frames_storage.size, node_leaf->layer.frames_storage.keys);
        BLO_write_int32_array(
            writer, node_leaf->layer.frames_storage.size,
            node_leaf->layer.frames_storage.values);
        break;
      }
      case GREASE_PENCIL_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup
        *>(node); BLO_write_struct(writer, GreasePencilLayerTreeGroup, group); break;
      }
    }
  }

  /* Write materials. */
  BLO_write_pointer_array(
      writer, grease_pencil->material_array_size, grease_pencil->material_array);
}

static void grease_pencil_blend_read_data(BlendDataReader *reader, ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;

  /* Read animation data. */
  BLO_read_data_address(reader, &grease_pencil->adt);
  BKE_animdata_blend_read_data(reader, grease_pencil->adt);

  /* Read drawings. */
  BLO_read_pointer_array(reader, (void **)&grease_pencil->drawing_array);
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = grease_pencil->drawing_array[i];
    switch (drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
        BLO_read_data_address(reader, &drawing);
        drawing->geometry.wrap().blend_read(*reader);
        break;
      }
      case GREASE_PENCIL_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
        BLO_read_data_address(reader, &drawing_reference);
        break;
      }
    }
  }

  /* Read layer tree. */
  BLO_read_pointer_array(reader, (void **)&grease_pencil->layer_tree_storage.nodes);
  for (int i = 0; i < grease_pencil->layer_tree_storage.nodes_num; i++) {
    GreasePencilLayerTreeNode *node = grease_pencil->layer_tree_storage.nodes[i];
    switch (node->type) {
      case GREASE_PENCIL_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf
        *>(node); BLO_read_data_address(reader, &node_leaf);
        /* Read layer data. */
        BLO_read_int32_array(
            reader, node_leaf->layer.frames_storage.size,
            &node_leaf->layer.frames_storage.keys);
        BLO_read_int32_array(
            reader, node_leaf->layer.frames_storage.size,
            &node_leaf->layer.frames_storage.values);
        break;
      }
      case GREASE_PENCIL_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup
        *>(node); BLO_read_data_address(reader, &group); break;
      }
    }
  }

  /* Read materials. */
  BLO_read_pointer_array(reader, (void **)&grease_pencil->material_array);

  /* TODO: Convert storage data to runtime structs. */
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

void *BKE_grease_pencil_add(Main *bmain, const char *name)
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(BKE_id_new(bmain, ID_GP, name));

  return grease_pencil;
}

BoundBox *BKE_grease_pencil_boundbox_get(Object *ob)
{
  BLI_assert(ob->type == OB_GREASE_PENCIL);
  const GreasePencil *grease_pencil = static_cast<const GreasePencil *>(ob->data);

  if (ob->runtime.bb != nullptr && (ob->runtime.bb->flag & BOUNDBOX_DIRTY) == 0) {
    return ob->runtime.bb;
  }

  if (ob->runtime.bb == nullptr) {
    ob->runtime.bb = MEM_cnew<BoundBox>(__func__);

    float3 min(FLT_MAX);
    float3 max(-FLT_MAX);

    for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
      GreasePencilDrawingOrReference *drawing_or_ref = grease_pencil->drawing_array[i];
      switch (drawing_or_ref->type) {
        case GREASE_PENCIL_DRAWING: {
          GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
          const blender::bke::CurvesGeometry &curves = drawing->geometry.wrap();

          if (!curves.bounds_min_max(min, max)) {
            min = float3(-1);
            max = float3(1);
          }
          break;
        }
        case GREASE_PENCIL_DRAWING_REFERENCE: {
          /* TODO */
          break;
        }
      }
    }

    BKE_boundbox_init_from_minmax(ob->runtime.bb, min, max);
  }

  return ob->runtime.bb;
}

namespace blender::bke::gpencil {

LayerGroup &TreeNode::as_group()
{
  return *static_cast<LayerGroup *>(this);
}

Layer &TreeNode::as_layer()
{
  return *static_cast<Layer *>(this);
}

}  // namespace blender::bke::gpencil

blender::Span<GreasePencilDrawingOrReference *> GreasePencil::drawings() const
{
  return blender::Span<GreasePencilDrawingOrReference *>{this->drawing_array,
                                                         this->drawing_array_size};
}

void GreasePencil::foreach_visible_drawing(
    int frame,
    blender::FunctionRef<void(GreasePencilDrawing &, blender::bke::gpencil::Layer &)> function)
{
  blender::Span<GreasePencilDrawingOrReference *> drawings = this->drawings();
  this->runtime->root_group().foreach_layer_pre_order([&](blender::bke::gpencil::Layer &layer) {
    if (layer.frames().contains(frame)) {
      int index = layer.frames().lookup(frame);
      GreasePencilDrawingOrReference *drawing_or_reference = drawings[index];
      if (drawing_or_reference->type == GREASE_PENCIL_DRAWING) {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(
            drawing_or_reference);
        function(*drawing, layer);
      }
    }
  });
}
