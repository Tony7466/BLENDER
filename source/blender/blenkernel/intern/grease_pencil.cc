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
#include "BLI_stack.hh"

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
  using namespace blender;

  printf("grease_pencil_copy_data\n");
  GreasePencil *grease_pencil_dst = (GreasePencil *)id_dst;
  const GreasePencil *grease_pencil_src = (GreasePencil *)id_src;

  /* Duplicate drawing array. */
  grease_pencil_dst->drawing_array_size = grease_pencil_src->drawing_array_size;
  grease_pencil_dst->drawing_array = MEM_cnew_array<GreasePencilDrawingOrReference *>(
      grease_pencil_src->drawing_array_size, __func__);
  for (int i = 0; i < grease_pencil_src->drawing_array_size; i++) {
    const GreasePencilDrawingOrReference *src_drawing_or_ref = grease_pencil_src->drawing_array[i];
    switch (src_drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        const GreasePencilDrawing *src_drawing = reinterpret_cast<const GreasePencilDrawing *>(
            src_drawing_or_ref);
        grease_pencil_dst->drawing_array[i] = reinterpret_cast<GreasePencilDrawingOrReference *>(
            MEM_cnew<GreasePencilDrawing>(__func__));
        GreasePencilDrawing *dst_drawing = reinterpret_cast<GreasePencilDrawing *>(
            grease_pencil_dst->drawing_array[i]);
        dst_drawing->base.type = src_drawing->base.type;
        dst_drawing->base.flag = src_drawing->base.flag;
        new (&dst_drawing->geometry) CurvesGeometry(src_drawing->geometry.wrap());
        break;
      }
      case GREASE_PENCIL_DRAWING_REFERENCE: {
        const GreasePencilDrawingReference *src_drawing_reference =
            reinterpret_cast<const GreasePencilDrawingReference *>(src_drawing_or_ref);
        grease_pencil_dst->drawing_array[i] = reinterpret_cast<GreasePencilDrawingOrReference *>(
            MEM_dupallocN(src_drawing_reference));
        break;
      }
    }
  }

  if (grease_pencil_src->runtime) {
    grease_pencil_dst->runtime = MEM_new<bke::GreasePencilRuntime>(__func__,
                                                                   *grease_pencil_src->runtime);
  }
}

static void grease_pencil_free_data(ID *id)
{
  printf("grease_pencil_free_data\n");
  GreasePencil *grease_pencil = (GreasePencil *)id;
  BKE_animdata_free(&grease_pencil->id, false);

  grease_pencil->free_drawing_array();
  grease_pencil->free_layer_tree_storage();

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
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = grease_pencil->drawing_array[i];
    if (drawing_or_ref->type == GREASE_PENCIL_DRAWING_REFERENCE) {
      GreasePencilDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, drawing_reference->id_reference, IDWALK_CB_USER);
    }
  }
}

static void grease_pencil_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;

  /* Free storage if there is one. */
  grease_pencil->free_layer_tree_storage();
  grease_pencil->save_layer_tree_to_storage();

  /* Write animation data. */
  if (grease_pencil->adt) {
    BKE_animdata_blend_write(writer, grease_pencil->adt);
  }

  /* Write LibData */
  BLO_write_id_struct(writer, GreasePencil, id_address, &grease_pencil->id);
  BKE_id_blend_write(writer, &grease_pencil->id);

  /* Write drawings. */
  grease_pencil->write_drawing_array(writer);
  /* Write layer tree. */
  grease_pencil->write_layer_tree_storage(writer);
  /* Write materials. */
  BLO_write_pointer_array(
      writer, grease_pencil->material_array_size, grease_pencil->material_array);
}

static void grease_pencil_blend_read_data(BlendDataReader *reader, ID *id)
{
  using namespace blender::bke::gpencil;
  GreasePencil *grease_pencil = (GreasePencil *)id;

  /* Read animation data. */
  BLO_read_data_address(reader, &grease_pencil->adt);
  BKE_animdata_blend_read_data(reader, grease_pencil->adt);

  /* Read drawings. */
  grease_pencil->read_drawing_array(reader);
  /* Read layer tree. */
  grease_pencil->read_layer_tree_storage(reader);
  /* Read materials. */
  BLO_read_pointer_array(reader, (void **)&grease_pencil->material_array);

  /* TODO: Convert storage data to runtime structs. */
  grease_pencil->runtime = MEM_new<blender::bke::GreasePencilRuntime>(__func__);
  grease_pencil->load_layer_tree_from_storage();
}

static void grease_pencil_blend_read_lib(BlendLibReader *reader, ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BLO_read_id_address(reader, grease_pencil->id.lib, &grease_pencil->material_array[i]);
  }
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = grease_pencil->drawing_array[i];
    if (drawing_or_ref->type == GREASE_PENCIL_DRAWING_REFERENCE) {
      GreasePencilDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
      BLO_read_id_address(reader, grease_pencil->id.lib, &drawing_reference->id_reference);
    }
  }
}

static void grease_pencil_blend_read_expand(BlendExpander *expander, ID *id)
{
  GreasePencil *grease_pencil = (GreasePencil *)id;
  for (int i = 0; i < grease_pencil->material_array_size; i++) {
    BLO_expand(expander, grease_pencil->material_array[i]);
  }
  for (int i = 0; i < grease_pencil->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = grease_pencil->drawing_array[i];
    if (drawing_or_ref->type == GREASE_PENCIL_DRAWING_REFERENCE) {
      GreasePencilDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
      BLO_expand(expander, drawing_reference->id_reference);
    }
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

void GreasePencil::read_drawing_array(BlendDataReader *reader)
{
  BLO_read_pointer_array(reader, (void **)&this->drawing_array);
  for (int i = 0; i < this->drawing_array_size; i++) {
    BLO_read_data_address(reader, &this->drawing_array[i]);
    GreasePencilDrawingOrReference *drawing_or_ref = this->drawing_array[i];
    switch (drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
        drawing->geometry.wrap().blend_read(*reader);
        break;
      }
      case GREASE_PENCIL_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
        BLO_read_data_address(reader, &drawing_reference->id_reference);
        break;
      }
    }
  }
}

void GreasePencil::write_drawing_array(BlendWriter *writer)
{
  BLO_write_pointer_array(writer, this->drawing_array_size, this->drawing_array);
  for (int i = 0; i < this->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = this->drawing_array[i];
    switch (drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
        BLO_write_struct(writer, GreasePencilDrawing, drawing);
        drawing->geometry.wrap().blend_write(*writer, this->id);
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
}

void GreasePencil::free_drawing_array()
{
  for (int i = 0; i < this->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = this->drawing_array[i];
    switch (drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
        drawing->geometry.wrap().~CurvesGeometry();
        MEM_freeN(drawing);
        break;
      }
      case GREASE_PENCIL_DRAWING_REFERENCE: {
        GreasePencilDrawingReference *drawing_reference =
            reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref);
        MEM_freeN(drawing_reference);
        break;
      }
    }
  }
  MEM_freeN(this->drawing_array);
}

void GreasePencil::save_layer_tree_to_storage()
{
  using namespace blender::bke::gpencil;
  /* We always store the root group, so we have to add one here. */
  int num_tree_nodes = this->runtime->root_group().total_num_children() + 1;
  this->layer_tree_storage.nodes_num = num_tree_nodes;
  this->layer_tree_storage.nodes = MEM_cnew_array<GreasePencilLayerTreeNode *>(num_tree_nodes,
                                                                               __func__);

  int i = 0;
  this->runtime->root_group().save_to_storage(&this->layer_tree_storage.nodes[i++]);
  for (TreeNode &node : this->runtime->root_group().children_in_pre_order()) {
    GreasePencilLayerTreeNode **dst = &this->layer_tree_storage.nodes[i];
    if (node.is_group()) {
      LayerGroup &group = node.as_group();
      group.save_to_storage(dst);
    }
    else if (node.is_layer()) {
      Layer &layer = node.as_layer();
      layer.save_to_storage(dst);
    }
    i++;
  }
}

static void read_layer_node_recursive(blender::bke::gpencil::LayerGroup &current_group,
                                      GreasePencilLayerTreeNode **nodes,
                                      int index)
{
  using namespace blender::bke::gpencil;
  GreasePencilLayerTreeNode *node = nodes[index];
  switch (node->type) {
    case GREASE_PENCIL_LAYER_TREE_LEAF: {
      GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
      current_group.add_layer(Layer(node_leaf->base.name));
      break;
    }
    case GREASE_PENCIL_LAYER_TREE_GROUP: {
      GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
      LayerGroup new_group(group->base.name);
      for (int i = 0; i < group->children_num; i++) {
        read_layer_node_recursive(new_group, nodes, index + i + 1);
      }
      current_group.add_group(std::move(new_group));
      break;
    }
  }
}

void GreasePencil::load_layer_tree_from_storage()
{
  using namespace blender::bke::gpencil;
  if (this->layer_tree_storage.nodes_num == 0 || !this->layer_tree_storage.nodes) {
    return;
  }
  /* The first node should be the root group. */
  BLI_assert(
      reinterpret_cast<GreasePencilLayerTreeNode *>(this->layer_tree_storage.nodes[0])->type ==
      GREASE_PENCIL_LAYER_TREE_GROUP);
  read_layer_node_recursive(this->runtime->root_group(), this->layer_tree_storage.nodes, 1);
}

void GreasePencil::read_layer_tree_storage(BlendDataReader *reader)
{
  BLO_read_pointer_array(reader, (void **)&this->layer_tree_storage.nodes);
  for (int i = 0; i < this->layer_tree_storage.nodes_num; i++) {
    BLO_read_data_address(reader, &this->layer_tree_storage.nodes[i]);
    GreasePencilLayerTreeNode *node = this->layer_tree_storage.nodes[i];
    switch (node->type) {
      case GREASE_PENCIL_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
        BLO_read_data_address(reader, &node_leaf->base.name);
        /* Read layer data. */
        BLO_read_int32_array(
            reader, node_leaf->layer.frames_storage.size, &node_leaf->layer.frames_storage.keys);
        BLO_read_int32_array(
            reader, node_leaf->layer.frames_storage.size, &node_leaf->layer.frames_storage.values);
        break;
      }
      case GREASE_PENCIL_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
        BLO_read_data_address(reader, &group->base.name);
        break;
      }
    }
  }
}

void GreasePencil::write_layer_tree_storage(BlendWriter *writer)
{
  BLO_write_pointer_array(
      writer, this->layer_tree_storage.nodes_num, this->layer_tree_storage.nodes);
  for (int i = 0; i < this->layer_tree_storage.nodes_num; i++) {
    GreasePencilLayerTreeNode *node = this->layer_tree_storage.nodes[i];
    switch (node->type) {
      case GREASE_PENCIL_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
        BLO_write_struct(writer, GreasePencilLayerTreeLeaf, node_leaf);
        BLO_write_string(writer, node_leaf->base.name);
        /* Write layer data. */
        BLO_write_int32_array(
            writer, node_leaf->layer.frames_storage.size, node_leaf->layer.frames_storage.keys);
        BLO_write_int32_array(
            writer, node_leaf->layer.frames_storage.size, node_leaf->layer.frames_storage.values);
        break;
      }
      case GREASE_PENCIL_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
        BLO_write_struct(writer, GreasePencilLayerTreeGroup, group);
        BLO_write_string(writer, group->base.name);
        break;
      }
    }
  }
}

void GreasePencil::free_layer_tree_storage()
{
  if (this->layer_tree_storage.nodes_num == 0 || !this->layer_tree_storage.nodes) {
    return;
  }
  for (int i = 0; i < this->layer_tree_storage.nodes_num; i++) {
    GreasePencilLayerTreeNode *node = this->layer_tree_storage.nodes[i];
    if (node->name) {
      MEM_freeN(node->name);
    }
    switch (node->type) {
      case GREASE_PENCIL_LAYER_TREE_LEAF: {
        GreasePencilLayerTreeLeaf *node_leaf = reinterpret_cast<GreasePencilLayerTreeLeaf *>(node);
        if (node_leaf->layer.frames_storage.size > 0) {
          MEM_freeN(node_leaf->layer.frames_storage.keys);
          MEM_freeN(node_leaf->layer.frames_storage.values);
        }
        MEM_freeN(node_leaf);
        break;
      }
      case GREASE_PENCIL_LAYER_TREE_GROUP: {
        GreasePencilLayerTreeGroup *group = reinterpret_cast<GreasePencilLayerTreeGroup *>(node);
        MEM_freeN(group);
        break;
      }
    }
  }
  MEM_freeN(this->layer_tree_storage.nodes);
}
