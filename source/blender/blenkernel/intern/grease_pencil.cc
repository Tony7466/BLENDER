/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_anim_data.h"
#include "BKE_curves.hh"
#include "BKE_customdata.h"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_object.h"

#include "BLI_math_vector_types.hh"
#include "BLI_memarena.h"
#include "BLI_memory_utils.hh"
#include "BLI_polyfill_2d.h"
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
using blender::uint3;
using blender::Vector;

static void grease_pencil_init_data(ID *id)
{
  using namespace blender::bke;

  // printf("grease_pencil_init_data\n");
  GreasePencil *grease_pencil = (GreasePencil *)id;
  grease_pencil->runtime = MEM_new<GreasePencilRuntime>(__func__);
}

static void grease_pencil_copy_data(Main * /*bmain*/,
                                    ID *id_dst,
                                    const ID *id_src,
                                    const int /*flag*/)
{
  using namespace blender;

  // printf("grease_pencil_copy_data\n");
  GreasePencil *grease_pencil_dst = (GreasePencil *)id_dst;
  const GreasePencil *grease_pencil_src = (GreasePencil *)id_src;

  /* Duplicate material array. */
  grease_pencil_dst->material_array = static_cast<Material **>(
      MEM_dupallocN(grease_pencil_src->material_array));

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

        new (&dst_drawing->geometry) bke::CurvesGeometry(src_drawing->geometry.wrap());
        dst_drawing->runtime = MEM_new<bke::GreasePencilDrawingRuntime>(__func__);
        dst_drawing->runtime->triangles_cache = src_drawing->runtime->triangles_cache;
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

  /* Do not copy layer tree storage. */
  grease_pencil_dst->layer_tree_storage.nodes = nullptr;
  grease_pencil_dst->layer_tree_storage.nodes_num = 0;

  /* Duplicate runtime data. */
  if (grease_pencil_src->runtime) {
    grease_pencil_dst->runtime = MEM_new<bke::GreasePencilRuntime>(__func__,
                                                                   *grease_pencil_src->runtime);
  }
}

static void grease_pencil_free_data(ID *id)
{
  // printf("grease_pencil_free_data\n");
  GreasePencil *grease_pencil = (GreasePencil *)id;
  BKE_animdata_free(&grease_pencil->id, false);

  MEM_SAFE_FREE(grease_pencil->material_array);

  grease_pencil->free_drawing_array();
  grease_pencil->free_layer_tree_storage();

  BKE_grease_pencil_batch_cache_free(grease_pencil);

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
  /* Free the layer tree storage again after writing. */
  grease_pencil->free_layer_tree_storage();
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

  grease_pencil->runtime = MEM_new<blender::bke::GreasePencilRuntime>(__func__);
  grease_pencil->load_layer_tree_from_storage();
  /* Free the layer tree storage again after loading. */
  grease_pencil->free_layer_tree_storage();
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

void *BKE_grease_pencil_add(Main *bmain, const char *name)
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(BKE_id_new(bmain, ID_GP, name));

  return grease_pencil;
}

GreasePencil *BKE_grease_pencil_new_nomain()
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(BKE_id_new_nomain(ID_GP, nullptr));
  grease_pencil_init_data(&grease_pencil->id);
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

void BKE_grease_pencil_data_update(struct Depsgraph * /*depsgraph*/,
                                   struct Scene * /*scene*/,
                                   Object *object)
{
  /* Free any evaluated data and restore original data. */
  BKE_object_free_derived_caches(object);

  GreasePencil *grease_pencil = static_cast<GreasePencil *>(object->data);
  /* Evaluate modifiers. */
  /* TODO. */

  /* Assign evaluated object. */
  /* TODO: Get eval from modifiers geometry set. */
  GreasePencil *grease_pencil_eval = (GreasePencil *)BKE_id_copy_ex(
      nullptr, &grease_pencil->id, nullptr, LIB_ID_COPY_LOCALIZE);
  // if (grease_pencil_eval == nullptr) {
  //   grease_pencil_eval = BKE_grease_pencil_new_nomain();
  //   BKE_object_eval_assign_data(object, &grease_pencil_eval->id, true);
  // }
  BKE_object_eval_assign_data(object, &grease_pencil_eval->id, true);
}

/* Draw Cache */

void (*BKE_grease_pencil_batch_cache_dirty_tag_cb)(GreasePencil *grease_pencil,
                                                   int mode) = nullptr;
void (*BKE_grease_pencil_batch_cache_free_cb)(GreasePencil *grease_pencil) = nullptr;

void BKE_grease_pencil_batch_cache_dirty_tag(GreasePencil *grease_pencil, int mode)
{
  if (grease_pencil->runtime && grease_pencil->runtime->batch_cache) {
    BKE_grease_pencil_batch_cache_dirty_tag_cb(grease_pencil, mode);
  }
}

void BKE_grease_pencil_batch_cache_free(GreasePencil *grease_pencil)
{
  if (grease_pencil->runtime && grease_pencil->runtime->batch_cache) {
    BKE_grease_pencil_batch_cache_free_cb(grease_pencil);
  }
}

/* GreasePencilDrawing API */

blender::Span<blender::uint3> GreasePencilDrawing::triangles() const
{
  using namespace blender;
  const bke::GreasePencilDrawingRuntime &runtime = *this->runtime;
  runtime.triangles_cache.ensure([&](Vector<uint3> &cache) {
    MemArena *pf_arena = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, __func__);

    const bke::CurvesGeometry &curves = this->geometry.wrap();
    const Span<float3> positions = curves.positions();
    const offset_indices::OffsetIndices<int> points_by_curve = curves.points_by_curve();

    int total_triangles = 0;
    for (int curve_i : curves.curves_range()) {
      IndexRange points = points_by_curve[curve_i];
      if (points.size() > 2) {
        total_triangles += points.size() - 2;
      }
    }

    cache.resize(total_triangles);

    int t = 0;
    for (int curve_i : curves.curves_range()) {
      const IndexRange points = points_by_curve[curve_i];

      if (points.size() < 3) {
        continue;
      }

      const int num_trinagles = points.size() - 2;

      uint(*tris)[3] = static_cast<uint(*)[3]>(
          BLI_memarena_alloc(pf_arena, sizeof(*tris) * size_t(num_trinagles)));
      float(*projverts)[2] = static_cast<float(*)[2]>(
          BLI_memarena_alloc(pf_arena, sizeof(*projverts) * size_t(points.size())));

      /* TODO: calculate axis_mat properly. */
      float3x3 axis_mat;
      axis_dominant_v3_to_m3(axis_mat.ptr(), float3(0.0f, -1.0f, 0.0f));

      for (const int i : IndexRange(points.size())) {
        mul_v2_m3v3(projverts[i], axis_mat.ptr(), positions[points[i]]);
      }

      BLI_polyfill_calc_arena(projverts, points.size(), 0, tris, pf_arena);

      for (const int i : IndexRange(num_trinagles)) {
        cache[t] = uint3(tris[i]);
        t++;
      }

      BLI_memarena_clear(pf_arena);
    }

    BLI_memarena_free(pf_arena);
  });

  return this->runtime->triangles_cache.data().as_span();
}

void GreasePencilDrawing::tag_positions_changed()
{
  this->geometry.wrap().tag_positions_changed();
  this->runtime->triangles_cache.tag_dirty();
}

bool GreasePencilDrawing::has_stroke_buffer()
{
  return this->runtime->stroke_cache.size() > 0;
}

blender::Span<blender::bke::StrokePoint> GreasePencilDrawing::stroke_buffer()
{
  return this->runtime->stroke_cache.as_span();
}

/* GreasePencil API */

static void grease_pencil_grow_drawing_array_by(GreasePencil &self, const int add_capacity)
{
  BLI_assert(add_capacity > 0);
  const int new_drawing_array_size = self.drawing_array_size + add_capacity;
  GreasePencilDrawingOrReference **new_drawing_array =
      reinterpret_cast<GreasePencilDrawingOrReference **>(
          MEM_cnew_array<GreasePencilDrawingOrReference *>(new_drawing_array_size, __func__));

  blender::uninitialized_relocate_n(
      self.drawing_array, self.drawing_array_size, new_drawing_array);

  self.drawing_array = new_drawing_array;
  self.drawing_array_size = new_drawing_array_size;
}

static void grease_pencil_shrink_drawing_array_by(GreasePencil &self, const int remove_capacity)
{
  BLI_assert(remove_capacity > 0);
  const int new_drawing_array_size = self.drawing_array_size - remove_capacity;
  GreasePencilDrawingOrReference **new_drawing_array =
      reinterpret_cast<GreasePencilDrawingOrReference **>(
          MEM_cnew_array<GreasePencilDrawingOrReference *>(new_drawing_array_size, __func__));

  blender::uninitialized_move_n(self.drawing_array, new_drawing_array_size, new_drawing_array);
  MEM_freeN(self.drawing_array);

  self.drawing_array = new_drawing_array;
  self.drawing_array_size = new_drawing_array_size;
}

blender::Span<GreasePencilDrawingOrReference *> GreasePencil::drawings() const
{
  return blender::Span<GreasePencilDrawingOrReference *>{this->drawing_array,
                                                         this->drawing_array_size};
}

blender::MutableSpan<GreasePencilDrawingOrReference *> GreasePencil::drawings_for_write()
{
  return blender::MutableSpan<GreasePencilDrawingOrReference *>{this->drawing_array,
                                                                this->drawing_array_size};
}

void GreasePencil::add_empty_drawings(int n)
{
  using namespace blender;
  BLI_assert(n > 0);
  const int prev_size = this->drawings().size();
  grease_pencil_grow_drawing_array_by(*this, n);
  MutableSpan<GreasePencilDrawingOrReference *> new_drawings =
      this->drawings_for_write().drop_front(prev_size);
  for (const int i : IndexRange(new_drawings.size())) {
    new_drawings[i] = reinterpret_cast<GreasePencilDrawingOrReference *>(
        MEM_new<GreasePencilDrawing>(__func__));
    GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(new_drawings[i]);
    new (&drawing->geometry) bke::CurvesGeometry();
  }
}

void GreasePencil::remove_drawing(int index_to_remove)
{
  using namespace blender::bke::gpencil;
  /* In order to not change the indices of the drawings, we do the following to the drawing to be
   * removed:
   *  - If the drawing (A) is not the last one:
   *     1.1) Find any frames in the layers that reference the last drawing (B) and point them to
   *          A's index.
   *     1.2) Swap drawing A with drawing B.
   *  2) Destroy A and shrink the array by one.
   *  3) Remove any frames in the layers that reference the A's index.
   */
  BLI_assert(this->drawing_array_size > 0);
  BLI_assert(index_to_remove >= 0 && index_to_remove < this->drawing_array_size);

  /* Move the drawing that should be removed to the last index. */
  const int last_drawing_index = this->drawing_array_size - 1;
  if (index_to_remove != last_drawing_index) {
    this->root_group().foreach_layer_pre_order(
        [last_drawing_index, index_to_remove](Layer &layer) {
          blender::Map<int, int> &frames = layer.frames_for_write();
          for (auto item : frames.items()) {
            if (item.value == last_drawing_index) {
              item.value = index_to_remove;
            }
            else if (item.value == index_to_remove) {
              item.value = last_drawing_index;
            }
          }
        });
    std::swap(this->drawings_for_write()[index_to_remove],
              this->drawings_for_write()[last_drawing_index]);
  }

  /* Delete the last drawing. */
  GreasePencilDrawingOrReference *drawing_or_ref_to_remove =
      this->drawings_for_write()[last_drawing_index];
  switch (drawing_or_ref_to_remove->type) {
    case GREASE_PENCIL_DRAWING: {
      GreasePencilDrawing *drawing_to_remove = reinterpret_cast<GreasePencilDrawing *>(
          drawing_or_ref_to_remove);
      drawing_to_remove->geometry.wrap().~CurvesGeometry();
      MEM_delete(drawing_to_remove->runtime);
      drawing_to_remove->runtime = nullptr;
      MEM_freeN(drawing_to_remove);
      break;
    }
    case GREASE_PENCIL_DRAWING_REFERENCE: {
      GreasePencilDrawingReference *drawing_reference_to_remove =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_or_ref_to_remove);
      MEM_freeN(drawing_reference_to_remove);
      break;
    }
  }

  /* Remove any frame that points to the last drawing. */
  this->root_group().foreach_layer_pre_order([last_drawing_index](Layer &layer) {
    blender::Map<int, int> &frames = layer.frames_for_write();
    frames.remove_if([last_drawing_index](auto item) { return item.value == last_drawing_index; });
  });

  /* Shrink drawing array. */
  grease_pencil_shrink_drawing_array_by(*this, 1);
}

void GreasePencil::foreach_visible_drawing(
    int frame, blender::FunctionRef<void(GreasePencilDrawing &)> function)
{
  using namespace blender::bke::gpencil;

  blender::Span<GreasePencilDrawingOrReference *> drawings = this->drawings();
  for (TreeNode &node : this->root_group().children_in_pre_order()) {
    if (!node.is_layer()) {
      continue;
    }
    Layer &layer = node.as_layer();
    int index = layer.drawing_at(frame);
    if (index == -1) {
      continue;
    }
    GreasePencilDrawingOrReference *drawing_or_reference = drawings[index];
    if (drawing_or_reference->type == GREASE_PENCIL_DRAWING) {
      GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_reference);
      function(*drawing);
    }
    else if (drawing_or_reference->type == GREASE_PENCIL_DRAWING_REFERENCE) {
      /* TODO */
    }
  }
}

blender::bke::gpencil::Layer *GreasePencil::get_active_layer()
{
  using namespace blender::bke::gpencil;
  /* TOOD. For now get the first layer. */
  for (TreeNode &node : this->root_group().children_in_pre_order()) {
    if (!node.is_layer()) {
      continue;
    }
    return &node.as_layer();
  }
  return nullptr;
}

blender::bke::gpencil::LayerGroup &GreasePencil::root_group()
{
  BLI_assert(this->runtime != nullptr);
  return this->runtime->root_group();
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
        /* Initialize runtime data. */
        drawing->geometry.runtime = MEM_new<blender::bke::CurvesGeometryRuntime>(__func__);
        drawing->geometry.wrap().update_curve_types();
        drawing->runtime = MEM_new<blender::bke::GreasePencilDrawingRuntime>(__func__);
        drawing->runtime->triangles_cache.tag_dirty();
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
  if (this->drawing_array == nullptr || this->drawing_array_size == 0) {
    return;
  }
  for (int i = 0; i < this->drawing_array_size; i++) {
    GreasePencilDrawingOrReference *drawing_or_ref = this->drawing_array[i];
    switch (drawing_or_ref->type) {
      case GREASE_PENCIL_DRAWING: {
        GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_or_ref);
        drawing->geometry.wrap().~CurvesGeometry();
        MEM_delete(drawing->runtime);
        drawing->runtime = nullptr;
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
  this->drawing_array = nullptr;
  this->drawing_array_size = 0;
}

static void save_tree_node_to_storage(blender::bke::gpencil::TreeNode &node,
                                      GreasePencilLayerTreeNode *dst)
{
  dst->type = node.type;
  copy_v3_v3_uchar(dst->color, node.color);
  dst->flag = node.flag;
  if (node.name) {
    dst->name = BLI_strdup(node.name);
  }
}

static void save_layer_to_storage(blender::bke::gpencil::Layer &node,
                                  GreasePencilLayerTreeNode **dst)
{
  using namespace blender;
  GreasePencilLayerTreeLeaf *new_leaf = MEM_cnew<GreasePencilLayerTreeLeaf>(__func__);
  /* Save properties. */
  save_tree_node_to_storage(node, &new_leaf->base);

  /* Save frames map. */
  int frames_size = node.frames().size();
  new_leaf->layer.frames_storage.size = frames_size;
  new_leaf->layer.frames_storage.keys = MEM_cnew_array<int>(frames_size, __func__);
  new_leaf->layer.frames_storage.values = MEM_cnew_array<int>(frames_size, __func__);

  MutableSpan<int> keys{new_leaf->layer.frames_storage.keys, frames_size};
  MutableSpan<int> values{new_leaf->layer.frames_storage.values, frames_size};
  keys.copy_from(node.sorted_keys());
  for (int i : keys.index_range()) {
    values[i] = node.frames().lookup(keys[i]);
  }

  /* Store pointer. */
  *dst = reinterpret_cast<GreasePencilLayerTreeNode *>(new_leaf);
}

static void save_layer_group_to_storage(blender::bke::gpencil::LayerGroup &node,
                                        GreasePencilLayerTreeNode **dst)
{
  GreasePencilLayerTreeGroup *new_group = MEM_cnew<GreasePencilLayerTreeGroup>(__func__);
  /* Save properties. */
  save_tree_node_to_storage(node, &new_group->base);

  /* Save number of children. */
  new_group->children_num = node.num_children();

  /* Store pointer. */
  *dst = reinterpret_cast<GreasePencilLayerTreeNode *>(new_group);
}

void GreasePencil::save_layer_tree_to_storage()
{
  using namespace blender::bke::gpencil;
  /* We always store the root group, so we have to add one here. */
  int num_tree_nodes = this->root_group().total_num_children() + 1;
  this->layer_tree_storage.nodes_num = num_tree_nodes;
  this->layer_tree_storage.nodes = MEM_cnew_array<GreasePencilLayerTreeNode *>(num_tree_nodes,
                                                                               __func__);

  int i = 0;
  save_layer_group_to_storage(this->root_group(), &this->layer_tree_storage.nodes[i++]);
  for (TreeNode &node : this->root_group().children_in_pre_order()) {
    GreasePencilLayerTreeNode **dst = &this->layer_tree_storage.nodes[i];
    if (node.is_group()) {
      LayerGroup &group = node.as_group();
      save_layer_group_to_storage(group, dst);
    }
    else if (node.is_layer()) {
      Layer &layer = node.as_layer();
      save_layer_to_storage(layer, dst);
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
      Layer new_layer(node_leaf->base.name);
      for (int i = 0; i < node_leaf->layer.frames_storage.size; i++) {
        new_layer.insert_frame(node_leaf->layer.frames_storage.keys[i],
                               node_leaf->layer.frames_storage.values[i]);
      }
      current_group.add_layer(std::move(new_layer));
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
  GreasePencilLayerTreeNode *root = reinterpret_cast<GreasePencilLayerTreeNode *>(
      this->layer_tree_storage.nodes[0]);
  BLI_assert(root->type == GREASE_PENCIL_LAYER_TREE_GROUP);
  for (int i = 0; i < reinterpret_cast<GreasePencilLayerTreeGroup *>(root)->children_num; i++) {
    read_layer_node_recursive(this->runtime->root_group(), this->layer_tree_storage.nodes, i + 1);
  }
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
  if (this->layer_tree_storage.nodes_num == 0 || this->layer_tree_storage.nodes == nullptr) {
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
  this->layer_tree_storage.nodes = nullptr;
  this->layer_tree_storage.nodes_num = 0;
}
