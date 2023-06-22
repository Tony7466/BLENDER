/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "draw_sculpt.hh"

#include "draw_pbvh.h"

#include "BKE_paint.h"
#include "BKE_pbvh.h"
#include "DRW_pbvh.hh"

namespace blender::draw {

struct SculptCallbackData {
  bool use_wire;
  bool fast_mode;

  PBVHAttrReq *attrs;
  int attrs_len;

  Vector<SculptBatch> batches;
};

static void sculpt_draw_cb(SculptCallbackData *data,
                           PBVHBatches *batches,
                           PBVH_GPU_Args *pbvh_draw_args)
{
  if (!batches) {
    return;
  }

  SculptBatch batch = {};

  int primcount;
  if (data->use_wire) {
    batch.batch = DRW_pbvh_lines_get(
        batches, data->attrs, data->attrs_len, pbvh_draw_args, &primcount, data->fast_mode);
  }
  else {
    batch.batch = DRW_pbvh_tris_get(
        batches, data->attrs, data->attrs_len, pbvh_draw_args, &primcount, data->fast_mode);
  }

  batch.material_slot = drw_pbvh_material_index_get(batches);

  data->batches.append(batch);
}

static Vector<SculptBatch> sculpt_batches_get_ex(
    Object *ob, bool use_wire, bool use_materials, PBVHAttrReq *attrs, int attrs_len)
{
  /* PBVH should always exist for non-empty meshes, created by depsgraph eval. */
  PBVH *pbvh = ob->sculpt ? ob->sculpt->pbvh : nullptr;
  if (!pbvh) {
    return {};
  }

  /* TODO(Miguel Pozo): Don't use global context. */
  const DRWContextState *drwctx = DRW_context_state_get();
  RegionView3D *rv3d = drwctx->rv3d;
  const bool navigating = rv3d && (rv3d->rflag & RV3D_NAVIGATING);

  Paint *paint = nullptr;
  if (drwctx->evil_C != nullptr) {
    paint = BKE_paint_get_active_from_context(drwctx->evil_C);
  }

  /* Frustum planes to show only visible PBVH nodes. */
  float4 draw_planes[6];
  PBVHFrustumPlanes draw_frustum = {reinterpret_cast<float(*)[4]>(draw_planes), 6};
  float4 update_planes[6];
  PBVHFrustumPlanes update_frustum = {reinterpret_cast<float(*)[4]>(update_planes), 6};

  /* TODO: take into account partial redraw for clipping planes. */
  DRW_view_frustum_planes_get(DRW_view_default_get(), draw_frustum.planes);
  /* Transform clipping planes to object space. Transforming a plane with a
   * 4x4 matrix is done by multiplying with the transpose inverse.
   * The inverse cancels out here since we transform by inverse(obmat). */
  float4x4 tmat = math::transpose(float4x4(ob->object_to_world));
  for (int i : IndexRange(6)) {
    draw_planes[i] = tmat * draw_planes[i];
    update_planes[i] = draw_planes[i];
  }

  if (paint && (paint->flags & PAINT_SCULPT_DELAY_UPDATES)) {
    if (navigating) {
      BKE_pbvh_get_frustum_planes(pbvh, &update_frustum);
    }
    else {
      BKE_pbvh_set_frustum_planes(pbvh, &update_frustum);
    }
  }

  /* Fast mode to show low poly multires while navigating. */
  bool fast_mode = false;
  if (paint && (paint->flags & PAINT_FAST_NAVIGATE)) {
    fast_mode = navigating;
  }

  /* Update draw buffers only for visible nodes while painting.
   * But do update them otherwise so navigating stays smooth. */
  bool update_only_visible = rv3d && !(rv3d->rflag & RV3D_PAINTING);
  if (paint && (paint->flags & PAINT_SCULPT_DELAY_UPDATES)) {
    update_only_visible = true;
  }

  Mesh *mesh = static_cast<Mesh *>(ob->data);
  BKE_pbvh_update_normals(pbvh, mesh->runtime->subdiv_ccg);

  SculptCallbackData data;
  data.use_wire = use_wire;
  data.fast_mode = fast_mode;
  data.attrs = attrs;
  data.attrs_len = attrs_len;

  BKE_pbvh_draw_cb(pbvh,
                   update_only_visible,
                   &update_frustum,
                   &draw_frustum,
                   (void (*)(void *, PBVHBatches *, PBVH_GPU_Args *))sculpt_draw_cb,
                   &data,
                   use_materials,
                   attrs,
                   attrs_len);

  return data.batches;
}

Vector<SculptBatch> sculpt_batches_get(
    Object *ob, bool use_wire, bool use_mask, bool use_fset, bool use_color, bool use_uv)
{
  PBVHAttrReq attrs[16] = {0};
  int attrs_len = 0;

  /* NOTE: these are NOT #eCustomDataType, they are extended values, ASAN may warn about this. */
  attrs[attrs_len++].type = (eCustomDataType)CD_PBVH_CO_TYPE;
  attrs[attrs_len++].type = (eCustomDataType)CD_PBVH_NO_TYPE;

  if (use_mask) {
    attrs[attrs_len++].type = (eCustomDataType)CD_PBVH_MASK_TYPE;
  }

  if (use_fset) {
    attrs[attrs_len++].type = (eCustomDataType)CD_PBVH_FSET_TYPE;
  }

  Mesh *me = BKE_object_get_original_mesh(ob);

  if (use_color) {
    const CustomDataLayer *layer = BKE_id_attributes_color_find(&me->id,
                                                                me->active_color_attribute);
    if (layer) {
      attrs[attrs_len].type = eCustomDataType(layer->type);
      attrs[attrs_len].domain = BKE_id_attribute_domain(&me->id, layer);
      STRNCPY(attrs[attrs_len].name, layer->name);
      attrs_len++;
    }
  }

  if (use_uv) {
    int layer_i = CustomData_get_active_layer_index(&me->ldata, CD_PROP_FLOAT2);
    if (layer_i != -1) {
      CustomDataLayer *layer = me->ldata.layers + layer_i;
      attrs[attrs_len].type = CD_PROP_FLOAT2;
      attrs[attrs_len].domain = ATTR_DOMAIN_CORNER;
      STRNCPY(attrs[attrs_len].name, layer->name);
      attrs_len++;
    }
  }

  return sculpt_batches_get_ex(ob, use_wire, false, attrs, attrs_len);
}

}  // namespace blender::draw
