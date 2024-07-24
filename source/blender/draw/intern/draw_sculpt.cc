/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "draw_sculpt.hh"

#include "draw_attributes.hh"
#include "draw_pbvh.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh_types.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"

#include "DRW_pbvh.hh"

namespace blender::draw {

float3 SculptBatch::debug_color()
{
  static float3 colors[9] = {
      {1.0f, 0.2f, 0.2f},
      {0.2f, 1.0f, 0.2f},
      {0.2f, 0.2f, 1.0f},
      {1.0f, 1.0f, 0.2f},
      {0.2f, 1.0f, 1.0f},
      {1.0f, 0.2f, 1.0f},
      {1.0f, 0.7f, 0.2f},
      {0.2f, 1.0f, 0.7f},
      {0.7f, 0.2f, 1.0f},
  };

  return colors[debug_index % 9];
}

static Vector<SculptBatch> sculpt_batches_get_ex(const Object *ob,
                                                 const bool use_wire,
                                                 const Set<pbvh::AttributeRequest> &attrs)
{
  /* pbvh::Tree should always exist for non-empty meshes, created by depsgraph eval. */
  bke::pbvh::Tree *pbvh = ob->sculpt ? ob->sculpt->pbvh.get() : nullptr;
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

  /* Frustum planes to show only visible pbvh::Tree nodes. */
  float4 draw_planes[6];
  PBVHFrustumPlanes draw_frustum = {reinterpret_cast<float(*)[4]>(draw_planes), 6};
  float4 update_planes[6];
  PBVHFrustumPlanes update_frustum = {reinterpret_cast<float(*)[4]>(update_planes), 6};

  /* TODO: take into account partial redraw for clipping planes. */
  DRW_view_frustum_planes_get(DRW_view_default_get(), draw_frustum.planes);
  /* Transform clipping planes to object space. Transforming a plane with a
   * 4x4 matrix is done by multiplying with the transpose inverse.
   * The inverse cancels out here since we transform by inverse(obmat). */
  float4x4 tmat = math::transpose(ob->object_to_world());
  for (int i : IndexRange(6)) {
    draw_planes[i] = tmat * draw_planes[i];
    update_planes[i] = draw_planes[i];
  }

  if (paint && (paint->flags & PAINT_SCULPT_DELAY_UPDATES)) {
    if (navigating) {
      bke::pbvh::get_frustum_planes(*pbvh, &update_frustum);
    }
    else {
      bke::pbvh::set_frustum_planes(*pbvh, &update_frustum);
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

  const Mesh *mesh = static_cast<const Mesh *>(ob->data);
  bke::pbvh::update_normals(*pbvh, mesh->runtime->subdiv_ccg.get());

  const IndexMask nodes_to_update;  // TODO: Gather based on flags with index mask result

  IndexMaskMemory memory;
  const IndexMask visible_nodes = pbvh::calc_visible_nodes(*pbvh, *rv3d, memory);

  Span<gpu::Batch *> batches;
  if (use_wire) {
    batches = pbvh::ensure_lines_batches(
        *ob,
        {{}, fast_mode},
        nodes_to_update,
        reinterpret_cast<draw::pbvh::DrawCache &>(*pbvh->draw_data));
  }
  else {
    batches = pbvh::ensure_tris_batches(
        *ob,
        {attrs, fast_mode},
        nodes_to_update,
        reinterpret_cast<draw::pbvh::DrawCache &>(*pbvh->draw_data));
  }

  Vector<SculptBatch> result_batches(nodes_to_update.size());
  nodes_to_update.foreach_index([&](const int i, const int pos) {
    result_batches[pos] = {};
    result_batches[pos].batch = batches[i];
    result_batches[pos].material_slot = 0;  // TODO
    result_batches[pos].debug_index = pos;
  });

  return result_batches;
}

Vector<SculptBatch> sculpt_batches_get(const Object *ob, SculptBatchFeature features)
{
  Set<pbvh::AttributeRequest> attrs;

  attrs.add(pbvh::CustomRequest::Position);
  attrs.add(pbvh::CustomRequest::Normal);
  if (features & SCULPT_BATCH_MASK) {
    attrs.add(pbvh::CustomRequest::Mask);
  }
  if (features & SCULPT_BATCH_FACE_SET) {
    attrs.add(pbvh::CustomRequest::FaceSet);
  }

  const Mesh *mesh = BKE_object_get_original_mesh(ob);
  const bke::AttributeAccessor attributes = mesh->attributes();

  if (features & SCULPT_BATCH_VERTEX_COLOR) {
    if (const char *name = mesh->active_color_attribute) {
      if (const std::optional<bke::AttributeMetaData> meta_data = attributes.lookup_meta_data(
              name))
      {
        attrs.add(pbvh::GenericRequest{name, meta_data->data_type, meta_data->domain});
      }
    }
  }

  if (features & SCULPT_BATCH_UV) {
    if (const char *name = CustomData_get_active_layer_name(&mesh->corner_data, CD_PROP_FLOAT2)) {
      attrs.add(pbvh::GenericRequest{name, CD_PROP_FLOAT2, bke::AttrDomain::Corner});
    }
  }

  return sculpt_batches_get_ex(ob, features & SCULPT_BATCH_WIREFRAME, attrs);
}

Vector<SculptBatch> sculpt_batches_per_material_get(const Object *ob,
                                                    Span<const GPUMaterial *> materials)
{
  BLI_assert(ob->type == OB_MESH);
  const Mesh *mesh = static_cast<const Mesh *>(ob->data);

  DRW_Attributes draw_attrs;
  DRW_MeshCDMask cd_needed;
  DRW_mesh_get_attributes(*ob, *mesh, materials.data(), materials.size(), &draw_attrs, &cd_needed);

  Set<pbvh::AttributeRequest> attrs;

  attrs.add(pbvh::CustomRequest::Position);
  attrs.add(pbvh::CustomRequest::Normal);

  for (int i = 0; i < draw_attrs.num_requests; i++) {
    const DRW_AttributeRequest &req = draw_attrs.requests[i];
    attrs.add(pbvh::GenericRequest{req.attribute_name, req.cd_type, req.domain});
  }

  /* UV maps are not in attribute requests. */
  for (uint i = 0; i < 32; i++) {
    if (cd_needed.uv & (1 << i)) {
      int layer_i = CustomData_get_layer_index_n(&mesh->corner_data, CD_PROP_FLOAT2, i);
      CustomDataLayer *layer = layer_i != -1 ? mesh->corner_data.layers + layer_i : nullptr;
      if (layer) {
        attrs.add(pbvh::GenericRequest{layer->name, CD_PROP_FLOAT2, bke::AttrDomain::Corner});
      }
    }
  }

  return sculpt_batches_get_ex(ob, false, attrs);
}

}  // namespace blender::draw
