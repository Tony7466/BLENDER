/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_math_vector.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"

#include "sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

template<typename Func>
inline void to_static_color_type(const eCustomDataType type, const Func &func)
{
  switch (type) {
    case CD_PROP_COLOR:
      func(MPropCol());
      break;
    case CD_PROP_BYTE_COLOR:
      func(MLoopCol());
      break;
    default:
      BLI_assert_unreachable();
      break;
  }
}

template<typename T> void to_float(const T &src, float dst[4]);

template<> void to_float(const MLoopCol &src, float dst[4])
{
  rgba_uchar_to_float(dst, reinterpret_cast<const uchar *>(&src));
  srgb_to_linearrgb_v3_v3(dst, dst);
}
template<> void to_float(const MPropCol &src, float dst[4])
{
  copy_v4_v4(dst, src.color);
}

template<typename T> void from_float(const float src[4], T &dst);

template<> void from_float(const float src[4], MLoopCol &dst)
{
  float temp[4];
  linearrgb_to_srgb_v3_v3(temp, src);
  temp[3] = src[3];
  rgba_float_to_uchar(reinterpret_cast<uchar *>(&dst), temp);
}
template<> void from_float(const float src[4], MPropCol &dst)
{
  copy_v4_v4(dst.color, src);
}

template<typename T>
static void vertex_color_get(const SculptSession &ss, const int index, float r_color[4])
{
  if (pbvh.color_domain == ATTR_DOMAIN_CORNER) {
    int count = 0;
    zero_v4(r_color);
    for (const int i_face : pbvh.pmap[index]) {
      const IndexRange face = pbvh.faces[i_face];
      Span<T> colors{static_cast<const T *>(pbvh.color_layer->data) + face.start(), face.size()};
      Span<int> face_verts = pbvh.corner_verts.slice(face);

      for (const int i : IndexRange(face.size())) {
        if (face_verts[i] == index) {
          float temp[4];
          to_float(colors[i], temp);

          add_v4_v4(r_color, temp);
          count++;
        }
      }
    }

    if (count) {
      mul_v4_fl(r_color, 1.0f / float(count));
    }
  }
  else {
    to_float(static_cast<T *>(pbvh.color_layer->data)[index], r_color);
  }
}

template<typename T>
static void vertex_color_set(SculptSession &ss, const int index, const float color[4])
{
  if (pbvh.color_domain == ATTR_DOMAIN_CORNER) {
    for (const int i_face : pbvh.pmap[index]) {
      const IndexRange face = pbvh.faces[i_face];
      MutableSpan<T> colors{static_cast<T *>(pbvh.color_layer->data) + face.start(), face.size()};
      Span<int> face_verts = pbvh.corner_verts.slice(face);

      for (const int i : IndexRange(face.size())) {
        if (face_verts[i] == index) {
          from_float(color, colors[i]);
        }
      }
    }
  }
  else {
    from_float(color, static_cast<T *>(pbvh.color_layer->data)[index]);
  }
}

}  // namespace blender::ed::sculpt_paint

void SCULPT_swap_colors(SculptSession *ss,
                        const blender::Span<int> indices,
                        blender::MutableSpan<blender::flaot4> r_colors)
{
  blender::ed::sculpt_paint::to_static_color_type(
      eCustomDataType(ss->color_layer->type), [&](auto dummy) {
        using T = decltype(dummy);
        T *pbvh_colors = static_cast<T *>(pbvh->color_layer->data);
        for (const int i : IndexRange(indices_num)) {
          T temp = pbvh_colors[indices[i]];
          blender::ed::sculpt_paint::from_float(r_colors[i], pbvh_colors[indices[i]]);
          blender::ed::sculpt_paint::to_float(temp, r_colors[i]);
        }
      });
}

void SCULPT_vertex_color_get(const SculptSession *ss, PBVHVertRef vertex, float r_color[4])
{
  blender::ed::sculpt_paint::to_static_color_type(
      eCustomDataType(ss->color_layer->type), [&](auto dummy) {
        using T = decltype(dummy);
        blender::ed::sculpt_paint::vertex_color_get<T>(*pbvh, vertex, r_color);
      });
}

void SCULPT_vertex_color_set(SculptSession *ss, PBVHVertRef vertex, const float color[4])
{
  blender::ed::sculpt_paint::to_static_color_type(
      eCustomDataType(ss->color_layer->type), [&](auto dummy) {
        using T = decltype(dummy);
        blender::ed::sculpt_paint::vertex_color_set<T>(*pbvh, vertex, color);
      });
}

void SCULPT_store_colors(SculptSession *ss,
                         const blender::Span<int> indices,
                         blender::MutableSpan<blender::flaot4> r_colors)
{
  blender::ed::sculpt_paint::to_static_color_type(
      eCustomDataType(ss->color_layer->type), [&](auto dummy) {
        using T = decltype(dummy);
        T *pbvh_colors = static_cast<T *>(ss->color_layer->data);
        for (const int i : IndexRange(indices_num)) {
          blender::ed::sculpt_paint::to_float(pbvh_colors[indices[i]], r_colors[i]);
        }
      });
}

void SCULPT_store_colors_vertex(SculptSession *ss,
                                const blender::Span<int> indices,
                                blender::MutableSpan<blender::flaot4> r_colors)
{
  if (ss->color_domain == ATTR_DOMAIN_POINT) {
    SCULPT_store_colors(ss, indices, indices_num, r_colors);
  }
  else {
    blender::ed::sculpt_paint::to_static_color_type(
        eCustomDataType(ss->color_layer->type), [&](auto dummy) {
          using T = decltype(dummy);
          for (const int i : IndexRange(indices_num)) {
            blender::ed::sculpt_paint::vertex_color_get<T>(*ss, indices[i], r_colors[i]);
          }
        });
  }
}

bool BKE_pbvh_get_color_layer(Mesh *me, CustomDataLayer **r_layer, eAttrDomain *r_domain)
{
  *r_layer = BKE_id_attribute_search(
      &me->id, me->active_color_attribute, CD_MASK_COLOR_ALL, ATTR_DOMAIN_MASK_COLOR);
  *r_domain = *r_layer ? BKE_id_attribute_domain(&me->id, *r_layer) : ATTR_DOMAIN_POINT;
  return *r_layer != nullptr;
}

SculptColorWriteInfo SCULPT_color_get_for_write(SculptSession *ss)
{
  Mesh *mesh = BKE_pbvh_get_mesh(ss->pbvh);
  const char *active_color = mesh->active_color_attribute;
  if (!active_color) {
    return {}
  }
  CustomDataLayer *layer = BKE_id_attribute_search(
      &mesh->id, mesh->active_color_attribute, CD_MASK_COLOR_ALL, ATTR_DOMAIN_MASK_COLOR);
  if (!layer) {
    return {};
  }
  SculptColorWriteInfo info;
  switch (BKE_pbvh_type(ss->pbvh)) {
    case PBVH_FACES: {
      info.layer = info.layer = static_cast<float *>(CustomData_get_layer_named_for_write(
          &mesh->vert_data, CD_PROP_CO, active_color, mesh->totvert));
      break;
    }
    case PBVH_BMESH:
      info.bm_offset = CustomData_get_offset(&BKE_pbvh_get_bmesh(ss->pbvh)->vdata, CD_PAINT_MASK);
      break;
    case PBVH_GRIDS:
      break;
  }
  return info;
}
