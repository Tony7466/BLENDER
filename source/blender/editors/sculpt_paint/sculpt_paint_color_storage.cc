/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_math_vector.h"
#include "BLI_vector.hh"

#include "BKE_mesh.hh"
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

template<typename T> float4 to_float(const T &src);

template<> float4 to_float(const MLoopCol &src)
{
  float4 dst;
  rgba_uchar_to_float(dst, reinterpret_cast<const uchar *>(&src));
  srgb_to_linearrgb_v3_v3(dst, dst);
  return dst;
}
template<> float4 to_float(const MPropCol &src)
{
  float4 dst;
  copy_v4_v4(dst, src.color);
  return dst;
}

template<typename T> T from_float(const float src[4]);

template<> MLoopCol from_float(const float src[4], MLoopCol &dst)
{
  MLoopCol dst;
  float temp[4];
  linearrgb_to_srgb_v3_v3(temp, src);
  temp[3] = src[3];
  rgba_float_to_uchar(reinterpret_cast<uchar *>(&dst), temp);
  return dst;
}
template<> MPropCol from_float(const float src[4])
{
  MPropCol dst;
  copy_v4_v4(dst.color, src);
  return dst;
}

static float4 color_get_vert(const SculptSession &ss, const int index)
{
  switch (color_write.domain) {
    case ATTR_DOMAIN_POINT: {
      to_static_color_type(color_write.data_type, [&](auto dummy) {
        using T = decltype(dummy);
        color_write.layer.typed<T>()[vert] = from_float(color);
      });
      break;
    }
    case ATTR_DOMAIN_CORNER: {
      Vector<int> corners;
      for (const int face : ss.pmap[vert]) {
        corners.append(
            bke::mesh::face_find_corner_from_vert(ss.faces[face], ss.corner_verts, vert));
      }
      to_static_color_type(color_write.data_type, [&](auto dummy) {
        using T = decltype(dummy);
        color_write.layer.typed<T>().fill_indices(corners, from_float(color));
      });
      break;
    }
    default:
      BLI_assert_unreachable();
  }

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

static void color_set_vert(SculptColorWriteInfo &color_write,
                           const SculptSession &ss,
                           const int vert,
                           const float4 &color)
{
  switch (color_write.domain) {
    case ATTR_DOMAIN_POINT: {
      to_static_color_type(color_write.data_type, [&](auto dummy) {
        using T = decltype(dummy);
        color_write.layer.typed<T>()[vert] = from_float(color);
      });
      break;
    }
    case ATTR_DOMAIN_CORNER: {
      Vector<int> corners;
      for (const int face : ss.pmap[vert]) {
        corners.append(
            bke::mesh::face_find_corner_from_vert(ss.faces[face], ss.corner_verts, vert));
      }
      to_static_color_type(color_write.data_type, [&](auto dummy) {
        using T = decltype(dummy);
        color_write.layer.typed<T>().fill_indices(corners, from_float(color));
      });
      break;
    }
    default:
      BLI_assert_unreachable();
  }
}

}  // namespace blender::ed::sculpt_paint

void SCULPT_swap_colors(SculptSession *ss,
                        const blender::Span<int> indices,
                        blender::MutableSpan<blender::float4> r_colors)
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

float4 SCULPT_vertex_color_get(const SculptSession *ss,
                               const SculptColorInfo &color_info,
                               PBVHVertRef vertex)
{
  return blender::ed::sculpt_paint::color_get_vert<T>(*ss, vertex.i);
}

void SCULPT_vertex_color_set(SculptSession *ss, PBVHVertRef vertex, const float color[4])
{
  blender::ed::sculpt_paint::to_static_color_type(
      eCustomDataType(ss->color_layer->type), [&](auto dummy) {
        using T = decltype(dummy);
        blender::ed::sculpt_paint::color_set_vert<T>(*ss, vertex, color);
      });
}

void SCULPT_store_colors(SculptSession *ss,
                         const blender::Span<int> indices,
                         blender::MutableSpan<blender::float4> r_colors)
{
  blender::ed::sculpt_paint::to_static_color_type(
      eCustomDataType(ss->color_layer->type), [&](auto dummy) {
        using T = decltype(dummy);
        T *pbvh_colors = static_cast<T *>(ss->color_layer->data);
        for (const int i : indices.index_range()) {
          blender::ed::sculpt_paint::to_float(pbvh_colors[indices[i]], r_colors[i]);
        }
      });
}

void SCULPT_store_colors_vertex(SculptSession *ss,
                                const blender::Span<int> indices,
                                blender::MutableSpan<blender::float4> r_colors)
{
  if (ss->color_domain == ATTR_DOMAIN_POINT) {
    SCULPT_store_colors(ss, indices, r_colors);
  }
  else {
    blender::ed::sculpt_paint::to_static_color_type(
        eCustomDataType(ss->color_layer->type), [&](auto dummy) {
          using T = decltype(dummy);
          for (const int i : indices.index_range()) {
            blender::ed::sculpt_paint::color_get_vert<T>(*ss, indices[i], r_colors[i]);
          }
        });
  }
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
  info.data_type = eCustomDataType(layer->type);
  info.domain = BKE_id_attribute_domain(&mesh->id, layer);
  switch (BKE_pbvh_type(ss->pbvh)) {
    case PBVH_FACES: {
      info.layer = info.layer = static_cast<float *>(CustomData_get_layer_named_for_write(
          &mesh->vert_data, CD_PROP_CO, active_color, mesh->totvert));
      break;
    }
    case PBVH_BMESH:
      info.bm_offset = CustomData_get_offset_named(&BKE_pbvh_get_bmesh(ss->pbvh)->vdata,
                                                   CD_PAINT_MASK);
      break;
    case PBVH_GRIDS:
      BLI_assert_unreachable();
      break;
  }
  return info;
}
