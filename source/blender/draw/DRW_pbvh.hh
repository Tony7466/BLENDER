/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

#include "BLI_index_mask_fwd.hh"
#include "BLI_map.hh"
#include "BLI_span.hh"
#include "BLI_struct_equality_utils.hh"

// TODO: Remove this include
#include "BKE_pbvh_api.hh"

namespace blender::gpu {
class Batch;
class IndexBuf;
class VertBuf;
}  // namespace blender::gpu
struct Mesh;
struct CustomData;
struct Object;
struct SubdivCCG;
struct BMesh;
struct BMFace;
struct RegionView3D;
namespace blender::bke {
enum class AttrDomain : int8_t;
namespace pbvh {
class Node;
class DrawCache;
class Tree;
}  // namespace pbvh
}  // namespace blender::bke

namespace blender {
namespace draw::pbvh {

class GenericRequest {
 public:
  std::string name;
  eCustomDataType type;
  bke::AttrDomain domain;
  GenericRequest(const StringRef name, const eCustomDataType type, const bke::AttrDomain domain)
      : name(name), type(type), domain(domain)
  {
  }
  BLI_STRUCT_EQUALITY_OPERATORS_3(GenericRequest, type, domain, name);
};

enum class CustomRequest : int8_t {
  Position,
  Normal,
  Mask,
  FaceSet,
};

using AttributeRequest = std::variant<CustomRequest, GenericRequest>;

}  // namespace draw::pbvh

template<> struct DefaultHash<draw::pbvh::AttributeRequest> {
  uint64_t operator()(const draw::pbvh::AttributeRequest &value) const;
};

namespace draw::pbvh {

struct ViewportRequest {
  Vector<AttributeRequest> attributes;
  bool use_coarse_grids;
  BLI_STRUCT_EQUALITY_OPERATORS_2(ViewportRequest, attributes, use_coarse_grids);
  uint64_t hash() const
  {
    return get_default_hash(attributes, use_coarse_grids);
  }
};

// TODO: Try to move back to cc file.
class DrawCache : public bke::pbvh::DrawCache {
 public:
  Vector<int> visible_tri_count;
  Vector<bool> use_flat_layout;
  Vector<int> material_indices;

  Vector<gpu::IndexBuf *> lines_ibos;
  Vector<gpu::IndexBuf *> lines_ibos_coarse;
  Vector<gpu::IndexBuf *> tris_ibos;
  Vector<gpu::IndexBuf *> tris_ibos_coarse;
  Map<AttributeRequest, Vector<gpu::VertBuf *>> attribute_vbos;

  Vector<gpu::Batch *> lines_batches;
  Vector<gpu::Batch *> lines_batches_coarse;
  Map<ViewportRequest, Vector<gpu::Batch *>> tris_batches;

  ~DrawCache() override = default;
};

Span<gpu::Batch *> ensure_tris_batches(const Object &object,
                                       const ViewportRequest &request,
                                       const IndexMask &nodes_to_update,
                                       DrawCache &draw_data);
Span<gpu::Batch *> ensure_lines_batches(const Object &object,
                                        const ViewportRequest &request,
                                        const IndexMask &nodes_to_update,
                                        DrawCache &draw_data);

Span<int> ensure_material_indices(const Object &object, DrawCache &draw_data);

}  // namespace draw::pbvh
}  // namespace blender
