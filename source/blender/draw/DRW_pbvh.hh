/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

#include <variant>

#include "BLI_index_mask_fwd.hh"
#include "BLI_string_ref.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_vector.hh"

#include "DNA_customdata_types.h"

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

namespace blender::draw::pbvh {

class DrawCache;

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

struct ViewportRequest {
  Vector<AttributeRequest> attributes;
  bool use_coarse_grids;
  BLI_STRUCT_EQUALITY_OPERATORS_2(ViewportRequest, attributes, use_coarse_grids);
  uint64_t hash() const;
};

DrawCache &ensure_draw_data(std::unique_ptr<bke::pbvh::DrawCache> &ptr);

void mark_attributes_dirty(const Object &object, const IndexMask &node_mask, DrawCache &draw_data);
void remove_node_tags(bke::pbvh::Tree &pbvh, const IndexMask &node_mask);

Span<gpu::Batch *> ensure_tris_batches(const Object &object,
                                       const ViewportRequest &request,
                                       const IndexMask &nodes_to_update,
                                       DrawCache &draw_data);
Span<gpu::Batch *> ensure_lines_batches(const Object &object,
                                        const ViewportRequest &request,
                                        const IndexMask &nodes_to_update,
                                        DrawCache &draw_data);

Span<int> ensure_material_indices(const Object &object, DrawCache &draw_data);

}  // namespace blender::draw::pbvh
