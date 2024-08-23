/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

/* Needed for BKE_ccg.hh. */
#include "BLI_assert.h"
#include "BLI_index_mask.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_virtual_array.hh"

#include "DNA_customdata_types.h"

#include "BKE_ccg.hh"

namespace blender::gpu {
class Batch;
class IndexBuf;
class VertBuf;
}  // namespace blender::gpu
struct Mesh;
struct CustomData;
struct SubdivCCG;
struct BMesh;
struct BMFace;
struct RegionView3D;
namespace blender::bke {
enum class AttrDomain : int8_t;
namespace pbvh {
class Node;
class DrawCache;
}  // namespace pbvh
}  // namespace blender::bke

namespace blender::draw::pbvh {

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

class DrawCache;

struct ViewportRequest {
  Set<AttributeRequest> attributes;
  bool use_coarse_grids;
};

// TODO: Try to move back to cc file.
class DrawCache : public bke::pbvh::DrawCache {
 public:
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

IndexMask calc_visible_nodes(const bke::pbvh::Tree &pbvh,
                             const RegionView3D &rv3d,
                             IndexMaskMemory &memory);

}  // namespace blender::draw::pbvh
