/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

/* Needed for BKE_ccg.hh. */
#include "BLI_assert.h"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_virtual_array.hh"

#include "DNA_customdata_types.h"

#include "BKE_ccg.hh"
#include "BKE_pbvh.hh"

namespace blender::gpu {
class Batch;
}
struct Mesh;
struct CustomData;
struct SubdivCCG;
struct BMesh;
struct BMFace;
namespace blender::bke {
enum class AttrDomain : int8_t;
namespace pbvh {
class Node;
}
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

struct PBVHDrawData;

struct ViewportRequest {
  Set<AttributeRequest> attributes;
  bool use_coarse_grids;
};

Span<gpu::Batch *> ensure_tris_batches(const ViewportRequest &request,
                                       const Object &object,
                                       const IndexMask &nodes_to_update,
                                       PBVHDrawData &draw_data);

}  // namespace blender::draw::pbvh
