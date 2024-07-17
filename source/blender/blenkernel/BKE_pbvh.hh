/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BLI_array.hh"
#include "BLI_bounds_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

struct BMesh;
struct BMVert;
struct BMFace;
namespace blender::draw::pbvh {
struct PBVHBatches;
}

/* #PBVHNodeFlags is needed by `DRW_render.hh` and `draw_cache.cc`. */
enum PBVHNodeFlags {
  PBVH_Leaf = 1 << 0,

  PBVH_UpdateNormals = 1 << 1,
  PBVH_UpdateBB = 1 << 2,
  PBVH_UpdateDrawBuffers = 1 << 4,
  PBVH_UpdateRedraw = 1 << 5,
  PBVH_UpdateMask = 1 << 6,
  PBVH_UpdateVisibility = 1 << 8,

  PBVH_RebuildDrawBuffers = 1 << 9,
  PBVH_FullyHidden = 1 << 10,
  PBVH_FullyMasked = 1 << 11,
  PBVH_FullyUnmasked = 1 << 12,

  PBVH_UpdateTopology = 1 << 13,
  PBVH_UpdateColor = 1 << 14,
  PBVH_RebuildPixels = 1 << 15,
  PBVH_TexLeaf = 1 << 16,
  /** Used internally by `pbvh_bmesh.cc`. */
  PBVH_TopologyUpdated = 1 << 17,
};
ENUM_OPERATORS(PBVHNodeFlags, PBVH_TopologyUpdated);

/* A few C++ methods to play nice with sets and maps. */
#define PBVH_REF_CXX_METHODS(Class) \
  bool operator==(const Class b) const \
  { \
    return i == b.i; \
  } \
  uint64_t hash() const \
  { \
    return i; \
  }

struct PBVHVertRef {
  intptr_t i;

  PBVH_REF_CXX_METHODS(PBVHVertRef)
};

#define PBVH_REF_NONE -1LL

namespace blender::bke::pbvh {

class Tree;

enum class Type {
  Mesh,
  Grids,
  BMesh,
};

/* NOTE: this structure is getting large, might want to split it into
 * union'd structs */
class Node {
  friend Tree;
  /* Opaque handle for drawing code */
  draw::pbvh::PBVHBatches *draw_batches = nullptr;

  /** Axis aligned min and max of all vertex positions in the node. */
  Bounds<float3> bounds = {};
  /** Bounds from the start of current brush stroke. */
  Bounds<float3> bounds_orig = {};

  /* For internal nodes, the offset of the children in the blender::bke::pbvh::Tree
   * 'nodes' array. */
  int children_offset = 0;

  /* List of primitives for this node. Semantics depends on
   * blender::bke::pbvh::Tree type:
   *
   * - Type::Mesh: Indices into the #blender::bke::pbvh::Tree::corner_tris array.
   * - Type::Grids: Multires grid indices.
   * - Type::BMesh: Unused.  See Node.bm_faces.
   *
   * NOTE: This is a pointer inside of blender::bke::pbvh::Tree.prim_indices; it
   * is not allocated separately per node.
   */
  Span<int> prim_indices;

  /* Array of indices into the mesh's vertex array. Contains the
   * indices of all vertices used by faces that are within this
   * node's bounding box.
   *
   * Note that a vertex might be used by a multiple faces, and
   * these faces might be in different leaf nodes. Such a vertex
   * will appear in the vert_indices array of each of those leaf
   * nodes.
   *
   * In order to support cases where you want access to multiple
   * nodes' vertices without duplication, the vert_indices array
   * is ordered such that the first part of the array, up to
   * index 'uniq_verts', contains "unique" vertex indices. These
   * vertices might not be truly unique to this node, but if
   * they appear in another node's vert_indices array, they will
   * be above that node's 'uniq_verts' value.
   *
   * Used for leaf nodes in a mesh-based blender::bke::pbvh::Tree (not multires.)
   */
  Array<int, 0> vert_indices;
  /** The number of vertices in #vert_indices not shared with (owned by) another node. */
  int uniq_verts = 0;

  /* Array of indices into the Mesh's corner array.
   * Type::Mesh only.
   */
  Array<int, 0> corner_indices;

  /* An array mapping face corners into the vert_indices
   * array. The array is sized to match 'totprim', and each of
   * the face's corners gets an index into the vert_indices
   * array, in the same order as the corners in the original
   * triangle.
   *
   * Used for leaf nodes in a mesh-based blender::bke::pbvh::Tree (not multires.)
   */
  Array<int3, 0> face_vert_indices;

  /* Indicates whether this node is a leaf or not; also used for
   * marking various updates that need to be applied. */
  PBVHNodeFlags flag = PBVHNodeFlags(0);

  /* Used for ray-casting: how close the bounding-box is to the ray point. */
  float tmin = 0.0f;

  /* Dyntopo */

  /* Set of pointers to the BMFaces used by this node.
   * NOTE: Type::BMesh only. Faces are always triangles
   * (dynamic topology forcibly triangulates the mesh).
   */
  Set<BMFace *, 0> bm_faces;
  Set<BMVert *, 0> bm_unique_verts;
  Set<BMVert *, 0> bm_other_verts;

  /* Deprecated. Stores original coordinates of triangles. */
  float (*bm_orco)[3] = nullptr;
  int (*bm_ortri)[3] = nullptr;
  BMVert **bm_orvert = nullptr;
  int bm_tot_ortri = 0;

  /* Used to store the brush color during a stroke and composite it over the original color */
  PBVHColorBufferNode color_buffer;
  PBVHPixelsNode pixels;

  /* Used to flash colors of updated node bounding boxes in
   * debug draw mode (when G.debug_value / bpy.app.debug_value is 889).
   */
  int debug_draw_gen = 0;
};

class Tree {
  friend Node;
  Type type_;
  BMesh *bm_;

  Vector<Node> nodes;

  /* Memory backing for Node.prim_indices. */
  Array<int> prim_indices;

  /* Mesh data. The evaluated deform mesh for mesh sculpting, and the base mesh for grids. */
  Mesh *mesh;

  /** Local array used when not sculpting base mesh positions directly. */
  Array<float3> vert_positions_deformed;
  /** Local array used when not sculpting base mesh positions directly. */
  Array<float3> vert_normals_deformed;
  /** Local array used when not sculpting base mesh positions directly. */
  Array<float3> face_normals_deformed;

  MutableSpan<float3> vert_positions;
  Span<float3> vert_normals;
  Span<float3> face_normals;

  /* Grid Data */
  SubdivCCG *subdiv_ccg;

  /* flag are verts/faces deformed */
  bool deformed;

  /* Dynamic topology */
  float bm_max_edge_len;
  float bm_min_edge_len;
  int cd_vert_node_offset;
  int cd_face_node_offset;

  float planes[6][4];
  int num_planes;

  BMLog *bm_log;

  PBVHPixels pixels;

 public:
  ~Tree();

  Type type() const
  {
    return this->type_;
  }
};

}  // namespace blender::bke::pbvh

void BKE_pbvh_draw_debug_cb(blender::bke::pbvh::Tree &pbvh,
                            void (*draw_fn)(blender::bke::pbvh::Node *node,
                                            void *user_data,
                                            const float bmin[3],
                                            const float bmax[3],
                                            PBVHNodeFlags flag),
                            void *user_data);
