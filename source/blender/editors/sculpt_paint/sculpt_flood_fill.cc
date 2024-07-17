#include "DNA_mesh_types.h"

#include "paint_intern.hh"
#include "sculpt_intern.hh"

/* -------------------------------------------------------------------- */
/** \name Sculpt Flood Fill API
 *
 * Iterate over connected vertices, starting from one or more initial vertices.
 * \{ */

namespace blender::ed::sculpt_paint {

namespace flood_fill {

FillData init_fill(SculptSession &ss)
{
  SCULPT_vertex_random_access_ensure(ss);
  FillData data;
  data.visited_verts.resize(SCULPT_vertex_count_get(ss));
  return data;
}

void add_initial(FillData &flood, PBVHVertRef vertex)
{
  flood.queue.push(vertex);
}

void FillDataMesh::add_initial(const int vertex)
{
  this->queue.push(vertex);
}

void FillDataGrids::add_initial(const SubdivCCGCoord vertex)
{
  this->queue.push(vertex);
}

void FillDataBMesh::add_initial(BMVert *vertex)
{
  this->queue.push(vertex);
}

void add_and_skip_initial(FillData &flood, PBVHVertRef vertex)
{
  flood.queue.push(vertex);
  flood.visited_verts[vertex.i].set(vertex.i);
}

void FillDataMesh::add_and_skip_initial(const int vertex, const int index)
{
  this->queue.push(vertex);
  this->visited_verts[index].set();
}

void FillDataGrids::add_and_skip_initial(const SubdivCCGCoord vertex, const int index)
{
  this->queue.push(vertex);
  this->visited_verts[index].set();
}

void FillDataBMesh::add_and_skip_initial(BMVert *vertex, const int index)
{
  this->queue.push(vertex);
  this->visited_verts[index].set();
}

void add_initial_with_symmetry(const Object &ob,
                               const SculptSession &ss,
                               FillData &flood,
                               PBVHVertRef vertex,
                               const float radius)
{
  if (radius <= 0.0f) {
    if (vertex.i != PBVH_REF_NONE) {
      add_initial(flood, vertex);
    }
    return;
  }

  /* Add active vertex and symmetric vertices to the queue. */
  const char symm = SCULPT_mesh_symmetry_xyz_get(ob);
  for (char i = 0; i <= symm; ++i) {
    if (!SCULPT_is_symmetry_iteration_valid(i, symm)) {
      continue;
    }
    PBVHVertRef v = {PBVH_REF_NONE};

    if (i == 0) {
      v = vertex;
    }
    else {
      BLI_assert(radius > 0.0f);
      const float radius_squared = (radius == FLT_MAX) ? FLT_MAX : radius * radius;
      float3 location;
      flip_v3_v3(location, SCULPT_vertex_co_get(ss, vertex), ePaintSymmetryFlags(i));
      v = nearest_vert_calc(ob, location, radius_squared, false);
    }

    if (v.i != PBVH_REF_NONE) {
      add_initial(flood, v);
    }
  }
}

void FillDataMesh::add_initial_with_symmetry(const Object &object,
                                             const SculptSession &ss,
                                             const int vertex,
                                             const float radius)
{
  if (radius <= 0.0f) {
    this->add_initial(vertex);
    return;
  }

  const Mesh &mesh = *static_cast<const Mesh *>(object.data);
  const Span<float3> vert_positions = BKE_pbvh_get_vert_positions(*ss.pbvh);
  const bke::AttributeAccessor attributes = mesh.attributes();
  VArraySpan<bool> hide_vert = *attributes.lookup<bool>(".hide_vert", bke::AttrDomain::Point);

  const char symm = SCULPT_mesh_symmetry_xyz_get(object);
  for (char i = 0; i <= symm; ++i) {
    if (!SCULPT_is_symmetry_iteration_valid(i, symm)) {
      continue;
    }

    std::optional<int> vert_to_add;
    if (i == 0) {
      vert_to_add = vertex;
    }
    else {
      BLI_assert(radius > 0.0f);
      const float radius_squared = (radius == FLT_MAX) ? FLT_MAX : radius * radius;
      float3 location;
      flip_v3_v3(location, vert_positions[vertex], ePaintSymmetryFlags(i));
      vert_to_add = nearest_vert_calc_mesh(
          *ss.pbvh, vert_positions, hide_vert, location, radius_squared, false);
    }

    if (vert_to_add) {
      this->add_initial(*vert_to_add);
    }
  }
}

void FillDataGrids::add_initial_with_symmetry(const Object &object,
                                              const SculptSession &ss,
                                              SubdivCCGCoord vertex,
                                              float radius)
{
  if (radius <= 0.0f) {
    this->add_initial(vertex);
    return;
  }

  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

  const char symm = SCULPT_mesh_symmetry_xyz_get(object);
  for (char i = 0; i <= symm; ++i) {
    if (!SCULPT_is_symmetry_iteration_valid(i, symm)) {
      continue;
    }

    std::optional<SubdivCCGCoord> vert_to_add;
    if (i == 0) {
      vert_to_add = vertex;
    }
    else {
      BLI_assert(radius > 0.0f);
      const float radius_squared = (radius == FLT_MAX) ? FLT_MAX : radius * radius;
      float3 location;
      CCGElem *elem = ss.subdiv_ccg->grids[vertex.grid_index];
      flip_v3_v3(
          location, CCG_grid_elem_co(key, elem, vertex.x, vertex.y), ePaintSymmetryFlags(i));
      vert_to_add = nearest_vert_calc_grids(*ss.pbvh, subdiv_ccg, location, radius_squared, false);
    }

    if (vert_to_add) {
      this->add_initial(*vert_to_add);
    }
  }
}

void FillDataBMesh::add_initial_with_symmetry(const Object &object,
                                              const SculptSession &ss,
                                              BMVert *vertex,
                                              float radius)
{
  if (radius <= 0.0f) {
    this->add_initial(vertex);
    return;
  }

  const char symm = SCULPT_mesh_symmetry_xyz_get(object);
  for (char i = 0; i <= symm; ++i) {
    if (!SCULPT_is_symmetry_iteration_valid(i, symm)) {
      continue;
    }

    std::optional<BMVert *> vert_to_add;
    if (i == 0) {
      vert_to_add = vertex;
    }
    else {
      BLI_assert(radius > 0.0f);
      const float radius_squared = (radius == FLT_MAX) ? FLT_MAX : radius * radius;
      float3 location;
      flip_v3_v3(location, vertex->co, ePaintSymmetryFlags(i));
      vert_to_add = nearest_vert_calc_bmesh(*ss.pbvh, location, radius_squared, false);
    }

    if (vert_to_add) {
      this->add_initial(*vert_to_add);
    }
  }
}

void add_active(const Object &ob, const SculptSession &ss, FillData &flood, float radius)
{
  add_initial_with_symmetry(ob, ss, flood, SCULPT_active_vertex_get(ss), radius);
}

void FillDataMesh::add_active(const Object &object, const SculptSession &ss, float radius)
{
  PBVHVertRef active_vert = SCULPT_active_vertex_get(ss);
  this->add_initial_with_symmetry(object, ss, active_vert.i, radius);
}

void FillDataGrids::add_active(const Object &object, const SculptSession &ss, float radius)
{
  PBVHVertRef active_vert = SCULPT_active_vertex_get(ss);

  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  const int grid_index = active_vert.i / key.grid_area;
  const int index_in_grid = active_vert.i - grid_index * key.grid_area;
  SubdivCCGCoord coord{};
  coord.grid_index = grid_index;
  coord.x = index_in_grid % key.grid_size;
  coord.y = index_in_grid / key.grid_size;

  this->add_initial_with_symmetry(object, ss, coord, radius);
}

void FillDataBMesh::add_active(const Object &object, const SculptSession &ss, float radius)
{
  PBVHVertRef active_vert = SCULPT_active_vertex_get(ss);
  this->add_initial_with_symmetry(object, ss, (BMVert *)active_vert.i, radius);
}

void execute(SculptSession &ss,
             FillData &flood,
             FunctionRef<bool(PBVHVertRef from_v, PBVHVertRef to_v, bool is_duplicate)> func)
{
  while (!flood.queue.empty()) {
    PBVHVertRef from_v = flood.queue.front();
    flood.queue.pop();

    SculptVertexNeighborIter ni;
    SCULPT_VERTEX_DUPLICATES_AND_NEIGHBORS_ITER_BEGIN (ss, from_v, ni) {
      const PBVHVertRef to_v = ni.vertex;
      int to_v_i = BKE_pbvh_vertex_to_index(*ss.pbvh, to_v);

      if (flood.visited_verts[to_v_i]) {
        continue;
      }

      if (!hide::vert_visible_get(ss, to_v)) {
        continue;
      }

      flood.visited_verts[BKE_pbvh_vertex_to_index(*ss.pbvh, to_v)].set();

      if (func(from_v, to_v, ni.is_duplicate)) {
        flood.queue.push(to_v);
      }
    }
    SCULPT_VERTEX_NEIGHBORS_ITER_END(ni);
  }
}

}  // namespace flood_fill

}  // namespace blender::ed::sculpt_paint

/** \} */
