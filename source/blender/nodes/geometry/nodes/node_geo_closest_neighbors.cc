/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_array_utils.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_bvhutils.h"
#include "BKE_pointcloud.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_closest_neighbors_cc {

using TargetElement = GeometryNodeClosestNeighborsTargetType;

NODE_STORAGE_FUNCS(NodeGeometryClosestNeighbors)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on({0});
  b.add_input<decl::Vector>("Source Position")
      .implicit_field_on(implicit_field_inputs::position, {0});
  b.add_input<decl::Float>("Max Distance")
      .default_value(100.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .supports_field()
      .field_on({0});
  b.add_input<decl::Int>("Max Neighbors")
      .default_value(10)
      .min(1)
      .max(1000)
      .subtype(PROP_UNSIGNED);
  b.add_input<decl::Geometry>("Target Geometry")
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud});

  b.add_output<decl::Geometry>("Geometry").propagate_all();
  b.add_output<decl::Int>("Neighbor Count").dependent_field({1, 2, 3}).field_on({0});
  b.add_output<decl::Int>("First Neighbor").dependent_field({1, 2, 3}).field_on({0});
  b.add_output<decl::Geometry>("Neighbors");
  b.add_output<decl::Int>("Source Index").field_source().field_on({3});
  b.add_output<decl::Int>("Target Index").field_source().field_on({3});
  b.add_output<decl::Vector>("Target Position").field_source().field_on({3});
  b.add_output<decl::Vector>("Target Normal").field_source().field_on({3});
  b.add_output<decl::Float>("Target Distance").field_source().field_on({3});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "target_element", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryClosestNeighbors *data = MEM_cnew<NodeGeometryClosestNeighbors>(__func__);
  node->storage = data;
}

static void node_update(bNodeTree * /*ntree*/, bNode * /*node*/)
{
  // const NodeGeometryClosestNeighbors &storage = node_storage(*node);
}

struct NeighborPointData {
  Vector<int> source_indices;
  Vector<int> target_indices;
  Vector<float3> target_positions;
  Vector<float3> target_normals;
  Vector<float> target_distances;
};
using ThreadLocalData = threading::EnumerableThreadSpecific<NeighborPointData>;

static void find_closest_neighbors(const IndexMask &mask,
                                   BVHTree *target_tree,
                                   BVHTree_NearestPointCallback nearest_cb,
                                   void *nearest_userdata,
                                   const VArray<float3> &source_positions,
                                   const VArray<float> &max_distances,
                                   const int max_neighbors,
                                   MutableSpan<int> &r_neighbor_counts,
                                   MutableSpan<int> &r_first_neighbors,
                                   ThreadLocalData &thread_storage)
{
  /* All pairs for the same source point are handled by the same thread,
   * so we can rely on pairs for each point to be in a contiguous array,
   * without additional sorting after the overlap search. */

  mask.foreach_index(GrainSize(128), [&](const int64_t source_i) {
    NeighborPointData &data = thread_storage.local();
    const float max_distance = max_distances[source_i];
    const float3 source_position = source_positions[source_i];

    /* Note: this is relative to the thread-local array,
     * final start index is computed when concatenating. */
    const int64_t first_neighbor = data.source_indices.size();

    /* Note the "distance" stored at first is actually the squared distance.
     * This way comparing distances avoids square roots.
     * After range queries the square roots are computed. */
    BLI_bvhtree_range_query_cpp(
        *target_tree,
        source_position,
        max_distance,
        [&](int target_i, const float3 &co, float /*dist_sq*/) {
          BVHTreeNearest nearest;
          nearest.index = -1;
          nearest.dist_sq = max_distance; /* #nearest_cb has an internal distance check. */
          nearest_cb(nearest_userdata, target_i, co, &nearest);

          const int64_t neighbor_count = data.source_indices.size() - first_neighbor;
          const Span<float> dist_sq_span = data.target_distances.as_span().slice(first_neighbor,
                                                                                 neighbor_count);
          const float *ptr = std::lower_bound(
              dist_sq_span.begin(), dist_sq_span.end(), nearest.dist_sq);
          const int index = ptr - dist_sq_span.data();
          if (index >= max_neighbors) {
            /* Already have enough closer contacts. */
            return;
          }
          if (neighbor_count == max_neighbors) {
            data.source_indices.pop_last();
            data.target_indices.pop_last();
            data.target_positions.pop_last();
            data.target_normals.pop_last();
            data.target_distances.pop_last();
          }
          data.source_indices.insert(index, (int)source_i);
          data.target_indices.insert(index, target_i);
          data.target_positions.insert(index, nearest.co);
          data.target_normals.insert(index, nearest.no);
          data.target_distances.insert(index, nearest.dist_sq);
        });

    /* Compute distances. */
    const IndexRange full_range{first_neighbor, data.source_indices.size() - first_neighbor};
    for (float &dist : data.target_distances.as_mutable_span().slice(full_range)) {
      dist = math::sqrt(dist);
    }

    /* Note: this is relative to the thread-local array,
     * final start index is computed when concatenating. */
    if (!r_first_neighbors.is_empty()) {
      r_first_neighbors[source_i] = first_neighbor;
    }
    if (!r_neighbor_counts.is_empty()) {
      r_neighbor_counts[source_i] = thread_storage.local().source_indices.size() - first_neighbor;
    }
  });
}

static void gather_thread_storage(ThreadLocalData &thread_storage,
                                  NeighborPointData &r_neighbors,
                                  MutableSpan<int> r_first_neighbors)
{

  int64_t total_neighbors = 0;
  for (const NeighborPointData &local_data : thread_storage) {
    const int64_t local_size = local_data.source_indices.size();
    BLI_assert(local_data.target_indices.size() == local_size);
    BLI_assert(local_data.target_positions.size() == local_size);
    BLI_assert(local_data.target_normals.size() == local_size);
    BLI_assert(local_data.target_distances.size() == local_size);
    total_neighbors += local_size;
  }
  const int64_t start_index = r_neighbors.source_indices.size();
  const int64_t new_size = start_index + total_neighbors;
  r_neighbors.source_indices.reserve(new_size);
  r_neighbors.target_indices.reserve(new_size);
  r_neighbors.target_positions.reserve(new_size);
  r_neighbors.target_normals.reserve(new_size);
  r_neighbors.target_distances.reserve(new_size);

  IndexRange current_range = {};
  for (NeighborPointData &local_data : thread_storage) {
    const int64_t local_size = local_data.source_indices.size();
    current_range = IndexRange(current_range.size(), local_size);

    r_neighbors.source_indices.extend(local_data.source_indices);
    r_neighbors.target_indices.extend(local_data.target_indices);
    r_neighbors.target_positions.extend(local_data.target_positions);
    r_neighbors.target_normals.extend(local_data.target_normals);
    r_neighbors.target_distances.extend(local_data.target_distances);

    /* Update the neighbor start indices. */
    if (!r_first_neighbors.is_empty()) {
      int prev_source_i = -1;
      for (const int source_i : local_data.source_indices) {
        if (source_i != prev_source_i) {
          r_first_neighbors[source_i] += (int)current_range.start();
          prev_source_i = source_i;
        }
      }
    }
  }
}

static void find_closest_neighbors_on_component(GeometryComponent &component,
                                                eAttrDomain domain,
                                                const Field<bool> &selection_field,
                                                const Field<float3> &source_position_field,
                                                const Field<float> &max_distance_field,
                                                const int max_neighbors,
                                                const GeometrySet &target,
                                                const TargetElement target_element,
                                                const AttributeIDRef first_neighbor_id,
                                                const AttributeIDRef neighbor_count_id,
                                                NeighborPointData &r_neighbors)
{
  const int domain_size = component.attribute_domain_size(domain);
  if (domain_size == 0) {
    return;
  }

  bke::GeometryFieldContext field_context{component, domain};
  fn::FieldEvaluator evaluator{field_context, domain_size};
  evaluator.set_selection(selection_field);
  evaluator.add(source_position_field);
  evaluator.add(max_distance_field);
  evaluator.evaluate();

  const IndexMask selection = evaluator.get_evaluated_selection_as_mask();
  if (selection.is_empty()) {
    return;
  }
  const VArray<float3> source_position_input = evaluator.get_evaluated<float3>(0);
  const VArray<float> max_distance_input = evaluator.get_evaluated<float>(1);

  MutableAttributeAccessor attributes = *component.attributes_for_write();
  SpanAttributeWriter<int> out_neighbor_count = attributes.lookup_or_add_for_write_only_span<int>(
      neighbor_count_id, domain);
  SpanAttributeWriter<int> out_first_neighbor = attributes.lookup_or_add_for_write_only_span<int>(
      first_neighbor_id, domain);

  ThreadLocalData thread_storage;

  switch (component.type()) {
    case GeometryComponent::Type::Mesh: {
      if (!target.has_mesh()) {
        break;
      }
      BVHCacheType cache_type = BVHTREE_FROM_VERTS;
      switch (target_element) {
        case GEO_NODE_CLOSEST_NEIGHBOR_TARGET_POINTS:
          cache_type = BVHTREE_FROM_VERTS;
          break;
        case GEO_NODE_CLOSEST_NEIGHBOR_TARGET_EDGES:
          cache_type = BVHTREE_FROM_EDGES;
          break;
        case GEO_NODE_CLOSEST_NEIGHBOR_TARGET_FACES:
          cache_type = BVHTREE_FROM_LOOPTRI;
          break;
      }
      BVHTreeFromMesh tree_data;
      BKE_bvhtree_from_mesh_get(&tree_data, target.get_mesh_for_read(), cache_type, 4);
      BLI_SCOPED_DEFER([&]() { free_bvhtree_from_mesh(&tree_data); });
      if (tree_data.tree == nullptr) {
        return;
      }

      find_closest_neighbors(selection,
                             tree_data.tree,
                             tree_data.nearest_callback,
                             &tree_data,
                             source_position_input,
                             max_distance_input,
                             max_neighbors,
                             out_neighbor_count.span,
                             out_first_neighbor.span,
                             thread_storage);
      break;
    }
    case GeometryComponent::Type::PointCloud: {
      if (!target.has_pointcloud()) {
        break;
      }
      BVHTreeFromPointCloud tree_data;
      BVHTree *tree = BKE_bvhtree_from_pointcloud_get(
          &tree_data, target.get_pointcloud_for_read(), 4);
      BLI_SCOPED_DEFER([&]() { free_bvhtree_from_pointcloud(&tree_data); });
      if (tree == nullptr) {
        return;
      }
      /* Pointcloud BVH tree has no nearest callback, lets roll our own. */
      auto nearest_cb = [](void *userdata, int index, const float co[3], BVHTreeNearest *nearest) {
        const BVHTreeFromPointCloud *data = static_cast<BVHTreeFromPointCloud *>(userdata);
        nearest->index = index;
        const float3 pos = data->coords[index];
        copy_v3_v3(nearest->co, pos);
        nearest->dist_sq = math::distance_squared(float3(co), pos);
        zero_v3(nearest->no);
      };
      find_closest_neighbors(selection,
                             tree,
                             nearest_cb,
                             &tree_data,
                             source_position_input,
                             max_distance_input,
                             max_neighbors,
                             out_neighbor_count.span,
                             out_first_neighbor.span,
                             thread_storage);
      break;
    }
  }

  gather_thread_storage(thread_storage, r_neighbors, out_first_neighbor.span);

  out_neighbor_count.finish();
  out_first_neighbor.finish();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet source = params.extract_input<GeometrySet>("Geometry");
  GeometrySet target = params.extract_input<GeometrySet>("Target Geometry");
  //  const NodeGeometryClosestNeighbors &storage = node_storage(params.node());
  const eAttrDomain domain = eAttrDomain(params.node().custom1);
  const TargetElement target_element = TargetElement(params.node().custom2);
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<float3> source_position_field = params.extract_input<Field<float3>>("Source Position");
  Field<float> max_distance_field = params.extract_input<Field<float>>("Max Distance");
  int max_neighbors = params.extract_input<int>("Max Neighbors");

  if (source.is_empty() || target.is_empty()) {
    params.set_default_remaining_outputs();
    return;
  }

  AnonymousAttributeIDPtr first_neighbor_id = params.get_output_anonymous_attribute_id_if_needed(
      "First Neighbor");
  AnonymousAttributeIDPtr neighbor_count_id = params.get_output_anonymous_attribute_id_if_needed(
      "Neighbor Count");

  NeighborPointData neighbors;

  static const Array<GeometryComponent::Type> types = {GeometryComponent::Type::Mesh,
                                                       GeometryComponent::Type::PointCloud,
                                                       GeometryComponent::Type::Curve};

  for (const GeometryComponent::Type type : types) {
    if (source.has(type)) {
      GeometryComponent &component = source.get_component_for_write(type);
      find_closest_neighbors_on_component(component,
                                          domain,
                                          selection_field,
                                          source_position_field,
                                          max_distance_field,
                                          max_neighbors,
                                          target,
                                          target_element,
                                          first_neighbor_id.get(),
                                          neighbor_count_id.get(),
                                          neighbors);
    }
  }

  PointCloud *points = BKE_pointcloud_new_nomain(neighbors.source_indices.size());
  MutableAttributeAccessor neighbors_attributes = points->attributes_for_write();
  AnonymousAttributeIDPtr source_index_id = params.get_output_anonymous_attribute_id_if_needed(
      "Source Index");
  AnonymousAttributeIDPtr target_index_id = params.get_output_anonymous_attribute_id_if_needed(
      "Target Index");
  AnonymousAttributeIDPtr target_position_id = params.get_output_anonymous_attribute_id_if_needed(
      "Target Position");
  AnonymousAttributeIDPtr target_normal_id = params.get_output_anonymous_attribute_id_if_needed(
      "Target Normal");
  AnonymousAttributeIDPtr target_distance_id = params.get_output_anonymous_attribute_id_if_needed(
      "Target Distance");

  neighbors_attributes.add<int>(
      source_index_id.get(),
      ATTR_DOMAIN_POINT,
      bke::AttributeInitVArray(VArray<int>::ForSpan(neighbors.source_indices)));
  neighbors_attributes.add<int>(
      target_index_id.get(),
      ATTR_DOMAIN_POINT,
      bke::AttributeInitVArray(VArray<int>::ForSpan(neighbors.target_indices)));
  neighbors_attributes.add<float3>(
      target_position_id.get(),
      ATTR_DOMAIN_POINT,
      bke::AttributeInitVArray(VArray<float3>::ForSpan(neighbors.target_positions)));
  neighbors_attributes.add<float3>(
      target_normal_id.get(),
      ATTR_DOMAIN_POINT,
      bke::AttributeInitVArray(VArray<float3>::ForSpan(neighbors.target_normals)));
  neighbors_attributes.add<float>(
      target_distance_id.get(),
      ATTR_DOMAIN_POINT,
      bke::AttributeInitVArray(VArray<float>::ForSpan(neighbors.target_distances)));

  GeometrySet out_neighbors = GeometrySet::create_with_pointcloud(points);

  params.set_output("Geometry", source);
  // if (neighbor_count_id) {
  //   params.set_output("Neighbor Count",
  //                     AnonymousAttributeFieldInput::Create<int>(neighbor_count_id, __func__));
  // }
  // if (first_neighbor_id) {
  //   params.set_output("First Neighbor",
  //                     AnonymousAttributeFieldInput::Create<int>(first_neighbor_id, __func__));
  // }
  params.set_output("Neighbors", std::move(out_neighbors));
  // if (source_index_id) {
  //   params.set_output("Source Index",
  //                     AnonymousAttributeFieldInput::Create<int>(source_index_id, __func__));
  // }
  // if (target_index_id) {
  //   params.set_output("Target Index",
  //                     AnonymousAttributeFieldInput::Create<int>(target_index_id, __func__));
  // }
  // if (target_position_id) {
  //   params.set_output("Target Position",
  //                     AnonymousAttributeFieldInput::Create<float3>(target_position_id,
  //                     __func__));
  // }
  // if (target_normal_id) {
  //   params.set_output("Target Normal",
  //                     AnonymousAttributeFieldInput::Create<float3>(target_normal_id, __func__));
  // }
  // if (target_distance_id) {
  //   params.set_output("Target Distance",
  //                     AnonymousAttributeFieldInput::Create<float>(target_distance_id,
  //                     __func__));
  // }

  params.set_default_remaining_outputs();
}

}  // namespace blender::nodes::node_geo_closest_neighbors_cc

void register_node_type_geo_closest_neighbors()
{
  namespace file_ns = blender::nodes::node_geo_closest_neighbors_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_CLOSEST_NEIGHBORS, "Closest Neighbors", NODE_CLASS_GEOMETRY);
  blender::bke::node_type_size_preset(&ntype, blender::bke::eNodeSizePreset::MIDDLE);
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  node_type_storage(&ntype,
                    "NodeGeometryClosestNeighbors",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  nodeRegisterType(&ntype);
}
