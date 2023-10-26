/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.hh"

#include "BLI_array_utils.hh"
#include "BLI_sort.hh"
#include "BLI_index_mask.hh"
#include "BLI_task.hh"

#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sort_elements_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Bool>("Selection").default_value(true).field_on_all().hide_value();
  b.add_input<decl::Int>("Group ID").field_on_all().hide_value();
  b.add_input<decl::Float>("Sort Weight").field_on_all().hide_value();

  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = ATTR_DOMAIN_POINT;
}

static bool indices_are_range(const Span<int> indices, const IndexRange range)
{
  if (indices.size() != range.size()) {
    return false;
  }
  return threading::parallel_reduce(
      range,
      4096,
      true,
      [&](const IndexRange range, const bool init) {
        if (init) {
          return std::equal(indices.begin(), indices.end(), range.begin());
        }
        return true;
      },
      [](const bool a, const bool b) { return a && b; });
}

static void grouped_sort(const OffsetIndices<int> offsets,
                         const Span<float> weights,
                         MutableSpan<int> indices)
{
  threading::parallel_for(offsets.index_range(), 250, [&](const IndexRange range) {
    for (const int group_index : range) {
      MutableSpan<int> group = indices.slice(offsets[group_index]);
      parallel_sort(group.begin(), group.end(), [&](const int index_a, const int index_b) {
        return weights[index_a] < weights[index_b];
      });
    }
  });
}

static void find_points_by_group_index(const Span<int> indices_of_curves,
                                       MutableSpan<int> r_offsets,
                                       MutableSpan<int> r_indices)
{
  offset_indices::build_reverse_offsets(indices_of_curves, r_offsets);
  Array<int> counts(r_offsets.size(), 0);

  for (const int64_t index : indices_of_curves.index_range()) {
    const int curve_index = indices_of_curves[index];
    r_indices[r_offsets[curve_index] + counts[curve_index]] = int(index);
    counts[curve_index]++;
  }
}

static int identifiers_to_indices(MutableSpan<int> r_identifiers_to_indices)
{
  const VectorSet<int> deduplicated_groups(r_identifiers_to_indices);
  threading::parallel_for(
      r_identifiers_to_indices.index_range(), 2048, [&](const IndexRange range) {
        for (int &value : r_identifiers_to_indices.slice(range)) {
          value = deduplicated_groups.index_of(value);
        }
      });

  Array<int> indices(deduplicated_groups.size());
  std::iota(indices.begin(), indices.end(), 0);
  parallel_sort(indices.begin(), indices.end(), [&](const int index_a, const int index_b) {
    return deduplicated_groups[index_a] < deduplicated_groups[index_b];
  });

  threading::parallel_for(
      r_identifiers_to_indices.index_range(), 2048, [&](const IndexRange range) {
        for (int &value : r_identifiers_to_indices.slice(range)) {
          value = indices[value];
        }
      });
  return deduplicated_groups.size();
}

static std::optional<VArray<int>> sorted_indices(const Mesh &mesh,
                                                const Field<bool> selection_field,
                                                const Field<int> group_id_field,
                                                const Field<float> weight_field,
                                                const eAttrDomain domain)
{
  const int domain_size = mesh.attributes().domain_size(domain);
  if (domain_size == 0) {
    printf("Zero domain.\n");
    return std::nullopt;
  }

  const bke::MeshFieldContext context(mesh, domain);
  FieldEvaluator evaluator(context, domain_size);
  evaluator.set_selection(selection_field);
  evaluator.add(group_id_field);
  evaluator.add(weight_field);
  evaluator.evaluate();
  const IndexMask mask = evaluator.get_evaluated_selection_as_mask();
  const VArray<int> group_id = evaluator.get_evaluated<int>(0);
  const VArray<float> weight = evaluator.get_evaluated<float>(1);

  if (group_id.is_single() && weight.is_single()) {
    printf("Single sort data.\n");
    return std::nullopt;
  }
  if (mask.is_empty()) {
    printf("Empty selection.\n");
    return std::nullopt;
  }

  Array<float> gathered_weight(mask.size());
  array_utils::gather(weight, mask, gathered_weight.as_mutable_span());

  Array<int> gathered_indices(domain_size);

  Array<int> indices(domain_size);

  if (group_id.is_single()) {
    printf("Single group sort.\n");
    array_utils::fill_index_range<int>(gathered_indices);
    grouped_sort(Span<int>({0, domain_size}), gathered_weight, gathered_indices);
  } else {
    printf("Multi group sort.\n");
    Array<int> gathered_group_id(mask.size());
    array_utils::gather(group_id, mask, gathered_group_id.as_mutable_span());
    const int total_groups = identifiers_to_indices(gathered_group_id);
    Array<int> gathered_offsets(total_groups + 1, 0);
    find_points_by_group_index(gathered_group_id, gathered_offsets, gathered_indices);
    grouped_sort(OffsetIndices<int>(gathered_offsets), gathered_weight, gathered_indices);
  }

  IndexMaskMemory memory;
  const IndexMask unselected = mask.complement(IndexRange(domain_size), memory);

  unselected.foreach_segment(GrainSize(2048), [&](const IndexMaskSegment segment) {
    const IndexRange segment_range = segment.index_range().shift(segment.first());
    MutableSpan<int> segment_indices = indices.as_mutable_span().slice(segment_range);
    std::iota(segment_indices.begin(), segment_indices.end(), segment_range.first());
  });

  mask.foreach_index_optimized<int>(GrainSize(2048), [&](const int index, const int pos){
    indices[index] = gathered_indices[pos];
  });

  if (indices_are_range(indices, indices.index_range())) {
    printf("No sort.\n");
    return std::nullopt;
  }

  return VArray<int>::ForContainer(std::move(indices));
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  const Field<int> group_id_field = params.extract_input<Field<int>>("Group ID");
  const Field<float> weight_field = params.extract_input<Field<float>>("Sort Weight");
  const eAttrDomain domain = eAttrDomain(params.node().custom1);

  const bke::AnonymousAttributePropagationInfo propagation_info =params.get_output_propagation_info("Geometry");
  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    geometry_set.keep_only_during_modify({GeometryComponent::Type::Mesh});
    /* TODO: All components with this domain. */
    const Mesh *src_mesh = geometry_set.get_mesh();
    if (src_mesh == nullptr) {
      return;
    }
    const std::optional<VArray<int>> indices = sorted_indices(*src_mesh, selection_field, group_id_field, weight_field, domain);
    if (!indices.has_value()) {
      return;
    }
    Mesh *dst_mesh = geometry_set.get_mesh_for_write();
    switch (bke::GeometryComponent::Type()) {
      case bke::GeometryComponent::Type::Mesh:
        switch (domain) {
          case ATTR_DOMAIN_POINT:
            //...
            break;
          //case ATTR_DOMAIN_POINT:
            break;
          //case ATTR_DOMAIN_POINT:
            break;
          default:
            BLI_assert_unreachable();
            break;
        }
        break;
      default: /// todo: delete thie
        break;
    }
  });

  params.set_output("Geometry", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "",
                    rna_enum_attribute_domain_items,
                    NOD_inline_enum_accessors(custom1),
                    ATTR_DOMAIN_POINT);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SORT_ELEMENTS, "Sort Elements", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sort_elements_cc
