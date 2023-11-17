/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_mesh.hh"

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_sort.hh"
#include "BLI_task.hh"

#include "DNA_mesh_types.h"

#include "GEO_reorder.hh"

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
      range.index_range(),
      4096,
      true,
      [&](const IndexRange part, const bool is_range) {
        const Span<int> local_indices = indices.slice(part);
        const IndexRange local_range = range.slice(part);
        return is_range &&
               std::equal(local_indices.begin(), local_indices.end(), local_range.begin());
      },
      std::logical_and());
}

static void grouped_sort(const OffsetIndices<int> offsets,
                         const Span<float> weights,
                         MutableSpan<int> indices)
{
  const auto comparator = [&](const int index_a, const int index_b) {
    const float weight_a = weights[index_a];
    const float weight_b = weights[index_b];
    if (UNLIKELY(weight_a == weight_b)) {
      /* Approach to make it stable. */
      return index_a < index_b;
    }
    return weight_a < weight_b;
  };

  threading::parallel_for(offsets.index_range(), 250, [&](const IndexRange range) {
    for (const int group_index : range) {
      MutableSpan<int> group = indices.slice(offsets[group_index]);
      parallel_sort(group.begin(), group.end(), comparator);
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

template<typename T, typename Func>
static void parallel_transform(MutableSpan<T> values, const int64_t grain_size, const Func &func)
{
  threading::parallel_for(values.index_range(), grain_size, [&](const IndexRange range) {
    MutableSpan<T> values_range = values.slice(range);
    std::transform(values_range.begin(), values_range.end(), values_range.begin(), func);
  });
}

static int identifiers_to_indices(MutableSpan<int> r_identifiers_to_indices)
{
  const VectorSet<int> deduplicated_identifiers(r_identifiers_to_indices);
  parallel_transform(r_identifiers_to_indices, 2048, [&](const int identifier) {
    return deduplicated_identifiers.index_of(identifier);
  });

  Array<int> indices(deduplicated_identifiers.size());
  array_utils::fill_index_range<int>(indices);
  parallel_sort(indices.begin(), indices.end(), [&](const int index_a, const int index_b) {
    return deduplicated_identifiers[index_a] < deduplicated_identifiers[index_b];
  });

  parallel_transform(
      r_identifiers_to_indices, 2048, [&](const int index) { return indices[index]; });
  return deduplicated_identifiers.size();
}

static std::optional<Array<int>> sorted_indices(const bke::GeometryComponent &component,
                                                const Field<bool> selection_field,
                                                const Field<int> group_id_field,
                                                const Field<float> weight_field,
                                                const eAttrDomain domain)
{
  const int domain_size = component.attribute_domain_size(domain);
  if (domain_size == 0) {
    return std::nullopt;
  }

  const bke::GeometryFieldContext context(component, domain);
  FieldEvaluator evaluator(context, domain_size);
  evaluator.set_selection(selection_field);
  evaluator.add(group_id_field);
  evaluator.add(weight_field);
  evaluator.evaluate();
  const IndexMask mask = evaluator.get_evaluated_selection_as_mask();
  const VArray<int> group_id = evaluator.get_evaluated<int>(0);
  const VArray<float> weight = evaluator.get_evaluated<float>(1);

  if (group_id.is_single() && weight.is_single()) {
    return std::nullopt;
  }
  if (mask.is_empty()) {
    return std::nullopt;
  }

  Array<float> weight_span(domain_size);
  array_utils::copy(weight, mask, weight_span.as_mutable_span());

  Array<int> gathered_indices(mask.size());

  if (group_id.is_single()) {
    mask.to_indices<int>(gathered_indices);
    grouped_sort(Span({0, int(mask.size())}), weight_span, gathered_indices);
  }
  else {
    Array<int> gathered_group_id(mask.size());
    array_utils::gather(group_id, mask, gathered_group_id.as_mutable_span());
    const int total_groups = identifiers_to_indices(gathered_group_id);
    Array<int> gathered_offsets(total_groups + 1, 0);
    find_points_by_group_index(gathered_group_id, gathered_offsets, gathered_indices);
    parallel_transform<int>(gathered_indices, 2048, [&](const int pos) { return mask[pos]; });
    if (!weight.is_single()) {
      grouped_sort(OffsetIndices<int>(gathered_offsets), weight_span, gathered_indices);
    }
  }

  IndexMaskMemory memory;
  const IndexMask unselected = mask.complement(IndexRange(domain_size), memory);

  Array<int> indices(domain_size);

  unselected.foreach_index_optimized<int>(GrainSize(2048),
                                          [&](const int index) { indices[index] = index; });
  mask.foreach_index_optimized<int>(GrainSize(2048), [&](const int index, const int pos) {
    indices[index] = gathered_indices[pos];
  });

  if (indices_are_range(indices, indices.index_range())) {
    return std::nullopt;
  }

  return indices;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  const Field<int> group_id_field = params.extract_input<Field<int>>("Group ID");
  const Field<float> weight_field = params.extract_input<Field<float>>("Sort Weight");
  const eAttrDomain domain = eAttrDomain(params.node().custom1);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    for (const auto [type, domains] : geometry::reorder_supports().items()) {
      if (!domains.contains(domain)) {
        continue;
      }
      const bke::GeometryComponent *src_component = geometry_set.get_component(type);
      if (src_component == nullptr || src_component->is_empty()) {
        continue;
      }
      const std::optional<Array<int>> indices = sorted_indices(
          *src_component, selection_field, group_id_field, weight_field, domain);
      if (!indices.has_value()) {
        continue;
      }
      bke::GeometryComponent &dst_component = geometry::reordered_component_copy(
          *src_component, *indices, domain);
      geometry_set.remove(type);
      geometry_set.add(dst_component);
      dst_component.remove_user_and_delete_if_last();
    }
  });

  params.set_output("Geometry", std::move(geometry_set));
}

template<typename T>
static Array<EnumPropertyItem> items_value_in(const Span<T> values,
                                              const EnumPropertyItem *src_items)
{
  Vector<EnumPropertyItem> items;
  for (const EnumPropertyItem *item = src_items; item->identifier != nullptr; item++) {
    if (values.contains(T(item->value))) {
      items.append(*item);
    }
  }
  items.append({0, nullptr, 0, nullptr, nullptr});
  return items.as_span();
}

static void node_rna(StructRNA *srna)
{
  static const Array<EnumPropertyItem> supported_items = items_value_in<eAttrDomain>(
      {ATTR_DOMAIN_POINT,
       ATTR_DOMAIN_EDGE,
       ATTR_DOMAIN_FACE,
       ATTR_DOMAIN_CURVE,
       ATTR_DOMAIN_INSTANCE},
      rna_enum_attribute_domain_items);

  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "",
                    supported_items.data(),
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
