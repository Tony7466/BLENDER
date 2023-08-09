/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_task.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include <fmt/format.h>

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_index_map_cc {

NODE_STORAGE_FUNCS(NodeAccumulateField)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Source");

  b.add_input<decl::Int>("Keys").field_on_all().multi_input();
  b.add_input<decl::Int>("Search Keys").field_on_all().multi_input();

  b.add_output<decl::Int>("Index").dependent_field({2});
  b.add_output<decl::Bool>("Is Valid").dependent_field({2});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "domain", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = ATTR_DOMAIN_POINT;
}

using IndexMapKey = Span<int>;

struct IndexMapHash {
  Span<int> keys;
  int key_size;

  uint64_t operator()(const IndexMapKey value) const
  {
    return value.hash();
  }

  uint64_t operator()(const int index) const
  {
    const IndexMapKey key = keys.slice(index * key_size, key_size);
    return key.hash();
  }
};

struct IndexMapEquality {
  Span<int> keys;
  int key_size;
  ;

  bool operator()(const IndexMapKey value_a, const int index_b) const
  {
    const IndexMapKey key_b = keys.slice(index_b * key_size, key_size);
    return value_a == key_b;
  }

  bool operator()(const int index_a, const int index_b) const
  {
    const IndexMapKey key_a = keys.slice(index_a * key_size, key_size);
    const IndexMapKey key_b = keys.slice(index_b * key_size, key_size);
    return key_a == key_b;
  }
};

static Array<int> resegmentation(const Span<Span<int>> segments,
                                 const int segment_size,
                                 const int element_size)
{
  Array<int> result(segment_size * element_size);
  Vector<int> src_element;
  src_element.reserve(element_size);
  for (const int index_in_segment : IndexRange(segment_size)) {
    for (const int segment_index : IndexRange(element_size)) {
      src_element.append(segments[segment_index][index_in_segment]);
    }
    MutableSpan<int> dst_element = result.as_mutable_span().slice(index_in_segment * element_size,
                                                                  element_size);
    std::copy(src_element.begin(), src_element.end(), dst_element.begin());
    src_element.clear();
  }
  return result;
}

static bool component_is_available(const GeometrySet &geometry,
                                   const GeometryComponent::Type type,
                                   const eAttrDomain domain)
{
  if (!geometry.has(type)) {
    return false;
  }
  const GeometryComponent &component = *geometry.get_component(type);
  return component.attribute_domain_size(domain) != 0;
}

static const GeometryComponent *find_source_component(const GeometrySet &geometry,
                                                      const eAttrDomain domain)
{
  static const Array<GeometryComponent::Type> supported_types = {
      GeometryComponent::Type::Mesh,
      GeometryComponent::Type::PointCloud,
      GeometryComponent::Type::Curve,
      GeometryComponent::Type::Instance};
  for (const GeometryComponent::Type src_type : supported_types) {
    if (component_is_available(geometry, src_type, domain)) {
      return geometry.get_component(src_type);
    }
  }

  return nullptr;
}

class IndexMapFunction : public mf::MultiFunction {
 private:
  Array<int> stored_keys_;
  using IndexSet = Set<int, 0, DefaultProbingStrategy, IndexMapHash, IndexMapEquality>;
  IndexSet set_;

  mf::Signature signature_;
  Array<std::string> signature_input_names_;

 public:
  IndexMapFunction(const GeometrySet &geometry,
                   const eAttrDomain source_domain,
                   Vector<Field<int>> store_keys)
  {
    const int total_keys = store_keys.size();
    if (!this->create_map(geometry, source_domain, std::move(store_keys))) {
      throw std::runtime_error("cannot create map");
    }

    mf::SignatureBuilder builder{"Index Map", signature_};
    signature_input_names_.reinitialize(total_keys);
    for (const int index : IndexRange(total_keys)) {
      signature_input_names_[index] = fmt::format("Key to Lookup {}", index + 1);
      builder.single_input<int>(signature_input_names_[index].c_str());
    }

    builder.single_output<int>("Index");
    builder.single_output<bool>("Is Valid");

    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const int total_keys = signature_input_names_.size();
    Array<VArraySpan<int>> search_key_arrays(total_keys);
    Array<Span<int>> search_keys(total_keys);
    for (const int index : IndexRange(total_keys)) {
      const StringRef input_name(signature_input_names_[index]);
      search_key_arrays[index] = params.readonly_single_input<int>(index, input_name);
      search_keys[index] = search_key_arrays[index];
    }

    MutableSpan<int> indices = params.uninitialized_single_output<int>(total_keys, "Index");
    MutableSpan<bool> is_valid = params.uninitialized_single_output<bool>(total_keys + 1,
                                                                          "Is Valid");

    const Array<int> all_keys = resegmentation(search_keys, mask.min_array_size(), total_keys);
    mask.foreach_index([&](const int index) {
      const IndexMapKey key = all_keys.as_span().slice(index * total_keys, total_keys);
      indices[index] = set_.lookup_key_default_as(key, -1);
    });
  }

 protected:
  bool create_map(const GeometrySet &geometry,
                  const eAttrDomain source_domain,
                  Vector<Field<int>> store_keys)
  {
    const GeometryComponent *component = find_source_component(geometry, source_domain);
    if (component == nullptr) {
      return false;
    }
    bke::GeometryFieldContext geometry_context(*component, source_domain);
    const int domain_size = component->attributes()->domain_size(source_domain);
    FieldEvaluator evaluator(geometry_context, domain_size);

    const int key_size = store_keys.size();
    Array<int> keys_in_segments(key_size * domain_size);
    Array<Span<int>> segments(key_size);
    for (const int index : store_keys.index_range()) {
      MutableSpan<int> keys_segment = keys_in_segments.as_mutable_span().slice(index * domain_size,
                                                                               domain_size);
      segments[index] = keys_segment;
      evaluator.add_with_destination(std::move(store_keys[index]), keys_segment);
    }
    evaluator.evaluate();

    stored_keys_ = resegmentation(segments, domain_size, key_size);

    const IndexMapHash hash{stored_keys_.as_span(), key_size};
    const IndexMapEquality equal{stored_keys_.as_span(), key_size};

    set_ = IndexSet(hash, equal);
    set_.reserve(domain_size);
    Array<int> indices(domain_size);
    std::iota(indices.begin(), indices.end(), 0);
    set_.add_multiple(indices);
    return true;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Source");
  const eAttrDomain domain = eAttrDomain(params.node().custom1);

  const Vector<fn::ValueOrField<int>> store_keys =
      params.extract_input<Vector<fn::ValueOrField<int>>>("Keys");
  const Vector<fn::ValueOrField<int>> search_keys =
      params.extract_input<Vector<fn::ValueOrField<int>>>("Search Keys");

  if (store_keys.size() != search_keys.size()) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Number of inputs to search and store keys non the same"));
    params.set_default_remaining_outputs();
    return;
  }

  if (store_keys.is_empty()) {
    params.set_default_remaining_outputs();
    return;
  }

  Vector<Field<int>> store_key_fields;
  store_key_fields.reserve(store_keys.size());
  for (const fn::ValueOrField<int> key_value_field : store_keys) {
    store_key_fields.append(key_value_field.as_field());
  }

  Vector<GField> search_key_fields;
  search_key_fields.reserve(search_keys.size());
  for (const fn::ValueOrField<int> key_value_field : search_keys) {
    search_key_fields.append(key_value_field.as_field());
  }

  std::shared_ptr<IndexMapFunction> fn;
  try {
    fn = std::make_shared<IndexMapFunction>(geometry_set, domain, store_key_fields);
  }
  catch (const std::runtime_error &) {
    params.set_default_remaining_outputs();
    return;
  }

  auto op = FieldOperation::Create(std::move(fn), std::move(search_key_fields));

  params.set_output("Index", Field<int>(op, 0));
  params.set_output("Is Valid", Field<bool>(std::move(op), 1));
}

}  // namespace blender::nodes::node_geo_index_map_cc

void register_node_type_geo_index_map()
{
  namespace file_ns = blender::nodes::node_geo_index_map_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INDEX_MAP, "Index Map", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.initfunc = file_ns::node_init;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
