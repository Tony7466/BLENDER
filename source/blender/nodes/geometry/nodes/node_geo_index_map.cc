/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_task.hh"

#include "UI_interface.h"
#include "UI_resources.h"

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

struct IndexMapHash {
  Span<int> keys_;
  int key_size_;

  uint64_t operator()(const int index) const
  {
    const Span<int> key = keys_.slice(index * key_size_, key_size_);
    return key.hash();
  }
};

struct IndexMapEquality {
  Span<int> keys_;
  int key_size_;

  bool operator()(const int index_a, const int index_b) const
  {
    const Span<int> key_a = keys_.slice(index_a * key_size_, key_size_);
    const Span<int> key_b = keys_.slice(index_b * key_size_, key_size_);
    return key_a == key_b;
  }
};

static Array<int> resegmentation(const Span<int> segments,
                                 const int segment_size,
                                 const int element_size)
{
  Array<int> result(segment_size * element_size);
  Vector<int> src_element;
  src_element.reserve(element_size);
  for (const int index_in_segment : IndexRange(segment_size)) {
    for (const int segment_index : IndexRange(element_size)) {
      const int index = segment_index * segment_size + index_in_segment;
      src_element.append(segments[index]);
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
  Vector<std::string> signature_input_names_;

 public:
  IndexMapFunction(const GeometrySet &geometry,
                   const eAttrDomain source_domain,
                   Vector<Field<int>> store_keys)
  {
    if (!this->create_map(geometry, source_domain, std::move(store_keys))) {
      throw std::runtime_error("cannot create map");
    }

    mf::SignatureBuilder builder{"Index Map", signature_};
    for (const int index : store_keys.index_range()) {
      std::string name = fmt::format("Key to Lookup {}", index + 1);
      builder.single_input<int>(name.c_str());
      signature_input_names_.append(std::move(name));
    }

    builder.single_output<int>("Index");
    builder.single_output<bool>("Is Valid");

    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  { /*
     Vector<VArraySpan<int>> search_keys;
     search_keys.reserve(signature_input_names_.size());
     for (const int i : signature_input_names_.index_range()) {
       search_keys.append(params.readonly_single_input<int>(i, signature_input_names_[i].c_str()));
     }

     MutableSpan<int> indices = params.uninitialized_single_output<int>(total_key_, "Index");
     MutableSpan<bool> is_valid = params.uninitialized_single_output<bool>(total_key_ + 1,
                                                                           "Is Valid");
     const int min_size = mask.min_array_size();

     Array<int> to_search_keys(min_size * total_key_);
     Array<int64_t> to_search_hashs(min_size);
     build_generic_keys(search_keys.as_span(),
                        mask,
                        total_key_,
                        to_search_keys.as_mutable_span(),
                        to_search_hashs.as_mutable_span());

     mask.foreach_index([&](const int index){
       indices[index] = set_map_.index_of_try_as(&to_search_keys[index * total_key_],
     to_search_hashs[index]); if (indices[index] == -1) { indices[index] = 0; is_valid[index] =
     false;
       }
       else {
         is_valid[index] = true;
       }
     });*/
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
    for (const int index : store_keys.index_range()) {
      MutableSpan<int> keys_segment = keys_in_segments.as_mutable_span().slice(index * domain_size,
                                                                               domain_size);
      evaluator.add_with_destination(std::move(store_keys[index]), keys_segment);
    }
    evaluator.evaluate();

    stored_keys_ = resegmentation(keys_in_segments, domain_size, key_size);

    const IndexMapHash hash{stored_keys_.as_span(), key_size};
    const IndexMapEquality equal{stored_keys_.as_span(), key_size};

    set_ = IndexSet(hash, equal);
    set_.reserve(domain_size);
    Array<int> indices(domain_size);
    std::iota(indices.begin(), indices.end(), 0);
    set_.add_multiple(indices);
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
    fn = std::make_shared<IndexMapFunction>(geometry_set, domain, std::move(store_key_fields));
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
