/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_task.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_index_map_cc {

NODE_STORAGE_FUNCS(NodeAccumulateField)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Source"));

  b.add_input<decl::Int>(N_("Keys")).field_on_all().multi_input();
  b.add_input<decl::Int>(N_("Search Keys")).field_on_all().multi_input();

  b.add_output<decl::Int>(N_("Index")).field_on_all().dependent_field({2});
  b.add_output<decl::Bool>(N_("Is Valid")).field_on_all().dependent_field({2});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "domain", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = ATTR_DOMAIN_POINT;
}

struct IndexMapHash {
  int total_key;

  uint64_t operator()(const int *value) const
  {
    int64_t hash = int64_t(*value);
    for (const int index : IndexRange(total_key).drop_front(1)) {
      hash = get_default_hash_2<int64_t>(hash, int64_t(*(value + index)));
    }
    return hash;
  }
};

struct IndexMapEquality {
  int total_key;

  bool operator()(const int *a, const int *b) const
  {
    for (const int index : IndexRange(total_key)) {
      if (*(a + index) != *(b + index)) {
        return false;
      }
    }
    return true;
  }
};

static void build_generic_keys(const Span<VArraySpan<int>> keys,
                               const IndexMask mask,
                               const int total_key,
                               MutableSpan<int> r_values,
                               MutableSpan<int64_t> r_hashs)
{
  threading::parallel_for(mask.index_range(), 1024, [&](const IndexRange range) {
    for (const int key_i : keys.index_range()) {
      const VArraySpan<int> &key = keys[key_i];
      for (const int index : mask.slice(range)) {
        r_values[index * total_key + key_i] = key[index];
      }
    }
    for (const int index : mask.slice(range)) {
      r_hashs[index] = int64_t(r_values[index * total_key]);
    }
    for (const int key_i : keys.index_range().drop_front(1)) {
      for (const int index : mask.slice(range)) {
        const int64_t previos = r_hashs[index];
        const int64_t another = int64_t(r_values[index * total_key + key_i]);
        const int64_t new_hash = get_default_hash_2<int64_t>(previos, another);
        r_hashs[index] = new_hash;
      }
    }
  });
}

class IndexMapFunction : public mf::MultiFunction {
 private:
  const GeometrySet geometry_;
  const eAttrDomain source_domain_;

  const int total_key_;

  Array<std::string> signature_input_names_;

  using Set = VectorSet<const int *, DefaultProbingStrategy, IndexMapHash, IndexMapEquality>;
  Set set_map_;
  Array<int> keys_;

  std::optional<bke::GeometryFieldContext> geometry_context_;
  std::unique_ptr<FieldEvaluator> evaluator_;

  mf::Signature signature_;

 public:
  IndexMapFunction(GeometrySet geometry, eAttrDomain source_domain, Vector<Field<int>> store_keys)
      : geometry_(std::move(geometry)),
        source_domain_(source_domain),
        total_key_(store_keys.size()),
        signature_input_names_(total_key_),
        set_map_(IndexMapHash{total_key_}, IndexMapEquality{total_key_})
  {
    const GeometryComponent *component = bke::find_source_component(geometry_, source_domain_);
    if (!component) {
      throw std::runtime_error("cannot find component");
    }

    geometry_context_.emplace(bke::GeometryFieldContext(*component, source_domain_));
    const int domain_size = geometry_context_->attributes()->domain_size(source_domain_);
    evaluator_ = std::make_unique<FieldEvaluator>(*geometry_context_, domain_size);

    this->create_map(std::move(store_keys), IndexRange(domain_size));

    mf::SignatureBuilder signature{"Index Map", signature_};
    for (const int i : signature_input_names_.index_range()) {
      std::stringstream name;
      name << "Key " << i;
      signature_input_names_[i] = name.str();
    }

    for (const std::string &name : signature_input_names_) {
      signature.single_input<int>(name.c_str());
    }

    signature.single_output<int>("Index");
    signature.single_output<bool>("Is Valid");

    this->set_signature(&signature_);
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
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

    for (const int index : mask) {
      indices[index] = set_map_.index_of_try_as(&to_search_keys[index * total_key_],
                                                to_search_hashs[index]);
      if (indices[index] == -1) {
        indices[index] = 0;
        is_valid[index] = false;
      }
      else {
        is_valid[index] = true;
      }
    }
  }

 protected:
  void create_map(Vector<Field<int>> store_keys, const IndexRange range)
  {
    for (const Field<int> &field : store_keys) {
      evaluator_->add(field);
    }

    evaluator_->evaluate();

    Array<VArraySpan<int>> store_keys_value(total_key_);
    for (const int i : IndexRange(total_key_)) {
      store_keys_value[i] = evaluator_->get_evaluated<int>(i);
    }

    keys_.reinitialize(range.size() * total_key_);
    Array<int64_t> hashs(range.size());
    build_generic_keys(store_keys_value.as_span(),
                       range,
                       total_key_,
                       keys_.as_mutable_span(),
                       hashs.as_mutable_span());

    set_map_.clear();
    set_map_.reserve(range.size());
    for (const int index : range) {
      set_map_.add(&keys_[index * total_key_], hashs[index]);
    }
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Source");

  const eAttrDomain domain = eAttrDomain(params.node().custom1);

  Vector<ValueOrField<int>> store_keys = params.extract_input<Vector<fn::ValueOrField<int>>>(
      "Keys");
  Vector<ValueOrField<int>> search_keys = params.extract_input<Vector<fn::ValueOrField<int>>>(
      "Search Keys");

  if (store_keys.size() != search_keys.size()) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Number of inputs search and store keys non equal"));
    params.set_default_remaining_outputs();
    return;
  }

  if (store_keys.is_empty()) {
    params.set_default_remaining_outputs();
    return;
  }

  Vector<Field<int>> store_key_fields;
  store_key_fields.reserve(store_keys.size());
  for (ValueOrField<int> key_value_field : store_keys) {
    store_key_fields.append(key_value_field.as_field());
  }

  Vector<GField> search_key_fields;
  search_key_fields.reserve(search_keys.size());
  for (ValueOrField<int> key_value_field : search_keys) {
    search_key_fields.append(key_value_field.as_field());
  }

  std::shared_ptr<IndexMapFunction> fn;
  try {
    fn = std::make_shared<IndexMapFunction>(
        std::move(geometry_set), domain, std::move(store_key_fields));
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
