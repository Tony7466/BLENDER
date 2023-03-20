/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <random>

#include "BLI_task.hh"

#include "BKE_attribute_math.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sample_random_index_cc {

NODE_STORAGE_FUNCS(NodeGeometrySampleIndex);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"));

  b.add_input<decl::Float>(N_("Probability")).hide_value().field_on_all();
  b.add_input<decl::Int>(N_("ID")).implicit_field(implicit_field_inputs::id_or_index);;
  b.add_input<decl::Int>(N_("Seed")).field_on_all();

  b.add_output<decl::Int>(N_("Index")).dependent_field({2, 3});
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "domain", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = ATTR_DOMAIN_POINT;
}

static bool component_is_available(const GeometrySet &geometry,
                                   const GeometryComponentType type,
                                   const eAttrDomain domain)
{
  if (!geometry.has(type)) {
    return false;
  }
  const GeometryComponent &component = *geometry.get_component_for_read(type);
  return component.attribute_domain_size(domain) != 0;
}

static const GeometryComponent *find_source_component(const GeometrySet &geometry,
                                                      const eAttrDomain domain)
{
  /* Choose the other component based on a consistent order, rather than some more complicated
   * heuristic. This is the same order visible in the spreadsheet and used in the ray-cast node. */
  static const Array<GeometryComponentType> supported_types = {GEO_COMPONENT_TYPE_MESH,
                                                               GEO_COMPONENT_TYPE_POINT_CLOUD,
                                                               GEO_COMPONENT_TYPE_CURVE,
                                                               GEO_COMPONENT_TYPE_INSTANCES};
  for (const GeometryComponentType src_type : supported_types) {
    if (component_is_available(geometry, src_type, domain)) {
      return geometry.get_component_for_read(src_type);
    }
  }

  return nullptr;
}

class SampleRandomIndexFieldInput : public mf::MultiFunction {
  const GeometryComponent &component_;
  const eAttrDomain domain_;

  mf::Signature signature_;
  Array<float> probability_;
  std::discrete_distribution<int> discrete_dist_;

 public:
  SampleRandomIndexFieldInput(const GeometryComponent &component,
                              const eAttrDomain domain,
                              Field<float> probability)
      : component_(component),
        domain_(domain)
  {
    mf::SignatureBuilder builder{"Sample Random Index", signature_};
    builder.single_input<int>("ID");
    builder.single_input<int>("Seed");
    builder.single_output<int>("Index");
    this->set_signature(&signature_);

    const int domain_size = component_.attribute_domain_size(domain_);
    const bke::GeometryFieldContext field_context{component_, domain};
    FieldEvaluator evaluator(field_context, domain_size);
    probability_.reinitialize(domain_size);
    evaluator.add_with_destination(std::move(probability), probability_.as_mutable_span());
    evaluator.evaluate();
    
    float total = 0.0f;
    for (const float &weight : probability_){
      total += math::abs(weight);
    }

    if (total <= 0.0001){
      for (float &weight : probability_){
        weight += 1;
      }
    }

    discrete_dist_ = std::discrete_distribution<int>(probability_.begin(), probability_.end());
  }

  void call(IndexMask mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<int> &ids = params.readonly_single_input<int>(0, "ID");
    const VArray<int> &seeds = params.readonly_single_input<int>(1, "Seed");
    MutableSpan<int> dst = params.uninitialized_single_output<int>(2, "Index");

    std::discrete_distribution<int> discrete_dist = discrete_dist_;

    for (const int index : mask){
      const int &id = ids[index];
      const int &seed = seeds[index];
      std::seed_seq seed_sequence{id, seed};
      std::mt19937 gen(seed_sequence);
      discrete_dist.reset();
      const int result_index = discrete_dist(gen);
      dst[index] = result_index;
    }
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry");
  
  const eAttrDomain domain = static_cast<eAttrDomain>(params.node().custom1);
  
  Field<float> probability_field = params.extract_input<Field<float>>("Probability");
  Field<int> id_field = params.extract_input<Field<int>>("ID");
  Field<int> seed_field = params.extract_input<Field<int>>("Seed");

  const GeometryComponent *component_for_sample = find_source_component(geometry, domain);

  if (component_for_sample == nullptr){
    params.set_default_remaining_outputs();
    return;
  }

  auto fn = std::make_shared<SampleRandomIndexFieldInput>(*component_for_sample, domain, std::move(probability_field));
  auto op = FieldOperation::Create(std::move(fn), {std::move(id_field), std::move(seed_field)});
  params.set_output<Field<int>>("Index", Field<int>(std::move(op)));
}

}  // namespace blender::nodes::node_geo_sample_random_index_cc

void register_node_type_geo_sample_random_index()
{
  namespace file_ns = blender::nodes::node_geo_sample_random_index_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_RANDOM_INDEX, "Sample Random Index", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
