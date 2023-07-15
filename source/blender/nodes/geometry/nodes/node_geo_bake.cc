/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bake_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Geometry");
}

class LazyFunctionForBakeNode : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForBakeNode(const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
      : node_(node)
  {
    debug_name_ = "Bake";

    MutableSpan<int> lf_index_by_bsocket = own_lf_graph_info.mapping.lf_index_by_bsocket;

    const bNodeSocket &input_bsocket = node.input_socket(0);
    const bNodeSocket &output_bsocket = node.output_socket(0);
    const CPPType &type = CPPType::get<GeometrySet>();

    lf_index_by_bsocket[input_bsocket.index_in_tree()] = inputs_.append_and_get_index_as(
        "Geometry", type);
    lf_index_by_bsocket[output_bsocket.index_in_tree()] = outputs_.append_and_get_index_as(
        "Geometry", type);
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const final
  {
    GeometrySet geometry = params.extract_input<GeometrySet>(0);
    params.set_output(0, std::move(geometry));
  }
};

class LazyFunctionForBakeNodeInputUsage : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForBakeNodeInputUsage(const bNode &node) : node_(node)
  {
    debug_name_ = "Bake Input Usage";
    outputs_.append_as("Usage", CPPType::get<ValueOrField<bool>>());
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const final
  {
    params.set_output(0, true);
  }
};

}  // namespace blender::nodes::node_geo_bake_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_bake_node_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;
  BLI_assert(node.type == GEO_NODE_BAKE);
  return std::make_unique<file_ns::LazyFunctionForBakeNode>(node, own_lf_graph_info);
}

std::unique_ptr<LazyFunction> get_bake_node_input_usage_lazy_function(const bNode &node)
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;
  BLI_assert(node.type == GEO_NODE_BAKE);
  return std::make_unique<file_ns::LazyFunctionForBakeNodeInputUsage>(node);
}

}  // namespace blender::nodes

void register_node_type_geo_bake()
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BAKE, "Bake", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
