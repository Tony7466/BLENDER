/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bundle_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("A");
  b.add_input<decl::Geometry>("B");
  b.add_output<decl::Geometry>("Bundle").propagate_all();
}

class LazyFunctionForBundle : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForBundle(const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
      : node_(node)
  {
    debug_name_ = "Bundle";
    for (const int i : node.input_sockets().index_range()) {
      const bNodeSocket &bsocket = node.input_socket(i);
      lf_graph_info.mapping.lf_index_by_bsocket[bsocket.index_in_tree()] =
          inputs_.append_and_get_index_as(
              bsocket.name, *bsocket.typeinfo->geometry_nodes_cpp_type, lf::ValueUsage::Maybe);
    }
    for (const int i : node.output_sockets().index_range()) {
      const bNodeSocket &bsocket = node.output_socket(i);
      lf_graph_info.mapping.lf_index_by_bsocket[bsocket.index_in_tree()] =
          outputs_.append_and_get_index_as(bsocket.name,
                                           *bsocket.typeinfo->geometry_nodes_cpp_type);
    }
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const final
  {
    params.set_output(0, GeometrySet());
  }
};

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BUNDLE, "Bundle", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_bundle_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_bundle_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
{
  using namespace node_geo_bundle_cc;
  return std::make_unique<LazyFunctionForBundle>(node, lf_graph_info);
}

}  // namespace blender::nodes
