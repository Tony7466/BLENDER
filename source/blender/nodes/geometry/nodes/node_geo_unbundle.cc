/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_unbundle_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bundle>("Bundle");
  b.add_output<decl::Geometry>("A");
  b.add_output<decl::Geometry>("B");
}

class LazyFunctionForUnbundle : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForUnbundle(const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
      : node_(node)
  {
    debug_name_ = "Unbundle";
    for (const int i : node.input_sockets().index_range()) {
      const bNodeSocket &bsocket = node.input_socket(i);
      lf_graph_info.mapping.lf_index_by_bsocket[bsocket.index_in_tree()] =
          inputs_.append_and_get_index_as(
              bsocket.name, *bsocket.typeinfo->geometry_nodes_cpp_type, lf::ValueUsage::Used);
    }
    for (const int i : node.output_sockets().index_range()) {
      const bNodeSocket &bsocket = node.output_socket(i);
      lf_graph_info.mapping.lf_index_by_bsocket[bsocket.index_in_tree()] =
          outputs_.append_and_get_index_as(bsocket.name,
                                           *bsocket.typeinfo->geometry_nodes_cpp_type);
    }
  }

  void execute_impl(lf::Params & /*params*/, const lf::Context & /*context*/) const final
  {
    // const int outputs_num = node_.output_sockets().size();
    // const int bundle_param = 0;

    // BLI_assert(params.get_dynamic_inputs_num(bundle_param) == outputs_num);

    // for (const int i : IndexRange(outputs_num)) {
    //   const lf::Slot input_slot{bundle_param, i};
    //   const lf::Slot output_slot{i};
    //   if (params.get_output_usage(output_slot) != lf::ValueUsage::Used) {
    //     continue;
    //   }
    //   if (GeometrySet *geometry = params.try_get_input_data_ptr_or_request<GeometrySet>(
    //           input_slot))
    //   {
    //     params.set_output(output_slot, std::move(*geometry));
    //   }
    // }
  }
};

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_UNBUNDLE, "Unbundle", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_unbundle_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_unbundle_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
{
  using namespace node_geo_unbundle_cc;
  return std::make_unique<LazyFunctionForUnbundle>(node, lf_graph_info);
}

}  // namespace blender::nodes
