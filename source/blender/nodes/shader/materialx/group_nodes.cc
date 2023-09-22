/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "group_nodes.h"
#include "node_parser.h"

#include "BLI_vector.hh"

namespace blender::nodes::materialx {

NodeItem GroupNodeParser::compute()
{
  NodeItem res = empty();

  const bNodeTree *ngroup = reinterpret_cast<const bNodeTree *>(node_->id);
  ngroup->ensure_topology_cache();
  const bNode *node_out = ngroup->group_output_node();
  if (!node_out) {
    return res;
  }

  MaterialX::GraphElement *graph = graph_;
#ifdef USE_MATERIALX_NODEGRAPH
  std::string name = MaterialX::createValidName(ngroup->id.name);
  MaterialX::NodeGraphPtr group_graph = graph_->getChildOfType<MaterialX::NodeGraph>(name);
  if (!group_graph) {
    CLOG_INFO(LOG_MATERIALX_SHADER, 1, "<nodegraph name=%s>", name.c_str());
    group_graph = graph_->addChild<MaterialX::NodeGraph>(name);
  }
  graph = group_graph.get();
#endif

  NodeItem out =
      GroupOutputNodeParser(graph,
                                       depsgraph_,
                                       material_,
                                       node_out,
                                       socket_out_,
                                       NodeItem::Type::Any,
                                       this,
                                       export_image_fn_)
          .compute_full();

#ifdef USE_MATERIALX_NODEGRAPH
  /* We have to be in NodeParser's graph_, therefore copying output */
  res.output = out.output;
#else
  res = out;
#endif
  return res;
}

NodeItem GroupNodeParser::compute_full()
{
  NodeItem res = compute();
  if (NodeItem::is_arithmetic(to_type_)) {
    res = res.convert(to_type_);
  }
  return res;
}

NodeItem GroupOutputNodeParser::compute()
{
#ifdef USE_MATERIALX_NODEGRAPH
  Vector<NodeItem> values;
  for (auto socket_in : node_->input_sockets()) {
    NodeItem value = get_input_value(socket_in->index(), NodeItem::Type::Any);
    if (value.value) {
      value = create_node("constant", value.type(), {{"value", value}});
    }
    values.append(value);
  }
  Vector<NodeItem> outputs;
  for (int i = 0; i < values.size(); ++i) {
    if (values[i]) {
      outputs.append(create_output("output" + std::to_string(i + 1), values[i]));
    }
  }
  return outputs[socket_out_->index()];
#else
  return get_input_value(socket_out_->index(), NodeItem::Type::Any);
#endif
}

NodeItem GroupOutputNodeParser::compute_full()
{
#ifdef USE_MATERIALX_NODEGRAPH
  NodeItem res = empty();

  /* Checking if output was already computed */
  res.output = graph_->getOutput("output" + std::to_string(socket_out_->index() + 1));
  if (res.output) {
    return res;
  }

  CLOG_INFO(LOG_MATERIALX_SHADER,
            1,
            "%s [%d] => %s",
            node_->name,
            node_->typeinfo->type,
            NodeItem::type(to_type_).c_str());

  res = compute();
  return res;
#else
  return compute();
#endif
}

NodeItem GroupInputNodeParser::compute()
{
#ifdef USE_MATERIALX_NODEGRAPH
  NodeItem value = group_parser_->get_input_link(socket_out_->index(), to_type_);
  if (!value) {
    return empty();
  }

  if (value.value) {
    value = create_node("constant", value.type(), {{"value", value}});
  }
  return create_input("input" + std::to_string(socket_out_->index() + 1), value);
#else
  return group_parser_->get_input_link(socket_out_->index(), to_type_);
#endif
}

NodeItem GroupInputNodeParser::compute_full()
{
#ifdef USE_MATERIALX_NODEGRAPH
  NodeItem res = empty();

  /* Checking if output was already computed */
  res.input = graph_->getInput("input" + std::to_string(socket_out_->index() + 1));
  if (res.input) {
    return res;
  }

  CLOG_INFO(LOG_MATERIALX_SHADER,
            1,
            "%s [%d] => %s",
            node_->name,
            node_->typeinfo->type,
            NodeItem::type(to_type_).c_str());

  res = compute();
  return res;
#else
  return compute();
#endif
}

}  // namespace blender::nodes::materialx
