/* SPDX-FileCopyrightText: 2011 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_FileOutputNode.h"

#include "BLI_string.h"

namespace blender::compositor {

FileOutputNode::FileOutputNode(bNode *editor_node) : Node(editor_node)
{
  /* pass */
}

void FileOutputNode::convert_to_operations(NodeConverter &converter,
                                           const CompositorContext &context) const
{
  for (NodeInput *input : inputs_) {
    if (input->is_linked()) {
      converter.add_node_input_preview(input);
      break;
    }
  }

  /* Only File Outputs in the root node tree are considered. */
  const bool is_in_root_node_tree = this->get_bnodetree() == context.get_bnodetree();
  if (!context.is_rendering() || !is_in_root_node_tree) {
    return;
  }

  Vector<FileOutputInput> inputs;
  for (NodeInput *input : inputs_) {
    auto *storage = static_cast<NodeImageMultiFileSocket *>(input->get_bnode_socket()->storage);
    inputs.append(FileOutputInput(storage, input->get_data_type()));
  }

  auto *storage = static_cast<const NodeImageMultiFile *>(this->get_bnode()->storage);
  auto *output_operation = new FileOutputOperation(&context, storage, inputs);
  converter.add_operation(output_operation);

  for (int i = 0; i < inputs_.size(); i++) {
    converter.map_input_socket(inputs_[i], output_operation->get_input_socket(i));
  }
}

}  // namespace blender::compositor
