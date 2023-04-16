/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2011 Blender Foundation. */

#include "COM_KuwaharaNode.h"
#include "COM_KuwaharaOperation.h"

namespace blender::compositor {

void KuwaharaNode::convert_to_operations(NodeConverter &converter,
                                         const CompositorContext & /*context*/) const
{
  const bNode *node = this->get_bnode();
  const NodeKuwaharaData *data = (const NodeKuwaharaData *)node->storage;

  KuwaharaOperation *operation = new KuwaharaOperation();
  operation->set_kernel_size(data->kernel_size);
  operation->set_variation(data->variation);

  converter.add_operation(operation);
  converter.map_input_socket(get_input_socket(0), operation->get_input_socket(0));
  converter.map_output_socket(get_output_socket(0), operation->get_output_socket());
}

}  // namespace blender::compositor
