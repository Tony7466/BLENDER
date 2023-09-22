/* SPDX-FileCopyrightText: 2011 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_TextureNoiseNode.h"

#include "COM_TextureNoiseOperation.h"

namespace blender::compositor {

void TextureNoiseNode::convert_to_operations(NodeConverter &converter,
                                             const CompositorContext & /*context*/) const
{
  TextureNoiseOperation *color_operation = new TextureNoiseOperation();
  converter.add_operation(color_operation);

  NodeInput *vector_input = this->get_input_socket(0);
  // NodeInput *w_input = this->get_input_socket(1);
  NodeInput *scale_input = this->get_input_socket(2);
  NodeInput *detail_input = this->get_input_socket(3);
  NodeInput *roughness_input = this->get_input_socket(4);
  NodeInput *lacunarity_input = this->get_input_socket(5);
  NodeInput *distortion_input = this->get_input_socket(6);

  // NodeOutput *value_output = this->get_output_socket(0);
  NodeOutput *color_output = this->get_output_socket(1);

  converter.map_input_socket(vector_input, color_operation->get_input_socket(0));
  converter.map_input_socket(scale_input, color_operation->get_input_socket(1));
  converter.map_input_socket(detail_input, color_operation->get_input_socket(2));
  converter.map_input_socket(roughness_input, color_operation->get_input_socket(3));
  converter.map_input_socket(lacunarity_input, color_operation->get_input_socket(4));
  converter.map_input_socket(distortion_input, color_operation->get_input_socket(5));

  converter.map_output_socket(color_output, color_operation->get_output_socket());
}

}  // namespace blender::compositor
