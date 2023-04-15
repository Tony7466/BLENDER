/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "node_geometry_util.hh"

#include "BLI_hash.h"

#include "UI_interface.h"
#include "UI_resources.h"

// Wrap the code in a unique namespace to avoid naming conflicts
namespace blender::nodes::node_geo_input_rna_cc {

// Declare the node's inputs and outputs
static void node_declare(NodeDeclarationBuilder &b){
  // Add an output socket of type decl::Float with the name "Value"
  b.add_output<decl::Float>(N_("Value"));
}

// Define the node's UI layout
static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr){

  // UI similar to editors\space_graph\graph_buttons.c

  // Target ID
  uiLayout *row;
  row = uiLayoutRow(layout, true);
  uiLayoutSetRedAlert(row, false); // You can set this based on your custom validation
  uiLayoutSetAlignment(row, UI_LAYOUT_ALIGN_LEFT);
  uiTemplateAnyID(row, ptr, "id_ptr", "id_type", "");

  // Target Property
  PointerRNA id_ptr = RNA_pointer_get(ptr, "id_ptr");
  if (id_ptr.data) {
    PointerRNA root_ptr;

    // Get pointer for resolving the property selected
    RNA_id_pointer_create(reinterpret_cast<ID*>(id_ptr.data), &root_ptr);

    // RNA path
    uiLayout *row;
    row = uiLayoutRow(layout, true);
    uiLayoutSetRedAlert(row, false); // You can set this based on your custom validation
    uiTemplatePathBuilder(row, ptr, "rna_path", &root_ptr, "");
  }
}

// Define node output
static void node_build_multi_function(NodeMultiFunctionBuilder &builder){
  
  // Access node storage
  const bNode &bnode = builder.node();
  NodeInputRna *node_storage = static_cast<NodeInputRna *>(bnode.storage);

  // define node output 
  float x = 1.0;
  builder.construct_and_set_matching_fn<mf::CustomMF_Constant<float>>(x);
}

// Initialize the node's storage
static void node_init(bNodeTree * /*tree*/, bNode *node){
  // Allocate a new NodeInputRna object and assign it to the node's storage
  NodeInputRna *data = MEM_cnew<NodeInputRna>(__func__);

  // Set the default ID type to ID_OB (Object)
  data->id_type = ID_OB;

  node->storage = data;
}

}  // namespace blender::nodes::node_geo_input_rna_cc

// Register the node type
void register_node_type_geo_input_rna(){
  // Define a namespace alias for convenience
  namespace file_ns = blender::nodes::node_geo_input_rna_cc;
  // Create a bNodeType object
  static bNodeType ntype;
  // Set the node's base properties
  geo_node_type_base(&ntype, GEO_NODE_INPUT_RNA, "RNA Path", 0);
  // Assign the node's functions
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  // Set the node's storage functions
  node_type_storage(
    &ntype, "NodeInputRna", node_free_standard_storage, node_copy_standard_storage);
  // Set the node's multi-function builder
  ntype.build_multi_function = file_ns::node_build_multi_function;
  // Set the node's UI layout function
  ntype.draw_buttons = file_ns::node_layout;
  // Register the node type
  nodeRegisterType(&ntype);
}