/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_add_node_search.hh"
#include "NOD_geometry.h"
#include "NOD_socket.h"

#include "node_geometry_util.hh"

namespace blender::nodes {
std::string socket_identifier_for_serial_loop_item(const NodeSerialLoopItem &item)
{
  return "Item_" + std::to_string(item.identifier);
}

static std::unique_ptr<SocketDeclaration> socket_declaration_for_serial_loop_item(
    const NodeSerialLoopItem &item,
    const eNodeSocketInOut in_out,
    const int corresponding_input = -1)
{
  const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);

  std::unique_ptr<SocketDeclaration> decl;

  auto handle_field_decl = [&](SocketDeclaration &decl) {
    if (in_out == SOCK_IN) {
      decl.input_field_type = InputSocketFieldType::IsSupported;
    }
    else {
      decl.output_field_dependency = OutputFieldDependency::ForPartiallyDependentField(
          {corresponding_input});
    }
  };

  switch (socket_type) {
    case SOCK_FLOAT:
      decl = std::make_unique<decl::Float>();
      handle_field_decl(*decl);
      break;
    case SOCK_VECTOR:
      decl = std::make_unique<decl::Vector>();
      handle_field_decl(*decl);
      break;
    case SOCK_RGBA:
      decl = std::make_unique<decl::Color>();
      handle_field_decl(*decl);
      break;
    case SOCK_BOOLEAN:
      decl = std::make_unique<decl::Bool>();
      handle_field_decl(*decl);
      break;
    case SOCK_ROTATION:
      decl = std::make_unique<decl::Rotation>();
      handle_field_decl(*decl);
      break;
    case SOCK_INT:
      decl = std::make_unique<decl::Int>();
      handle_field_decl(*decl);
      break;
    case SOCK_STRING:
      decl = std::make_unique<decl::String>();
      break;
    case SOCK_GEOMETRY:
      decl = std::make_unique<decl::Geometry>();
      break;
    case SOCK_OBJECT:
      decl = std::make_unique<decl::Object>();
      break;
    case SOCK_IMAGE:
      decl = std::make_unique<decl::Image>();
      break;
    case SOCK_COLLECTION:
      decl = std::make_unique<decl::Collection>();
      break;
    case SOCK_MATERIAL:
      decl = std::make_unique<decl::Material>();
      break;
    default:
      BLI_assert_unreachable();
      break;
  }

  decl->name = item.name ? item.name : "";
  decl->identifier = socket_identifier_for_serial_loop_item(item);
  decl->in_out = in_out;
  return decl;
}

void socket_declarations_for_serial_loop_items(const Span<NodeSerialLoopItem> items,
                                               NodeDeclaration &r_declaration)
{
  for (const int i : items.index_range()) {
    const NodeSerialLoopItem &item = items[i];
    r_declaration.inputs.append(socket_declaration_for_serial_loop_item(item, SOCK_IN));
    r_declaration.outputs.append(
        socket_declaration_for_serial_loop_item(item, SOCK_OUT, r_declaration.inputs.size() - 1));
  }
  r_declaration.inputs.append(decl::create_extend_declaration(SOCK_IN));
  r_declaration.outputs.append(decl::create_extend_declaration(SOCK_OUT));
}
}  // namespace blender::nodes
namespace blender::nodes::node_geo_serial_loop_output_cc {

NODE_STORAGE_FUNCS(NodeGeometrySerialLoopOutput);

static void node_declare_dynamic(const bNodeTree & /*node_tree*/,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  NodeDeclarationBuilder b{r_declaration};
  b.add_input<decl::Bool>(N_("Break Before")).hide_value();
  b.add_input<decl::Bool>(N_("Break After")).hide_value();

  const NodeGeometrySerialLoopOutput &storage = node_storage(node);
  socket_declarations_for_serial_loop_items(storage.items_span(), r_declaration);
}

static void search_node_add_ops(GatherAddNodeSearchParams &params)
{
  AddNodeItem item;
  item.ui_name = IFACE_("Serial Loop Zone");
  item.description = TIP_("Add a new serial loop input and output nodes to the node tree");
  item.add_fn = [](const bContext &C, bNodeTree &node_tree, float2 cursor) {
    bNode *input = nodeAddNode(&C, &node_tree, "GeometryNodeSerialLoopInput");
    bNode *output = nodeAddNode(&C, &node_tree, "GeometryNodeSerialLoopOutput");
    static_cast<NodeGeometrySerialLoopInput *>(input->storage)->output_node_id =
        output->identifier;

    input->locx = cursor.x / UI_SCALE_FAC - 150;
    input->locy = cursor.y / UI_SCALE_FAC + 20;
    output->locx = cursor.x / UI_SCALE_FAC + 150;
    output->locy = cursor.y / UI_SCALE_FAC + 20;

    return Vector<bNode *>({input, output});
  };
  params.add_item(std::move(item));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySerialLoopOutput *data = MEM_cnew<NodeGeometrySerialLoopOutput>(__func__);

  data->next_identifier = 0;

  data->items = MEM_cnew_array<NodeSerialLoopItem>(1, __func__);
  data->items[0].name = BLI_strdup(DATA_("Geometry"));
  data->items[0].socket_type = SOCK_GEOMETRY;
  data->items[0].identifier = data->next_identifier++;
  data->items_num = 1;

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  NodeGeometrySerialLoopOutput &storage = node_storage(*node);
  for (NodeSerialLoopItem &item : storage.items_span()) {
    MEM_SAFE_FREE(item.name);
  }
  MEM_SAFE_FREE(storage.items);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometrySerialLoopOutput &src_storage = node_storage(*src_node);
  NodeGeometrySerialLoopOutput *dst_storage = MEM_cnew<NodeGeometrySerialLoopOutput>(__func__);

  dst_storage->items = MEM_cnew_array<NodeSerialLoopItem>(src_storage.items_num, __func__);
  dst_storage->items_num = src_storage.items_num;
  dst_storage->active_index = src_storage.active_index;
  dst_storage->next_identifier = src_storage.next_identifier;
  for (const int i : IndexRange(src_storage.items_num)) {
    if (char *name = src_storage.items[i].name) {
      dst_storage->items[i].identifier = src_storage.items[i].identifier;
      dst_storage->items[i].name = BLI_strdup(name);
      dst_storage->items[i].socket_type = src_storage.items[i].socket_type;
    }
  }

  dst_node->storage = dst_storage;
}

}  // namespace blender::nodes::node_geo_serial_loop_output_cc

blender::Span<NodeSerialLoopItem> NodeGeometrySerialLoopOutput::items_span() const
{
  return blender::Span<NodeSerialLoopItem>(items, items_num);
}

blender::MutableSpan<NodeSerialLoopItem> NodeGeometrySerialLoopOutput::items_span()
{
  return blender::MutableSpan<NodeSerialLoopItem>(items, items_num);
}

void register_node_type_geo_serial_loop_output()
{
  namespace file_ns = blender::nodes::node_geo_serial_loop_output_cc;

  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_SERIAL_LOOP_OUTPUT, "Serial Loop Output", NODE_CLASS_INTERFACE);
  ntype.initfunc = file_ns::node_init;
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.gather_add_node_search_ops = file_ns::search_node_add_ops;
  node_type_storage(&ntype,
                    "NodeGeometrySerialLoopOutput",
                    file_ns::node_free_storage,
                    file_ns::node_copy_storage);
  nodeRegisterType(&ntype);
}
