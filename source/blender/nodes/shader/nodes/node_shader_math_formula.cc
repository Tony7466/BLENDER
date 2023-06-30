
#include "node_shader_util.hh"

#include "NOD_math_functions.hh"
#include "NOD_socket_search_link.hh"
#include "RNA_enum_types.h"
#include "UI_interface.h"
#include "UI_resources.h"

#include "DEG_depsgraph_query.h"
#include "NOD_common.h"
#include "NOD_socket.h"

#include <regex>

/* **************** SCALAR MATH ******************** */
namespace blender::nodes::node_shader_math_formula_cc {
NODE_STORAGE_FUNCS(NodeMathFormula);

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  NodeMathFormula *data = MEM_cnew<NodeMathFormula>(__func__);
  node->storage = data;

  data->active_index = 0;
  data->data_type = SOCK_FLOAT;

  data->formula[0] = '\0';
  data->items = nullptr;
  data->items_num = 0;
  data->next_identifier = 0;
}

static void node_free_storage(bNode *node)
{
  if (!node->storage) {
    return;
  }
  NodeMathFormula &storage = node_storage(*node);
  for (NodeMathFormulaItem &item : storage.items_span()) {
    MEM_SAFE_FREE(item.name);
  }
  MEM_SAFE_FREE(storage.items);
  MEM_freeN(node->storage);
}

static void sh_node_math_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "formula", 0, "", ICON_NONE);
  uiItemR(layout, ptr, "data_type", 0, "", ICON_NONE);
}

class SocketSearchOp {
 public:
  std::string socket_name;
  NodeMathOperation mode = NODE_MATH_ADD;
  void operator()(LinkSearchOpParams &params)
  {
    bNode &node = params.add_node("ShaderNodeMathFormula");
    node.custom1 = mode;
    params.update_and_connect_available_socket(node, socket_name);
  }
};

static void sh_node_math_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (!params.node_tree().typeinfo->validate_link(
          static_cast<eNodeSocketDatatype>(params.other_socket().type), SOCK_FLOAT))
  {
    return;
  }
}

static const mf::MultiFunction *get_base_multi_function(const bNode & /*node*/)
{
  return nullptr;
}

class FormulaWrapperFunction : public mf::MultiFunction {
 private:
  // const mf::MultiFunction &fn_;
  const char *formula_;

 public:
  FormulaWrapperFunction(const char *formula) : formula_(formula)
  {
    // this->set_signature(&fn.signature());
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    // fn_.call(mask, params, context);

    /* Assumes the output parameter is the last one. */
    const int output_param_index = this->param_amount() - 1;
    /* This has actually been initialized in the call above. */
    MutableSpan<float> results = params.uninitialized_single_output<float>(output_param_index);

    mask.foreach_index_optimized<int>([&](const int i) {
      float &value = results[i];
      CLAMP(value, 0.0f, 1.0f);
    });
  }
};

static void sh_node_math_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *base_function = get_base_multi_function(builder.node());

  //   const bool clamp_output = builder.node().custom2 != 0;
  //   if (clamp_output) {
  //     builder.construct_and_set_matching_fn<ClampWrapperFunction>(*base_function);
  //   }
  //   else {
  //     builder.set_matching_fn(base_function);
  //   }
}
template<class dcl_type>
static void node_add_socket(const char *identifier,
                            const char *name,
                            eNodeSocketInOut in_out,
                            NodeDeclaration &r_declaration)
{
  std::unique_ptr<dcl_type> socket = std::make_unique<dcl_type>();
  socket->identifier = identifier;
  socket->name = name;
  socket->in_out = in_out;
  in_out == SOCK_IN ? r_declaration.inputs.append(std::move(socket)) :
                      r_declaration.outputs.append(std::move(socket));
}

static void node_add_socket_type(const char *identifier,
                                 const char *name,
                                 int8_t data_type,
                                 eNodeSocketInOut in_out,
                                 NodeDeclaration &r_declaration)
{
  switch (data_type) {
    case SOCK_FLOAT: {
      node_add_socket<decl::Float>(name, name, in_out, r_declaration);
      break;
    }
    case SOCK_VECTOR: {
      node_add_socket<decl::Vector>(name, name, in_out, r_declaration);
      break;
    }
    case SOCK_BOOLEAN: {
      node_add_socket<decl::Bool>(name, name, in_out, r_declaration);
      break;
    }
    case SOCK_INT: {
      node_add_socket<decl::Int>(name, name, in_out, r_declaration);
      break;
    }
    case SOCK_ROTATION: {
      node_add_socket<decl::Rotation>(name, name, in_out, r_declaration);
      break;
    }
    case SOCK_RGBA: {
      node_add_socket<decl::Color>(name, name, in_out, r_declaration);
      break;
    }
    default:
      break;
  }
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeMathFormula &src_storage = node_storage(*src_node);
  NodeMathFormula *dst_storage = MEM_cnew<NodeMathFormula>(__func__);

  dst_storage->items = MEM_cnew_array<NodeMathFormulaItem>(src_storage.items_num, __func__);
  dst_storage->items_num = src_storage.items_num;
  dst_storage->active_index = src_storage.active_index;
  dst_storage->next_identifier = src_storage.next_identifier;
  BLI_strncpy(dst_storage->formula, src_storage.formula, ARRAY_SIZE(src_storage.formula));

  for (const int i : IndexRange(src_storage.items_num)) {
    if (char *name = src_storage.items[i].name) {
      dst_storage->items[i].identifier = src_storage.items[i].identifier;
      dst_storage->items[i].name = BLI_strdup(name);
      dst_storage->items[i].socket_type = src_storage.items[i].socket_type;
      dst_storage->items[i].attribute_domain = src_storage.items[i].attribute_domain;
    }
  }

  dst_node->storage = dst_storage;
}

static void node_declare_dynamic(const bNodeTree & /*node_tree*/,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  auto &data = node_storage(node);

  for (auto &item : data.items_span()) {
    node_add_socket_type(item.name, item.name, item.socket_type, SOCK_IN, r_declaration);
  }

  r_declaration.inputs.append(decl::create_extend_declaration(SOCK_IN));

  node_add_socket_type("Value", "Value", data.data_type, SOCK_OUT, r_declaration);
}

void add_item(NodeMathFormula *math_formula, const short socket_type, const char *name, int index)
{

  NodeMathFormulaItem *old_items = math_formula->items;
  math_formula->items = MEM_cnew_array<NodeMathFormulaItem>(math_formula->items_num + 1, __func__);
  for (const int i : blender::IndexRange(index)) {
    math_formula->items[i] = old_items[i];
  }
  for (const int i : blender::IndexRange(index, math_formula->items_num - index)) {
    math_formula->items[i + 1] = old_items[i];
  }

  const char *defname = nodeStaticSocketLabel(socket_type, 0);
  NodeMathFormulaItem &added_item = math_formula->items[index];
  added_item.identifier = math_formula->next_identifier++;
  added_item.socket_type = socket_type;
  added_item.name = (char *)name;
  math_formula->items_num++;

  MEM_SAFE_FREE(old_items);
}

void node_math_formula_update(bNodeTree *ntree, bNode *node)
{
  auto &data = node_storage(*node);
  char *formula = data.formula;

  auto get_identifier_len = [](char *string) {
    int len = 0;
    while (string) {
      char cur_char = string[0];
      if (cur_char >= 'a' && cur_char <= 'z') {
      }
      else if (cur_char >= 'A' && cur_char <= 'Z') {
      }
      else if (cur_char == '_') {
      }
      else {
        return len;
      }
      string++;
      len++;
    }
    return 0;
  };

  auto identifier_inserted = [&](const char *string) {
    for (auto &item : data.items_span())
      {
        if (STREQ(string, item.name)) {
          return true;
        }
      }
    return false;
  };
  char identifier[256];
  while (*formula) {
    const int len = get_identifier_len(formula);

    if (len < 1) {
      formula++;
      continue;
    }

    BLI_strncpy_rlen(identifier, formula, len + 1);
    if (!identifier_inserted(identifier)) {
      add_item(&data, SOCK_FLOAT, BLI_strdup(identifier), data.items_num);
    }
    formula = formula + len;
  }

  update_node_declaration_and_sockets(*ntree, *node);
}

}  // namespace blender::nodes::node_shader_math_formula_cc

void register_node_type_sh_math_formula()
{

  namespace file_ns = blender::nodes::node_shader_math_formula_cc;

  static bNodeType ntype;

  sh_fn_node_type_base(&ntype, SH_NODE_MATH_FORMULA, "Formula", NODE_CLASS_CONVERTER);
  ntype.initfunc = file_ns::node_init;
  // ntype.initfunc = file_ns::node_init;
  ntype.declare = file_ns::sh_node_math_declare;
  // ntype.labelfunc = node_math_label;
  // ntype.gpu_fn = file_ns::gpu_shader_math;
  ntype.updatefunc = file_ns::node_math_formula_update;

  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.build_multi_function = file_ns::sh_node_math_build_multi_function;

  node_type_storage(
      &ntype, "NodeMathFormula", file_ns::node_free_storage, file_ns::node_copy_storage);

  nodeRegisterType(&ntype);
}

blender::Span<NodeMathFormulaItem> NodeMathFormula::items_span() const
{
  return blender::Span<NodeMathFormulaItem>(items, items_num);
}

blender::MutableSpan<NodeMathFormulaItem> NodeMathFormula::items_span()
{
  return blender::MutableSpan<NodeMathFormulaItem>(items, items_num);
}

blender::IndexRange NodeMathFormula::items_range() const
{
  return blender::IndexRange(items_num);
}

NodeMathFormulaItem *NOD_math_formula_add_item(NodeMathFormula *math_formula,
                                               short socket_type,
                                               const char *name)
{
  auto identifier_inserted = [](NodeMathFormula &math_formula, const char *string) {
    for (auto &item : math_formula.items_span()) {
      if (STREQ(string, item.name)) {
        return true;
      }
    }
    return false;
  };
  if (identifier_inserted(*math_formula, name)) {
    return nullptr;
  }
  else {
    namespace file_ns = blender::nodes::node_shader_math_formula_cc;

    file_ns::add_item(math_formula, SOCK_FLOAT, name, math_formula->items_num);
    return &math_formula->items[math_formula->items_num - 1];
  }
}

bool NOD_math_formula_contains_item(NodeMathFormula * /*math_formula*/,
                                    const NodeMathFormulaItem * /*item*/)
{
  return true;
}
NodeMathFormulaItem *NOD_math_formula_get_active_item(NodeMathFormula *math_formula)
{
  return &math_formula->items[math_formula->active_index];
}
void NOD_math_formula_set_active_item(NodeMathFormula *math_formula, NodeMathFormulaItem *item)
{
  auto find_idx_fn = [](NodeMathFormula &math_formula, NodeMathFormulaItem *item_) {
    int idx = 0;
    for (auto &item : math_formula.items_span()) {
      if (&item == item_) {
        return idx;
      }
    }
    return 0;
  };

  math_formula->active_index = find_idx_fn(*math_formula, item);
}
NodeMathFormulaItem *NOD_math_formula_find_item(NodeMathFormula *math_formula, const char *name)
{
  auto find_idx_fn = [](NodeMathFormula &math_formula, const char *name) {
    int idx = 0;
    for (auto &item : math_formula.items_span()) {
      if (STREQ(item.name, name)) {
        return idx;
      }
    }
    return -1;
  };
  if (find_idx_fn(*math_formula, name) >= 0) {
    math_formula->items[find_idx_fn(*math_formula, name)];
  }
  return nullptr;
}

NodeMathFormulaItem *NOD_math_formula_insert_item(NodeMathFormula *math_formula,
                                                  short socket_type,
                                                  const char *name,
                                                  int index)
{
  auto identifier_inserted = [](NodeMathFormula &math_formula, const char *string) {
    for (auto &item : math_formula.items_span()) {
      if (STREQ(string, item.name)) {
        return true;
      }
    }
    return false;
  };
  if (identifier_inserted(*math_formula, name)) {
    return nullptr;
  }
  else {
    namespace file_ns = blender::nodes::node_shader_math_formula_cc;

    file_ns::add_item(math_formula, SOCK_FLOAT, name, index);
    return &math_formula->items[index];
  }
}
NodeMathFormulaItem *NOD_math_formula_add_item_from_socket(NodeMathFormula * /*math_formula*/,
                                                           const bNode * /*from_node*/,
                                                           const bNodeSocket * /*from_sock*/)
{
  return nullptr;
}
NodeMathFormulaItem *NOD_math_formula_insert_item_from_socket(NodeMathFormula * /*math_formula*/,
                                                              const bNode * /*from_node*/,
                                                              const bNodeSocket * /*from_sock*/,
                                                              int /*index*/)
{
  return nullptr;
}
void NOD_math_formula_remove_item(NodeMathFormula *math_formula, NodeMathFormulaItem *item)
{

  const int index = item - math_formula->items;
  if (index < 0 || index >= math_formula->items_num) {
    return;
  }

  NodeMathFormulaItem *old_items = math_formula->items;
  math_formula->items = MEM_cnew_array<NodeMathFormulaItem>(math_formula->items_num - 1, __func__);
  for (const int i : blender::IndexRange(index)) {
    math_formula->items[i] = old_items[i];
  }
  for (const int i : blender::IndexRange(index, math_formula->items_num - index).drop_front(1)) {
    math_formula->items[i - 1] = old_items[i];
  }

  MEM_SAFE_FREE(old_items[index].name);

  math_formula->items_num--;
  MEM_SAFE_FREE(old_items);
}

void NOD_math_formula_clear_items(NodeMathFormula *math_formula)
{
  for (NodeMathFormulaItem &item : math_formula->items_span()) {
    MEM_SAFE_FREE(item.name);
  }
  MEM_SAFE_FREE(math_formula->items);
  math_formula->items = nullptr;
  math_formula->items_num = 0;
}

void NOD_math_formula_move_item(NodeMathFormula *math_formula, int from_index, int to_index)
{
  BLI_assert(from_index >= 0 && from_index < math_formula->items_num);
  BLI_assert(to_index >= 0 && to_index < math_formula->items_num);

  if (from_index == to_index) {
    return;
  }

  const int direction = from_index < to_index ? 1 : -1;
  const NodeMathFormulaItem tmp = math_formula->items[from_index];
  for (int i = from_index; i != to_index; i = i + direction) {
    math_formula->items[i] = math_formula->items[i + direction];
  }
  math_formula->items[to_index] = tmp;
}

bNode *NOD_math_formula_find_node_by_item(bNodeTree *ntree, const NodeMathFormulaItem *item)
{
  ntree->ensure_topology_cache();
  for (bNode *node : ntree->nodes_by_type("ShaderNodeMathFormula")) {
    NodeMathFormula *math_formula = static_cast<NodeMathFormula *>(node->storage);
    if (math_formula->items_span().contains_ptr(item)) {
      return node;
    }
  }
  return nullptr;
}

void NOD_math_formula_rename_item(const bNode *node,
                                  NodeMathFormulaItem *dest_item,
                                  const char *new_name)
{
printf("aaaa");
  NodeMathFormula *math_formula = static_cast<NodeMathFormula *>(node->storage);
  for (auto &item : math_formula->items_span()) {
    if (STREQ(item.name, new_name)) {
      // Name in use
      return;
    }
  }
  auto is_identifier = [](const char *string) {
    while (string[0]!='\0') {
      char cur_char = string[0];
      if (cur_char >= 'a' && cur_char <= 'z') {
      }
      else if (cur_char >= 'A' && cur_char <= 'Z') {
      }
      else if (cur_char == '_') {
      }
      else {
        return false;
      }
      string++;
    }
    return true;
  };
  if (!is_identifier(new_name)) {
    return;
  }
  char *new_formula = BLI_str_replaceN(math_formula->formula, dest_item->name, new_name);
  BLI_strncpy(math_formula->formula, new_formula, ARRAY_SIZE(NodeMathFormula().formula));
  MEM_delete(new_formula);

  MEM_delete(dest_item->name);
  dest_item->name = BLI_strdup(new_name);
}
