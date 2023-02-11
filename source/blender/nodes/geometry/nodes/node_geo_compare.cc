/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <cmath>

#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "RNA_enum_types.h"

#include "NOD_socket_search_link.hh"

#include "FN_multi_function_signature.hh"

namespace blender::nodes::node_geo_compare_cc {

NODE_STORAGE_FUNCS(NodeFunctionCompare)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>(N_("A")).min(-10000.0f).max(10000.0f);
  b.add_input<decl::Float>(N_("B")).min(-10000.0f).max(10000.0f);

  b.add_input<decl::Int>(N_("A"), "A_INT");
  b.add_input<decl::Int>(N_("B"), "B_INT");

  b.add_input<decl::Vector>(N_("A"), "A_VEC3");
  b.add_input<decl::Vector>(N_("B"), "B_VEC3");

  b.add_input<decl::Color>(N_("A"), "A_COL");
  b.add_input<decl::Color>(N_("B"), "B_COL");

  b.add_input<decl::String>(N_("A"), "A_STR");
  b.add_input<decl::String>(N_("B"), "B_STR");

  b.add_input<decl::Material>(N_("A"), "A_MAT");
  b.add_input<decl::Material>(N_("B"), "B_MAT");

  b.add_input<decl::Image>(N_("A"), "A_IMG");
  b.add_input<decl::Image>(N_("B"), "B_IMG");

  b.add_input<decl::Object>(N_("A"), "A_OBJ");
  b.add_input<decl::Object>(N_("B"), "B_OBJ");

  b.add_input<decl::Collection>(N_("A"), "A_COLL");
  b.add_input<decl::Collection>(N_("B"), "B_COLL");

  b.add_input<decl::Float>(N_("C")).default_value(0.9f);
  b.add_input<decl::Float>(N_("Angle")).default_value(0.0872665f).subtype(PROP_ANGLE);
  b.add_input<decl::Float>(N_("Epsilon")).default_value(0.001).min(-10000.0f).max(10000.0f);

  b.add_output<decl::Bool>(N_("Result"));
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  const NodeFunctionCompare &data = node_storage(*static_cast<const bNode *>(ptr->data));
  uiItemR(layout, ptr, "data_type", 0, "", ICON_NONE);
  if (data.data_type == SOCK_VECTOR) {
    uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);
  }
  uiItemR(layout, ptr, "operation", 0, "", ICON_NONE);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  NodeFunctionCompare *data = (NodeFunctionCompare *)node->storage;
  const eNodeSocketDatatype node_data_type = static_cast<eNodeSocketDatatype>(data->data_type);

  bNodeSocket *sock_comp = (bNodeSocket *)BLI_findlink(&node->inputs, 18);
  bNodeSocket *sock_angle = (bNodeSocket *)BLI_findlink(&node->inputs, 19);
  bNodeSocket *sock_epsilon = (bNodeSocket *)BLI_findlink(&node->inputs, 20);

  LISTBASE_FOREACH (bNodeSocket *, socket, &node->inputs) {
    nodeSetSocketAvailability(ntree, socket, socket->type == node_data_type);
  }

  nodeSetSocketAvailability(ntree,
                            sock_epsilon,
                            ELEM(data->operation, NODE_COMPARE_EQUAL, NODE_COMPARE_NOT_EQUAL) &&
                                !ELEM(data->data_type,
                                      SOCK_INT,
                                      SOCK_STRING,
                                      SOCK_OBJECT,
                                      SOCK_COLLECTION,
                                      SOCK_MATERIAL,
                                      SOCK_IMAGE));

  nodeSetSocketAvailability(ntree,
                            sock_comp,
                            ELEM(data->mode, NODE_COMPARE_MODE_DOT_PRODUCT) &&
                                data->data_type == SOCK_VECTOR);

  nodeSetSocketAvailability(ntree,
                            sock_angle,
                            ELEM(data->mode, NODE_COMPARE_MODE_DIRECTION) &&
                                node_data_type == SOCK_VECTOR);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeFunctionCompare *data = MEM_cnew<NodeFunctionCompare>(__func__);
  data->operation = NODE_COMPARE_GREATER_THAN;
  data->data_type = SOCK_FLOAT;
  data->mode = NODE_COMPARE_MODE_ELEMENT;
  node->storage = data;
}

class SocketSearchOp {
 public:
  std::string socket_name;
  eNodeSocketDatatype data_type;
  NodeCompareOperation operation;
  NodeCompareMode mode = NODE_COMPARE_MODE_ELEMENT;
  void operator()(LinkSearchOpParams &params)
  {
    bNode &node = params.add_node("GeometryNodeCompare");
    node_storage(node).data_type = data_type;
    node_storage(node).operation = operation;
    node_storage(node).mode = mode;
    params.update_and_connect_available_socket(node, socket_name);
  }
};

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const eNodeSocketDatatype type = static_cast<eNodeSocketDatatype>(params.other_socket().type);

  const eNodeSocketDatatype mode_type = (type == SOCK_BOOLEAN) ? SOCK_INT : type;
  const bool compare_only_type = ELEM(type, SOCK_STRING, SOCK_IMAGE, SOCK_MATERIAL, SOCK_OBJECT);

  const std::string socket_name = params.in_out() == SOCK_IN ? "A" : "Result";

  for (const EnumPropertyItem *item = rna_enum_node_compare_operation_items;
       item->identifier != nullptr;
       item++) {
    if (item->name != nullptr && item->identifier[0] != '\0') {
      if (!compare_only_type &&
          ELEM(item->value, NODE_COMPARE_COLOR_BRIGHTER, NODE_COMPARE_COLOR_DARKER)) {
        params.add_item(IFACE_(item->name),
                        SocketSearchOp{socket_name,
                                       SOCK_RGBA,
                                       static_cast<NodeCompareOperation>(item->value)});
      }
      else if ((!compare_only_type) ||
               (compare_only_type &&
                ELEM(item->value, NODE_COMPARE_EQUAL, NODE_COMPARE_NOT_EQUAL))) {
        params.add_item(IFACE_(item->name),
                        SocketSearchOp{socket_name,
                                       mode_type,
                                       static_cast<NodeCompareOperation>(item->value)});
      }
    }
  }
  /* Add Angle socket. */
  if (!compare_only_type && params.in_out() == SOCK_IN) {
    params.add_item(
        IFACE_("Angle"),
        SocketSearchOp{
            "Angle", SOCK_VECTOR, NODE_COMPARE_GREATER_THAN, NODE_COMPARE_MODE_DIRECTION});
  }
}

static void node_label(const bNodeTree * /*tree*/, const bNode *node, char *label, int maxlen)
{
  const NodeFunctionCompare *data = (NodeFunctionCompare *)node->storage;
  const char *name;
  bool enum_label = RNA_enum_name(rna_enum_node_compare_operation_items, data->operation, &name);
  if (!enum_label) {
    name = "Unknown";
  }
  BLI_strncpy(label, IFACE_(name), maxlen);
}

static float component_average(float3 a)
{
  return (a.x + a.y + a.z) / 3.0f;
}

template<typename T> void compare_fields(GeoNodeExecParams &params, const StringRef type_name);

template<> void compare_fields<float>(GeoNodeExecParams &params, const StringRef type_name)
{
  static std::string input_a = "A" + type_name;
  static std::string input_b = "B" + type_name;

  Field<float> value_a = params.extract_input<Field<float>>(input_a);
  Field<float> value_b = params.extract_input<Field<float>>(input_b);

  static auto exec_preset_all = mf::build::exec_presets::AllSpanOrSingle();
  static auto exec_preset_first_two = mf::build::exec_presets::SomeSpanOrSingle<0, 1>();

  Vector<GField> inputs;
  inputs.reserve(3);
  inputs.append(std::move(value_a));
  inputs.append(std::move(value_b));

  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  mf::MultiFunction *cmp_fn;
  switch (storage.operation) {
    case NODE_COMPARE_LESS_THAN: {
      static auto fn = mf::build::SI2_SO<float, float, bool>(
          "Less Than", [](float a, float b) { return a < b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_LESS_EQUAL: {
      static auto fn = mf::build::SI2_SO<float, float, bool>(
          "Less Equal", [](float a, float b) { return a <= b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_GREATER_THAN: {
      static auto fn = mf::build::SI2_SO<float, float, bool>(
          "Greater Than", [](float a, float b) { return a > b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_GREATER_EQUAL: {
      static auto fn = mf::build::SI2_SO<float, float, bool>(
          "Greater Equal", [](float a, float b) { return a >= b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_EQUAL: {
      Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
      inputs.append(std::move(epsilon));

      static auto fn = mf::build::SI3_SO<float, float, float, bool>(
          "Equal",
          [](float a, float b, float epsilon) { return std::abs(a - b) <= epsilon; },
          exec_preset_first_two);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_NOT_EQUAL:
      Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
      inputs.append(std::move(epsilon));

      static auto fn = mf::build::SI3_SO<float, float, float, bool>(
          "Not Equal",
          [](float a, float b, float epsilon) { return std::abs(a - b) > epsilon; },
          exec_preset_first_two);
      cmp_fn = &fn;
      break;
  }
  Field<bool> result(FieldOperation::Create(*cmp_fn, std::move(inputs)), 0);
  params.set_output("Result", std::move(result));
}

template<> void compare_fields<int>(GeoNodeExecParams &params, const StringRef type_name)
{
  static std::string input_a = "A" + type_name;
  static std::string input_b = "B" + type_name;

  Field<int> value_a = params.extract_input<Field<int>>(input_a);
  Field<int> value_b = params.extract_input<Field<int>>(input_b);

  static auto exec_preset_all = mf::build::exec_presets::AllSpanOrSingle();
  static auto exec_preset_first_two = mf::build::exec_presets::SomeSpanOrSingle<0, 1>();

  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  mf::MultiFunction *cmp_fn;
  switch (storage.operation) {
    case NODE_COMPARE_LESS_THAN: {
      static auto fn = mf::build::SI2_SO<int, int, bool>(
          "Less Than", [](int a, int b) { return a < b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_LESS_EQUAL: {
      static auto fn = mf::build::SI2_SO<int, int, bool>(
          "Less Equal", [](int a, int b) { return a <= b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_GREATER_THAN: {
      static auto fn = mf::build::SI2_SO<int, int, bool>(
          "Greater Than", [](int a, int b) { return a > b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_GREATER_EQUAL: {
      static auto fn = mf::build::SI2_SO<int, int, bool>(
          "Greater Equal", [](int a, int b) { return a >= b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_EQUAL: {
      static auto fn = mf::build::SI2_SO<int, int, bool>(
          "Equal", [](int a, int b) { return a == b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_NOT_EQUAL: {
      static auto fn = mf::build::SI2_SO<int, int, bool>(
          "Not Equal", [](int a, int b) { return a != b; }, exec_preset_all);
      cmp_fn = &fn;
      break;
    }
  }
  Field<bool> result(FieldOperation::Create(*cmp_fn, {std::move(value_a), std::move(value_b)}), 0);
  params.set_output("Result", std::move(result));
}

template<> void compare_fields<float3>(GeoNodeExecParams &params, const StringRef type_name)
{
  static std::string input_a = "A" + type_name;
  static std::string input_b = "B" + type_name;

  Field<float3> value_a = params.extract_input<Field<float3>>(input_a);
  Field<float3> value_b = params.extract_input<Field<float3>>(input_b);

  Vector<GField> inputs;
  inputs.reserve(3);
  inputs.append(std::move(value_a));
  inputs.append(std::move(value_b));

  static auto exec_preset_all = mf::build::exec_presets::AllSpanOrSingle();
  static auto exec_preset_first_two = mf::build::exec_presets::SomeSpanOrSingle<0, 1>();

  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  mf::MultiFunction *cmp_fn;
  switch (storage.operation) {
    case NODE_COMPARE_LESS_THAN:
      switch (storage.mode) {
        case NODE_COMPARE_MODE_AVERAGE: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Less Than - Average",
              [](float3 a, float3 b) { return component_average(a) < component_average(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DOT_PRODUCT: {
          Field<float> c = params.extract_input<Field<float>>("C");
          inputs.append(std::move(c));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Less Than - Dot Product",
              [](float3 a, float3 b, float comp) { return math::dot(a, b) < comp; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DIRECTION: {
          Field<float> angle = params.extract_input<Field<float>>("Angle");
          inputs.append(std::move(angle));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Less Than - Direction",
              [](float3 a, float3 b, float angle) { return angle_v3v3(a, b) < angle; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_ELEMENT: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Less Than - Element-wise",
              [](float3 a, float3 b) { return a.x < b.x && a.y < b.y && a.z < b.z; },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_LENGTH: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Less Than - Length",
              [](float3 a, float3 b) { return math::length(a) < math::length(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
      }
      break;
    case NODE_COMPARE_LESS_EQUAL:
      switch (storage.mode) {
        case NODE_COMPARE_MODE_AVERAGE: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Less Equal - Average",
              [](float3 a, float3 b) { return component_average(a) <= component_average(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DOT_PRODUCT: {
          Field<float> c = params.extract_input<Field<float>>("C");
          inputs.append(std::move(c));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Less Equal - Dot Product",
              [](float3 a, float3 b, float comp) { return math::dot(a, b) <= comp; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DIRECTION: {
          Field<float> angle = params.extract_input<Field<float>>("Angle");
          inputs.append(std::move(angle));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Less Equal - Direction",
              [](float3 a, float3 b, float angle) { return angle_v3v3(a, b) <= angle; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_ELEMENT: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Less Equal - Element-wise",
              [](float3 a, float3 b) { return a.x <= b.x && a.y <= b.y && a.z <= b.z; },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_LENGTH: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Less Equal - Length",
              [](float3 a, float3 b) { return math::length(a) <= math::length(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
      }
      break;
    case NODE_COMPARE_GREATER_THAN:
      switch (storage.mode) {
        case NODE_COMPARE_MODE_AVERAGE: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Greater Than - Average",
              [](float3 a, float3 b) { return component_average(a) > component_average(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DOT_PRODUCT: {
          Field<float> c = params.extract_input<Field<float>>("C");
          inputs.append(std::move(c));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Greater Than - Dot Product",
              [](float3 a, float3 b, float comp) { return math::dot(a, b) > comp; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DIRECTION: {
          Field<float> angle = params.extract_input<Field<float>>("Angle");
          inputs.append(std::move(angle));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Greater Than - Direction",
              [](float3 a, float3 b, float angle) { return angle_v3v3(a, b) > angle; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_ELEMENT: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Greater Than - Element-wise",
              [](float3 a, float3 b) { return a.x > b.x && a.y > b.y && a.z > b.z; },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_LENGTH: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Greater Than - Length",
              [](float3 a, float3 b) { return math::length(a) > math::length(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
      }
      break;
    case NODE_COMPARE_GREATER_EQUAL:
      switch (storage.mode) {
        case NODE_COMPARE_MODE_AVERAGE: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Greater Equal - Average",
              [](float3 a, float3 b) { return component_average(a) >= component_average(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DOT_PRODUCT: {
          Field<float> c = params.extract_input<Field<float>>("C");
          inputs.append(std::move(c));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Greater Equal - Dot Product",
              [](float3 a, float3 b, float comp) { return math::dot(a, b) >= comp; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DIRECTION: {
          Field<float> angle = params.extract_input<Field<float>>("Angle");
          inputs.append(std::move(angle));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Greater Equal - Direction",
              [](float3 a, float3 b, float angle) { return angle_v3v3(a, b) >= angle; },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_ELEMENT: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Greater Equal - Element-wise",
              [](float3 a, float3 b) { return a.x >= b.x && a.y >= b.y && a.z >= b.z; },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_LENGTH: {
          static auto fn = mf::build::SI2_SO<float3, float3, bool>(
              "Greater Equal - Length",
              [](float3 a, float3 b) { return math::length(a) >= math::length(b); },
              exec_preset_all);
          cmp_fn = &fn;
          break;
        }
      }
      break;
    case NODE_COMPARE_EQUAL:
      switch (storage.mode) {
        case NODE_COMPARE_MODE_AVERAGE: {
          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Equal - Average",
              [](float3 a, float3 b, float epsilon) {
                return abs(component_average(a) - component_average(b)) <= epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DOT_PRODUCT: {
          Field<float> c = params.extract_input<Field<float>>("C");
          inputs.append(std::move(c));

          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI4_SO<float3, float3, float, float, bool>(
              "Equal - Dot Product",
              [](float3 a, float3 b, float comp, float epsilon) {
                return abs(math::dot(a, b) - comp) <= epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DIRECTION: {
          Field<float> angle = params.extract_input<Field<float>>("Angle");
          inputs.append(std::move(angle));

          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI4_SO<float3, float3, float, float, bool>(
              "Equal - Direction",
              [](float3 a, float3 b, float angle, float epsilon) {
                return abs(angle_v3v3(a, b) - angle) <= epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_ELEMENT: {
          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Equal - Element-wise",
              [](float3 a, float3 b, float epsilon) {
                return abs(a.x - b.x) <= epsilon && abs(a.y - b.y) <= epsilon &&
                       abs(a.z - b.z) <= epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_LENGTH: {
          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Equal - Length",
              [](float3 a, float3 b, float epsilon) {
                return abs(math::length(a) - math::length(b)) <= epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
      }
      break;
    case NODE_COMPARE_NOT_EQUAL:
      switch (storage.mode) {
        case NODE_COMPARE_MODE_AVERAGE: {
          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Not Equal - Average",
              [](float3 a, float3 b, float epsilon) {
                return abs(component_average(a) - component_average(b)) > epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DOT_PRODUCT: {
          Field<float> c = params.extract_input<Field<float>>("C");
          inputs.append(std::move(c));

          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI4_SO<float3, float3, float, float, bool>(
              "Not Equal - Dot Product",
              [](float3 a, float3 b, float comp, float epsilon) {
                return abs(math::dot(a, b) - comp) >= epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_DIRECTION: {
          Field<float> angle = params.extract_input<Field<float>>("Angle");
          inputs.append(std::move(angle));

          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI4_SO<float3, float3, float, float, bool>(
              "Not Equal - Direction",
              [](float3 a, float3 b, float angle, float epsilon) {
                return abs(angle_v3v3(a, b) - angle) > epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_ELEMENT: {
          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Not Equal - Element-wise",
              [](float3 a, float3 b, float epsilon) {
                return abs(a.x - b.x) > epsilon || abs(a.y - b.y) > epsilon ||
                       abs(a.z - b.z) > epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
        case NODE_COMPARE_MODE_LENGTH: {
          Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
          inputs.append(std::move(epsilon));

          static auto fn = mf::build::SI3_SO<float3, float3, float, bool>(
              "Not Equal - Length",
              [](float3 a, float3 b, float epsilon) {
                return abs(math::length(a) - math::length(b)) > epsilon;
              },
              exec_preset_first_two);
          cmp_fn = &fn;
          break;
        }
      }
      break;
  }
  Field<bool> result(FieldOperation::Create(*cmp_fn, std::move(inputs)), 0);
  params.set_output("Result", std::move(result));
}

template<>
void compare_fields<ColorGeometry4f>(GeoNodeExecParams &params, const StringRef type_name)
{
  static std::string input_a = "A" + type_name;
  static std::string input_b = "B" + type_name;

  Field<ColorGeometry4f> value_a = params.extract_input<Field<ColorGeometry4f>>(input_a);
  Field<ColorGeometry4f> value_b = params.extract_input<Field<ColorGeometry4f>>(input_b);

  static auto exec_preset_all = mf::build::exec_presets::AllSpanOrSingle();
  static auto exec_preset_first_two = mf::build::exec_presets::SomeSpanOrSingle<0, 1>();

  Vector<GField> inputs;
  inputs.reserve(3);
  inputs.append(std::move(value_a));
  inputs.append(std::move(value_b));

  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  mf::MultiFunction *cmp_fn;
  switch (storage.operation) {
    case NODE_COMPARE_EQUAL: {
      Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
      inputs.append(std::move(epsilon));

      static auto fn = mf::build::SI3_SO<ColorGeometry4f, ColorGeometry4f, float, bool>(
          "Equal",
          [](ColorGeometry4f a, ColorGeometry4f b, float epsilon) {
            return abs(a.r - b.r) <= epsilon && abs(a.g - b.g) <= epsilon &&
                   abs(a.b - b.b) <= epsilon;
          },
          exec_preset_first_two);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_NOT_EQUAL: {
      Field<float> epsilon = params.extract_input<Field<float>>("Epsilon");
      inputs.append(std::move(epsilon));

      static auto fn = mf::build::SI3_SO<ColorGeometry4f, ColorGeometry4f, float, bool>(
          "Not Equal",
          [](ColorGeometry4f a, ColorGeometry4f b, float epsilon) {
            return abs(a.r - b.r) > epsilon || abs(a.g - b.g) > epsilon ||
                   abs(a.b - b.b) > epsilon;
          },
          exec_preset_first_two);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_COLOR_BRIGHTER: {
      static auto fn = mf::build::SI2_SO<ColorGeometry4f, ColorGeometry4f, bool>(
          "Brighter",
          [](ColorGeometry4f a, ColorGeometry4f b) {
            return rgb_to_grayscale(a) > rgb_to_grayscale(b);
          },
          exec_preset_all);
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_COLOR_DARKER: {
      static auto fn = mf::build::SI2_SO<ColorGeometry4f, ColorGeometry4f, bool>(
          "Darker",
          [](ColorGeometry4f a, ColorGeometry4f b) {
            return rgb_to_grayscale(a) < rgb_to_grayscale(b);
          },
          exec_preset_all);
      cmp_fn = &fn;
      break;
    }
  }
  Field<bool> result(FieldOperation::Create(*cmp_fn, std::move(inputs)), 0);
  params.set_output("Result", std::move(result));
}

template<> void compare_fields<std::string>(GeoNodeExecParams &params, const StringRef type_name)
{
  static std::string input_a = "A" + type_name;
  static std::string input_b = "B" + type_name;

  Field<std::string> value_a = params.extract_input<Field<std::string>>(input_a);
  Field<std::string> value_b = params.extract_input<Field<std::string>>(input_b);

  static auto exec_preset_all = mf::build::exec_presets::AllSpanOrSingle();
  static auto exec_preset_first_two = mf::build::exec_presets::SomeSpanOrSingle<0, 1>();

  Vector<GField> inputs;
  inputs.reserve(2);
  inputs.append(std::move(value_a));
  inputs.append(std::move(value_b));

  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  mf::MultiFunction *cmp_fn;
  switch (storage.operation) {
    case NODE_COMPARE_EQUAL: {
      static auto fn = mf::build::SI2_SO<std::string, std::string, bool>(
          "Equal", [](std::string a, std::string b) { return a == b; });
      cmp_fn = &fn;
      break;
    }
    case NODE_COMPARE_NOT_EQUAL: {
      static auto fn = mf::build::SI2_SO<std::string, std::string, bool>(
          "Not Equal", [](std::string a, std::string b) { return a != b; });
      cmp_fn = &fn;
      break;
    }
  }
  Field<bool> result(FieldOperation::Create(*cmp_fn, std::move(inputs)), 0);
  params.set_output("Result", std::move(result));
}

template<typename T>
static void compare_pointers(GeoNodeExecParams &params, const StringRef type_name)
{
  static std::string input_a = "A" + type_name;
  static std::string input_b = "B" + type_name;

  const T value_a = params.extract_input<T>(input_a);
  const T value_b = params.extract_input<T>(input_b);

  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  switch (storage.operation) {
    case NODE_COMPARE_EQUAL: {
      params.set_output("Result", value_a == value_b);
      break;
    }
    case NODE_COMPARE_NOT_EQUAL: {
      params.set_output("Result", value_a != value_b);
      break;
    }
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const NodeFunctionCompare &storage = *reinterpret_cast<NodeFunctionCompare *>(
      params.node().storage);
  const eNodeSocketDatatype data_type = static_cast<eNodeSocketDatatype>(storage.data_type);

  switch (data_type) {
    case SOCK_FLOAT: {
      compare_fields<float>(params, "");
      break;
    }
    case SOCK_INT: {
      compare_fields<int>(params, "_INT");
      break;
    }
    case SOCK_VECTOR: {
      compare_fields<float3>(params, "_VEC3");
      break;
    }
    case SOCK_RGBA: {
      compare_fields<ColorGeometry4f>(params, "_COL");
      break;
    }
    case SOCK_STRING: {
      compare_fields<std::string>(params, "_STR");
      break;
    }
    case SOCK_OBJECT: {
      compare_pointers<Object *>(params, "_OBJ");
      break;
    }
    case SOCK_COLLECTION: {
      compare_pointers<Collection *>(params, "_COLL");
      break;
    }
    case SOCK_MATERIAL: {
      compare_pointers<Material *>(params, "_MAT");
      break;
    }
    case SOCK_IMAGE: {
      compare_pointers<Image *>(params, "_IMG");
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }
}

}  // namespace blender::nodes::node_geo_compare_cc

void register_node_type_geo_compare()
{
  namespace file_ns = blender::nodes::node_geo_compare_cc;

  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_COMPARE, "Compare", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  // ntype.labelfunc = file_ns::node_label;
  ntype.updatefunc = file_ns::node_update;
  ntype.initfunc = file_ns::node_init;
  node_type_storage(
      &ntype, "NodeFunctionCompare", node_free_standard_storage, node_copy_standard_storage);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;
  nodeRegisterType(&ntype);
}
