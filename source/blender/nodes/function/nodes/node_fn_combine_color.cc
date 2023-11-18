/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_fn_combine_color_cc {

NODE_STORAGE_FUNCS(NodeCombSepColor)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();

  const bNode *node = b.node_or_null();
  if (node != nullptr) {
    const NodeCombSepColor &storage = node_storage(*node);
    const NodeCombSepColorMode mode = NodeCombSepColorMode(storage.mode);

    switch (mode) {
      case NODE_COMBSEP_COLOR_RGB:
        b.add_input<decl::Float>("Red").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        b.add_input<decl::Float>("Green").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        b.add_input<decl::Float>("Blue").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        break;
      case NODE_COMBSEP_COLOR_HSL:
        b.add_input<decl::Float>("Hue").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        b.add_input<decl::Float>("Saturation").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        b.add_input<decl::Float>("Lightness").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        break;
      case NODE_COMBSEP_COLOR_HSV:
        b.add_input<decl::Float>("Hue").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        b.add_input<decl::Float>("Saturation").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        b.add_input<decl::Float>("Value").min(0.0f).max(1.0f).subtype(PROP_FACTOR);
        break;
      default: {
        BLI_assert_unreachable();
        break;
      }
    }
  }
  b.add_input<decl::Float>("Alpha").default_value(1.0f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);

  b.add_output<decl::Color>("Color");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeCombSepColor *data = MEM_cnew<NodeCombSepColor>(__func__);
  data->mode = NODE_COMBSEP_COLOR_RGB;
  node->storage = data;
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  const NodeCombSepColor &storage = node_storage(bnode);

  static auto rgba_fn = mf::build::SI4_SO<float, float, float, float, ColorGeometry4f>(
      "RGB", [](float r, float g, float b, float a) { return ColorGeometry4f(r, g, b, a); });
  static auto hsva_fn = mf::build::SI4_SO<float, float, float, float, ColorGeometry4f>(
      "HSV", [](float h, float s, float v, float a) {
        ColorGeometry4f r_color;
        hsv_to_rgb(h, s, v, &r_color.r, &r_color.g, &r_color.b);
        r_color.a = a;
        return r_color;
      });
  static auto hsla_fn = mf::build::SI4_SO<float, float, float, float, ColorGeometry4f>(
      "HSL", [](float h, float s, float l, float a) {
        ColorGeometry4f color;
        hsl_to_rgb(h, s, l, &color.r, &color.g, &color.b);
        color.a = a;
        return color;
      });

  switch (storage.mode) {
    case NODE_COMBSEP_COLOR_RGB:
      return &rgba_fn;
    case NODE_COMBSEP_COLOR_HSV:
      return &hsva_fn;
    case NODE_COMBSEP_COLOR_HSL:
      return &hsla_fn;
  }

  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Mode of color processing",
                    rna_enum_node_combsep_color_items,
                    NOD_storage_enum_accessors(mode));
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_COMBINE_COLOR, "Combine Color", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  node_type_storage(
      &ntype, "NodeCombSepColor", node_free_standard_storage, node_copy_standard_storage);
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;

  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_combine_color_cc
