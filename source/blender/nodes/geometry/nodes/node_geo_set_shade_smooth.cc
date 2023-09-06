/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_set_shade_smooth_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Bool>("Smooth Edge").field_on_all().default_value(true);
  b.add_input<decl::Bool>("Smooth Face").field_on_all().default_value(true);
  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

enum class SmoothMode {
  Face = 0,
  Edge = 1,
  Both = 2,
};

static void node_update(bNodeTree *ntree, bNode *node)
{
  const SmoothMode mode = SmoothMode(node->custom1);
  bNodeSocket *edge = static_cast<bNodeSocket *>(node->inputs.first)->next->next;
  bNodeSocket *face = edge->next;
  bke::nodeSetSocketAvailability(ntree, edge, ELEM(mode, SmoothMode::Both, SmoothMode::Edge));
  bke::nodeSetSocketAvailability(ntree, face, ELEM(mode, SmoothMode::Both, SmoothMode::Face));
}

/**
 * When the `sharp_face` attribute doesn't exist, all faces are considered smooth. If all faces
 * are selected and the sharp value is a constant false value, we can remove the attribute instead
 * as an optimization to avoid storing it and propagating it in the future.
 */
static bool try_removing_sharp_attribute(Mesh &mesh,
                                         const StringRef name,
                                         const Field<bool> &selection_field,
                                         const Field<bool> &sharp_field)
{
  if (selection_field.node().depends_on_input() || sharp_field.node().depends_on_input()) {
    return false;
  }
  const bool selection = fn::evaluate_constant_field(selection_field);
  if (!selection) {
    return true;
  }
  const bool sharp = fn::evaluate_constant_field(sharp_field);
  if (sharp) {
    return false;
  }
  mesh.attributes_for_write().remove(name);
  return true;
}

static void set_sharp(Mesh &mesh,
                      const eAttrDomain domain,
                      const StringRef name,
                      const Field<bool> &selection_field,
                      const Field<bool> &sharp_field)
{
  const int domain_size = mesh.attributes().domain_size(domain);
  if (mesh.attributes().domain_size(domain) == 0) {
    return;
  }
  if (try_removing_sharp_attribute(mesh, name, selection_field, sharp_field)) {
    return;
  }

  MutableAttributeAccessor attributes = mesh.attributes_for_write();
  AttributeWriter<bool> sharp = attributes.lookup_or_add_for_write<bool>(name, domain);

  const bke::MeshFieldContext field_context{mesh, domain};
  fn::FieldEvaluator evaluator{field_context, domain_size};
  evaluator.set_selection(selection_field);
  evaluator.add_with_destination(sharp_field, sharp.varray);
  evaluator.evaluate();

  sharp.finish();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  const SmoothMode mode = SmoothMode(params.node().custom1);
  Field<bool> smooth_edge = ELEM(mode, SmoothMode::Both, SmoothMode::Edge) ?
                                params.extract_input<Field<bool>>("Smooth Edge") :
                                fn::make_constant_field<bool>(false);
  Field<bool> smooth_face = ELEM(mode, SmoothMode::Both, SmoothMode::Face) ?
                                params.extract_input<Field<bool>>("Smooth Face") :
                                fn::make_constant_field<bool>(false);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (Mesh *mesh = geometry_set.get_mesh_for_write()) {
      if (ELEM(mode, SmoothMode::Both, SmoothMode::Edge)) {
        set_sharp(*mesh,
                  ATTR_DOMAIN_EDGE,
                  "sharp_edge",
                  selection_field,
                  fn::invert_boolean_field(smooth_edge));
      }
      if (ELEM(mode, SmoothMode::Both, SmoothMode::Face)) {
        set_sharp(*mesh,
                  ATTR_DOMAIN_FACE,
                  "sharp_face",
                  selection_field,
                  fn::invert_boolean_field(smooth_face));
      }
    }
  });
  params.set_output("Geometry", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {int(SmoothMode::Both), "BOTH", 0, "Both", ""},
      {int(SmoothMode::Edge), "EDGE", 0, "Edge", ""},
      {int(SmoothMode::Face), "FACE", 0, "Face", "Face"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna, "mode", "Mode", "", mode_items, NOD_inline_enum_accessors(custom1));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SET_SHADE_SMOOTH, "Set Shade Smooth", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.updatefunc = node_update;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_shade_smooth_cc
