/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_length_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve").supported_type(
      {GeometryComponent::Type::Curve, GeometryComponent::Type::GreasePencil});
  b.add_output<decl::Float>("Length");
}

static float curves_total_length(const bke::CurvesGeometry &curves)
{
  const VArray<bool> cyclic = curves.cyclic();
  curves.ensure_evaluated_lengths();
  
  float total_length = 0.0f;
  for (const int i : curves.curves_range()) {
    total_length += curves.evaluated_length_total_for_curve(i, cyclic[i]);
  }
  return total_length;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");
  float length = 0.0f;
  if (geometry_set.has_curves()) {
    const Curves &curves_id = *geometry_set.get_curves();
    const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
    length += curves_total_length(curves);
  }
  else if (geometry_set.has_grease_pencil()) {
    using namespace bke::greasepencil;
    GreasePencil &grease_pencil = *geometry_set.get_grease_pencil_for_write();
    for (const Layer *layer : grease_pencil.layers()) {
      Drawing *drawing = grease_pencil.get_editable_drawing_at(layer,
                                                               grease_pencil.runtime->eval_frame);
      if (drawing == nullptr) {
        continue;
      }
      const bke::CurvesGeometry &curves = drawing->strokes();
      length += curves_total_length(curves);
    }
  }
  else {
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Length", length);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_CURVE_LENGTH, "Curve Length", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_length_cc
