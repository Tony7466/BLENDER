/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"

#include "BLI_task.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_curves.hh"
#include "BKE_mesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sample_material_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"));
  b.add_input<decl::Geometry>(N_("Material Source"));
  b.add_input<decl::Bool>(N_("Selection")).default_value(true).hide_value().field_on({0});
  b.add_input<decl::Int>(N_("Material Index")).default_value(1).field_on({0});
  /* TODO: It's possible to avoid propagation from second geometry input? Or i miss something? */
  b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<int> index_field = params.extract_input<Field<int>>("Material Index");

  geometry.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const MeshComponent *component = geometry_set.get_component_for_read<MeshComponent>();
    Mesh *mesh = geometry_set.get_mesh_for_write();
    if (!mesh) {
      return;
    }

    bke::GeometryFieldContext field_context{*component, ATTR_DOMAIN_FACE};
    fn::FieldEvaluator evaluator{field_context, mesh->totpoly};
    evaluator.set_selection(selection_field);
    evaluator.add(index_field);
    evaluator.evaluate();

    const IndexMask selection = evaluator.get_evaluated_selection_as_mask();
    if (selection.is_empty()) {
      return;
    }
    
    bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
    SpanAttributeWriter<int> dst_indices_attribute = attributes.lookup_or_add_for_write_only_span<int>("material_index", ATTR_DOMAIN_FACE);
    MutableSpan<int> dst_indices = dst_indices_attribute.span;
    
    const VArray<int> &indices = evaluator.get_evaluated<int>(0);
    
    indices.materialize(selection, dst_indices);
    
    dst_indices_attribute.finish();
  });

  params.set_output("Geometry", std::move(geometry));
}

}  // namespace blender::nodes::node_geo_sample_material_cc

void register_node_type_geo_sample_material()
{
  namespace file_ns = blender::nodes::node_geo_sample_material_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_MATERIAL, "Sample Material", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
