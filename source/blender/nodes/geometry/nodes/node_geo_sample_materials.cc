/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BKE_attribute.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sample_materials_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"));
  b.add_input<decl::Geometry>(N_("Material Source"));
  b.add_input<decl::Bool>(N_("Selection")).default_value(true).hide_value().field_on({0});
  b.add_input<decl::Int>(N_("Material Index")).field_on({0});
  /* TODO: It's possible to avoid propagation from second geometry input? Or i miss something? */
  b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry");
  GeometrySet source_geometry = params.extract_input<GeometrySet>("Material Source");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<int> index_field = params.extract_input<Field<int>>("Material Index");

  const Mesh *source_mesh = source_geometry.get_mesh_for_read();

  if (source_mesh == nullptr) {
    params.set_output("Geometry", std::move(geometry));
    return;
  }

  if (source_mesh->totcol == 0) {
    params.set_output("Geometry", std::move(geometry));
    return;
  }

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

    const Span<Material *> original_materials(mesh->mat, mesh->totcol);

    Vector<Material *> materials = original_materials;

    const auto append_material = [source_mesh, &materials](const int index) -> int {
      const int limited_index = math::clamp<int>(index, 0, source_mesh->totcol - 1);
      Material *material = source_mesh->mat[limited_index];
      return materials.append_non_duplicates_and_get_index(material);
    };

    bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
    SpanAttributeWriter<int> dst_indices_attribute =
        attributes.lookup_or_add_for_write_only_span<int>("material_index", ATTR_DOMAIN_FACE);
    MutableSpan<int> dst_indices = dst_indices_attribute.span;

    const VArray<int> &indices = evaluator.get_evaluated<int>(0);
    devirtualize_varray(indices, [&](auto indices) {
      selection.foreach_index(
          [&](const int index) { dst_indices[index] = append_material(indices[index]); });
    });
    dst_indices_attribute.finish();

    const int result_tot_material = materials.size();
    const Span<Material *> new_materials = materials.as_span().drop_front(mesh->totcol);

    if (new_materials.is_empty()) {
      return;
    }

    Array<Material *> old_src_material = original_materials;
    mesh->mat = static_cast<Material **>(
        MEM_recallocN_id(mesh->mat, sizeof(void *) * result_tot_material, "matarray"));
    MutableSpan<Material *> r_materials(mesh->mat, result_tot_material);
    r_materials.drop_front(mesh->totcol).copy_from(new_materials);
    mesh->totcol = result_tot_material;
  });

  params.set_output("Geometry", std::move(geometry));
}

}  // namespace blender::nodes::node_geo_sample_materials_cc

void register_node_type_geo_sample_materials()
{
  namespace file_ns = blender::nodes::node_geo_sample_materials_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_MATERIALS, "Sample Materials", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
