/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_extract_elements.hh"

#include "BLI_index_mask.hh"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

namespace blender::geometry {

Array<Mesh *> extract_mesh_vertices(const Mesh &mesh,
                                    const IndexMask &mask,
                                    const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.verts_num);
  Array<Mesh *> elements(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int vert_i, const int element_i) {
    Mesh *element = BKE_mesh_new_nomain(1, 0, 0, 0);
    BKE_mesh_copy_parameters_for_eval(element, &mesh);

    /* TODO: Propagate attributes from other domains. Same for other functions. */
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{vert_i},
                           element->attributes_for_write());
    elements[element_i] = element;
  });

  return elements;
}

Array<Mesh *> extract_mesh_edges(const Mesh &mesh,
                                 const IndexMask &mask,
                                 const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.edges_num);
  Array<Mesh *> elements(mask.size(), nullptr);

  const Span<int2> src_edges = mesh.edges();
  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int edge_i, const int element_i) {
    Mesh *element = BKE_mesh_new_nomain(2, 1, 0, 0);
    BKE_mesh_copy_parameters_for_eval(element, &mesh);

    MutableSpan<int2> element_edges = element->edges_for_write();
    element_edges[0] = {0, 1};

    const int2 &src_edge = src_edges[edge_i];
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>({src_edge[0], src_edge[1]}),
                           element->attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Edge,
                           bke::AttrDomain::Edge,
                           bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                           Span<int>{edge_i},
                           element->attributes_for_write());

    elements[element_i] = element;
  });

  return elements;
}

Array<Mesh *> extract_mesh_faces(const Mesh &mesh,
                                 const IndexMask &mask,
                                 const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= mesh.faces_num);
  Array<Mesh *> elements(mask.size(), nullptr);

  const Span<int> src_corner_verts = mesh.corner_verts();
  const Span<int> src_corner_edges = mesh.corner_edges();
  const OffsetIndices<int> src_faces = mesh.faces();

  const bke::AttributeAccessor src_attributes = mesh.attributes();

  mask.foreach_index(GrainSize(32), [&](const int face_i, const int element_i) {
    const IndexRange src_face = src_faces[face_i];
    const int verts_num = src_face.size();

    Mesh *element = BKE_mesh_new_nomain(verts_num, verts_num, 1, verts_num);
    BKE_mesh_copy_parameters_for_eval(element, &mesh);

    MutableSpan<int2> element_edges = element->edges_for_write();
    MutableSpan<int> element_corner_verts = element->corner_verts_for_write();
    MutableSpan<int> element_corner_edges = element->corner_edges_for_write();
    MutableSpan<int> element_face_offsets = element->face_offsets_for_write();

    for (const int i : IndexRange(verts_num)) {
      element_edges[i] = {i, i + 1};
      element_corner_verts[i] = i;
      element_corner_edges[i] = i;
    }
    element_edges.last()[1] = 0;
    element_face_offsets[0] = 0;
    element_face_offsets[1] = verts_num;

    Array<int> old_corner_indices(verts_num);
    Array<int> old_edge_indices(verts_num);
    Array<int> old_vert_indices(verts_num);
    for (const int i : IndexRange(verts_num)) {
      const int src_corner_i = src_face[i];
      const int src_edge_i = src_corner_edges[src_corner_i];
      const int src_vert_i = src_corner_verts[src_corner_i];
      old_corner_indices[i] = src_corner_i;
      old_edge_indices[i] = src_edge_i;
      old_vert_indices[i] = src_vert_i;
    }

    bke::MutableAttributeAccessor element_attributes = element->attributes_for_write();
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           old_vert_indices,
                           element_attributes);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Edge,
                           bke::AttrDomain::Edge,
                           bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                           old_edge_indices,
                           element_attributes);
    bke::gather_attributes(
        src_attributes,
        bke::AttrDomain::Corner,
        bke::AttrDomain::Corner,
        bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_edge", ".corner_vert"}),
        old_corner_indices,
        element_attributes);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Face,
                           bke::AttrDomain::Face,
                           attribute_filter,
                           Span<int>{face_i},
                           element_attributes);

    elements[element_i] = element;
  });

  return elements;
}

Array<PointCloud *> extract_pointcloud_points(const PointCloud &pointcloud,
                                              const IndexMask &mask,
                                              const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= pointcloud.totpoint);
  Array<PointCloud *> elements(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = pointcloud.attributes();

  mask.foreach_index(GrainSize(32), [&](const int point_i, const int element_i) {
    PointCloud *element = BKE_pointcloud_new_nomain(1);
    element->totcol = pointcloud.totcol;
    element->mat = static_cast<Material **>(MEM_dupallocN(pointcloud.mat));

    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{point_i},
                           element->attributes_for_write());
    elements[element_i] = element;
  });

  return elements;
}

Array<Curves *> extract_curves_points(const Curves &curves,
                                      const IndexMask &mask,
                                      const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= curves.geometry.point_num);
  Array<Curves *> elements(mask.size(), nullptr);

  const bke::CurvesGeometry &src_curves = curves.geometry.wrap();
  const bke::AttributeAccessor src_attributes = src_curves.attributes();

  mask.foreach_index(GrainSize(32), [&](const int point_i, const int element_i) {
    /* TODO: Use src curve type. */
    Curves *element = bke::curves_new_nomain_single(1, CURVE_TYPE_POLY);
    bke::curves_copy_parameters(curves, *element);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{point_i},
                           element->geometry.wrap().attributes_for_write());
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Curve,
                           bke::AttrDomain::Curve,
                           attribute_filter,
                           Span<int>{point_i},
                           element->geometry.wrap().attributes_for_write());
    elements[element_i] = element;
  });

  return elements;
}

Array<Curves *> extract_curves(const Curves &curves,
                               const IndexMask &mask,
                               const bke::AttributeFilter &attribute_filter)
{
  BLI_assert(mask.min_array_size() <= curves.geometry.curve_num);
  Array<Curves *> elements(mask.size(), nullptr);

  const bke::CurvesGeometry &src_curves = curves.geometry.wrap();
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  const OffsetIndices<int> src_points_by_curve = src_curves.points_by_curve();

  mask.foreach_index(GrainSize(32), [&](const int curve_i, const int element_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const int points_num = src_points.size();
    Curves *element = bke::curves_new_nomain_single(points_num, CURVE_TYPE_POLY);
    bke::MutableAttributeAccessor element_attributes =
        element->geometry.wrap().attributes_for_write();
    bke::curves_copy_parameters(curves, *element);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           src_points,
                           element_attributes);
    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Curve,
                           bke::AttrDomain::Curve,
                           attribute_filter,
                           Span<int>{curve_i},
                           element_attributes);
    element->geometry.wrap().update_curve_types();
    elements[element_i] = element;
  });

  return elements;
}

Array<bke::Instances *> extract_instances(const bke::Instances &instances,
                                          const IndexMask &mask,
                                          const bke::AttributeFilter &attribute_filter)
{
  using bke::Instances;
  BLI_assert(mask.min_array_size() <= instances.instances_num());
  Array<Instances *> elements(mask.size(), nullptr);

  const bke::AttributeAccessor src_attributes = instances.attributes();
  const Span<bke::InstanceReference> src_references = instances.references();
  const Span<int> src_reference_handles = instances.reference_handles();
  const Span<float4x4> src_transforms = instances.transforms();

  mask.foreach_index(GrainSize(32), [&](const int instance_i, const int element_i) {
    const int old_handle = src_reference_handles[instance_i];
    const bke::InstanceReference &old_reference = src_references[old_handle];
    const float4x4 &old_transform = src_transforms[instance_i];

    Instances *element = new Instances();
    const int new_handle = element->add_new_reference(old_reference);
    element->add_instance(new_handle, old_transform);

    bke::gather_attributes(src_attributes,
                           bke::AttrDomain::Instance,
                           bke::AttrDomain::Instance,
                           bke::attribute_filter_with_skip_ref(
                               attribute_filter, {".reference_index", "instance_transform"}),
                           Span<int>{instance_i},
                           element->attributes_for_write());

    elements[element_i] = element;
  });

  return elements;
}

Array<GreasePencil *> extract_greasepencil_layers(const GreasePencil &grease_pencil,
                                                  const IndexMask &mask,
                                                  const bke::AttributeFilter &attribute_filter)
{
  using namespace bke::greasepencil;
  BLI_assert(mask.min_array_size() <= grease_pencil.layers().size());

  Array<GreasePencil *> elements(mask.size(), nullptr);
  const bke::AttributeAccessor src_attributes = grease_pencil.attributes();
  const Span<const Layer *> src_layers = grease_pencil.layers();

  mask.foreach_index(GrainSize(32), [&](const int layer_i, const int element_i) {
    GreasePencil *element = BKE_grease_pencil_new_nomain();
    element->material_array = static_cast<Material **>(
        MEM_dupallocN(grease_pencil.material_array));
    element->material_array_num = grease_pencil.material_array_num;

    const Layer &src_layer = *src_layers[layer_i];
    const Drawing *src_drawing = grease_pencil.get_eval_drawing(src_layer);

    if (src_drawing) {
      Layer &new_layer = element->add_layer(src_layer.name());
      Drawing &drawing = *element->insert_frame(new_layer, element->runtime->eval_frame);
      drawing.strokes_for_write() = src_drawing->strokes();

      bke::gather_attributes(src_attributes,
                             bke::AttrDomain::Layer,
                             bke::AttrDomain::Layer,
                             attribute_filter,
                             Span<int>{layer_i},
                             element->attributes_for_write());
    }

    elements[element_i] = element;
  });

  return elements;
}

Array<GreasePencil *> extract_greasepencil_layer_points(
    const GreasePencil &grease_pencil,
    int layer_i,
    const IndexMask &mask,
    const bke::AttributeFilter &attribute_filter)
{
  using namespace bke::greasepencil;
  const Layer &src_layer = *grease_pencil.layer(layer_i);
  const Drawing &src_drawing = *grease_pencil.get_eval_drawing(src_layer);
  const bke::CurvesGeometry &src_curves = src_drawing.strokes();
  const bke::AttributeAccessor src_layer_attributes = grease_pencil.attributes();
  const bke::AttributeAccessor src_curves_attributes = src_curves.attributes();

  Array<GreasePencil *> elements(mask.size(), nullptr);
  mask.foreach_index(GrainSize(32), [&](const int point_i, const int element_i) {
    GreasePencil *element = BKE_grease_pencil_new_nomain();
    element->material_array = static_cast<Material **>(
        MEM_dupallocN(grease_pencil.material_array));
    element->material_array_num = grease_pencil.material_array_num;

    Layer &new_layer = element->add_layer(src_layer.name());
    Drawing &drawing = *element->insert_frame(new_layer, element->runtime->eval_frame);
    bke::CurvesGeometry &new_curves = drawing.strokes_for_write();
    new_curves.resize(1, 1);
    new_curves.offsets_for_write().last() = 1;

    bke::gather_attributes(src_layer_attributes,
                           bke::AttrDomain::Layer,
                           bke::AttrDomain::Layer,
                           attribute_filter,
                           Span<int>{layer_i},
                           element->attributes_for_write());
    bke::gather_attributes(src_curves_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           Span<int>{point_i},
                           new_curves.attributes_for_write());

    new_curves.update_curve_types();

    elements[element_i] = element;
  });

  return elements;
}

Array<GreasePencil *> extract_greasepencil_layer_curves(
    const GreasePencil &grease_pencil,
    const int layer_i,
    const IndexMask &mask,
    const bke::AttributeFilter &attribute_filter)
{
  using namespace bke::greasepencil;
  const Layer &src_layer = *grease_pencil.layer(layer_i);
  const Drawing &src_drawing = *grease_pencil.get_eval_drawing(src_layer);
  const bke::CurvesGeometry &src_curves = src_drawing.strokes();
  const bke::AttributeAccessor src_layer_attributes = grease_pencil.attributes();
  const bke::AttributeAccessor src_curves_attributes = src_curves.attributes();
  const OffsetIndices<int> src_points_by_curve = src_curves.points_by_curve();

  Array<GreasePencil *> elements(mask.size(), nullptr);
  mask.foreach_index(GrainSize(32), [&](const int curve_i, const int element_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const int points_num = src_points.size();

    GreasePencil *element = BKE_grease_pencil_new_nomain();
    element->material_array = static_cast<Material **>(
        MEM_dupallocN(grease_pencil.material_array));
    element->material_array_num = grease_pencil.material_array_num;

    Layer &new_layer = element->add_layer(src_layer.name());
    Drawing &drawing = *element->insert_frame(new_layer, element->runtime->eval_frame);
    bke::CurvesGeometry &new_curves = drawing.strokes_for_write();

    new_curves.resize(points_num, 1);
    bke::gather_attributes(src_curves_attributes,
                           bke::AttrDomain::Point,
                           bke::AttrDomain::Point,
                           attribute_filter,
                           src_points,
                           new_curves.attributes_for_write());
    bke::gather_attributes(src_curves_attributes,
                           bke::AttrDomain::Curve,
                           bke::AttrDomain::Curve,
                           attribute_filter,
                           Span<int>{curve_i},
                           new_curves.attributes_for_write());
    bke::gather_attributes(src_layer_attributes,
                           bke::AttrDomain::Layer,
                           bke::AttrDomain::Layer,
                           attribute_filter,
                           Span<int>{layer_i},
                           element->attributes_for_write());

    new_curves.update_curve_types();
    elements[element_i] = element;
  });

  return elements;
}

}  // namespace blender::geometry
