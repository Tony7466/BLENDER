/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_anonymous_attribute_id.hh"
#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_multi_value_map.hh"

namespace blender::geometry {

const MultiValueMap<bke::GeometryComponent::Type, eAttrDomain> &reorder_supports()
{
  const static MultiValueMap<bke::GeometryComponent::Type, eAttrDomain>
      supported_types_and_domains = []() {
        MultiValueMap<bke::GeometryComponent::Type, eAttrDomain> supported_types_and_domains;
        supported_types_and_domains.add_multiple(
            bke::GeometryComponent::Type::Mesh,
            {ATTR_DOMAIN_POINT, ATTR_DOMAIN_EDGE, ATTR_DOMAIN_FACE});
        supported_types_and_domains.add(bke::GeometryComponent::Type::Curve, ATTR_DOMAIN_CURVE);
        supported_types_and_domains.add(bke::GeometryComponent::Type::PointCloud,
                                        ATTR_DOMAIN_POINT);
        supported_types_and_domains.add(bke::GeometryComponent::Type::Instance,
                                        ATTR_DOMAIN_INSTANCE);
        return supported_types_and_domains;
      }();
  return supported_types_and_domains;
}

template<typename T, typename Func>
static void parallel_transform(MutableSpan<T> values, const int64_t grain_size, const Func &func)
{
  threading::parallel_for(values.index_range(), grain_size, [&](const IndexRange range) {
    MutableSpan<T> values_range = values.slice(range);
    std::transform(values_range.begin(), values_range.end(), values_range.begin(), func);
  });
}

static void reorder_attributes_group_to_group(const bke::AttributeAccessor src_attributes,
                                              const eAttrDomain domain,
                                              const OffsetIndices<int> src_offsets,
                                              const OffsetIndices<int> dst_offsets,
                                              const Span<int> old_by_new_map,
                                              bke::MutableAttributeAccessor dst_attributes)
{
  src_attributes.for_all(
      [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
        if (meta_data.domain != domain) {
          return true;
        }
        const GVArray src = *src_attributes.lookup(id, domain);
        bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
            id, domain, meta_data.data_type);
        if (!dst) {
          return true;
        }

        threading::parallel_for(old_by_new_map.index_range(), 1024, [&](const IndexRange range) {
          for (const int new_i : range) {
            const int old_i = old_by_new_map[new_i];
            array_utils::copy(src.slice(src_offsets[old_i]), dst.span.slice(dst_offsets[new_i]));
          }
        });

        dst.finish();
        return true;
      });
}

static void reorder_offset_indices(const Span<int> old_offsets,
                                   const Span<int> old_by_new_map,
                                   MutableSpan<int> new_offsets)
{
  Array<int> old_counts(old_offsets.size());
  array_utils::copy(old_offsets, old_counts.as_mutable_span());
  offset_indices::reduce_offsets_to_counts(old_counts.as_mutable_span());
  array_utils::gather(old_counts.as_span().drop_back(1), old_by_new_map, new_offsets.drop_back(1));
  offset_indices::accumulate_counts_to_offsets(new_offsets);
}

static void reorder_attributes(const bke::AttributeAccessor src_attributes,
                               const eAttrDomain domain,
                               const Span<int> old_by_new_map,
                               bke::MutableAttributeAccessor dst_attributes)
{
  src_attributes.for_all(
      [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
        if (meta_data.domain != domain) {
          return true;
        }
        const GVArray src = *src_attributes.lookup(id, domain);
        bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
            id, domain, meta_data.data_type);
        if (!dst) {
          return true;
        }

        bke::attribute_math::convert_to_static_type(meta_data.data_type, [&](auto dummy) {
          using T = decltype(dummy);
          array_utils::gather(src.typed<T>(), old_by_new_map, dst.span.typed<T>());
        });

        dst.finish();
        return true;
      });
}

static Array<int> invert_permutation(const Span<int> permutation)
{
  Array<int> data(permutation.size());
  threading::parallel_for(permutation.index_range(), 2048, [&](const IndexRange range) {
    for (const int64_t i : range) {
      data[permutation[i]] = i;
    }
  });
  return data;
}

static void reorder_customdata_groups(CustomData &data,
                                      const OffsetIndices<int> old_offsets,
                                      const OffsetIndices<int> new_offsets,
                                      const Span<int> old_by_new_map)
{
  const int elements_num = new_offsets.total_size();
  const int groups_num = old_by_new_map.size();
  CustomData new_data;
  CustomData_copy_layout(&data, &new_data, CD_MASK_ALL, CD_CONSTRUCT, elements_num);
  for (const int new_i : IndexRange(groups_num)) {
    const int old_i = old_by_new_map[new_i];
    const IndexRange old_range = old_offsets[old_i];
    const IndexRange new_range = new_offsets[new_i];
    BLI_assert(old_range.size() == new_range.size());
    CustomData_copy_data(&data, &new_data, old_range.start(), new_range.start(), old_range.size());
  }
  CustomData_free(&data, elements_num);
  data = new_data;
}

static void reorder_mesh_verts(const Mesh &src_mesh,
                               const Span<int> old_by_new_map,
                               Mesh &dst_mesh)
{
  reorder_attributes(
      src_mesh.attributes(), ATTR_DOMAIN_POINT, old_by_new_map, dst_mesh.attributes_for_write());

  const Array<int> new_by_old_map = invert_permutation(old_by_new_map);
  const auto old_index_to_new = [&](const int old_i) { return new_by_old_map[old_i]; };
  parallel_transform(dst_mesh.edges_for_write().cast<int>(), 4098, old_index_to_new);
  parallel_transform(dst_mesh.corner_verts_for_write(), 4098, old_index_to_new);
}

static void reorder_mesh_edges(const Mesh &src_mesh,
                               const Span<int> old_by_new_map,
                               Mesh &dst_mesh)
{
  reorder_attributes(
      src_mesh.attributes(), ATTR_DOMAIN_EDGE, old_by_new_map, dst_mesh.attributes_for_write());

  const Array<int> new_by_old_map = invert_permutation(old_by_new_map);
  parallel_transform(dst_mesh.corner_edges_for_write(), 4098, [&](const int old_i) {
    return new_by_old_map[old_i];
  });
}

static void reorder_mesh_faces(const Mesh &src_mesh,
                               const Span<int> old_by_new_map,
                               Mesh &dst_mesh)
{
  reorder_attributes(
      src_mesh.attributes(), ATTR_DOMAIN_FACE, old_by_new_map, dst_mesh.attributes_for_write());

  const Span<int> old_offsets = src_mesh.face_offsets();
  MutableSpan<int> new_offsets = dst_mesh.face_offsets_for_write();
  reorder_offset_indices(old_offsets, old_by_new_map, new_offsets);

  reorder_attributes_group_to_group(src_mesh.attributes(),
                                    ATTR_DOMAIN_CORNER,
                                    old_offsets,
                                    new_offsets.as_span(),
                                    old_by_new_map,
                                    dst_mesh.attributes_for_write());
}

static void reorder_mesh(const bke::GeometryComponent &src_component,
                         const Span<int> old_by_new_map,
                         const eAttrDomain domain,
                         bke::GeometryComponent &dst_component)
{
  const bke::MeshComponent *src_mesh_component = dynamic_cast<const bke::MeshComponent *>(
      &src_component);
  bke::MeshComponent *dst_mesh_component = dynamic_cast<bke::MeshComponent *>(&dst_component);
  if (ELEM(nullptr, src_mesh_component, dst_mesh_component)) {
    return;
  }
  const Mesh &src_mesh = *src_mesh_component->get();
  Mesh &dst_mesh = *dst_mesh_component->get_for_write();
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      reorder_mesh_verts(src_mesh, old_by_new_map, dst_mesh);
      break;
    case ATTR_DOMAIN_EDGE:
      reorder_mesh_edges(src_mesh, old_by_new_map, dst_mesh);
      break;
    case ATTR_DOMAIN_FACE:
      reorder_mesh_faces(src_mesh, old_by_new_map, dst_mesh);
      break;
    default:
      break;
  }
  BKE_mesh_tag_topology_changed(&dst_mesh);
}

static void reorder_points(const bke::GeometryComponent &src_component,
                           const Span<int> old_by_new_map,
                           const eAttrDomain domain,
                           bke::GeometryComponent &dst_component)
{
  const bke::PointCloudComponent *src_points_component =
      dynamic_cast<const bke::PointCloudComponent *>(&src_component);
  bke::PointCloudComponent *dst_points_component = dynamic_cast<bke::PointCloudComponent *>(
      &dst_component);
  if (ELEM(nullptr, src_points_component, dst_points_component)) {
    return;
  }
  if (domain != ATTR_DOMAIN_POINT) {
    return;
  }

  const PointCloud &src_pointcloud = *src_points_component->get();
  PointCloud &dst_pointcloud = *dst_points_component->get_for_write();

  reorder_attributes(
      src_pointcloud.attributes(), domain, old_by_new_map, dst_pointcloud.attributes_for_write());

  dst_pointcloud.tag_positions_changed();
  dst_pointcloud.tag_radii_changed();
}

static void reorder_curves(const bke::GeometryComponent &src_component,
                           const Span<int> old_by_new_map,
                           const eAttrDomain domain,
                           bke::GeometryComponent &dst_component)
{
  const bke::CurveComponent *src_curves_component = dynamic_cast<const bke::CurveComponent *>(
      &src_component);
  bke::CurveComponent *dst_curves_component = dynamic_cast<bke::CurveComponent *>(&dst_component);
  if (ELEM(nullptr, src_curves_component, dst_curves_component)) {
    return;
  }
  if (domain != ATTR_DOMAIN_CURVE) {
    return;
  }

  const bke::CurvesGeometry &src_curves = src_curves_component->get()->geometry.wrap();
  bke::CurvesGeometry &dst_curves = dst_curves_component->get_for_write()->geometry.wrap();

  reorder_attributes(
      src_curves.attributes(), domain, old_by_new_map, dst_curves.attributes_for_write());

  const Span<int> old_offsets = src_curves.offsets();
  MutableSpan<int> new_offsets = dst_curves.offsets_for_write();
  reorder_offset_indices(old_offsets, old_by_new_map, new_offsets);

  reorder_attributes_group_to_group(src_curves.attributes(),
                                    domain,
                                    old_offsets,
                                    new_offsets.as_span(),
                                    old_by_new_map,
                                    dst_curves.attributes_for_write());
  dst_curves.tag_topology_changed();
}

static void reorder_instaces(const bke::GeometryComponent &src_component,
                             const Span<int> old_by_new_map,
                             const eAttrDomain domain,
                             bke::GeometryComponent &dst_component)
{
  const bke::InstancesComponent *src_instances_component =
      dynamic_cast<const bke::InstancesComponent *>(&src_component);
  bke::InstancesComponent *dst_instances_component = dynamic_cast<bke::InstancesComponent *>(
      &dst_component);
  if (ELEM(nullptr, src_instances_component, dst_instances_component)) {
    return;
  }
  if (domain != ATTR_DOMAIN_CURVE) {
    return;
  }

  const bke::Instances &src_instances = *src_instances_component->get();
  bke::Instances &dst_instances = *dst_instances_component->get_for_write();

  reorder_attributes(
      src_instances.attributes(), domain, old_by_new_map, dst_instances.attributes_for_write());

  const Span<int> old_reference_handles = src_instances.reference_handles();
  MutableSpan<int> new_reference_handles = dst_instances.reference_handles();
  array_utils::gather(old_reference_handles, old_by_new_map, new_reference_handles);

  const Span<float4x4> old_transforms = src_instances.transforms();
  MutableSpan<float4x4> new_transforms = dst_instances.transforms();
  array_utils::gather(old_transforms, old_by_new_map, new_transforms);
}

bke::GeometryComponent &reordered_component_copy(const bke::GeometryComponent &src_component,
                                                 const Span<int> old_by_new_map,
                                                 const eAttrDomain domain)
{
  bke::GeometryComponent &dst_component = *src_component.copy();

  reorder_mesh(src_component, old_by_new_map, domain, dst_component);
  reorder_points(src_component, old_by_new_map, domain, dst_component);
  reorder_curves(src_component, old_by_new_map, domain, dst_component);
  reorder_instaces(src_component, old_by_new_map, domain, dst_component);

  return dst_component;
}

}  // namespace blender::geometry
