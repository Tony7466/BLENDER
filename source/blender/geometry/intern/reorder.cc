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

  offset_indices::gather_group_sizes(old_offsets, old_by_new_map, new_offsets);
  offset_indices::accumulate_counts_to_offsets(new_offsets);
}

static void reorder_attributes(const bke::AttributeAccessor src_attributes,
                               const eAttrDomain domain,
                               const Span<int> old_by_new_map,
                               bke::MutableAttributeAccessor dst_attributes)
{
  bke::gather_attributes(src_attributes, domain, {}, {}, old_by_new_map, dst_attributes);
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

static void reorder_mesh_verts(const Mesh &src_mesh,
                               const Span<int> old_by_new_map,
                               Mesh &dst_mesh)
{
  reorder_attributes(
      src_mesh.attributes(), ATTR_DOMAIN_POINT, old_by_new_map, dst_mesh.attributes_for_write());

  const Array<int> new_by_old_map = invert_permutation(old_by_new_map);

  array_utils::gather(new_by_old_map.as_span(),
                      dst_mesh.edges().cast<int>(),
                      dst_mesh.edges_for_write().cast<int>());
  array_utils::gather(
      new_by_old_map.as_span(), dst_mesh.corner_verts(), dst_mesh.corner_verts_for_write());
}

static void reorder_mesh_edges(const Mesh &src_mesh,
                               const Span<int> old_by_new_map,
                               Mesh &dst_mesh)
{
  reorder_attributes(
      src_mesh.attributes(), ATTR_DOMAIN_EDGE, old_by_new_map, dst_mesh.attributes_for_write());

  const Array<int> new_by_old_map = invert_permutation(old_by_new_map);

  array_utils::gather(
      new_by_old_map.as_span(), dst_mesh.corner_edges(), dst_mesh.corner_edges_for_write());
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

static void reorder_mesh(const Mesh &src_mesh,
                         const Span<int> old_by_new_map,
                         const eAttrDomain domain,
                         Mesh &dst_mesh)
{
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

static void reorder_points(const PointCloud &src_pointcloud,
                           const Span<int> old_by_new_map,
                           PointCloud &dst_pointcloud)
{
  reorder_attributes(src_pointcloud.attributes(),
                     ATTR_DOMAIN_POINT,
                     old_by_new_map,
                     dst_pointcloud.attributes_for_write());

  dst_pointcloud.tag_positions_changed();
  dst_pointcloud.tag_radii_changed();
}

static void reorder_curves(const bke::CurvesGeometry &src_curves,
                           const Span<int> old_by_new_map,
                           bke::CurvesGeometry &dst_curves)
{
  reorder_attributes(src_curves.attributes(),
                     ATTR_DOMAIN_CURVE,
                     old_by_new_map,
                     dst_curves.attributes_for_write());

  const Span<int> old_offsets = src_curves.offsets();
  MutableSpan<int> new_offsets = dst_curves.offsets_for_write();
  reorder_offset_indices(old_offsets, old_by_new_map, new_offsets);

  reorder_attributes_group_to_group(src_curves.attributes(),
                                    ATTR_DOMAIN_POINT,
                                    old_offsets,
                                    new_offsets.as_span(),
                                    old_by_new_map,
                                    dst_curves.attributes_for_write());
  dst_curves.tag_topology_changed();
}

static void reorder_instaces(const bke::Instances &src_instances,
                             const Span<int> old_by_new_map,
                             bke::Instances &dst_instances)
{
  reorder_attributes(src_instances.attributes(),
                     ATTR_DOMAIN_INSTANCE,
                     old_by_new_map,
                     dst_instances.attributes_for_write());

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
  BLI_assert(!src_component.is_empty());
  bke::GeometryComponent &dst_component = *src_component.copy();

  if (const bke::MeshComponent *src_mesh_component = dynamic_cast<const bke::MeshComponent *>(
          &src_component))
  {
    bke::MeshComponent &dst_mesh_component = static_cast<bke::MeshComponent &>(dst_component);
    reorder_mesh(
        *src_mesh_component->get(), old_by_new_map, domain, *dst_mesh_component.get_for_write());
  }
  else if (const bke::PointCloudComponent *src_points_component =
               dynamic_cast<const bke::PointCloudComponent *>(&src_component))
  {
    bke::PointCloudComponent &dst_points_component = static_cast<bke::PointCloudComponent &>(
        dst_component);
    BLI_assert(domain == ATTR_DOMAIN_POINT);
    reorder_points(
        *src_points_component->get(), old_by_new_map, *dst_points_component.get_for_write());
  }
  else if (const bke::CurveComponent *src_curves_component =
               dynamic_cast<const bke::CurveComponent *>(&src_component))
  {
    bke::CurveComponent &dst_curves_component = static_cast<bke::CurveComponent &>(dst_component);
    BLI_assert(domain == ATTR_DOMAIN_CURVE);
    reorder_curves(src_curves_component->get()->geometry.wrap(),
                   old_by_new_map,
                   dst_curves_component.get_for_write()->geometry.wrap());
  }
  else if (const bke::InstancesComponent *src_instances_component =
               dynamic_cast<const bke::InstancesComponent *>(&src_component))
  {
    bke::InstancesComponent &dst_instances_component = static_cast<bke::InstancesComponent &>(
        dst_component);
    BLI_assert(domain == ATTR_DOMAIN_INSTANCE);
    reorder_instaces(
        *src_instances_component->get(), old_by_new_map, *dst_instances_component.get_for_write());
  }

  return dst_component;
}

}  // namespace blender::geometry
