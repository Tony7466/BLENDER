/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

GizmosComponent::GizmosComponent() : GeometryComponent(GEO_COMPONENT_TYPE_GIZMO) {}

GizmosComponent::~GizmosComponent()
{
  this->clear();
}

GeometryComponent *GizmosComponent::copy() const
{
  GizmosComponent *new_component = new GizmosComponent();
  if (gizmos_ != nullptr) {
    new_component->gizmos_ = gizmos_->copy();
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return new_component;
}

void GizmosComponent::clear()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (gizmos_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      delete gizmos_;
    }
    gizmos_ = nullptr;
  }
}

bool GizmosComponent::has_gizmos() const
{
  return gizmos_ != nullptr;
}

const PointCloud *GizmosComponent::get_for_read() const
{
  return pointcloud_;
}

PointCloud *GizmosComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    gizmos_ = gizmos_->copy();
    ownership_ = GeometryOwnershipType::Owned;
  }
  return gizmos_;
}

bool GizmosComponent::is_empty() const
{
  return gizmos_ == nullptr;
}

bool GizmosComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void GizmosComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    gizmos_ = gizmos_->copy();
    ownership_ = GeometryOwnershipType::Owned;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Access
 * \{ */

namespace blender::bke {

static void tag_component_positions_changed(void *owner)
{
  PointCloud &points = *static_cast<PointCloud *>(owner);
  points.tag_positions_changed();
}

static void tag_component_radius_changed(void *owner)
{
  PointCloud &points = *static_cast<PointCloud *>(owner);
  points.tag_radii_changed();
}

/**
 * In this function all the attribute providers for a point cloud component are created. Most data
 * in this function is statically allocated, because it does not change over time.
 */
static ComponentAttributeProviders create_attribute_providers_for_point_cloud()
{
  static CustomDataAccessInfo point_access = {
      [](void *owner) -> CustomData * {
        PointCloud *pointcloud = static_cast<PointCloud *>(owner);
        return &pointcloud->pdata;
      },
      [](const void *owner) -> const CustomData * {
        const PointCloud *pointcloud = static_cast<const PointCloud *>(owner);
        return &pointcloud->pdata;
      },
      [](const void *owner) -> int {
        const PointCloud *pointcloud = static_cast<const PointCloud *>(owner);
        return pointcloud->totpoint;
      }};

  static BuiltinCustomDataLayerProvider position("position",
                                                 ATTR_DOMAIN_POINT,
                                                 CD_PROP_FLOAT3,
                                                 CD_PROP_FLOAT3,
                                                 BuiltinAttributeProvider::Creatable,
                                                 BuiltinAttributeProvider::NonDeletable,
                                                 point_access,
                                                 tag_component_positions_changed);
  static BuiltinCustomDataLayerProvider radius("radius",
                                               ATTR_DOMAIN_POINT,
                                               CD_PROP_FLOAT,
                                               CD_PROP_FLOAT,
                                               BuiltinAttributeProvider::Creatable,
                                               BuiltinAttributeProvider::Deletable,
                                               point_access,
                                               tag_component_radius_changed);
  static BuiltinCustomDataLayerProvider id("id",
                                           ATTR_DOMAIN_POINT,
                                           CD_PROP_INT32,
                                           CD_PROP_INT32,
                                           BuiltinAttributeProvider::Creatable,
                                           BuiltinAttributeProvider::Deletable,
                                           point_access,
                                           nullptr);
  static CustomDataAttributeProvider point_custom_data(ATTR_DOMAIN_POINT, point_access);
  return ComponentAttributeProviders({&position, &radius, &id}, {&point_custom_data});
}

static AttributeAccessorFunctions get_pointcloud_accessor_functions()
{
  static const ComponentAttributeProviders providers =
      create_attribute_providers_for_point_cloud();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PointCloud &pointcloud = *static_cast<const PointCloud *>(owner);
    switch (domain) {
      case ATTR_DOMAIN_POINT:
        return pointcloud.totpoint;
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return domain == ATTR_DOMAIN_POINT;
  };
  fn.adapt_domain = [](const void * /*owner*/,
                       const blender::GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) {
    if (from_domain == to_domain && from_domain == ATTR_DOMAIN_POINT) {
      return varray;
    }
    return blender::GVArray{};
  };
  return fn;
}

static const AttributeAccessorFunctions &get_pointcloud_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_pointcloud_accessor_functions();
  return fn;
}

}  // namespace blender::bke

blender::bke::AttributeAccessor PointCloud::attributes() const
{
  return blender::bke::AttributeAccessor(this,
                                         blender::bke::get_pointcloud_accessor_functions_ref());
}

blender::bke::MutableAttributeAccessor PointCloud::attributes_for_write()
{
  return blender::bke::MutableAttributeAccessor(
      this, blender::bke::get_pointcloud_accessor_functions_ref());
}

std::optional<blender::bke::AttributeAccessor> PointCloudComponent::attributes() const
{
  return blender::bke::AttributeAccessor(pointcloud_,
                                         blender::bke::get_pointcloud_accessor_functions_ref());
}

std::optional<blender::bke::MutableAttributeAccessor> PointCloudComponent::attributes_for_write()
{
  PointCloud *pointcloud = this->get_for_write();
  return blender::bke::MutableAttributeAccessor(
      pointcloud, blender::bke::get_pointcloud_accessor_functions_ref());
}

/** \} */
