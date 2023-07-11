/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#if 0

#  include "DNA_volume_types.h"

#  include "BKE_volume_geometry.hh"

#  include "attribute_access_volume.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name VolumeGeometry
 * \{ */

VolumeGeometry::VolumeGeometry() {}

/**
 * \note Expects `dst` to be initialized, since the original attributes must be freed.
 */
static void copy_volume_geometry(VolumeGeometry &dst, const VolumeGeometry &src)
{
  dst.grid = static_cast<VolumeGeometryGrid *>(MEM_dupallocN(src.grid));
}

VolumeGeometry::VolumeGeometry(const VolumeGeometry &other) : VolumeGeometry()
{
  copy_volume_geometry(*this, other);
}

VolumeGeometry &VolumeGeometry::operator=(const VolumeGeometry &other)
{
  if (this != &other) {
    copy_volume_geometry(*this, other);
  }
  return *this;
}

/* The source should be empty, but in a valid state so that using it further will work. */
static void move_volume_geometry(VolumeGeometry &dst, VolumeGeometry &src)
{
  std::swap(dst.grid, src.grid);
  src.grid = nullptr;
}

VolumeGeometry::VolumeGeometry(VolumeGeometry &&other) : VolumeGeometry()
{
  move_volume_geometry(*this, other);
}

VolumeGeometry &VolumeGeometry::operator=(VolumeGeometry &&other)
{
  if (this != &other) {
    move_volume_geometry(*this, other);
  }
  return *this;
}

VolumeGeometry::~VolumeGeometry()
{
  if (grid) {
    MEM_delete(grid);
  }
}

int VolumeGeometry::domain_size(eAttrDomain domain) const
{
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      return grid ? int(grid->active_voxel_num()) : 0;
    default:
      return 0;
  }
}

#  ifdef WITH_OPENVDB
void VolumeGeometry::set_grid(const openvdb::GridBase::Ptr &grid_ptr)
{
  if (grid == nullptr) {
    grid = MEM_new<VolumeGeometryGrid>(__func__);
  }
  grid->grid_ = grid_ptr;
}
#  endif

/**
 * In this function all the attribute providers for a volume component are created.
 * Most data in this function is statically allocated, because it does not change over time.
 */
static ComponentAttributeProviders create_attribute_providers_for_volume()
{
  static VolumeGridAccessInfo grid_access = {
      [](void *owner) -> VolumeGeometryGrid & {
        return *static_cast<VolumeGeometryGrid *>(owner);
      },
      [](const void *owner) -> const VolumeGeometryGrid & {
        return *static_cast<const VolumeGeometryGrid *>(owner);
      },
  };

  static auto update_on_change = [](void * /*owner*/) {};

  // static BuiltinVolumeAttributeProvider position("position",
  //                                                ATTR_DOMAIN_POINT,
  //                                                CD_PROP_FLOAT3,
  //                                                BuiltinAttributeProvider::NonCreatable,
  //                                                BuiltinAttributeProvider::NonDeletable,
  //                                                grid_access,
  //                                                update_on_change);

  static VolumeGridValueAttributeProvider value(
      "value", ATTR_DOMAIN_POINT, grid_access, update_on_change);

  // static VolumeAttributeProvider voxel_custom_data(ATTR_DOMAIN_POINT, grid_access);

  // return ComponentAttributeProviders({&position}, {&voxel_custom_data});
  return ComponentAttributeProviders({&value}, {});
}

static AttributeAccessorFunctions get_volume_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_volume();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  /* Set domain callbacks that are not defined yet. */
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const VolumeGeometry &geometry = *static_cast<const VolumeGeometry *>(owner);
    return geometry.domain_size(domain);
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return ELEM(domain, ATTR_DOMAIN_POINT);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const VolumeGeometry &geometry = *static_cast<const VolumeGeometry *>(owner);
    return geometry.adapt_domain(varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_volume_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_volume_accessor_functions();
  return fn;
}

AttributeAccessor VolumeGeometry::attributes() const
{
  return AttributeAccessor(grid, get_volume_accessor_functions_ref());
}

MutableAttributeAccessor VolumeGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(grid, get_volume_accessor_functions_ref());
}

GVArray VolumeGeometry::adapt_domain(const GVArray &varray, eAttrDomain from, eAttrDomain to) const
{
  if (from == to) {
    return varray;
  }
  return {};
}

void VolumeGeometry::blend_read_data(BlendDataReader & /*reader*/)
{
  grid = nullptr;
}

void VolumeGeometry::blend_write(BlendWriter & /*writer*/, ID & /*id*/) {}

/** \} */

}  // namespace blender::bke

#endif
