/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_volume.h"

#include "attribute_access_intern.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace {

template<typename OpType>
static auto grid_type_operation(const VolumeGridType grid_type, OpType &&op)
{
  switch (grid_type) {
    case VOLUME_GRID_FLOAT:
      return op.template operator()<openvdb::FloatGrid>();
    case VOLUME_GRID_VECTOR_FLOAT:
      return op.template operator()<openvdb::Vec3fGrid>();
    case VOLUME_GRID_BOOLEAN:
      return op.template operator()<openvdb::BoolGrid>();
    case VOLUME_GRID_DOUBLE:
      return op.template operator()<openvdb::DoubleGrid>();
    case VOLUME_GRID_INT:
      return op.template operator()<openvdb::Int32Grid>();
    case VOLUME_GRID_INT64:
      return op.template operator()<openvdb::Int64Grid>();
    case VOLUME_GRID_VECTOR_INT:
      return op.template operator()<openvdb::Vec3IGrid>();
    case VOLUME_GRID_VECTOR_DOUBLE:
      return op.template operator()<openvdb::Vec3dGrid>();
    case VOLUME_GRID_MASK:
      return op.template operator()<openvdb::MaskGrid>();
    case VOLUME_GRID_POINTS:
      return op.template operator()<openvdb::points::PointDataGrid>();
    case VOLUME_GRID_UNKNOWN:
      break;
  }

  /* Should never be called. */
  BLI_assert_msg(0, "should never be reached");
  return op.template operator()<openvdb::FloatGrid>();
}

}  // namespace

class VolumeGeometryGrid {
 public:
  VolumeGeometryGrid();
  ~VolumeGeometryGrid();

  VolumeGridType type() const;

  int64_t active_voxel_num() const;

  template<typename OpType> auto grid_type_operation(OpType &&op)
  {
    return ::grid_type_operation(type(), op);
  }

#ifdef WITH_OPENVDB
  openvdb::GridBase::Ptr grid_;
#endif
};

namespace blender::bke {

/**
 * Utility to group together multiple functions that are used to access custom data on geometry
 * components in a generic way.
 */
struct VolumeGridAccessInfo {
  using GridGetter = VolumeGeometryGrid &(*)(void *owner);
  using ConstGridGetter = const VolumeGeometryGrid &(*)(const void *owner);

  GridGetter get_grid;
  ConstGridGetter get_const_grid;
};

class BuiltinVolumeAttributeProvider final : public BuiltinAttributeProvider {
  using UpdateOnChange = void (*)(void *owner);
  const VolumeGridAccessInfo grid_access_;
  const UpdateOnChange update_on_change_;

 public:
  BuiltinVolumeAttributeProvider(std::string attribute_name,
                                 const eAttrDomain domain,
                                 const eCustomDataType attribute_type,
                                 const CreatableEnum creatable,
                                 const DeletableEnum deletable,
                                 const VolumeGridAccessInfo grid_access,
                                 const UpdateOnChange update_on_write,
                                 const AttributeValidator validator = {})
      : BuiltinAttributeProvider(
            std::move(attribute_name), domain, attribute_type, creatable, deletable, validator),
        grid_access_(grid_access),
        update_on_change_(update_on_write)
  {
  }

  GAttributeReader try_get_for_read(const void *owner) const final;
  GAttributeWriter try_get_for_write(void *owner) const final;
  bool try_delete(void *owner) const final;
  bool try_create(void *owner, const AttributeInit &initializer) const final;
  bool exists(const void *owner) const final;
};

/**
 * An attribute provider for custom volume grids.
 */
class VolumeAttributeProvider final : public DynamicAttributesProvider {
 private:
  static constexpr uint64_t supported_types_mask = CD_MASK_PROP_ALL;
  const eAttrDomain domain_;
  const VolumeGridAccessInfo grid_access_;

 public:
  VolumeAttributeProvider(const eAttrDomain domain, const VolumeGridAccessInfo grid_access)
      : domain_(domain), grid_access_(grid_access)
  {
  }

  GAttributeReader try_get_for_read(const void *owner,
                                    const AttributeIDRef &attribute_id) const final;

  GAttributeWriter try_get_for_write(void *owner, const AttributeIDRef &attribute_id) const final;

  bool try_delete(void *owner, const AttributeIDRef &attribute_id) const final;

  bool try_create(void *owner,
                  const AttributeIDRef &attribute_id,
                  eAttrDomain domain,
                  const eCustomDataType data_type,
                  const AttributeInit &initializer) const final;

  bool foreach_attribute(const void *owner, const AttributeForeachCallback callback) const final;

  void foreach_domain(const FunctionRef<void(eAttrDomain)> callback) const final
  {
    callback(domain_);
  }

 private:
  bool type_is_supported(eCustomDataType data_type) const
  {
    return ((1ULL << data_type) & supported_types_mask) != 0;
  }
};

}  // namespace blender::bke
