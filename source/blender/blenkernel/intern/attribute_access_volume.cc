/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_attribute.hh"
#include "BKE_volume_attribute.hh"
#include "BKE_volume_openvdb.hh"

#include "attribute_access_volume.hh"

#include "intern/volume_grids.hh"

#if 0
/* -------------------------------------------------------------------- */
/** \name Volume Geometry Grid
 * \{ */

VolumeGeometryGrid::VolumeGeometryGrid() : grid_(nullptr) {}
VolumeGeometryGrid::~VolumeGeometryGrid() {}

VolumeGridType VolumeGeometryGrid::type() const
{
  if (grid_ == nullptr) {
    return VOLUME_GRID_UNKNOWN;
  }

  if (grid_->isType<openvdb::FloatGrid>()) {
    return VOLUME_GRID_FLOAT;
  }
  if (grid_->isType<openvdb::Vec3fGrid>()) {
    return VOLUME_GRID_VECTOR_FLOAT;
  }
  if (grid_->isType<openvdb::BoolGrid>()) {
    return VOLUME_GRID_BOOLEAN;
  }
  if (grid_->isType<openvdb::DoubleGrid>()) {
    return VOLUME_GRID_DOUBLE;
  }
  if (grid_->isType<openvdb::Int32Grid>()) {
    return VOLUME_GRID_INT;
  }
  if (grid_->isType<openvdb::Int64Grid>()) {
    return VOLUME_GRID_INT64;
  }
  if (grid_->isType<openvdb::Vec3IGrid>()) {
    return VOLUME_GRID_VECTOR_INT;
  }
  if (grid_->isType<openvdb::Vec3dGrid>()) {
    return VOLUME_GRID_VECTOR_DOUBLE;
  }
  if (grid_->isType<openvdb::MaskGrid>()) {
    return VOLUME_GRID_MASK;
  }
  if (grid_->isType<openvdb::points::PointDataGrid>()) {
    return VOLUME_GRID_POINTS;
  }
  return VOLUME_GRID_UNKNOWN;
}

int64_t VolumeGeometryGrid::active_voxel_num() const
{
#  ifdef WITH_OPENVDB
  return grid_->activeVoxelCount();
#  else
  return 0;
#  endif
}

/** \} */
#endif

#if 0

#  ifdef WITH_OPENVDB

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Attribute Provider Declaration
 * \{ */

// GAttributeReader VolumeGridValueAttributeProvider::try_get_for_read(const void *owner) const
//{
//   // const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
//   const VolumeGridVector &grids = grid_access_.get_const_grids(owner);
//   GVArray varray = get_volume_varray<VArrayImpl_For_VolumeGridValue>(grids);
//   return {varray, domain_, nullptr};
// }
//
// GAttributeWriter VolumeGridValueAttributeProvider::try_get_for_write(void *owner) const
//{
//   // VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
//   VolumeGridVector &grids = grid_access_.get_grids(owner);
//   GVMutableArray varray = get_volume_vmutablearray<VArrayImpl_For_VolumeGridValue>(grids);
//   return {varray, domain_, nullptr};
// }
//
// bool VolumeGridValueAttributeProvider::try_delete(void * /*owner*/) const
//{
//   if (deletable_ != Deletable) {
//     return false;
//   }
//   /* Not supported. */
//   //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
//   //  if (grid.remove_attribute(???)) {
//   //    if (update_on_change_ != nullptr) {
//   //      update_on_change_(owner);
//   //    }
//   //  }
//   //  return true;
//   return false;
// }
//
// bool VolumeGridValueAttributeProvider::try_create(void * /*owner*/,
//                                                   const AttributeInit & /*initializer*/) const
//{
//   if (createable_ != Creatable) {
//     return false;
//   }
//   /* Not supported. */
//   //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
//   //  if (grid.add_attribute(???)) {
//   //    if (update_on_change_ != nullptr) {
//   //      update_on_change_(owner);
//   //    }
//   //    return true;
//   //  }
//   return false;
// }
//
// bool VolumeGridValueAttributeProvider::exists(const void * /*owner*/) const
//{
//   return true;
// }

GAttributeReader VolumeGridPositionAttributeProvider::try_get_for_read(const void *owner) const
{
  const VolumeGridVector &grids = grid_access_.get_const_grids(owner);
  GVArray varray = get_volume_varray<VArrayImpl_For_VolumeGridPosition>(
      grids, AttributeIDRef{"position"}, true);
  return {varray, domain_, nullptr};
}

GAttributeWriter VolumeGridPositionAttributeProvider::try_get_for_write(void * /*owner*/) const
{
  return {};
}

bool VolumeGridPositionAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool VolumeGridPositionAttributeProvider::try_create(void * /*owner*/,
                                                     const AttributeInit & /*initializer*/) const
{
  return false;
}

bool VolumeGridPositionAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

GAttributeReader VolumeCustomAttributeProvider::try_get_for_read(
    const void *owner, const AttributeIDRef &attribute_id) const
{
  const VolumeGridVector &grids = grid_access_.get_const_grids(owner);
  GVArray varray = get_volume_varray<VArrayImpl_For_VolumeGridValue>(grids, attribute_id, false);
  return {varray, domain_, nullptr};
}

GAttributeWriter VolumeCustomAttributeProvider::try_get_for_write(
    void *owner, const AttributeIDRef &attribute_id) const
{
  VolumeGridVector &grids = grid_access_.get_grids(owner);
  GVMutableArray varray = get_volume_vmutablearray<VArrayImpl_For_VolumeGridValue>(grids,
                                                                                   attribute_id);
  return {varray, domain_, nullptr};
}

bool VolumeCustomAttributeProvider::try_delete(void *owner,
                                               const AttributeIDRef &attribute_id) const
{
  if (!attribute_id) {
    return false;
  }
  VolumeGridVector &grids = grid_access_.get_grids(owner);
  bool result = false;
  grids.remove_if([attribute_id, &result](VolumeGrid &grid) {
    result = true;
    return grid.name() == attribute_id.name();
  });
  if (result && update_on_change_ != nullptr) {
    update_on_change_(owner);
  }
  return false;
}

static VolumeGridType grid_type_for_custom_data_type(const eCustomDataType data_type)
{
  switch (data_type) {
    case CD_PROP_FLOAT:
      return VOLUME_GRID_FLOAT;
    // case CD_PROP_FLOAT2:        return
    case CD_PROP_FLOAT3:
      return VOLUME_GRID_VECTOR_FLOAT;
    case CD_PROP_INT32:
      return VOLUME_GRID_INT;
    // case CD_PROP_INT32_2D:        return
    // case CD_PROP_COLOR:        return
    case CD_PROP_BOOL:
      return VOLUME_GRID_BOOLEAN;
    // case CD_PROP_INT8:        return
    // case CD_PROP_BYTE_COLOR:        return
    // case CD_PROP_QUATERNION:        return
    // case CD_PROP_STRING:        return
    default:
      return VOLUME_GRID_UNKNOWN;
  }
}

static eCustomDataType custom_data_type_for_grid_type(const VolumeGridType grid_type)
{
  switch (grid_type) {
    case VOLUME_GRID_FLOAT:
      return CD_PROP_FLOAT;
    case VOLUME_GRID_VECTOR_FLOAT:
      return CD_PROP_FLOAT3;
    case VOLUME_GRID_INT:
      return CD_PROP_INT32;
    case VOLUME_GRID_BOOLEAN:
      return CD_PROP_BOOL;
    default:
      return CD_NUMTYPES;
  }
}

static openvdb::GridBase::Ptr add_generic_grid(VolumeGridVector &grids,
                                               const VolumeGridType grid_type,
                                               const eCDAllocType /*alloctype*/,
                                               const openvdb::GridBase::Ptr &grid_template,
                                               const AttributeIDRef &attribute_id)
{
  openvdb::GridBase::Ptr result = nullptr;
  volume_grid_to_static_type_tag(grid_type, [&](auto tag) {
    using GridType = typename decltype(tag)::type;
    if (grid_template) {
      typename GridType::Ptr typed_grid = GridType::create(*grid_template);
      result = typed_grid;
      volume_grid_to_static_type_tag(BKE_volume_grid_type_openvdb(*grid_template), [&](auto tag) {
        using GridType = typename decltype(tag)::type;
        typename GridType::Ptr typed_template = openvdb::GridBase::grid<GridType>(grid_template);
        BLI_assert(typed_template != nullptr);
        typed_grid->topologyUnion(*typed_template);
      });
    }
    else {
      result = GridType::create();
    }
  });
  if (!result) {
    return nullptr;
  }

  if (attribute_id.is_anonymous()) {
    const AnonymousAttributeID &anonymous_id = attribute_id.anonymous_id();
    result->setName(anonymous_id.name());
  }
  else {
    result->setName(attribute_id.name());
  }
  grids.emplace_back(VolumeGrid{result});

  return result;
}

static openvdb::GridBase::Ptr add_generic_grid_with_varray(
    VolumeGridVector &grids,
    const VolumeGridType grid_type,
    const eCDAllocType /*alloctype*/,
    const openvdb::GridBase::Ptr &grid_template,
    const AttributeIDRef &attribute_id,
    const GVArray &varray)
{
  openvdb::GridBase::Ptr result = add_generic_grid(
      grids, grid_type, CD_CONSTRUCT, grid_template, attribute_id);
  if (result != nullptr) {
    GVMutableArray dst_varray = get_volume_vmutablearray<VArrayImpl_For_VolumeGridValue>(
        *result, grid_type);
    volume_grid_to_static_type_tag(grid_type, [&](auto tag) {
      using GridType = typename decltype(tag)::type;
      using VArrayImplType = VArrayImpl_For_VolumeGridValue<GridType>;
      using AttributeType = typename VArrayImplType::AttributeType;

      VArray<AttributeType> typed_varray = varray.typed<AttributeType>();
      dst_varray.try_assign_VArray(typed_varray);
    });
  }
  return result;
}

static openvdb::GridBase::Ptr add_generic_grid_with_data(
    VolumeGridVector &grids,
    const VolumeGridType grid_type,
    const eCDAllocType /*alloctype*/,
    const openvdb::GridBase::Ptr &grid_template,
    const AttributeIDRef &attribute_id,
    const void *data,
    const ImplicitSharingInfo * /*sharing_info*/)
{
  openvdb::GridBase::Ptr result = add_generic_grid(
      grids, grid_type, CD_CONSTRUCT, grid_template, attribute_id);
  if (result != nullptr) {
    GVMutableArray dst_varray = get_volume_vmutablearray<VArrayImpl_For_VolumeGridValue>(
        *result, grid_type);
    dst_varray.set_all(data);
  }
  return result;
}

static bool add_grid_from_attribute_init(const AttributeIDRef &attribute_id,
                                         VolumeGridVector &grids,
                                         const VolumeGridType grid_type,
                                         const openvdb::GridBase::Ptr &grid_template,
                                         const AttributeInit &initializer)
{
  switch (initializer.type) {
    case AttributeInit::Type::Construct: {
      const openvdb::GridBase::Ptr result = add_generic_grid(
          grids, grid_type, CD_CONSTRUCT, grid_template, attribute_id);
      return result != nullptr;
    }
    case AttributeInit::Type::DefaultValue: {
      const openvdb::GridBase::Ptr result = add_generic_grid(
          grids, grid_type, CD_SET_DEFAULT, grid_template, attribute_id);
      return result != nullptr;
    }
    case AttributeInit::Type::VArray: {
      const GVArray &varray =
          static_cast<const blender::bke::AttributeInitVArray &>(initializer).varray;
      const openvdb::GridBase::Ptr result = add_generic_grid_with_varray(
          grids, grid_type, CD_SET_DEFAULT, grid_template, attribute_id, varray);
      return result != nullptr;
    }
    case AttributeInit::Type::MoveArray: {
      void *data = static_cast<const blender::bke::AttributeInitMoveArray &>(initializer).data;
      const openvdb::GridBase::ConstPtr result = add_generic_grid_with_data(
          grids, grid_type, CD_CONSTRUCT, grid_template, attribute_id, data, nullptr);
      return result != nullptr;
    }
    case AttributeInit::Type::Shared: {
      const AttributeInitShared &init = static_cast<const blender::bke::AttributeInitShared &>(
          initializer);
      const openvdb::GridBase::ConstPtr result = add_generic_grid_with_data(grids,
                                                                            grid_type,
                                                                            CD_CONSTRUCT,
                                                                            grid_template,
                                                                            attribute_id,
                                                                            init.data,
                                                                            init.sharing_info);
      return result != nullptr;
    }
  }
  return false;
}

bool VolumeCustomAttributeProvider::try_create(void *owner,
                                               const AttributeIDRef &attribute_id,
                                               const eAttrDomain domain,
                                               const eCustomDataType data_type,
                                               const AttributeInit &initializer) const
{
  if (!attribute_id) {
    return false;
  }
  if (domain_ != domain) {
    return false;
  }
  if (!this->type_is_supported(data_type)) {
    return false;
  }
  VolumeGridVector &grids = grid_access_.get_grids(owner);
  if (grids.find_grid(attribute_id)) {
    return false;
  }

  const openvdb::GridBase::Ptr grid_template = grids.empty() ? nullptr : grids.front().grid();
  const VolumeGridType grid_type = grid_type_for_custom_data_type(data_type);
  if (!add_grid_from_attribute_init(attribute_id, grids, grid_type, grid_template, initializer)) {
    return false;
  }
  if (update_on_change_ != nullptr) {
    update_on_change_(owner);
  }
  return true;
}

bool VolumeCustomAttributeProvider::foreach_attribute(
    const void *owner, const AttributeForeachCallback callback) const
{
  const VolumeGridVector &grids = grid_access_.get_const_grids(owner);

  for (const VolumeGrid &grid : grids) {
    const AttributeIDRef attribute_id{grid.name()};
    const eCustomDataType data_type = custom_data_type_for_grid_type(grid.grid_type());
    if (data_type == CD_NUMTYPES) {
      continue;
    }
    const AttributeMetaData meta_data{ATTR_DOMAIN_VOXEL, data_type};
    callback(attribute_id, meta_data);
  }
  return true;
}

}  // namespace blender::bke

#  endif  // WITH_OPENVDB

#endif
