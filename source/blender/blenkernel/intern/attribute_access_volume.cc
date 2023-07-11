/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_volume_attribute.hh"

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

#ifdef WITH_OPENVDB

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Attribute Provider Declaration
 * \{ */

GAttributeReader VolumeGridValueAttributeProvider::try_get_for_read(const void *owner) const
{
  // const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
  const VolumeGridVector &grids = grid_access_.get_const_grids(owner);
  GVArray varray = get_volume_varray<VArrayImpl_For_VolumeGridValue>(grids);
  return {varray, domain_, nullptr};
}

GAttributeWriter VolumeGridValueAttributeProvider::try_get_for_write(void *owner) const
{
  // VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  VolumeGridVector &grids = grid_access_.get_grids(owner);
  GVMutableArray varray = get_volume_vmutablearray<VArrayImpl_For_VolumeGridValue>(grids);
  return {varray, domain_, nullptr};
}

bool VolumeGridValueAttributeProvider::try_delete(void * /*owner*/) const
{
  if (deletable_ != Deletable) {
    return false;
  }
  /* Not supported. */
  //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  //  if (grid.remove_attribute(???)) {
  //    if (update_on_change_ != nullptr) {
  //      update_on_change_(owner);
  //    }
  //  }
  //  return true;
  return false;
}

bool VolumeGridValueAttributeProvider::try_create(void * /*owner*/,
                                                  const AttributeInit & /*initializer*/) const
{
  if (createable_ != Creatable) {
    return false;
  }
  /* Not supported. */
  //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  //  if (grid.add_attribute(???)) {
  //    if (update_on_change_ != nullptr) {
  //      update_on_change_(owner);
  //    }
  //    return true;
  //  }
  return false;
}

bool VolumeGridValueAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

// GAttributeReader VolumeAttributeProvider::try_get_for_read(
//     const void *owner, const AttributeIDRef & /*attribute_id*/) const
//{
//   const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
//   MakeGridVArrayOp op{*grid.grid_};
//   grid.grid_type_operation(op);
//   return {op.result_, domain_, nullptr};
// }
//
// GAttributeWriter VolumeAttributeProvider::try_get_for_write(
//     void *owner, const AttributeIDRef & /*attribute_id*/) const
//{
//   VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
//   MakeGridVMutableArrayOp op{*grid.grid_};
//   grid.grid_type_operation(op);
//   return {op.result_, domain_, nullptr};
// }
//
// bool VolumeAttributeProvider::try_delete(void * /*owner*/,
//                                          const AttributeIDRef & /*attribute_id*/) const
//{
//   /* Not supported. */
//   //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
//   //  if (grid.remove_attribute(attribute_id)) {
//   //    if (update_on_change_ != nullptr) {
//   //      update_on_change_(owner);
//   //    }
//   //  }
//   //  return true;
//   return false;
// }
//
// bool VolumeAttributeProvider::try_create(void * /*owner*/,
//                                          const AttributeIDRef & /*attribute_id*/,
//                                          const eAttrDomain domain,
//                                          const eCustomDataType data_type,
//                                          const AttributeInit & /*initializer*/) const
//{
//   if (domain_ != domain) {
//     return false;
//   }
//   if (!this->type_is_supported(data_type)) {
//     return false;
//   }
//   /* Not supported. */
//   //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
//   //  if (grid.add_attribute(attribute_id)) {
//   //    if (update_on_change_ != nullptr) {
//   //      update_on_change_(owner);
//   //    }
//   //    return true;
//   //  }
//   return false;
// }
//
// bool VolumeAttributeProvider::foreach_attribute(const void * /*owner*/,
//                                                 const AttributeForeachCallback /*callback*/)
//                                                 const
//{
//   /* Not supported. */
//   return false;
//   //  const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
//   //  return grid.foreach_attribute(callback);
// }

}  // namespace blender::bke

#endif  // WITH_OPENVDB
