/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_volume_types.h"

#include "BLI_bounds_types.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_index_mask.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"

/** \file
 * \ingroup bke
 * \brief Volume data-block.
 */

namespace blender::bke {

/**
 * A C++ class that wraps the DNA struct for better encapsulation and ease of use. It inherits
 * directly from the struct rather than storing a pointer to avoid more complicated ownership
 * handling.
 */
class VolumeGeometry : public ::VolumeGeometry {
 public:
  VolumeGeometry();
  VolumeGeometry(const VolumeGeometry &other);
  VolumeGeometry(VolumeGeometry &&other);
  VolumeGeometry &operator=(const VolumeGeometry &other);
  VolumeGeometry &operator=(VolumeGeometry &&other);
  ~VolumeGeometry();

  /* --------------------------------------------------------------------
   * Accessors.
   */

  /**
   * The total number of control points in all curves.
   */
  int active_voxel_num() const;
  IndexRange active_voxel_range() const;

  /**
   * The largest and smallest position values of evaluated points.
   */
  std::optional<Bounds<float3>> bounds_min_max() const;

  /* --------------------------------------------------------------------
   * Operations.
   */

 public:
  /** Call after operations changing the grid tree topology. */
  void tag_tree_changed();

  AttributeAccessor attributes() const;
  MutableAttributeAccessor attributes_for_write();

  /* --------------------------------------------------------------------
   * Attributes.
   */

  GVArray adapt_domain(const GVArray &varray, eAttrDomain from, eAttrDomain to) const;
  template<typename T>
  VArray<T> adapt_domain(const VArray<T> &varray, eAttrDomain from, eAttrDomain to) const
  {
    return this->adapt_domain(GVArray(varray), from, to).typed<T>();
  }

  /* --------------------------------------------------------------------
   * File Read/Write.
   */

  void blend_read_data(BlendDataReader &reader);
  void blend_write(BlendWriter &writer, ID &id);

 protected:
  void free_grid(Volume);
};

}  // namespace blender::bke

inline blender::bke::VolumeGeometry &VolumeGeometry::wrap()
{
  return *reinterpret_cast<blender::bke::VolumeGeometry *>(this);
}
inline const blender::bke::VolumeGeometry &VolumeGeometry::wrap() const
{
  return *reinterpret_cast<const blender::bke::VolumeGeometry *>(this);
}

// XXX OLD API for reference

//  const blender::CPPType *BKE_volume_grid_cpp_type(const openvdb::GridBase &grid);
//  size_t BKE_volume_grid_active_voxels(const VolumeGrid *grid);
//  /* Tries to find a grid for the given attribute. */
//  const VolumeGrid *BKE_volume_grid_attribute_find_for_read(
//      const struct Volume *volume, const blender::bke::AttributeIDRef &attribute_id);
//  VolumeGrid *BKE_volume_grid_attribute_find_for_write(
//      struct Volume *volume, const blender::bke::AttributeIDRef &attribute_id);
//  VolumeGrid *BKE_volume_grid_attribute_add_vdb(Volume &volume,
//                                                const blender::bke::AttributeIDRef
//                                                &attribute_id, openvdb::GridBase::Ptr
//                                                vdb_grid);

// namespace blender {

// struct CPPTypeForGridTypeOp {
//  const CPPType *result = nullptr;

//  template<typename GridType> void operator()(const GridType &grid)
//  {
//    using ValueType = typename GridType::ValueType;
//    using AttributeType = typename volume_openvdb::GridValueConverter<ValueType>::AttributeType;
//    result = &CPPType::get<AttributeType>();
//  }
//};

//}  // namespace blender

// const blender::CPPType *BKE_volume_grid_cpp_type(const openvdb::GridBase &grid)
//{
//  blender::CPPTypeForGridTypeOp op;
//  if (grid.apply<SupportedVolumeVDBTypes>(op)) {
//    return op.result;
//  }
//  return nullptr;
//}

// const VolumeGrid *BKE_volume_grid_attribute_find_for_read(
//    const struct Volume *volume, const blender::bke::AttributeIDRef &attribute_id)
//{
//  return BKE_volume_grid_find_for_read(volume, attribute_id.name().data());
//}

// VolumeGrid *BKE_volume_grid_attribute_find_for_write(
//    struct Volume *volume, const blender::bke::AttributeIDRef &attribute_id)
//{
//  return BKE_volume_grid_find_for_write(volume, attribute_id.name().data());
//}

// VolumeGrid *BKE_volume_grid_attribute_add_vdb(Volume &volume,
//                                              const blender::bke::AttributeIDRef &attribute_id,
//                                              openvdb::GridBase::Ptr vdb_grid)
//{
//  return BKE_volume_grid_add_vdb(volume, attribute_id.name(), vdb_grid);
//}
