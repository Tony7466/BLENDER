/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "attribute_access_volume.hh"

/* -------------------------------------------------------------------- */
/** \name Volume Geometry Grid
 * \{ */

VolumeGeometryGrid::VolumeGeometryGrid() : grid_(nullptr) {}
VolumeGeometryGrid::~VolumeGeometryGrid() {}

int64_t VolumeGeometryGrid::active_voxel_num() const
{
#ifdef WITH_OPENVDB
  return grid_->activeVoxelCount();
#else
  return 0;
#endif
}

/** \} */

#ifdef WITH_OPENVDB

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Grid Value Converter
 * \{ */

namespace converter {

template<typename GridValueT> struct GridValueConverter {
  using GridValueType = GridValueT;
  using AttributeType = GridValueT;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return value;
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return value;
  }
};

template<> struct GridValueConverter<double> {
  using GridValueType = double;
  using AttributeType = float;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return double(value);
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return value;
  }
};
template<> struct GridValueConverter<openvdb::Vec3d> {
  using GridValueType = openvdb::Vec3d;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(float(value.x()), float(value.y()), float(value.z()));
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3d(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::Vec3i> {
  using GridValueType = openvdb::Vec3i;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(float(value.x()), float(value.y()), float(value.z()));
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3i(int(value.x), int(value.y), int(value.z));
  }
};

template<> struct GridValueConverter<openvdb::Vec3f> {
  using GridValueType = openvdb::Vec3f;
  using AttributeType = blender::float3;

  static AttributeType to_attribute(const GridValueType &value)
  {
    return float3(value.x(), value.y(), value.z());
  }
  static GridValueType to_grid(const AttributeType &value)
  {
    return openvdb::Vec3f(value.x, value.y, value.z);
  }
};

}  // namespace converter

/** \} */

/* -------------------------------------------------------------------- */
/** \name Volume Attribute Virtual Array
 * \{ */

template<typename _GridType>
class VArrayImpl_For_VolumeGrid final
    : public VMutableArrayImpl<
          typename converter::GridValueConverter<typename _GridType::ValueType>::AttributeType> {
 protected:
  using GridType = typename std::remove_cv<_GridType>::type;
  using TreeType = typename GridType::TreeType;
  using ValueType = typename GridType::ValueType;
  using Converter = converter::GridValueConverter<ValueType>;
  using AttributeType = typename Converter::AttributeType;
  using Accessor = typename GridType::Accessor;
  using ConstAccessor = typename GridType::ConstAccessor;
  using LeafNodeType = typename TreeType::LeafNodeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafRange = typename LeafManager::LeafRange;

  GridType &grid_;

 public:
  VArrayImpl_For_VolumeGrid(GridType &grid)
      : VMutableArrayImpl<AttributeType>(grid.activeVoxelCount()), grid_(grid)
  {
  }

  VArrayImpl_For_VolumeGrid(const GridType &grid)
      : VMutableArrayImpl<AttributeType>(grid.activeVoxelCount()),
        grid_(const_cast<GridType &>(grid))
  {
  }

  AttributeType get(const int64_t index) const override
  {
    /* XXX It is recommended that each thread gets its own accessor.
     * https://www.openvdb.org/documentation/doxygen/overview.html#subsecValueAccessor
     * This could significantly improve threading performance,
     * but the VArray does not know about threaded access itself.
     */
    ConstAccessor accessor = grid_.getConstAccessor();
    openvdb::Coord coord = LeafNodeType::offsetToLocalCoord(index);
    return Converter::to_attribute(accessor.getValue(coord));
  }

  void set(const int64_t index, const AttributeType value) override
  {
    Accessor accessor = grid_.getAccessor();
    openvdb::Coord coord = LeafNodeType::offsetToLocalCoord(index);
    accessor.setValueOnly(coord, Converter::to_grid(value));
  }

  void set_all(Span<AttributeType> src) override
  {
    // LeafManager leaf_mgr(*grid_->treePtr());

    // const LeafRange leaf_range = leaf_mgr.leafRange();
    // tbb::parallel_for(leaf_range, [&](const LeafRange &range) {
    //   for (const LeafNodeType &leaf : range) {
    //   }
    // });
  }

  // void materialize(const IndexMask &mask, AttributeType *dst) const override
  // {
  //   if (dverts_ == nullptr) {
  //     mask.foreach_index([&](const int i) { dst[i] = 0.0f; });
  //   }
  //   mask.foreach_index(GrainSize(4096), [&](const int64_t i) {
  //     if (const MDeformWeight *weight = this->find_weight_at_index(i)) {
  //       dst[i] = weight->weight;
  //     }
  //     else {
  //       dst[i] = 0.0f;
  //     }
  //   });
  // }

  // void materialize_to_uninitialized(const IndexMask &mask, AttributeType *dst) const override
  // {
  //   this->materialize(mask, dst);
  // }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Static Type Selector Templates
 * \{ */

namespace static_type_select {

using SupportedVolumeVDBTypes = openvdb::TypeList<openvdb::BoolGrid,
                                                  openvdb::DoubleGrid,
                                                  openvdb::FloatGrid,
                                                  openvdb::Int32Grid,
                                                  openvdb::Int64Grid,
                                                  openvdb::MaskGrid,
                                                  openvdb::Vec3DGrid,
                                                  openvdb::Vec3IGrid,
                                                  openvdb::Vec3SGrid>;

#  define SupportedVolumeCPPTypes float, int

template<typename OpType> static bool grid_operation(const openvdb::GridBase &grid, OpType &&op)
{
  return grid.apply<SupportedVolumeVDBTypes>(op);
}

template<typename OpType> static bool cpp_type_operation(const CPPType &type, OpType &&op)
{
  type.to_static_type<SupportedVolumeCPPTypes>(op);

  return type.is_any<SupportedVolumeCPPTypes>();
}

struct MakeGridVArrayOp {
  GVArray result;

  template<typename GridType> void operator()(const GridType &grid)
  {
    using ValueType = typename GridType::ValueType;
    using AttributeType = typename converter::GridValueConverter<ValueType>::AttributeType;
    using VArrayImplType = VArrayImpl_For_VolumeGrid<GridType>;
    result = VArray<AttributeType>::template For<VArrayImplType, const GridType>(std::move(grid));
  }
};

struct MakeGridVMutableArrayOp {
  GVMutableArray result;

  template<typename GridType> void operator()(GridType &grid)
  {
    using ValueType = typename GridType::ValueType;
    using AttributeType = typename converter::GridValueConverter<ValueType>::AttributeType;
    using VArrayImplType = VArrayImpl_For_VolumeGrid<GridType>;
    result = VMutableArray<AttributeType>::template For<VArrayImplType, GridType>(std::move(grid));
  }
};

struct CreateGridTypeOp {
  const AttributeInit &initializer;

  openvdb::GridBase::Ptr &result;

  template<typename CPPType> void operator()() const
  {
    using TreeType = typename openvdb::tree::Tree4<CPPType, 5, 4, 3>::Type;
    using GridType = typename openvdb::Grid<TreeType>;

    result = GridType::create();
  }

  void operator()() const {}
};

}  // namespace static_type_select

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Provider Declaration
 * \{ */

static GVArray make_grid_array_for_read(const openvdb::GridBase &grid)
{
  static_type_select::MakeGridVArrayOp op;
  if (static_type_select::grid_operation(grid, op)) {
    return op.result;
  }
  return {};
}

static GVMutableArray make_grid_array_for_write(openvdb::GridBase &grid)
{
  static_type_select::MakeGridVMutableArrayOp op;
  if (static_type_select::grid_operation(grid, op)) {
    return op.result;
  }
  return {};
}

// static openvdb::GridBase::Ptr create_grid_type(const CPPType &type,
//                                               const AttributeInit &initializer)
//{
//  openvdb::GridBase::Ptr result;
//  static_type_select::CreateGridTypeOp op{initializer, result};
//  if (static_type_select::cpp_type_operation(type, op)) {
//    return result;
//  }
//  return nullptr;
//}

GAttributeReader BuiltinVolumeAttributeProvider::try_get_for_read(const void *owner) const
{
  const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
  return {make_grid_array_for_read(*grid.grid_), domain_, nullptr};
}

GAttributeWriter BuiltinVolumeAttributeProvider::try_get_for_write(void *owner) const
{
  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  return {make_grid_array_for_write(*grid.grid_), domain_, nullptr};
}

bool BuiltinVolumeAttributeProvider::try_delete(void * /*owner*/) const
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

bool BuiltinVolumeAttributeProvider::try_create(void * /*owner*/,
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

bool BuiltinVolumeAttributeProvider::exists(const void * /*owner*/) const
{
  /* Not supported. */
  return false;
  //  const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
  //  return grid->has_attribute(???);
}

GAttributeReader VolumeAttributeProvider::try_get_for_read(
    const void *owner, const AttributeIDRef & /*attribute_id*/) const
{
  const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
  return {make_grid_array_for_read(*grid.grid_), domain_, nullptr};
}

GAttributeWriter VolumeAttributeProvider::try_get_for_write(
    void *owner, const AttributeIDRef & /*attribute_id*/) const
{
  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  return {make_grid_array_for_write(*grid.grid_), domain_, nullptr};
}

bool VolumeAttributeProvider::try_delete(void * /*owner*/,
                                         const AttributeIDRef & /*attribute_id*/) const
{
  /* Not supported. */
  //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  //  if (grid.remove_attribute(attribute_id)) {
  //    if (update_on_change_ != nullptr) {
  //      update_on_change_(owner);
  //    }
  //  }
  //  return true;
  return false;
}

bool VolumeAttributeProvider::try_create(void * /*owner*/,
                                         const AttributeIDRef & /*attribute_id*/,
                                         const eAttrDomain domain,
                                         const eCustomDataType data_type,
                                         const AttributeInit & /*initializer*/) const
{
  if (domain_ != domain) {
    return false;
  }
  if (!this->type_is_supported(data_type)) {
    return false;
  }
  /* Not supported. */
  //  VolumeGeometryGrid &grid = grid_access_.get_grid(owner);
  //  if (grid.add_attribute(attribute_id)) {
  //    if (update_on_change_ != nullptr) {
  //      update_on_change_(owner);
  //    }
  //    return true;
  //  }
  return false;
}

bool VolumeAttributeProvider::foreach_attribute(const void * /*owner*/,
                                                const AttributeForeachCallback /*callback*/) const
{
  /* Not supported. */
  return false;
  //  const VolumeGeometryGrid &grid = grid_access_.get_const_grid(owner);
  //  return grid.foreach_attribute(callback);
}

}  // namespace blender::bke

#endif  // WITH_OPENVDB
