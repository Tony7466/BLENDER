/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_volume_openvdb.hh"

#include "BKE_attribute.hh"
#include "BKE_volume_attribute.hh"
#include "BKE_volume_openvdb.hh"

#include "attribute_access_volume.hh"

#include "intern/volume_grids.hh"

#ifndef NDEBUG
#  define DEBUG_GRID_ATTRIBUTES
#endif

namespace blender::bke {

GAttributeGridReader VolumeCustomAttributeGridProvider::try_get_grid_for_read(
    const void *owner, const AttributeIDRef &attribute_id) const
{
  const VolumeGridVector &grids = grid_access_.get_const_grids(owner);
  const VolumeGrid *grid = grids.find_grid(attribute_id);
  return {{grid ? grid->grid() : nullptr}, domain_, nullptr};
}

GAttributeGridWriter VolumeCustomAttributeGridProvider::try_get_grid_for_write(
    void *owner, const AttributeIDRef &attribute_id) const
{
  VolumeGridVector &grids = grid_access_.get_grids(owner);
  VolumeGrid *grid = grids.find_grid(attribute_id);
  return {{grid ? grid->grid() : nullptr}, domain_, nullptr};
}

bool VolumeCustomAttributeGridProvider::try_delete(void *owner,
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

static openvdb::GridBase::Ptr add_generic_grid(const CPPType &value_type,
                                               const eCDAllocType /*alloctype*/)
{
  volume::field_to_static_type(value_type, [&](auto tag) {
    using ValueType = typename decltype(tag)::type;
    using GridType = volume::grid_types::GridCommon<ValueType>;
    using TreeType = typename GridType::TreeType;

    const ValueType &background_value = *static_cast<const ValueType *>(
        value_type.default_value());
    return GridType::create(background_value);
  });
  return nullptr;
}

template<typename ToTreeType, typename FromTreeType, bool is_convertible> struct ConvertGridOp {
  typename openvdb::Grid<ToTreeType>::Ptr operator()(const FromTreeType & /*from_tree*/)
  {
    return openvdb::Grid<ToTreeType>::create();
  }
};

template<typename ToTreeType, typename FromTreeType>
struct ConvertGridOp<ToTreeType, FromTreeType, true> {
  typename openvdb::Grid<ToTreeType>::Ptr operator()(const FromTreeType &from_tree)
  {
    /* Make a deep copy of the data with value casting. */
    typename ToTreeType::Ptr tree = std::make_shared<ToTreeType>(from_tree);
    return openvdb::Grid<ToTreeType>::create(tree);
  }
};

template<typename ToTreeType, typename FromTreeType>
struct ConvertGridOp<ToTreeType, FromTreeType, false> {
  typename openvdb::Grid<ToTreeType>::Ptr operator()(const FromTreeType & /*from_tree*/)
  {
    return openvdb::Grid<ToTreeType>::create();
  }
};

template<typename ToTreeType>
struct ConvertGridOp<ToTreeType, volume::grid_types::MaskTree, true> {
  typename openvdb::Grid<ToTreeType>::Ptr operator()(
      const volume::grid_types::MaskTree & /*from_tree*/)
  {
    return openvdb::Grid<ToTreeType>::create();
  }
};

static openvdb::GridBase::Ptr add_generic_grid_copy(const CPPType &value_type,
                                                    const volume::GGrid &data)
{
  /* Template build sanitization: nested static type dispatch creates a lot of code, which can
   * easily make builds run out of memory. Capturing a functor allows doing the 2nd type dispatch
   * for the grid template separately, avoiding combinatorial explosion. */
  std::function<openvdb::GridBase::Ptr(const openvdb::GridBase::ConstPtr &from_grid)> copy_fn =
      nullptr;

  volume::field_to_static_type(value_type, [&](auto tag) {
    using ValueType = typename decltype(tag)::type;
    using GridType = volume::grid_types::GridCommon<ValueType>;
    using TreeType = typename GridType::TreeType;

    const ValueType &background_value = *static_cast<const ValueType *>(
        value_type.default_value());

    copy_fn = [background_value](
                  const openvdb::GridBase::ConstPtr &from_grid) -> openvdb::GridBase::Ptr {
      openvdb::GridBase::Ptr result = nullptr;
      if (from_grid) {
        volume::grid_to_static_type(from_grid, [&](auto &typed_data) {
          using FromGridType = typename std::decay<decltype(typed_data)>::type;
          using FromTreeType = typename FromGridType::TreeType;
          using FromValueType = typename FromTreeType::ValueType;
          ConvertGridOp<TreeType, FromTreeType, std::is_convertible_v<FromValueType, ValueType>>
              convert;
          result = convert(typed_data.tree());
        });
      }
      else {
        result = GridType::create(background_value);
      }
      return result;
    };
  });

  if (copy_fn) {
    return copy_fn(data.grid_);
  }
  return nullptr;
}

static openvdb::GridBase::Ptr add_generic_grid_move(const CPPType & /*value_type*/,
                                                    const volume::GMutableGrid &data)
{
  return data.grid_;
}

static openvdb::GridBase::Ptr add_generic_grid_shared(const CPPType &value_type,
                                                      const volume::GMutableGrid &data,
                                                      const ImplicitSharingInfo *sharing_info)
{
  /* XXX May eventually use this, for now just rely on shared_ptr. */
  UNUSED_VARS(sharing_info);

  openvdb::GridBase::Ptr result = nullptr;
  volume::field_to_static_type(value_type, [&](auto tag) {
    using ValueType = typename decltype(tag)::type;
    using GridType = volume::grid_types::GridCommon<ValueType>;

    if (data) {
      /* Data must be same grid type */
      BLI_assert(data.grid_->isType<GridType>());
      typename GridType::Ptr typed_data = openvdb::GridBase::grid<GridType>(data.grid_);
      BLI_assert(typed_data != nullptr);

      /* Create grid that shares the tree. */
      result = std::make_shared<GridType>(typed_data->treePtr());
    }
  });
  return result;
}

static bool add_grid_from_attribute_init(const AttributeIDRef &attribute_id,
                                         VolumeGridVector &grids,
                                         const CPPType &value_type,
                                         const AttributeInit &initializer)
{
  openvdb::GridBase::Ptr result = nullptr;
  switch (initializer.type) {
    case AttributeInit::Type::Construct:
      result = add_generic_grid(value_type, CD_CONSTRUCT);
#ifdef DEBUG_GRID_ATTRIBUTES
      std::cout << "Constructed grid attribute " << attribute_id << std::endl;
#endif
      break;
    case AttributeInit::Type::DefaultValue:
      result = add_generic_grid(value_type, CD_SET_DEFAULT);
#ifdef DEBUG_GRID_ATTRIBUTES
      std::cout << "Default value grid attribute " << attribute_id << std::endl;
#endif
      break;
    case AttributeInit::Type::VArray:
    case AttributeInit::Type::MoveArray:
    case AttributeInit::Type::Shared:
      break;
    case AttributeInit::Type::Grid: {
      const volume::GGrid &data =
          static_cast<const blender::bke::AttributeInitGrid &>(initializer).grid;
      result = add_generic_grid_copy(value_type, data);
#ifdef DEBUG_GRID_ATTRIBUTES
      std::cout << "Copied grid to attribute " << attribute_id << std::endl;
      if (data.grid_) {
        data.grid_->print(std::cout, 3);
      }
#endif
      break;
    }
    case AttributeInit::Type::MoveGrid: {
      const volume::GMutableGrid &data =
          static_cast<const blender::bke::AttributeInitMoveGrid &>(initializer).grid;
      result = add_generic_grid_move(value_type, data);
#ifdef DEBUG_GRID_ATTRIBUTES
      std::cout << "Moved grid to attribute " << attribute_id << std::endl;
      if (data.grid_) {
        data.grid_->print(std::cout, 3);
      }
#endif
      break;
    }
    case AttributeInit::Type::SharedGrid: {
      const AttributeInitSharedGrid &init =
          static_cast<const blender::bke::AttributeInitSharedGrid &>(initializer);
      const openvdb::GridBase::ConstPtr result = add_generic_grid_shared(
          value_type, init.grid, init.sharing_info);
#ifdef DEBUG_GRID_ATTRIBUTES
      std::cout << "Shared grid to attribute " << attribute_id << std::endl;
      if (init.grid.grid_) {
        init.grid.grid_->print(std::cout, 3);
      }
#endif
      break;
    }
  }
  if (result == nullptr) {
    return false;
  }

  if (attribute_id.is_anonymous()) {
    const AnonymousAttributeID &anonymous_id = attribute_id.anonymous_id();
    result->setName(anonymous_id.name());
  }
  else {
    result->setName(attribute_id.name());
  }
  grids.emplace_back(VolumeGrid{result});
  return true;
}

bool VolumeCustomAttributeGridProvider::try_create(void *owner,
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

  const CPPType *value_type = custom_data_type_to_cpp_type(data_type);
  if (value_type == nullptr) {
    return false;
  }
  if (!add_grid_from_attribute_init(attribute_id, grids, *value_type, initializer)) {
    return false;
  }
  if (update_on_change_ != nullptr) {
    update_on_change_(owner);
  }
  return true;
}

bool VolumeCustomAttributeGridProvider::foreach_attribute(
    const void *owner, const AttributeForeachCallback callback) const
{
  const VolumeGridVector &grids = grid_access_.get_const_grids(owner);

  for (const VolumeGrid &grid : grids) {
    const AttributeIDRef attribute_id{grid.name()};
    const CPPType *type = volume::GGrid{grid.grid()}.value_type();
    if (type == nullptr) {
      continue;
    }
    const eCustomDataType data_type = cpp_type_to_custom_data_type(*type);
    if (data_type == CD_NUMTYPES) {
      continue;
    }
    const AttributeMetaData meta_data{ATTR_DOMAIN_VOXEL, data_type};
    callback(attribute_id, meta_data);
  }
  return true;
}

}  // namespace blender::bke

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
