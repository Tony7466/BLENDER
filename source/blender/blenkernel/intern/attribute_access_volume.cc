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
    using GridType = volume::grid_types::AttributeGrid<ValueType>;
    using Converter = volume::grid_types::Converter<GridType>;

    const ValueType &background_value = *static_cast<const ValueType *>(
        value_type.default_value());
    return GridType::create(Converter::single_value_to_grid(background_value));
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

template<typename ToTreeType> struct ConvertGridOp<ToTreeType, openvdb::MaskTree, true> {
  typename openvdb::Grid<ToTreeType>::Ptr operator()(const openvdb::MaskTree & /*from_tree*/)
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
    using GridType = volume::grid_types::AttributeGrid<ValueType>;
    using TreeType = typename GridType::TreeType;
    using Converter = volume::grid_types::Converter<GridType>;

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
        result = GridType::create(Converter::single_value_to_grid(background_value));
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
    using GridType = volume::grid_types::AttributeGrid<ValueType>;

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
