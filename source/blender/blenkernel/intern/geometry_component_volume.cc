/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

// !!! DEBUGGING !!!
// #define WITH_OPENVDB

#include <cstdint>

#include "DNA_volume_types.h"

#include "BLI_cpp_type.hh"

#include "BKE_attribute.h"
#include "BKE_geometry_set.hh"
#include "BKE_lib_id.h"
#include "BKE_volume.h"
#include "BKE_volume_geometry.hh"

#include "attribute_access_intern.hh"
#include "volume_openvdb.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/Grid.h>
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

VolumeComponent::VolumeComponent() : GeometryComponent(GeometryComponent::Type::Volume) {}

VolumeComponent::~VolumeComponent()
{
  this->clear();
}

GeometryComponent *VolumeComponent::copy() const
{
  VolumeComponent *new_component = new VolumeComponent();
  if (volume_ != nullptr) {
    new_component->volume_ = BKE_volume_copy_for_eval(volume_);
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return new_component;
}

void VolumeComponent::clear()
{
  BLI_assert(this->is_mutable() || this->is_expired());
  if (volume_ != nullptr) {
    if (ownership_ == GeometryOwnershipType::Owned) {
      BKE_id_free(nullptr, volume_);
    }
    volume_ = nullptr;
  }
}

bool VolumeComponent::has_volume() const
{
  return volume_ != nullptr;
}

void VolumeComponent::replace(Volume *volume, GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  volume_ = volume;
  ownership_ = ownership;
}

Volume *VolumeComponent::release()
{
  BLI_assert(this->is_mutable());
  Volume *volume = volume_;
  volume_ = nullptr;
  return volume;
}

const Volume *VolumeComponent::get_for_read() const
{
  return volume_;
}

Volume *VolumeComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    volume_ = BKE_volume_copy_for_eval(volume_);
    ownership_ = GeometryOwnershipType::Owned;
  }
  return volume_;
}

bool VolumeComponent::owns_direct_data() const
{
  return ownership_ == GeometryOwnershipType::Owned;
}

void VolumeComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  if (ownership_ != GeometryOwnershipType::Owned) {
    volume_ = BKE_volume_copy_for_eval(volume_);
    ownership_ = GeometryOwnershipType::Owned;
  }
}

#ifdef WITH_OPENVDB

/* -------------------------------------------------------------------- */
/** \name Attribute Provider Declaration
 * \{ */

template<typename _GridType>
class VArrayImpl_For_VolumeGrid final
    : public VMutableArrayImpl<typename volume_openvdb::GridValueConverter<
          typename _GridType::ValueType>::AttributeType> {
 protected:
  using GridType = typename std::remove_cv<_GridType>::type;
  using TreeType = typename GridType::TreeType;
  using ValueType = typename GridType::ValueType;
  using Converter = volume_openvdb::GridValueConverter<ValueType>;
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

/**
 * Utility to group together multiple functions that are used to access custom data on geometry
 * components in a generic way.
 */
struct VolumeGridAccessInfo {
  using GridGetter = Volume *(*)(void *owner);
  using ConstGridGetter = const Volume *(*)(const void *owner);

  GridGetter get_grids;
  ConstGridGetter get_const_grids;
};

namespace detail {
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
    using AttributeType = typename volume_openvdb::GridValueConverter<ValueType>::AttributeType;
    using VArrayImplType = VArrayImpl_For_VolumeGrid<GridType>;
    result = VArray<AttributeType>::template For<VArrayImplType, const GridType>(std::move(grid));
  }
};

struct MakeGridVMutableArrayOp {
  GVMutableArray result;

  template<typename GridType> void operator()(GridType &grid)
  {
    using ValueType = typename GridType::ValueType;
    using AttributeType = typename volume_openvdb::GridValueConverter<ValueType>::AttributeType;
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

}  // namespace detail

static GVArray make_grid_array_for_read(const openvdb::GridBase &grid)
{
  detail::MakeGridVArrayOp op;
  if (detail::grid_operation(grid, op)) {
    return op.result;
  }
  return {};
}

static GVMutableArray make_grid_array_for_write(openvdb::GridBase &grid)
{
  detail::MakeGridVMutableArrayOp op;
  if (detail::grid_operation(grid, op)) {
    return op.result;
  }
  return {};
}

static openvdb::GridBase::Ptr create_grid_type(const CPPType &type,
                                               const AttributeInit &initializer)
{
  openvdb::GridBase::Ptr result;
  detail::CreateGridTypeOp op{initializer, result};
  if (detail::cpp_type_operation(type, op)) {
    return result;
  }
  return nullptr;
}

class BuiltinVolumeGridProvider final : public BuiltinAttributeProvider {
  using UpdateOnChange = void (*)(void *owner);
  const VolumeGridAccessInfo grid_access_;
  const UpdateOnChange update_on_change_;

 public:
  BuiltinVolumeGridProvider(std::string attribute_name,
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

GAttributeReader BuiltinVolumeGridProvider::try_get_for_read(const void *owner) const
{
  const Volume *volume = grid_access_.get_const_grids(owner);
  if (volume == nullptr) {
    return {};
  }
  const VolumeGrid *grid = BKE_volume_grid_find_for_read(volume, name_.c_str());
  if (grid == nullptr) {
    return {};
  }

  openvdb::GridBase::ConstPtr vdb_grid = BKE_volume_grid_openvdb_for_read(volume, grid);
  return {make_grid_array_for_read(*vdb_grid), domain_, nullptr};
}

GAttributeWriter BuiltinVolumeGridProvider::try_get_for_write(void *owner) const
{
  Volume *volume = grid_access_.get_grids(owner);
  if (volume == nullptr) {
    return {};
  }
  VolumeGrid *grid = BKE_volume_grid_find_for_write(volume, name_.c_str());
  if (grid == nullptr) {
    return {};
  }

  std::function<void()> tag_modified_fn;
  if (update_on_change_ != nullptr) {
    tag_modified_fn = [owner, update = update_on_change_]() { update(owner); };
  }

  openvdb::GridBase::Ptr vdb_grid = BKE_volume_grid_openvdb_for_write(volume, grid, false);
  return {make_grid_array_for_write(*vdb_grid), domain_, nullptr};
}

bool BuiltinVolumeGridProvider::try_delete(void *owner) const
{
  if (deletable_ != Deletable) {
    return false;
  }
  Volume *volume = grid_access_.get_grids(owner);
  if (volume == nullptr) {
    return false;
  }
  VolumeGrid *grid = BKE_volume_grid_find_for_write(volume, name_.c_str());
  if (grid == nullptr) {
    return false;
  }

  BKE_volume_grid_remove(volume, grid);
  if (update_on_change_ != nullptr) {
    update_on_change_(owner);
  }
  return true;
}

bool BuiltinVolumeGridProvider::try_create(void *owner, const AttributeInit &initializer) const
{
  if (createable_ != Creatable) {
    return false;
  }
  Volume *volume = grid_access_.get_grids(owner);
  if (volume == nullptr) {
    return false;
  }
  const CPPType *cpptype = custom_data_type_to_cpp_type(data_type_);
  if (cpptype == nullptr) {
    return false;
  }
  openvdb::GridBase::Ptr vdb_grid = create_grid_type(*cpptype, initializer);
  if (vdb_grid == nullptr) {
    return false;
  }
  VolumeGrid *grid = BKE_volume_grid_attribute_add_vdb(*volume, name_.c_str(), vdb_grid);
  return grid != nullptr;
}

bool BuiltinVolumeGridProvider::exists(const void *owner) const
{
  const Volume *volume = grid_access_.get_const_grids(owner);
  if (volume == nullptr) {
    return false;
  }
  const VolumeGrid *grid = BKE_volume_grid_find_for_read(volume, name_.c_str());
  return grid != nullptr;
}

/**
 * An attribute provider for custom volume grids.
 */
class VolumeGridAttributeProvider final : public DynamicAttributesProvider {
 private:
  static constexpr uint64_t supported_types_mask = CD_MASK_PROP_ALL;
  const eAttrDomain domain_;
  const VolumeGridAccessInfo grid_access_;

 public:
  VolumeGridAttributeProvider(const eAttrDomain domain, const VolumeGridAccessInfo grid_access)
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

GAttributeReader VolumeGridAttributeProvider::try_get_for_read(
    const void *owner, const AttributeIDRef &attribute_id) const
{
  const Volume *volume = grid_access_.get_const_grids(owner);
  if (volume == nullptr) {
    return {};
  }
  const VolumeGrid *grid = BKE_volume_grid_attribute_find_for_read(volume, attribute_id);
  if (grid == nullptr) {
    return {};
  }

  openvdb::GridBase::ConstPtr vdb_grid = BKE_volume_grid_openvdb_for_read(volume, grid);
  return {make_grid_array_for_read(*vdb_grid), domain_, nullptr};
}

GAttributeWriter VolumeGridAttributeProvider::try_get_for_write(
    void *owner, const AttributeIDRef &attribute_id) const
{
  Volume *volume = grid_access_.get_grids(owner);
  if (volume == nullptr) {
    return {};
  }
  VolumeGrid *grid = BKE_volume_grid_attribute_find_for_write(volume, attribute_id);
  if (grid == nullptr) {
    return {};
  }

  openvdb::GridBase::Ptr vdb_grid = BKE_volume_grid_openvdb_for_write(volume, grid, false);
  return {make_grid_array_for_write(*vdb_grid), domain_, nullptr};
}

bool VolumeGridAttributeProvider::try_delete(void *owner, const AttributeIDRef &attribute_id) const
{
  Volume *volume = grid_access_.get_grids(owner);
  if (volume == nullptr) {
    return false;
  }
  VolumeGrid *grid = BKE_volume_grid_attribute_find_for_write(volume, attribute_id);
  if (grid == nullptr) {
    return false;
  }

  BKE_volume_grid_remove(volume, grid);
  return true;
}

bool VolumeGridAttributeProvider::try_create(void *owner,
                                             const AttributeIDRef &attribute_id,
                                             const eAttrDomain domain,
                                             const eCustomDataType data_type,
                                             const AttributeInit &initializer) const
{
  if (domain_ != domain) {
    return false;
  }
  if (!this->type_is_supported(data_type)) {
    return false;
  }
  Volume *volume = grid_access_.get_grids(owner);
  if (volume == nullptr) {
    return false;
  }
  const CPPType *cpptype = custom_data_type_to_cpp_type(data_type);
  if (cpptype == nullptr) {
    return false;
  }
  openvdb::GridBase::Ptr vdb_grid = create_grid_type(*cpptype, initializer);
  if (vdb_grid == nullptr) {
    return false;
  }
  VolumeGrid *grid = BKE_volume_grid_attribute_add_vdb(*volume, attribute_id, vdb_grid);
  return grid != nullptr;
}

bool VolumeGridAttributeProvider::foreach_attribute(const void *owner,
                                                    const AttributeForeachCallback callback) const
{
  const Volume *volume = grid_access_.get_const_grids(owner);
  if (volume == nullptr) {
    return false;
  }

  const int num_grids = BKE_volume_num_grids(volume);
  for (const int i : IndexRange(num_grids)) {
    const VolumeGrid *grid = BKE_volume_grid_get_for_read(volume, i);
    BLI_assert(grid != nullptr);
    openvdb::GridBase::ConstPtr vdb_grid = BKE_volume_grid_openvdb_for_read(volume, grid);
    if (vdb_grid == nullptr) {
      continue;
    }
    const CPPType *cpptype = BKE_volume_grid_cpp_type(*vdb_grid);
    if (cpptype == nullptr) {
      continue;
    }
    const eCustomDataType data_type = cpp_type_to_custom_data_type(*cpptype);
    if (!type_is_supported(data_type)) {
      continue;
    }

    /* XXX need to integrate AttributeID with the grid list */
    AttributeIDRef attr_id(BKE_volume_grid_name(grid));
    AttributeMetaData attr_meta = {ATTR_DOMAIN_POINT, data_type};
    if (!callback(attr_id, attr_meta)) {
      return false;
    }
  }

  return true;
}

/* -------------------------------------------------------------------- */
/** \name Attribute Access Helper Functions
 * \{ */

// static void tag_component_radii_changed(void *owner)
//{
//   CurvesGeometry &curves = *static_cast<CurvesGeometry *>(owner);
//   curves.tag_radii_changed();
// }

/** \} */

/**
 * In this function all the attribute providers for a volume component are created.
 * Most data in this function is statically allocated, because it does not change over time.
 */
static ComponentAttributeProviders create_attribute_providers_for_volume()
{
  static VolumeGridAccessInfo grid_access = {
      [](void *owner) -> Volume * {
        Volume &volume = *static_cast<Volume *>(owner);
        return &volume;
      },
      [](const void *owner) -> const Volume * {
        const Volume &volume = *static_cast<const Volume *>(owner);
        return &volume;
      },
  };

  static auto update_on_change = [](void *owner) {};

  static BuiltinVolumeGridProvider position("position",
                                            ATTR_DOMAIN_POINT,
                                            CD_PROP_FLOAT3,
                                            BuiltinAttributeProvider::NonCreatable,
                                            BuiltinAttributeProvider::NonDeletable,
                                            grid_access,
                                            update_on_change);

  static VolumeGridAttributeProvider voxel_custom_data(ATTR_DOMAIN_POINT, grid_access);

  return ComponentAttributeProviders({&position}, {&voxel_custom_data});
}

/** \} */

static AttributeAccessorFunctions get_volume_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_volume();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const VolumeGridVector &grids = *static_cast<const VolumeGridVector *>(owner);
    switch (domain) {
      case ATTR_DOMAIN_POINT:
        // BKE_volume_grid_active_voxels(???
        return 1;
      default:
        return 0;
    }
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
    const Volume &volume = *static_cast<const Volume *>(owner);

    return BKE_volume_adapt_domain(volume, varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_volume_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_volume_accessor_functions();
  return fn;
}

std::optional<AttributeAccessor> VolumeComponent::attributes() const
{
  return AttributeAccessor(volume_ ? &volume_->runtime.grids : nullptr,
                           blender::bke::get_volume_accessor_functions_ref());
}

std::optional<MutableAttributeAccessor> VolumeComponent::attributes_for_write()
{
  Volume *volume = this->get_for_write();
  return MutableAttributeAccessor(volume ? &volume->runtime.grids : nullptr,
                                  blender::bke::get_volume_accessor_functions_ref());
}

#else

std::optional<AttributeAccessor> VolumeComponent::attributes() const
{
  return {};
}

std::optional<MutableAttributeAccessor> VolumeComponent::attributes_for_write()
{
  return {};
}

#endif  // WITH_OPENVDB

/** \} */

}  // namespace blender::bke
