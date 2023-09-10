/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_generic_virtual_array.hh"
#include "BLI_generic_virtual_grid.hh"
#include "BLI_volume_openvdb.hh"

namespace blender {

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl
 * \{ */

// void GVGridImpl::materialize(const IndexMask &mask, void *dst) const
//{
//   mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
//     void *elem_dst = POINTER_OFFSET(dst, type_->size() * i);
//     this->get(i, elem_dst);
//   });
// }
//
// void GVGridImpl::materialize_to_uninitialized(const IndexMask &mask, void *dst) const
//{
//   mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
//     void *elem_dst = POINTER_OFFSET(dst, type_->size() * i);
//     this->get_to_uninitialized(i, elem_dst);
//   });
// }
//
// void GVGridImpl::materialize_compressed(const IndexMask &mask, void *dst) const
//{
//   mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
//     void *elem_dst = POINTER_OFFSET(dst, type_->size() * pos);
//     this->get(i, elem_dst);
//   });
// }
//
// void GVGridImpl::materialize_compressed_to_uninitialized(const IndexMask &mask, void *dst) const
//{
//   mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
//     void *elem_dst = POINTER_OFFSET(dst, type_->size() * pos);
//     this->get_to_uninitialized(i, elem_dst);
//   });
// }
//
// void GVGridImpl::get(const int64_t index, void *r_value) const
//{
//   type_->destruct(r_value);
//   this->get_to_uninitialized(index, r_value);
// }

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVMutableGridImpl
 * \{ */

// void GVMutableGridImpl::set_by_copy(const int64_t index, const void *value)
//{
//   BUFFER_FOR_CPP_TYPE_VALUE(*type_, buffer);
//   type_->copy_construct(value, buffer);
//   this->set_by_move(index, buffer);
//   type_->destruct(buffer);
// }
//
// void GVMutableGridImpl::set_by_relocate(const int64_t index, void *value)
//{
//   this->set_by_move(index, value);
//   type_->destruct(value);
// }
//
// void GVMutableGridImpl::set_all(const void *src)
//{
//   const CommonVGridInfo info = this->common_info();
//   if (info.type == CommonVGridInfo::Type::Span) {
//     type_->copy_assign_n(src, const_cast<void *>(info.data));
//   }
//   else {
//     for (int64_t i : IndexRange(size_)) {
//       this->set_by_copy(i, POINTER_OFFSET(src, type_->size() * i));
//     }
//   }
// }
//
// void GVMutableGrid::fill(const void *value)
//{
//   const CommonVGridInfo info = this->common_info();
//   if (info.type == CommonVGridInfo::Type::Span) {
//     this->type().fill_assign_n(value, const_cast<void *>(info.data), this->size());
//   }
//   else {
//     for (int64_t i : IndexRange(this->size())) {
//       this->set_by_copy(i, value);
//     }
//   }
// }

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl_For_Grid
 * \{ */

GVGridImpl_For_Grid::GVGridImpl_For_Grid(const GridType &grid)
    : GVMutableGridImpl(volume::grid_base_attribute_type(grid)), grid_(&grid)
{
}

// GVArray GVGridImpl_For_Grid::get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const
//{
//  return volume::get_varray_for_leaf(log2dim, origin, *grid_);
//}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl_For_SingleValueRef
 * \{ */

/* Generic virtual array where each element has the same value. The value is not owned. */

GVArray GVGridImpl_For_SingleValueRef::get_varray_for_leaf(uint32_t log2dim,
                                                           const int3 & /*origin*/) const
{
  const uint32_t num_voxels = 1 << 3 * log2dim;

  volume::field_to_static_type(this->type(), [&](auto tag) {
    using AttributeValueType = typename decltype(tag)::type;

    return VArray<AttributeValueType>::ForSingle(*static_cast<const AttributeValueType *>(value_),
                                                 num_voxels);
  });
  return {};
}

CommonVGridInfo GVGridImpl_For_SingleValueRef::common_info() const
{
  return CommonVGridInfo{CommonVGridInfo::Type::Single, true, value_};
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl_For_SingleValue
 * \{ */

/* Same as GVGridImpl_For_SingleValueRef, but the value is owned. */
class GVGridImpl_For_SingleValue : public GVGridImpl_For_SingleValueRef, NonCopyable, NonMovable {
 public:
  GVGridImpl_For_SingleValue(const CPPType &type, const void *value)
      : GVGridImpl_For_SingleValueRef(type)
  {
    value_ = MEM_mallocN_aligned(type.size(), type.alignment(), __func__);
    type.copy_construct(value, (void *)value_);
  }

  ~GVGridImpl_For_SingleValue() override
  {
    type_->destruct((void *)value_);
    MEM_freeN((void *)value_);
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl_For_SmallTrivialSingleValue
 * \{ */

/**
 * Contains an inline buffer that can store a single value of a trivial type.
 * This avoids the allocation that would be done by #GVGridImpl_For_SingleValue.
 */
template<int BufferSize> class GVGridImpl_For_SmallTrivialSingleValue : public GVGridImpl {
 private:
  AlignedBuffer<BufferSize, 8> buffer_;

 public:
  GVGridImpl_For_SmallTrivialSingleValue(const CPPType &type, const void *value) : GVGridImpl(type)
  {
    BLI_assert(type.is_trivial());
    BLI_assert(type.alignment() <= 8);
    BLI_assert(type.size() <= BufferSize);
    type.copy_construct(value, &buffer_);
  }

  GVArray get_varray_for_leaf(uint32_t log2dim, const int3 & /*origin*/) const
  {
    const uint32_t num_voxels = 1 << 3 * log2dim;

    volume::field_to_static_type(this->type(), [&](auto tag) {
      using AttributeValueType = typename decltype(tag)::type;

      return VArray<AttributeValueType>::ForSingle(
          *static_cast<const AttributeValueType *>(buffer_.ptr()), num_voxels);
    });
    return {};
  }

 private:
  CommonVGridInfo common_info() const override
  {
    return CommonVGridInfo{CommonVGridInfo::Type::Single, true, &buffer_};
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridCommon
 * \{ */

GVGridCommon::GVGridCommon(const GVGridCommon &other) : storage_(other.storage_)
{
  impl_ = this->impl_from_storage();
}

GVGridCommon::GVGridCommon(GVGridCommon &&other) noexcept : storage_(std::move(other.storage_))
{
  impl_ = this->impl_from_storage();
  other.storage_.reset();
  other.impl_ = nullptr;
}

GVGridCommon::GVGridCommon(const GVGridImpl *impl) : impl_(impl)
{
  storage_ = impl_;
}

GVGridCommon::GVGridCommon(std::shared_ptr<const GVGridImpl> impl) : impl_(impl.get())
{
  if (impl) {
    storage_ = std::move(impl);
  }
}

GVGridCommon::~GVGridCommon() = default;

GVArray GVGridCommon::get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const
{
  return impl_->get_varray_for_leaf(log2dim, origin);
}

// void GVGridCommon::materialize(void *dst) const
//{
//   this->materialize(IndexMask(impl_->size()), dst);
// }
//
// void GVGridCommon::materialize(const IndexMask &mask, void *dst) const
//{
//   impl_->materialize(mask, dst);
// }
//
// void GVGridCommon::materialize_to_uninitialized(void *dst) const
//{
//   this->materialize_to_uninitialized(IndexMask(impl_->size()), dst);
// }
//
// void GVGridCommon::materialize_to_uninitialized(const IndexMask &mask, void *dst) const
//{
//   BLI_assert(mask.min_array_size() <= impl_->size());
//   impl_->materialize_to_uninitialized(mask, dst);
// }
//
// void GVGridCommon::materialize_compressed(const IndexMask &mask, void *dst) const
//{
//   impl_->materialize_compressed(mask, dst);
// }
//
// void GVGridCommon::materialize_compressed_to_uninitialized(const IndexMask &mask, void *dst)
// const
//{
//   impl_->materialize_compressed_to_uninitialized(mask, dst);
// }

void GVGridCommon::copy_from(const GVGridCommon &other)
{
  if (this == &other) {
    return;
  }
  storage_ = other.storage_;
  impl_ = this->impl_from_storage();
}

void GVGridCommon::move_from(GVGridCommon &&other) noexcept
{
  if (this == &other) {
    return;
  }
  storage_ = std::move(other.storage_);
  impl_ = this->impl_from_storage();
  other.storage_.reset();
  other.impl_ = nullptr;
}

bool GVGridCommon::is_grid() const
{
  const CommonVGridInfo info = impl_->common_info();
  return info.type == CommonVGridInfo::Type::Grid;
}

#ifdef WITH_OPENVDB
GVGridCommon::GridType *GVGridCommon::get_internal_grid() const
{
  BLI_assert(this->is_grid());
  const CommonVGridInfo info = impl_->common_info();
  return static_cast<GridType *>(const_cast<void *>(info.data));
}
#endif

bool GVGridCommon::is_single() const
{
  const CommonVGridInfo info = impl_->common_info();
  return info.type == CommonVGridInfo::Type::Single;
}

void GVGridCommon::get_internal_single(void *r_value) const
{
  BLI_assert(this->is_single());
  const CommonVGridInfo info = impl_->common_info();
  this->type().copy_assign(info.data, r_value);
}

void GVGridCommon::get_internal_single_to_uninitialized(void *r_value) const
{
  impl_->type().default_construct(r_value);
  this->get_internal_single(r_value);
}

const GVGridImpl *GVGridCommon::impl_from_storage() const
{
  if (!storage_.has_value()) {
    return nullptr;
  }
  return storage_.extra_info().get_vgrid(storage_.get());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGrid
 * \{ */

GVGrid::GVGrid(const GVGrid &other) = default;

GVGrid::GVGrid(GVGrid &&other) noexcept = default;

GVGrid::GVGrid(const GVGridImpl *impl) : GVGridCommon(impl) {}

GVGrid::GVGrid(std::shared_ptr<const GVGridImpl> impl) : GVGridCommon(std::move(impl)) {}

GVGrid::GVGrid(vgrid_tag::single /* tag */, const CPPType &type, const void *value)
{
  if (type.is_trivial() && type.size() <= 16 && type.alignment() <= 8) {
    this->emplace<GVGridImpl_For_SmallTrivialSingleValue<16>>(type, value);
  }
  else {
    this->emplace<GVGridImpl_For_SingleValue>(type, value);
  }
}

GVGrid GVGrid::ForSingle(const CPPType &type, const void *value)
{
  return GVGrid(vgrid_tag::single{}, type, value);
}

GVGrid GVGrid::ForSingleRef(const CPPType &type, const void *value)
{
  return GVGrid(vgrid_tag::single_ref{}, type, value);
}

GVGrid GVGrid::ForSingleDefault(const CPPType &type)
{
  return GVGrid::ForSingleRef(type, type.default_value());
}

#ifdef WITH_OPENVDB
GVGrid GVGrid::ForGrid(const GridType &grid)
{
  return GVGrid(vgrid_tag::grid{}, grid);
}

GVGrid GVGrid::ForEmpty(const CPPType &type)
{
  openvdb::GridBase *grid = nullptr;
  volume::field_to_static_type(type, [&](auto type_tag) {
    using T = typename decltype(type_tag)::type;
    using GridType = volume::grid_types::AttributeGrid<T>;
    using Converter = volume::grid_types::Converter<GridType>;

    const T &value = *static_cast<const T *>(type.default_value());
    grid = new volume::grid_types::AttributeGrid<T>(Converter::single_value_to_grid(value));
  });
  BLI_assert(grid != nullptr);

  return GVGrid::ForGrid(*grid);
}
#endif

GVGrid &GVGrid::operator=(const GVGrid &other)
{
  this->copy_from(other);
  return *this;
}

GVGrid &GVGrid::operator=(GVGrid &&other) noexcept
{
  this->move_from(std::move(other));
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVMutableGrid
 * \{ */

GVMutableGrid::GVMutableGrid(const GVMutableGrid &other) = default;
GVMutableGrid::GVMutableGrid(GVMutableGrid &&other) noexcept = default;

GVMutableGrid::GVMutableGrid(GVMutableGridImpl *impl) : GVGridCommon(impl) {}

GVMutableGrid::GVMutableGrid(std::shared_ptr<GVMutableGridImpl> impl)
    : GVGridCommon(std::move(impl))
{
}

#ifdef WITH_OPENVDB
GVMutableGrid GVMutableGrid::ForGrid(GridType &grid)
{
  return GVMutableGrid::For<GVGridImpl_For_Grid_final>(grid);
}
#endif

GVMutableGrid::operator GVGrid() const &
{
  GVGrid varray;
  varray.copy_from(*this);
  return varray;
}

GVMutableGrid::operator GVGrid() &&noexcept
{
  GVGrid varray;
  varray.move_from(std::move(*this));
  return varray;
}

GVMutableGrid &GVMutableGrid::operator=(const GVMutableGrid &other)
{
  this->copy_from(other);
  return *this;
}

GVMutableGrid &GVMutableGrid::operator=(GVMutableGrid &&other) noexcept
{
  this->move_from(std::move(other));
  return *this;
}

GVMutableGridImpl *GVMutableGrid::get_implementation() const
{
  return this->get_impl();
}

// void GVMutableGrid::set_all(const void *src)
//{
//   this->get_impl()->set_all(src);
// }

#ifdef WITH_OPENVDB
GVMutableGrid::GridType *GVMutableGrid::get_internal_grid() const
{
  BLI_assert(this->is_grid());
  const CommonVGridInfo info = impl_->common_info();
  return static_cast<GridType *>(const_cast<void *>(info.data));
}
#endif

/** \} */

CommonVGridInfo GVGridImpl_For_SingleValueRef_final::common_info() const
{
  return CommonVGridInfo(CommonVGridInfo::Type::Single, false, value_);
}

}  // namespace blender
