/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <optional>

#include "BLI_any.hh"
#include "BLI_volume.hh"

namespace blender {

/** Forward declarations for generic virtual grids. */
class GVGrid;
class GVMutableGrid;

/**
 * Is used to quickly check if a vgrid is a grid or single value..
 */
struct CommonVGridInfo {
  enum class Type : uint8_t {
    /* Is not one of the common special types below. */
    Any,
    Grid,
    Single,
  };

  Type type = Type::Any;

  /** True when the #data becomes a dangling pointer when the virtual grid is destructed. */
  bool may_have_ownership = true;

  /**
   * Points either to nothing, a single value or OpenVDB grid, depending on #type.
   * If this is a mutable virtual grid, it is safe to cast away const.
   */
  const void *data;

  CommonVGridInfo() = default;
  CommonVGridInfo(const Type _type, const bool _may_have_ownership, const void *_data)
      : type(_type), may_have_ownership(_may_have_ownership), data(_data)
  {
  }
};

/**
 * Implements the specifics of how the elements of a virtual array are accessed. It contains a
 * bunch of virtual methods that are wrapped by #VGrid.
 */
template<typename T> class VGridImpl {
 public:
  VGridImpl() {}

  virtual ~VGridImpl() = default;

  virtual CommonVGridInfo common_info() const
  {
    return {};
  }

  /**
   * Get the element at #index. This does not return a reference, because the value may be computed
   * on the fly.
   */
  // virtual T get(int64_t index) const = 0;

  /**
   * Copy values from the virtual array into the provided span. The index of the value in the
   * virtual array is the same as the index in the span.
   */
  // virtual void materialize(const IndexMask &mask, T *dst) const
  //{
  //   mask.foreach_index([&](const int64_t i) { dst[i] = this->get(i); });
  // }

  /**
   * Same as #materialize but #r_span is expected to be uninitialized.
   */
  // virtual void materialize_to_uninitialized(const IndexMask &mask, T *dst) const
  //{
  //   mask.foreach_index([&](const int64_t i) { new (dst + i) T(this->get(i)); });
  // }

  /**
   * Copy values from the virtual array into the provided span. Contrary to #materialize, the index
   * in virtual array is not the same as the index in the output span. Instead, the span is filled
   * without gaps.
   */
  // virtual void materialize_compressed(const IndexMask &mask, T *dst) const
  //{
  //   mask.foreach_index([&](const int64_t i, const int64_t pos) { dst[pos] = this->get(i); });
  // }

  /**
   * Same as #materialize_compressed but #r_span is expected to be uninitialized.
   */
  // virtual void materialize_compressed_to_uninitialized(const IndexMask &mask, T *dst) const
  //{
  //   mask.foreach_index(
  //       [&](const int64_t i, const int64_t pos) { new (dst + pos) T(this->get(i)); });
  // }

  /**
   * If this virtual grid wraps another #GVGrid, this method should assign the wrapped array to the
   * provided reference. This allows losslessly converting between generic and typed virtual arrays
   * in all cases. Return true when the virtual grid was assigned and false when nothing was done.
   */
  virtual bool try_assign_GVGrid(GVGrid & /*vgrid*/) const
  {
    return false;
  }

  virtual VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const = 0;
};

/** Similar to #VGridImpl, but adds methods that allow modifying the referenced elements. */
template<typename T> class VMutableGridImpl : public VGridImpl<T> {
 public:
  using VGridImpl<T>::VGridImpl;

  /**
   * Assign the provided #value to the #index.
   */
  // virtual void set(int64_t index, T value) = 0;

  /**
   * Copy all elements from the provided span into the virtual array.
   */
  // virtual void set_all(Span<T> src)
  //{
  //   const CommonVGridInfo info = this->common_info();
  //   if (info.type == CommonVGridInfo::Type::Span) {
  //     initialized_copy_n(
  //         src.data(), this->size_, const_cast<T *>(static_cast<const T *>(info.data)));
  //   }
  //   else {
  //     const int64_t size = this->size_;
  //     for (int64_t i = 0; i < size; i++) {
  //       this->set(i, src[i]);
  //     }
  //   }
  // }

  /**
   * Similar to #VGridImpl::try_assign_GVGrid but for mutable virtual grids.
   */
  virtual bool try_assign_GVMutableGrid(GVMutableGrid & /*vgrid*/) const
  {
    return false;
  }
};

#ifdef WITH_OPENVDB

/**
 * A virtual grid implementation that wraps a OpenVDB grid. This implementation is used by
 * mutable and immutable grids to avoid code duplication.
 */
template<typename T> class VGridImpl_For_Grid : public VMutableGridImpl<T> {
 protected:
  using GridType = volume::grid_types::AttributeGrid<T>;
  using TreeType = typename GridType::TreeType;

  GridType *grid_ = nullptr;

 public:
  VGridImpl_For_Grid(GridType &grid) : grid_(&grid) {}

  VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
#  ifdef WITH_OPENVDB
    using Accessor = typename GridType::ConstAccessor;
    using Converter = volume::grid_types::Converter<GridType>;
    const uint32_t num_voxels = 1 << 3 * log2dim;

    Accessor accessor = grid_->getConstAccessor();
    return VArray<T>::ForFunc(num_voxels, [log2dim, origin, &accessor](const int64_t index) {
      const openvdb::Coord xyz = volume::offset_to_global_coord(log2dim, origin, int32_t(index));
      return Converter::single_value_to_attribute(accessor.getValue(xyz));
    });
#  else
    return {};
#  endif
  }

 protected:
  CommonVGridInfo common_info() const override
  {
    return CommonVGridInfo(CommonVGridInfo::Type::Grid, true, grid_);
  }
};

/**
 * A version of #VArrayImpl_For_Grid that can not be subclassed. This allows safely overwriting
 * the #may_have_ownership method.
 */
template<typename T> class VGridImpl_For_Grid_final final : public VGridImpl_For_Grid<T> {
 public:
  using VGridImpl_For_Grid<T>::VGridImpl_For_Grid;
  using GridType = typename VGridImpl_For_Grid<T>::GridType;

  VGridImpl_For_Grid_final(GridType &grid)
      /* Cast const away, because the implementation for const and non const spans is shared. */
      : VGridImpl_For_Grid<T>(grid)
  {
  }

 private:
  CommonVGridInfo common_info() const final
  {
    return CommonVGridInfo(CommonVGridInfo::Type::Grid, false, this->grid_);
  }
};
#endif

/**
 * A virtual array implementation that returns the same value for every location. This class is
 * final so that it can be devirtualized by the compiler in some cases (e.g. when
 * #devirtualize_VGrid is used).
 */
template<typename T> class VGridImpl_For_Single final : public VGridImpl<T> {
 private:
  T value_;

 public:
  VGridImpl_For_Single(T value) : value_(std::move(value)) {}

  VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 & /*origin*/) const override
  {
    const uint32_t num_voxels = 1 << 3 * log2dim;
    return VArray<T>::ForSingle(value_, num_voxels);
  }

 protected:
  CommonVGridInfo common_info() const override
  {
    return CommonVGridInfo(CommonVGridInfo::Type::Single, true, &value_);
  }
};

/**
 * Evalute a function at the voxel location.
 * The `GetFunc` should take a single coordinate argument and return the value at that location.
 */
template<typename T, typename GetFunc> class VGridImpl_For_CoordFunc final : public VGridImpl<T> {
 private:
  GetFunc get_func_;

 public:
  VGridImpl_For_CoordFunc(GetFunc get_func) : get_func_(std::move(get_func)) {}

  VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
#ifdef WITH_OPENVDB
    const uint32_t num_voxels = 1 << 3 * log2dim;
    return VArray<T>::ForFunc(num_voxels, [this, log2dim, origin](const int64_t index) -> T {
      const int3 coord = int3(volume::offset_to_global_coord(log2dim, origin, index).data());
      return get_func_(coord);
    });
#else
    return {};
#endif
  }
};

namespace detail {

/**
 * Struct that can be passed as `ExtraInfo` into an #Any.
 * This struct is only intended to be used by #VGridCommon.
 */
template<typename T> struct VGridAnyExtraInfo {
  /**
   * Gets the virtual grid that is stored at the given pointer.
   */
  const VGridImpl<T> *(*get_vgrid)(const void *buffer);

  template<typename StorageT> static constexpr VGridAnyExtraInfo get()
  {
    /* These are the only allowed types in the #Any. */
    static_assert(
        std::is_base_of_v<VGridImpl<T>, StorageT> ||
        is_same_any_v<StorageT, const VGridImpl<T> *, std::shared_ptr<const VGridImpl<T>>>);

    /* Depending on how the virtual grid implementation is stored in the #Any, a different
     * #get_vgrid function is required. */
    if constexpr (std::is_base_of_v<VGridImpl<T>, StorageT>) {
      return {[](const void *buffer) {
        return static_cast<const VGridImpl<T> *>((const StorageT *)buffer);
      }};
    }
    else if constexpr (std::is_same_v<StorageT, const VGridImpl<T> *>) {
      return {[](const void *buffer) { return *(const StorageT *)buffer; }};
    }
    else if constexpr (std::is_same_v<StorageT, std::shared_ptr<const VGridImpl<T>>>) {
      return {[](const void *buffer) { return ((const StorageT *)buffer)->get(); }};
    }
    else {
      BLI_assert_unreachable();
      return {};
    }
  }
};

}  // namespace detail

/**
 * Utility class to reduce code duplication for methods available on #VGrid and #VMutableGrid.
 * Deriving #VMutableGrid from #VGrid would have some issues:
 * - Static methods on #VGrid would also be available on #VMutableGrid.
 * - It would allow assigning a #VGrid to a #VMutableGrid under some circumstances which is not
 *   allowed and could result in hard to find bugs.
 */
template<typename T> class VGridCommon {
 public:
#ifdef WITH_OPENVDB
  using GridType = volume::grid_types::AttributeGrid<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;
  using GridValueType = typename GridType::ValueType;
  using Converter = volume::grid_types::Converter<GridType>;
#endif

 protected:
  /**
   * Store the virtual grid implementation in an #Any. This makes it easy to avoid a memory
   * allocation if the implementation is small enough and is copyable. This is the case for the
   * most common virtual grids.
   * Other virtual grid implementations are typically stored as #std::shared_ptr. That works even
   * when the implementation itself is not copyable and makes copying #VGridCommon cheaper.
   */
  using Storage = Any<blender::detail::VGridAnyExtraInfo<T>, 24, 8>;

  /**
   * Pointer to the currently contained virtual grid implementation. This is allowed to be null.
   */
  const VGridImpl<T> *impl_ = nullptr;
  /**
   * Does the memory management for the virtual grid implementation. It contains one of the
   * following:
   * - Inlined subclass of #VGridImpl.
   * - Non-owning pointer to a #VGridImpl.
   * - Shared pointer to a #VGridImpl.
   */
  Storage storage_;

 protected:
  VGridCommon() = default;

  /** Copy constructor. */
  VGridCommon(const VGridCommon &other) : storage_(other.storage_)
  {
    impl_ = this->impl_from_storage();
  }

  /** Move constructor. */
  VGridCommon(VGridCommon &&other) noexcept : storage_(std::move(other.storage_))
  {
    impl_ = this->impl_from_storage();
    other.storage_.reset();
    other.impl_ = nullptr;
  }

  /**
   * Wrap an existing #VGridImpl and don't take ownership of it. This should rarely be used in
   * practice.
   */
  VGridCommon(const VGridImpl<T> *impl) : impl_(impl)
  {
    storage_ = impl_;
  }

  /**
   * Wrap an existing #VGridImpl that is contained in a #std::shared_ptr. This takes ownership.
   */
  VGridCommon(std::shared_ptr<const VGridImpl<T>> impl) : impl_(impl.get())
  {
    if (impl) {
      storage_ = std::move(impl);
    }
  }

  /**
   * Replace the contained #VGridImpl.
   */
  template<typename ImplT, typename... Args> void emplace(Args &&...args)
  {
    /* Make sure we are actually constructing a #VGridImpl. */
    static_assert(std::is_base_of_v<VGridImpl<T>, ImplT>);
    if constexpr (std::is_copy_constructible_v<ImplT> && Storage::template is_inline_v<ImplT>) {
      /* Only inline the implementation when it is copyable and when it fits into the inline
       * buffer of the storage. */
      impl_ = &storage_.template emplace<ImplT>(std::forward<Args>(args)...);
    }
    else {
      /* If it can't be inlined, create a new #std::shared_ptr instead and store that in the
       * storage. */
      std::shared_ptr<const VGridImpl<T>> ptr = std::make_shared<ImplT>(
          std::forward<Args>(args)...);
      impl_ = &*ptr;
      storage_ = std::move(ptr);
    }
  }

  /** Utility to implement a copy assignment operator in a subclass. */
  void copy_from(const VGridCommon &other)
  {
    if (this == &other) {
      return;
    }
    storage_ = other.storage_;
    impl_ = this->impl_from_storage();
  }

  /** Utility to implement a move assignment operator in a subclass. */
  void move_from(VGridCommon &&other) noexcept
  {
    if (this == &other) {
      return;
    }
    storage_ = std::move(other.storage_);
    impl_ = this->impl_from_storage();
    other.storage_.reset();
    other.impl_ = nullptr;
  }

  /** Get a pointer to the virtual array implementation that is currently stored in #storage_, or
   * null. */
  const VGridImpl<T> *impl_from_storage() const
  {
    if (!storage_.has_value()) {
      return nullptr;
    }
    return storage_.extra_info().get_vgrid(storage_.get());
  }

 public:
  /** Return false when there is no virtual array implementation currently. */
  operator bool() const
  {
    return impl_ != nullptr;
  }

  CommonVGridInfo common_info() const
  {
    BLI_assert(*this);
    return impl_->common_info();
  }

  /** Return true when the virtual grid is stored as a OpenVDB grid internally. */
  bool is_grid() const
  {
    BLI_assert(*this);
    const CommonVGridInfo info = impl_->common_info();
    return info.type == CommonVGridInfo::Type::Grid;
  }

#ifdef WITH_OPENVDB
  /**
   * Returns the internally used grid of the virtual grid. This invokes undefined behavior if the
   * virtual grid is not stored as a grid internally.
   */
  const GridType *get_internal_grid() const
  {
    BLI_assert(this->is_grid());
    const CommonVGridInfo info = impl_->common_info();
    return static_cast<const GridType *>(info.data);
  }
#endif

  /** Return true when the virtual grid returns the same value for every index. */
  bool is_single() const
  {
    BLI_assert(*this);
    const CommonVGridInfo info = impl_->common_info();
    return info.type == CommonVGridInfo::Type::Single;
  }

  /**
   * Return the value that is returned for every location. This invokes undefined behavior if the
   * virtual grid would not return the same value for every location.
   */
  T get_internal_single() const
  {
    BLI_assert(this->is_single());
    const CommonVGridInfo info = impl_->common_info();
    return *static_cast<const T *>(info.data);
  }

  /**
   * Return the value that is returned for every index, if the array is stored as a single value.
   */
  std::optional<T> get_if_single() const
  {
    const CommonVGridInfo info = impl_->common_info();
    if (info.type != CommonVGridInfo::Type::Single) {
      return std::nullopt;
    }
    return *static_cast<const T *>(info.data);
  }

  /** See #GVGridImpl::try_assign_GVGrid. */
  bool try_assign_GVGrid(GVGrid &vgrid) const
  {
    return impl_->try_assign_GVGrid(vgrid);
  }

  /* Leaf size 2^(log2dim * 3) and origin define the transform to local coordinates and index
   * space. */
  VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const
  {
    return impl_->get_varray_for_leaf(log2dim, origin);
  }

  const VGridImpl<T> *get_implementation() const
  {
    return impl_;
  }
};

template<typename T> class VMutableGrid;

/**
 * Various tags to disambiguate constructors of virtual grids.
 * Generally it is easier to use `VGrid::For*` functions to construct virtual grids, but
 * sometimes being able to use the constructor can result in better performance For example, when
 * constructing the virtual grid directly in a vector. Without the constructor one would have to
 * construct the virtual grid first and then move it into the vector.
 */
namespace vgrid_tag {
struct grid {
};
struct single_ref {
};
struct single {
};
}  // namespace vgrid_tag

/**
 * A #VGrid wraps a virtual grid implementation and provides easy access to its elements. It can
 * be copied and moved. While it is relatively small, it should still be passed by reference if
 * possible.
 */
template<typename T> class VGrid : public VGridCommon<T> {
  friend VMutableGrid<T>;

 public:
#ifdef WITH_OPENVDB
  using GridType = typename VGridCommon<T>::GridType;
  using TreeType = typename VGridCommon<T>::TreeType;
  using GridPtr = typename VGridCommon<T>::GridPtr;
  using GridConstPtr = typename VGridCommon<T>::GridConstPtr;
  using GridValueType = typename VGridCommon<T>::GridValueType;
  using Converter = typename VGridCommon<T>::Converter;
#endif

 public:
  VGrid() = default;
  VGrid(const VGrid &other) = default;
  VGrid(VGrid &&other) noexcept = default;

  VGrid(const VGridImpl<T> *impl) : VGridCommon<T>(impl) {}

  VGrid(std::shared_ptr<const VGridImpl<T>> impl) : VGridCommon<T>(std::move(impl)) {}

#ifdef WITH_OPENVDB
  VGrid(vgrid_tag::grid /* tag */, const GridType &grid)
  {
    this->template emplace<VGridImpl_For_Grid_final<T>>(const_cast<GridType &>(grid));
  }
#endif

  VGrid(vgrid_tag::single /* tag */, T value)
  {
    this->template emplace<VGridImpl_For_Single<T>>(std::move(value));
  }

  /**
   * Construct a new virtual grid for a custom #VGridImpl.
   */
  template<typename ImplT, typename... Args> static VGrid For(Args &&...args)
  {
    static_assert(std::is_base_of_v<VGridImpl<T>, ImplT>);
    VGrid VGrid;
    VGrid.template emplace<ImplT>(std::forward<Args>(args)...);
    return VGrid;
  }

  /**
   * Construct a new virtual grid that has the same value at every location.
   */
  static VGrid ForSingle(T value)
  {
    return VGrid(vgrid_tag::single{}, std::move(value));
  }

#ifdef WITH_OPENVDB
  /**
   * Construct a new virtual array for an existing span. This does not take ownership of the
   * underlying memory.
   */
  static VGrid ForGrid(const GridType &grid)
  {
    return VGrid(vgrid_tag::grid{}, grid);
  }
#endif

  /**
   * Construct a new virtual that will invoke the provided function whenever an element is
   * accessed.
   */
  template<typename GetFunc> static VGrid ForFunc(GetFunc get_func)
  {
    return VGrid::For<VGridImpl_For_CoordFunc<T, decltype(get_func)>>(std::move(get_func));
  }

  VGrid &operator=(const VGrid &other)
  {
    this->copy_from(other);
    return *this;
  }

  VGrid &operator=(VGrid &&other) noexcept
  {
    this->move_from(std::move(other));
    return *this;
  }
};

/**
 * Similar to #VGrid but references a virtual grid that can be modified.
 */
template<typename T> class VMutableGrid : public VGridCommon<T> {
 public:
#ifdef WITH_OPENVDB
  using GridType = typename VGridCommon<T>::GridType;
  using TreeType = typename VGridCommon<T>::TreeType;
  using GridPtr = typename VGridCommon<T>::GridPtr;
  using GridConstPtr = typename VGridCommon<T>::GridConstPtr;
  using GridValueType = typename VGridCommon<T>::GridValueType;
  using Converter = typename VGridCommon<T>::Converter;
#endif

 public:
  VMutableGrid() = default;
  VMutableGrid(const VMutableGrid &other) = default;
  VMutableGrid(VMutableGrid &&other) noexcept = default;

  VMutableGrid(const VMutableGridImpl<T> *impl) : VGridCommon<T>(impl) {}

  VMutableGrid(std::shared_ptr<const VMutableGridImpl<T>> impl) : VGridCommon<T>(std::move(impl))
  {
  }

  /**
   * Construct a new virtual grid for a custom #VMutableGridImpl.
   */
  template<typename ImplT, typename... Args> static VMutableGrid For(Args &&...args)
  {
    static_assert(std::is_base_of_v<VMutableGridImpl<T>, ImplT>);
    VMutableGrid VGrid;
    VGrid.template emplace<ImplT>(std::forward<Args>(args)...);
    return VGrid;
  }

#ifdef WITH_OPENVDB
  /**
   * Construct a new virtual array for an existing span. This does not take ownership of the
   * span.
   */
  static VMutableGrid ForGrid(GridType &grid)
  {
    return VMutableGrid::For<VGridImpl_For_Grid_final<T>>(grid);
  }
#endif

  /** Convert to a #VGrid by copying. */
  operator VGrid<T>() const &
  {
    VGrid<T> VGrid;
    VGrid.copy_from(*this);
    return VGrid;
  }

  /** Convert to a #VGrid by moving. */
  operator VGrid<T>() &&noexcept
  {
    VGrid<T> VGrid;
    VGrid.move_from(std::move(*this));
    return VGrid;
  }

  VMutableGrid &operator=(const VMutableGrid &other)
  {
    this->copy_from(other);
    return *this;
  }

  VMutableGrid &operator=(VMutableGrid &&other) noexcept
  {
    this->move_from(std::move(other));
    return *this;
  }

#ifdef WITH_OPENVDB
  /**
   * Get access to the internal grid. This invokes undefined behavior if the #is_grid returned
   * false.
   */
  GridType &get_internal_grid() const
  {
    BLI_assert(this->is_grid());
    const CommonVGridInfo info = this->get_impl()->common_info();
    return *const_cast<GridType *>(static_cast<const GridType *>(info.data));
  }
#endif

  /** See #GVMutableGridImpl::try_assign_GVMutableGrid. */
  bool try_assign_GVMutableGrid(GVMutableGrid &vgrid) const
  {
    return this->get_impl()->try_assign_GVMutableGrid(vgrid);
  }

 private:
  /** Utility to get the pointer to the wrapped #VMutableGridImpl. */
  VMutableGridImpl<T> *get_impl() const
  {
    /* This cast is valid by the invariant that a #VMutableGrid->impl_ is always a
     * #VMutableGridImpl. */
    return (VMutableGridImpl<T> *)this->impl_;
  }
};

template<typename T> static constexpr bool is_VGrid_v = false;
template<typename T> static constexpr bool is_VGrid_v<VGrid<T>> = true;

template<typename T> static constexpr bool is_VMutableGrid_v = false;
template<typename T> static constexpr bool is_VMutableGrid_v<VMutableGrid<T>> = true;

}  // namespace blender
