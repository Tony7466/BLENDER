/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 *
 * A generic virtual array is the same as a virtual array, except for the fact that the data type
 * is only known at runtime.
 */

#include "BLI_generic_array.hh"
#include "BLI_timeit.hh"
#include "BLI_virtual_grid.hh"

namespace blender {

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl and #GVMutableGridImpl.
 * \{ */

class GVArray;
class GVMutableArray;
class GVGrid;
class GVGridImpl;
class GVMutableGrid;
class GVMutableGridImpl;

/* A generically typed version of #VGridImpl. */
class GVGridImpl {
 protected:
  const CPPType *type_;

 public:
  GVGridImpl(const CPPType &type);
  virtual ~GVGridImpl() = default;

  const CPPType &type() const;

  // virtual void get(int64_t index, void *r_value) const;
  // virtual void get_to_uninitialized(int64_t index, void *r_value) const = 0;

  virtual CommonVGridInfo common_info() const
  {
    return {};
  }

  virtual GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const = 0;

  // virtual void materialize(const IndexMask &mask, void *dst) const;
  // virtual void materialize_to_uninitialized(const IndexMask &mask, void *dst) const;

  // virtual void materialize_compressed(const IndexMask &mask, void *dst) const;
  // virtual void materialize_compressed_to_uninitialized(const IndexMask &mask, void *dst) const;

  virtual bool try_assign_VGrid(void * /*vgrid*/) const
  {
    return false;
  }
};

/* A generic version of #VMutableGridImpl. */
class GVMutableGridImpl : public GVGridImpl {
 public:
  GVMutableGridImpl(const CPPType &type) : GVGridImpl(type) {}

  // virtual void set_by_copy(int64_t index, const void *value);
  // virtual void set_by_relocate(int64_t index, void *value);
  // virtual void set_by_move(int64_t index, void *value) = 0;

  // virtual void set_all(const void *src);

  virtual bool try_assign_VMutableGrid(void * /*vgrid*/) const
  {
    return false;
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGrid and #GVMutableGrid
 * \{ */

namespace detail {
struct GVGridAnyExtraInfo {
  const GVGridImpl *(*get_vgrid)(const void *buffer) =
      [](const void * /*buffer*/) -> const GVGridImpl * { return nullptr; };

  template<typename StorageT> static constexpr GVGridAnyExtraInfo get();
};
}  // namespace detail

class GVMutableGrid;

/**
 * Utility class to reduce code duplication between #GVGrid and #GVMutableGrid.
 * It pretty much follows #VGridCommon. Don't use this class outside of this header.
 */
class GVGridCommon {
 public:
#ifdef WITH_OPENVDB
  using GridType = openvdb::GridBase;
#endif

 protected:
  /**
   * See #VGridCommon for more information. The inline buffer is a bit larger here, because
   * generic virtual array implementations often require a bit more space than typed ones.
   */
  using Storage = Any<detail::GVGridAnyExtraInfo, 40, 8>;

  const GVGridImpl *impl_ = nullptr;
  Storage storage_;

 protected:
  GVGridCommon() = default;
  GVGridCommon(const GVGridCommon &other);
  GVGridCommon(GVGridCommon &&other) noexcept;
  GVGridCommon(const GVGridImpl *impl);
  GVGridCommon(std::shared_ptr<const GVGridImpl> impl);
  ~GVGridCommon();

  template<typename ImplT, typename... Args> void emplace(Args &&...args);

  void copy_from(const GVGridCommon &other);
  void move_from(GVGridCommon &&other) noexcept;

  const GVGridImpl *impl_from_storage() const;

 public:
  const CPPType &type() const;
  operator bool() const;

  template<typename T> bool try_assign_VGrid(VGrid<T> &vgrid) const;
  bool may_have_ownership() const
  {
    return impl_->common_info().may_have_ownership;
  }

  /* Leaf size 2^(log2dim * 3) and origin define the transform to local coordinates and index
   * space. */
  GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const;

  // void materialize(void *dst) const;
  // void materialize(const IndexMask &mask, void *dst) const;

  // void materialize_to_uninitialized(void *dst) const;
  // void materialize_to_uninitialized(const IndexMask &mask, void *dst) const;

  // void materialize_compressed(const IndexMask &mask, void *dst) const;
  // void materialize_compressed_to_uninitialized(const IndexMask &mask, void *dst) const;

  CommonVGridInfo common_info() const;

  /**
   * Returns true when the virtual grid is stored as a grid internally.
   */
  bool is_grid() const;
  /**
   * Returns the internally used grid of the virtual grid. This invokes undefined behavior if the
   * virtual grid is not stored as a grid internally.
   */
#ifdef WITH_OPENVDB
  GridType *get_internal_grid() const;
#endif

  /**
   * Returns true when the virtual array returns the same value for every index.
   */
  bool is_single() const;
  /**
   * Copies the value that is used for every element into `r_value`, which is expected to point to
   * initialized memory. This invokes undefined behavior if the virtual array would not return the
   * same value for every index.
   */
  void get_internal_single(void *r_value) const;
  /**
   * Same as `get_internal_single`, but `r_value` points to initialized memory.
   */
  void get_internal_single_to_uninitialized(void *r_value) const;

  // void get(int64_t index, void *r_value) const;
  ///**
  // * Returns a copy of the value at the given index. Usually a typed virtual array should
  // * be used instead, but sometimes this is simpler when only a few indices are needed.
  // */
  // template<typename T> T get(int64_t index) const;
  // void get_to_uninitialized(int64_t index, void *r_value) const;
};

/** Generic version of #VGrid. */
class GVGrid : public GVGridCommon {
 private:
  friend GVMutableGrid;

 public:
#ifdef WITH_OPENVDB
  using GridType = GVGridCommon::GridType;
#endif

 public:
  GVGrid() = default;

  GVGrid(const GVGrid &other);
  GVGrid(GVGrid &&other) noexcept;
  GVGrid(const GVGridImpl *impl);
  GVGrid(std::shared_ptr<const GVGridImpl> impl);

#ifdef WITH_OPENVDB
  GVGrid(vgrid_tag::grid /* tag */, const GridType &grid);
#endif
  GVGrid(vgrid_tag::single_ref /* tag */, const CPPType &type, const void *value);
  GVGrid(vgrid_tag::single /* tag */, const CPPType &type, const void *value);

  template<typename T> GVGrid(const VGrid<T> &vgrid);
  template<typename T> VGrid<T> typed() const;

  template<typename ImplT, typename... Args> static GVGrid For(Args &&...args);

  static GVGrid ForSingle(const CPPType &type, const void *value);
  static GVGrid ForSingleRef(const CPPType &type, const void *value);
  static GVGrid ForSingleDefault(const CPPType &type);
#ifdef WITH_OPENVDB
  static GVGrid ForGrid(const GridType &grid);
  static GVGrid ForEmpty(const CPPType &type);
#endif

  GVGrid &operator=(const GVGrid &other);
  GVGrid &operator=(GVGrid &&other) noexcept;

  const GVGridImpl *get_implementation() const
  {
    return impl_;
  }
};

/** Generic version of #VMutableGrid. */
class GVMutableGrid : public GVGridCommon {
 public:
#ifdef WITH_OPENVDB
  using GridType = GVGridCommon::GridType;
#endif

 public:
  GVMutableGrid() = default;
  GVMutableGrid(const GVMutableGrid &other);
  GVMutableGrid(GVMutableGrid &&other) noexcept;
  GVMutableGrid(GVMutableGridImpl *impl);
  GVMutableGrid(std::shared_ptr<GVMutableGridImpl> impl);

  template<typename T> GVMutableGrid(const VMutableGrid<T> &vgrid);
  template<typename T> VMutableGrid<T> typed() const;

  template<typename ImplT, typename... Args> static GVMutableGrid For(Args &&...args);

#ifdef WITH_OPENVDB
  static GVMutableGrid ForGrid(GridType &grid);
#endif

  operator GVGrid() const &;
  operator GVGrid() &&noexcept;

  GVMutableGrid &operator=(const GVMutableGrid &other);
  GVMutableGrid &operator=(GVMutableGrid &&other) noexcept;

#ifdef WITH_OPENVDB
  GridType *get_internal_grid() const;
#endif

  template<typename T> bool try_assign_VMutableGrid(VMutableGrid<T> &vgrid) const;

  // void set_by_copy(int64_t index, const void *value);
  // void set_by_move(int64_t index, void *value);
  // void set_by_relocate(int64_t index, void *value);

  // void fill(const void *value);
  ///**
  // * Copy the values from the source buffer to all elements in the virtual array.
  // */
  // void set_all(const void *src);

  GVMutableGridImpl *get_implementation() const;

 private:
  GVMutableGridImpl *get_impl() const;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Conversions between generic and typed virtual arrays.
 * \{ */

/* Used to convert a typed virtual grid into a generic one. */
template<typename T> class GVGridImpl_For_VGrid : public GVGridImpl {
 protected:
  VGrid<T> vgrid_;

 public:
  GVGridImpl_For_VGrid(VGrid<T> vgrid) : GVGridImpl(CPPType::get<T>()), vgrid_(std::move(vgrid)) {}

 protected:
  CommonVGridInfo common_info() const override
  {
    return vgrid_.common_info();
  }

  bool try_assign_VGrid(void *vgrid) const override
  {
    *(VGrid<T> *)vgrid = vgrid_;
    return true;
  }

  GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
    return vgrid_.get_varray_for_leaf(log2dim, origin);
  }
};

/* Used to convert any generic virtual grid into a typed one. */
template<typename T> class VGridImpl_For_GVGrid : public VGridImpl<T> {
 protected:
  GVGrid vgrid_;

 public:
  VGridImpl_For_GVGrid(GVGrid vgrid) : vgrid_(std::move(vgrid))
  {
    BLI_assert(vgrid_);
    BLI_assert(vgrid_.type().template is<T>());
  }

  VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
    return vgrid_.get_varray_for_leaf(log2dim, origin).typed<T>();
  }

 protected:
  CommonVGridInfo common_info() const override
  {
    return vgrid_.common_info();
  }

  bool try_assign_GVGrid(GVGrid &vgrid) const override
  {
    vgrid = vgrid_;
    return true;
  }
};

/* Used to convert any typed virtual mutable grid into a generic one. */
template<typename T> class GVMutableGridImpl_For_VMutableGrid : public GVMutableGridImpl {
 protected:
  VMutableGrid<T> vgrid_;

 public:
  GVMutableGridImpl_For_VMutableGrid(VMutableGrid<T> vgrid)
      : GVMutableGridImpl(CPPType::get<T>()), vgrid_(std::move(vgrid))
  {
  }

  GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
    return vgrid_.get_varray_for_leaf(log2dim, origin);
  }

 protected:
  CommonVGridInfo common_info() const override
  {
    return vgrid_.common_info();
  }

  bool try_assign_VGrid(void *vgrid) const override
  {
    *(VGrid<T> *)vgrid = vgrid_;
    return true;
  }

  bool try_assign_VMutableGrid(void *vgrid) const override
  {
    *(VMutableGrid<T> *)vgrid = vgrid_;
    return true;
  }
};

/* Used to convert an generic mutable virtual grid into a typed one. */
template<typename T> class VMutableGridImpl_For_GVMutableGrid : public VMutableGridImpl<T> {
 protected:
  GVMutableGrid vgrid_;

 public:
  VMutableGridImpl_For_GVMutableGrid(GVMutableGrid vgrid) : vgrid_(vgrid)
  {
    BLI_assert(vgrid_);
    BLI_assert(vgrid_.type().template is<T>());
  }

  VArray<T> get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
    return vgrid_.get_varray_for_leaf(log2dim, origin).typed<T>();
  }

 private:
  CommonVGridInfo common_info() const override
  {
    return vgrid_.common_info();
  }

  bool try_assign_GVGrid(GVGrid &vgrid) const override
  {
    vgrid = vgrid_;
    return true;
  }

  bool try_assign_GVMutableGrid(GVMutableGrid &vgrid) const override
  {
    vgrid = vgrid_;
    return true;
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl_For_GSpan.
 * \{ */

#ifdef WITH_OPENVDB
class GVGridImpl_For_Grid : public GVMutableGridImpl {
 public:
  using GridType = openvdb::GridBase;

 protected:
  const GridType *grid_ = nullptr;

 public:
  GVGridImpl_For_Grid(const GridType &grid);

  GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override
  {
    return volume::get_varray_for_leaf(log2dim, origin, *grid_);
  }

 protected:
  GVGridImpl_For_Grid(const CPPType &type) : GVMutableGridImpl(type) {}

 public:
  CommonVGridInfo common_info() const override
  {
    return CommonVGridInfo{CommonVGridInfo::Type::Grid, true, grid_};
  }
};

class GVGridImpl_For_Grid_final final : public GVGridImpl_For_Grid {
 public:
  using GVGridImpl_For_Grid::GVGridImpl_For_Grid;

 private:
  CommonVGridInfo common_info() const override
  {
    return CommonVGridInfo(CommonVGridInfo::Type::Grid, false, grid_);
  }
};

template<> inline constexpr bool is_trivial_extended_v<GVGridImpl_For_Grid> = true;
#endif /* WITH_OPENVDB */

/** \} */

/* -------------------------------------------------------------------- */
/** \name #GVGridImpl_For_SingleValueRef.
 * \{ */

class GVGridImpl_For_SingleValueRef : public GVGridImpl {
 protected:
  const void *value_ = nullptr;

 public:
  GVGridImpl_For_SingleValueRef(const CPPType &type, const void *value)
      : GVGridImpl(type), value_(value)
  {
  }

  GVArray get_varray_for_leaf(uint32_t log2dim, const int3 &origin) const override;

 protected:
  GVGridImpl_For_SingleValueRef(const CPPType &type) : GVGridImpl(type) {}

  CommonVGridInfo common_info() const override;
};

class GVGridImpl_For_SingleValueRef_final final : public GVGridImpl_For_SingleValueRef {
 public:
  using GVGridImpl_For_SingleValueRef::GVGridImpl_For_SingleValueRef;

 private:
  CommonVGridInfo common_info() const override;
};

template<> inline constexpr bool is_trivial_extended_v<GVGridImpl_For_SingleValueRef_final> = true;

/** \} */

/* -------------------------------------------------------------------- */
/** \name Inline methods for #GVGridImpl.
 * \{ */

inline GVGridImpl::GVGridImpl(const CPPType &type) : type_(&type) {}

inline const CPPType &GVGridImpl::type() const
{
  return *type_;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Inline methods for #GVMutableGridImpl.
 * \{ */

template<typename T>
inline bool GVMutableGrid::try_assign_VMutableGrid(VMutableGrid<T> &vgrid) const
{
  BLI_assert(impl_->type().is<T>());
  return this->get_impl()->try_assign_VMutableGrid(&vgrid);
}

inline GVMutableGridImpl *GVMutableGrid::get_impl() const
{
  return const_cast<GVMutableGridImpl *>(static_cast<const GVMutableGridImpl *>(impl_));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Inline methods for #GVGridCommon.
 * \{ */

template<typename ImplT, typename... Args> inline void GVGridCommon::emplace(Args &&...args)
{
  static_assert(std::is_base_of_v<GVGridImpl, ImplT>);
  if constexpr (std::is_copy_constructible_v<ImplT> && Storage::template is_inline_v<ImplT>) {
    impl_ = &storage_.template emplace<ImplT>(std::forward<Args>(args)...);
  }
  else {
    std::shared_ptr<const GVGridImpl> ptr = std::make_shared<ImplT>(std::forward<Args>(args)...);
    impl_ = &*ptr;
    storage_ = std::move(ptr);
  }
}

template<typename T> inline bool GVGridCommon::try_assign_VGrid(VGrid<T> &vgrid) const
{
  BLI_assert(impl_->type().is<T>());
  return impl_->try_assign_VGrid(&vgrid);
}

inline const CPPType &GVGridCommon::type() const
{
  return impl_->type();
}

inline GVGridCommon::operator bool() const
{
  return impl_ != nullptr;
}

inline CommonVGridInfo GVGridCommon::common_info() const
{
  return impl_->common_info();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Inline methods for #GVGrid.
 * \{ */

#ifdef WITH_OPENVDB
inline GVGrid::GVGrid(vgrid_tag::grid /* tag */, const GridType &grid)
{
  /* Use const-cast because the underlying virtual grid implementation is shared between const
   * and non const data. */
  GridType &mutable_grid = const_cast<GridType &>(grid);
  this->emplace<GVGridImpl_For_Grid_final>(mutable_grid);
}
#endif

inline GVGrid::GVGrid(vgrid_tag::single_ref /* tag */, const CPPType &type, const void *value)
{
  this->emplace<GVGridImpl_For_SingleValueRef_final>(type, value);
}

namespace detail {
template<typename StorageT> constexpr GVGridAnyExtraInfo GVGridAnyExtraInfo::get()
{
  static_assert(std::is_base_of_v<GVGridImpl, StorageT> ||
                is_same_any_v<StorageT, const GVGridImpl *, std::shared_ptr<const GVGridImpl>>);

  if constexpr (std::is_base_of_v<GVGridImpl, StorageT>) {
    return {[](const void *buffer) {
      return static_cast<const GVGridImpl *>((const StorageT *)buffer);
    }};
  }
  else if constexpr (std::is_same_v<StorageT, const GVGridImpl *>) {
    return {[](const void *buffer) { return *(const StorageT *)buffer; }};
  }
  else if constexpr (std::is_same_v<StorageT, std::shared_ptr<const GVGridImpl>>) {
    return {[](const void *buffer) { return ((const StorageT *)buffer)->get(); }};
  }
  else {
    BLI_assert_unreachable();
    return {};
  }
}
}  // namespace detail

template<typename ImplT, typename... Args> inline GVGrid GVGrid::For(Args &&...args)
{
  static_assert(std::is_base_of_v<GVGridImpl, ImplT>);
  GVGrid vgrid;
  vgrid.template emplace<ImplT>(std::forward<Args>(args)...);
  return vgrid;
}

template<typename T> inline GVGrid::GVGrid(const VGrid<T> &vgrid)
{
  if (!vgrid) {
    return;
  }
  const CommonVGridInfo info = vgrid.common_info();
  if (info.type == CommonVGridInfo::Type::Single) {
    *this = GVGrid::ForSingle(CPPType::get<T>(), info.data);
    return;
  }
  /* Need to check for ownership, because otherwise the referenced data can be destructed when
   * #this is destructed. */
  if (info.type == CommonVGridInfo::Type::Grid && !info.may_have_ownership) {
#ifdef WITH_OPENVDB
    *this = GVGrid::ForGrid(*static_cast<const GridType *>(info.data));
#endif
    return;
  }
  if (vgrid.try_assign_GVGrid(*this)) {
    return;
  }
  *this = GVGrid::For<GVGridImpl_For_VGrid<T>>(vgrid);
}

template<typename T> inline VGrid<T> GVGrid::typed() const
{
  if (!*this) {
    return {};
  }
  BLI_assert(impl_->type().is<T>());
  const CommonVGridInfo info = this->common_info();
  if (info.type == CommonVGridInfo::Type::Single) {
    return VGrid<T>::ForSingle(*static_cast<const T *>(info.data));
  }
  /* Need to check for ownership, because otherwise the referenced data can be destructed when
   * #this is destructed. */
  if (info.type == CommonVGridInfo::Type::Grid && !info.may_have_ownership) {
#ifdef WITH_OPENVDB
    using ResultGridType = typename VGrid<T>::GridType;
    return VGrid<T>::ForGrid(*static_cast<const ResultGridType *>(info.data));
#else
    return {};
#endif
  }
  VGrid<T> vgrid;
  if (this->try_assign_VGrid(vgrid)) {
    return vgrid;
  }
  return VGrid<T>::template For<VGridImpl_For_GVGrid<T>>(*this);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Inline methods for #GVMutableGrid.
 * \{ */

template<typename ImplT, typename... Args> inline GVMutableGrid GVMutableGrid::For(Args &&...args)
{
  static_assert(std::is_base_of_v<GVMutableGridImpl, ImplT>);
  GVMutableGrid vgrid;
  vgrid.emplace<ImplT>(std::forward<Args>(args)...);
  return vgrid;
}

template<typename T> inline GVMutableGrid::GVMutableGrid(const VMutableGrid<T> &vgrid)
{
  if (!vgrid) {
    return;
  }
  const CommonVGridInfo info = vgrid.common_info();
  if (info.type == CommonVGridInfo::Type::Grid && !info.may_have_ownership) {
#ifdef WITH_OPENVDB
    *this = GVMutableGrid::ForGrid(*static_cast<GridType *>(const_cast<void *>(info.data)));
#endif
    return;
  }
  if (vgrid.try_assign_GVMutableGrid(*this)) {
    return;
  }
  *this = GVMutableGrid::For<GVMutableGridImpl_For_VMutableGrid<T>>(vgrid);
}

template<typename T> inline VMutableGrid<T> GVMutableGrid::typed() const
{
  if (!*this) {
    return {};
  }
  BLI_assert(this->type().is<T>());
  const CommonVGridInfo info = this->common_info();
  if (info.type == CommonVGridInfo::Type::Grid && !info.may_have_ownership) {
#ifdef WITH_OPENVDB
    return VMutableGrid<T>::ForGrid(const_cast<GridType *>(static_cast<const T *>(info.data)));
#else
    return {};
#endif
  }
  VMutableGrid<T> vgrid;
  if (this->try_assign_VMutableGrid(vgrid)) {
    return vgrid;
  }
  return VMutableGrid<T>::template For<VMutableGridImpl_For_GVMutableGrid<T>>(*this);
}

/** \} */

}  // namespace blender
