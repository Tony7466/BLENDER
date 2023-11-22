/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_color.hh"
#include "BLI_cpp_type.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_math_base.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

#ifdef WITH_OPENVDB
#  include "openvdb_fwd.hh"
#endif

namespace blender::bke::grid_types {

/* -------------------------------------------------------------------- */
/** \name Grid Type Converter
 *
 * Converter between Blender types and OpenVDB types.
 * \{ */

#ifdef WITH_OPENVDB

namespace detail {

template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_ReinterpretCast {
  static GridValueType single_value_to_grid(const FieldValueType &value)
  {
    return reinterpret_cast<const GridValueType &>(value);
  }

  static FieldValueType single_value_to_attribute(const GridValueType &value)
  {
    return reinterpret_cast<const FieldValueType &>(value);
  }

  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> values)
  {
    return {reinterpret_cast<FieldValueType *>(values.begin()), values.size()};
  }
};

template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_CopyConstruct {
  static GridValueType single_value_to_grid(const FieldValueType &value)
  {
    return GridValueType(value);
  }

  static FieldValueType single_value_to_attribute(const GridValueType &value)
  {
    return FieldValueType(value);
  }

  // XXX can't do this, will only work as a DerivedSpan varray.
  //  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
  //      const MutableSpan<LeafBufferValueType> values)
  //  {
  //    return {reinterpret_cast<FieldValueType *>(values.begin()), values.size()};
  //  }
};

template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_Vector3CopyConstruct {
  using GridBaseType = typename GridValueType::ValueType;
  using FieldBaseType = typename FieldValueType::base_type;

  static GridValueType single_value_to_grid(const FieldValueType &value)
  {
    return GridValueType(GridBaseType(value[0]), GridBaseType(value[1]), GridBaseType(value[2]));
  }

  static FieldValueType single_value_to_attribute(const GridValueType &value)
  {
    return FieldValueType(
        FieldBaseType(value[0]), FieldBaseType(value[1]), FieldBaseType(value[2]));
  }

  // XXX can't do this, will only work as a DerivedSpan varray.
  //  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
  //      const MutableSpan<LeafBufferValueType> values)
  //  {
  //    return {reinterpret_cast<FieldValueType *>(values.begin()), values.size()};
  //  }
};

/* Return default values without trying to convert. */
template<typename FieldValueType, typename GridValueType, typename LeafBufferValueType>
struct GridConverter_Dummy {
  static GridValueType single_value_to_grid(const FieldValueType & /*value*/)
  {
    return GridValueType();
  }

  static FieldValueType single_value_to_attribute(const GridValueType & /*value*/)
  {
    return FieldValueType();
  }

  static MutableSpan<FieldValueType> leaf_buffer_to_varray(
      const MutableSpan<LeafBufferValueType> /*values*/)
  {
    return {};
  }
};

}  // namespace detail

/* Default implementation, only used when grid and attribute types are exactly the same. */
template<typename T> struct GridConverter : public detail::GridConverter_ReinterpretCast<T, T, T> {
  using FieldValueType = T;
};

/* TODO Some base types (double, int3, ...) don't have a CPPType registered yet, and even then
 * there are other layers like attributes, customdata, node sockets, which need to support these
 * types. In the meantime the converters will use smaller base types for fields and attributes. */

template<>
struct GridConverter<float3>
    : public detail::GridConverter_ReinterpretCast<float3, openvdb::Vec3f, openvdb::Vec3f> {
  using FieldValueType = float3;
};

template<>
struct GridConverter<math::Quaternion>
    : public detail::
          GridConverter_ReinterpretCast<math::Quaternion, openvdb::Vec4f, openvdb::Vec4f> {
  using FieldValueType = math::Quaternion;
};

template<>
struct GridConverter<blender::ColorGeometry4f>
    : public detail::
          GridConverter_ReinterpretCast<blender::ColorGeometry4f, openvdb::Vec4f, openvdb::Vec4f> {
  using FieldValueType = blender::ColorGeometry4f;
};

template<>
struct GridConverter<std::string>
    : public detail::GridConverter_Dummy<std::string, openvdb::ValueMask, uint64_t> {
  using FieldValueType = std::string;
};

#endif /* WITH_OPENVDB */

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Grid
 *
 * Grid class template using Blender data types.
 * \{ */

#ifdef WITH_OPENVDB

using SupportedValueTypes = std::tuple<bool,
                                       float,
                                       float2,
                                       float3,
                                       /*float4,*/
                                       int32_t,
                                       int64_t,
                                       int2,
                                       /*int3,*/ /*int4,*/ uint32_t
                                       /*,uint2*/
                                       /*,uint3*/
                                       /*,uint4*/>;

using SupportedPointGridAttributeTypes = openvdb::TypeList<
    openvdb::points::TypedAttributeArray<bool, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<float, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<openvdb::Vec2f, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<openvdb::Vec3f, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<int32_t, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<int64_t, openvdb::points::NullCodec>,
    //    openvdb::points::TypedAttributeArray<openvdb::Vec2I, openvdb::points::NullCodec>,
    openvdb::points::TypedAttributeArray<uint32_t, openvdb::points::NullCodec>>;

namespace detail {

template<typename Func> struct FilterVoidOp {
  Func func;
  void operator()(TypeTag<void> /*type_tag*/) const {}
  template<typename T> void operator()(TypeTag<T> type_tag) const
  {
    func(type_tag);
  }
};

/* Helper function to turn a tuple into a parameter pack by means of the dummy argument. */
template<typename... Types, typename Func>
void field_to_static_type_resolve(std::tuple<Types...> /*dummy*/, const CPPType &type, Func func)
{
  FilterVoidOp<Func> wrapper{func};
  type.to_static_type_tag<Types...>(wrapper);
}

}  // namespace detail

/* Helper function to evaluate a function with a static field type. */
template<typename Func> void field_to_static_type(const CPPType &type, Func func)
{
  detail::field_to_static_type_resolve(grid_types::SupportedValueTypes(), type, func);
}

/**
 * Tree types for commonly used values.
 */
template<typename T> struct FieldValueTreeImpl;

template<> struct FieldValueTreeImpl<bool> {
  using Type = openvdb::BoolTree;
};
template<> struct FieldValueTreeImpl<float> {
  using Type = openvdb::FloatTree;
};
template<> struct FieldValueTreeImpl<float2> {
  using Type = openvdb::Vec2STree;
};
template<> struct FieldValueTreeImpl<float3> {
  using Type = openvdb::Vec3STree;
};
template<> struct FieldValueTreeImpl<double> {
  using Type = openvdb::DoubleTree;
};
template<> struct FieldValueTreeImpl<double3> {
  using Type = openvdb::Vec3DTree;
};
template<> struct FieldValueTreeImpl<int8_t> {
  using Type = openvdb::Int8Grid;
};
template<> struct FieldValueTreeImpl<int32_t> {
  using Type = openvdb::Int32Tree;
};
template<> struct FieldValueTreeImpl<int64_t> {
  using Type = openvdb::Int64Tree;
};
template<> struct FieldValueTreeImpl<int2> {
  using Type = openvdb::Vec2ITree;
};
template<> struct FieldValueTreeImpl<int3> {
  using Type = openvdb::Vec3ITree;
};
template<> struct FieldValueTreeImpl<uint32_t> {
  using Type = openvdb::UInt32Tree;
};
template<> struct FieldValueTreeImpl<ColorGeometry4f> {
  using Type = openvdb::Vec4fTree;
};
template<> struct FieldValueTreeImpl<blender::ColorGeometry4b> {
  using Type = openvdb::UInt32Tree;
};
template<> struct FieldValueTreeImpl<math::Quaternion> {
  using Type = openvdb::Vec4fTree;
};
/* Stub class for string attributes, not supported. */
template<> struct FieldValueTreeImpl<std::string> {
  using Type = openvdb::MaskTree;
};

template<typename T>
using FieldValueGridImpl = openvdb::Grid<typename FieldValueTreeImpl<T>::Type>;

#else /* WITH_OPENVDB */

template<typename T> struct FieldValueGridImpl {
};

#endif /* WITH_OPENVDB */

template<typename T> class FieldValueGrid : public ImplicitSharingMixin {
 public:
  using FieldValueType = T;
  using GridType = FieldValueGridImpl<T>;

 private:
  std::shared_ptr<GridType> grid_;

 public:
  FieldValueGrid() : grid_(nullptr) {}
  FieldValueGrid(const FieldValueGrid<T> &other) : grid_(other.grid) {}
  /* Takes ownership of the grid, which must not be shared. */
  FieldValueGrid(const std::shared_ptr<GridType> &grid) : grid_(grid)
  {
    BLI_assert(grid_);
  }
  virtual ~FieldValueGrid() = default;

  void delete_self() override
  {
    delete this;
  }

  void delete_data_only() override
  {
    this->grid_.reset();
  }

  bool operator==(const FieldValueGrid<T> &other) const
  {
    return this->grid_ == other.grid_;
  }
  bool operator!=(const FieldValueGrid<T> &other) const
  {
    return this->grid_ != other.grid_;
  }

  std::shared_ptr<const GridType> get_grid() const
  {
    return grid_;
  }

  std::shared_ptr<GridType> get_grid_for_write() const
  {
    if (is_mutable()) {
      return grid_;
    }

    return grid_ ? grid_->deepCopy() : nullptr;
  }
};

}  // namespace blender::bke::grid_types

/** \} */

/* -------------------------------------------------------------------- */
/** \name Grid Utility Functions
 * \{ */

namespace blender::bke::grid_types {

template<typename T> bool get_background_value(const FieldValueGrid<T> &grid, T &r_value)
{
#ifdef WITH_OPENVDB
  if constexpr (std::is_same_v<T, std::string>) {
    return false;
  }
  else {
    using Converter = GridConverter<T>;
    r_value = Converter::single_value_to_attribute(grid.get_grid()->background());
    return true;
  }
#else
  return false;
#endif /* WITH_OPENVDB */
}

template<typename T> FieldValueGrid<T> *make_empty_grid(const T background_value)
{
#ifdef WITH_OPENVDB
  using GridType = typename FieldValueGrid<T>::GridType;

  std::shared_ptr<GridType> grid;
  if constexpr (std::is_same_v<T, std::string>) {
    grid = nullptr;
  }
  else {
    using Converter = GridConverter<T>;
    grid = GridType::create(Converter::single_value_to_grid(background_value));
  }
  return new FieldValueGrid<T>(grid);
#else
  return nullptr;
#endif /* WITH_OPENVDB */
}

/** \} */

}  // namespace blender::bke::grid_types
