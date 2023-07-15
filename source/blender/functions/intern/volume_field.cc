/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_cpp_type.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_resource_scope.hh"

#include "FN_volume_field.hh"

#ifdef WITH_OPENVDB

/* -------------------------------------------------------------------- */
/** \name Grid Value Converter
 * \{ */

namespace blender::converter {

template<typename GridValueT> struct GridValueConverter {
  using GridValueType = GridValueT;
  using FieldType = GridValueT;

  static FieldType to_field(const GridValueType &value)
  {
    return value;
  }
  static GridValueType to_grid(const FieldType &value)
  {
    return value;
  }
};

template<> struct GridValueConverter<openvdb::Vec3d> {
  using GridValueType = openvdb::Vec3d;
  using FieldType = blender::double3;

  static FieldType to_field(const GridValueType &value)
  {
    return double3(value.x(), value.y(), value.z());
  }
  static GridValueType to_grid(const FieldType &value)
  {
    return openvdb::Vec3d(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::Vec3i> {
  using GridValueType = openvdb::Vec3i;
  using FieldType = blender::int3;

  static FieldType to_field(const GridValueType &value)
  {
    return int3(value.x(), value.y(), value.z());
  }
  static GridValueType to_grid(const FieldType &value)
  {
    return openvdb::Vec3i(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::Vec3f> {
  using GridValueType = openvdb::Vec3f;
  using FieldType = blender::float3;

  static FieldType to_field(const GridValueType &value)
  {
    return float3(value.x(), value.y(), value.z());
  }
  static GridValueType to_grid(const FieldType &value)
  {
    return openvdb::Vec3f(value.x, value.y, value.z);
  }
};

template<> struct GridValueConverter<openvdb::PointDataIndex32> {
  using GridValueType = openvdb::PointDataIndex32;
  using FieldType = int32_t;

  static FieldType to_field(const GridValueType &value)
  {
    return int32_t(value);
  }
  static GridValueType to_grid(const FieldType &value)
  {
    return GridValueType(value);
  }
};

/* TODO add more */
using SupportedGridValueTypes =
    std::tuple<float, double, openvdb::Int32, openvdb::Vec3f, openvdb::Vec3i>;
using SupportedFieldTypes = std::tuple<float, double, int32_t, float3, int3>;

}  // namespace blender::converter

/** \} */

namespace blender::volume_mask {

bool VolumeMask::is_empty() const
{
  return grid_.empty();
}

int64_t VolumeMask::min_voxel_count() const
{
  return grid_.activeVoxelCount();
}

}  // namespace blender::volume_mask

namespace blender::fn {

VolumeGrid VolumeGrid::create(ResourceScope &scope, const CPPType &type, const int64_t voxel_count)
{
  // std::tuple<> SupportedGridCppTypes;

  /* Note: OpenVDB uses default allocator, there does not seem to be an easy way to utilize the
   * scope's linear allocator here.
   * https://github.com/AcademySoftwareFoundation/openvdb/issues/1002
   */

  openvdb::GridBase::Ptr result;
  // type.to_static_type_tag(

  //  volume_grid_to_static_type_tag(grid_type, [&](auto tag) {
  //  using GridType = typename decltype(tag)::type;
  //  if (grid_template) {
  //    typename GridType::Ptr typed_grid = GridType::create(*grid_template);
  //    result = typed_grid;

  //    volume_grid_to_static_type_tag(BKE_volume_grid_type_openvdb(*grid_template), [&](auto tag)
  //    {
  //      using GridType = typename decltype(tag)::type;
  //      typename GridType::Ptr typed_template = openvdb::GridBase::grid<GridType>(grid_template);
  //      BLI_assert(typed_template != nullptr);
  //      typed_grid->topologyUnion(*typed_template);
  //    });
  //  }
  //  else {
  //    result = GridType::create();
  //  }
  //});
  // if (!result) {
  //  return nullptr;
  //}

  // if (attribute_id.is_anonymous()) {
  //   const AnonymousAttributeID &anonymous_id = attribute_id.anonymous_id();
  //   result->setName(anonymous_id.name());
  // }
  // else {
  //   result->setName(attribute_id.name());
  // }
  // grids.emplace_back(VolumeGrid{result});

  openvdb::GridBase::Ptr &scope_grid = scope.add_value<openvdb::GridBase::Ptr>(std::move(result));
  return VolumeGrid{scope_grid};
}

VolumeGrid VolumeGrid::create(ResourceScope &scope,
                              const CPPType &type,
                              const void *background_value)
{
  return VolumeGrid{};
}

}  // namespace blender::fn

#else

namespace blender::volume_mask {

bool VolumeMask::is_empty() const
{
  return true;
}

int64_t VolumeMask::min_voxel_count() const
{
  return 0;
}

}  // namespace blender::volume_mask

namespace blender::fn {

VolumeGrid VolumeGrid::create(ResourceScope &scope, const CPPType &type, const int64_t voxel_count)
{
  return VolumeGrid{};
}

VolumeGrid VolumeGrid::create(ResourceScope &scope,
                              const CPPType &type,
                              const void *background_value)
{
  return VolumeGrid{};
}

}  // namespace blender::fn

#endif
