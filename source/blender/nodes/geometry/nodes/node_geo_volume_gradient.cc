/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"

#include "BKE_type_conversions.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_virtual_array.hh"

#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Interpolation.h>
#endif

namespace blender::nodes::node_geo_volume_gradient_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(CTX_N_(BLT_I18NCONTEXT_ID_ID, "Volume"))
      .translation_context(BLT_I18NCONTEXT_ID_ID)
      .supported_type(GeometryComponent::Type::Volume);
  b.add_input<decl::Float>(N_("Value")).supports_field();

  b.add_output<decl::Geometry>(CTX_N_(BLT_I18NCONTEXT_ID_ID, "Volume"))
      .translation_context(BLT_I18NCONTEXT_ID_ID)
      .supported_type(GeometryComponent::Type::Volume);
  b.add_output<decl::Vector>(N_("Vector")).dependent_field({1});
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (!U.experimental.use_new_volume_nodes) {
    return;
  }
  blender::nodes::search_link_ops_for_basic_node(params);
}

#ifdef WITH_OPENVDB

static const blender::CPPType *vdb_grid_type_to_cpp_type(const VolumeGridType grid_type)
{
  switch (grid_type) {
    case VOLUME_GRID_FLOAT:
      return &CPPType::get<float>();
    case VOLUME_GRID_VECTOR_FLOAT:
      return &CPPType::get<float3>();
    case VOLUME_GRID_INT:
      return &CPPType::get<int>();
    case VOLUME_GRID_BOOLEAN:
      return &CPPType::get<bool>();
    default:
      break;
  }
  return nullptr;
}

template<typename GridT>
void sample_grid(openvdb::GridBase::ConstPtr base_grid,
                 const Span<float3> positions,
                 const IndexMask &mask,
                 GMutableSpan dst,
                 const GeometryNodeSampleVolumeInterpolationMode interpolation_mode)
{
  using ValueT = typename GridT::ValueType;
  using AccessorT = typename GridT::ConstAccessor;
  const typename GridT::ConstPtr grid = openvdb::gridConstPtrCast<GridT>(base_grid);
  AccessorT accessor = grid->getConstAccessor();

  auto sample_data = [&](auto sampler) {
    mask.foreach_index([&](const int64_t i) {
      const float3 &pos = positions[i];
      ValueT value = sampler.wsSample(openvdb::Vec3R(pos.x, pos.y, pos.z));

      /* Special case for vector. */
      if constexpr (std::is_same_v<GridT, openvdb::VectorGrid>) {
        openvdb::Vec3f vec = static_cast<openvdb::Vec3f>(value);
        dst.typed<float3>()[i] = float3(vec.asV());
      }
      else {
        dst.typed<ValueT>()[i] = value;
      }
    });
  };

  switch (interpolation_mode) {
    case GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_TRILINEAR: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::BoxSampler> sampler(
          accessor, grid->transform());
      sample_data(sampler);
      break;
    }
    case GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_TRIQUADRATIC: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::QuadraticSampler> sampler(
          accessor, grid->transform());
      sample_data(sampler);
      break;
    }
    case GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_NEAREST:
    default: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::PointSampler> sampler(
          accessor, grid->transform());
      sample_data(sampler);
      break;
    }
  }
}

class SampleVolumeFunction : public mf::MultiFunction {
  openvdb::GridBase::ConstPtr base_grid_;
  VolumeGridType grid_type_;
  GeometryNodeSampleVolumeInterpolationMode interpolation_mode_;
  mf::Signature signature_;

 public:
  SampleVolumeFunction(openvdb::GridBase::ConstPtr base_grid,
                       GeometryNodeSampleVolumeInterpolationMode interpolation_mode)
      : base_grid_(std::move(base_grid)), interpolation_mode_(interpolation_mode)
  {
    grid_type_ = BKE_volume_grid_type_openvdb(*base_grid_);
    const CPPType *grid_cpp_type = vdb_grid_type_to_cpp_type(grid_type_);
    BLI_assert(grid_cpp_type != nullptr);
    mf::SignatureBuilder builder{"Sample Volume", signature_};
    builder.single_input<float3>("Position");
    builder.single_output("Value", *grid_cpp_type);
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArraySpan<float3> positions = params.readonly_single_input<float3>(0, "Position");
    GMutableSpan dst = params.uninitialized_single_output(1, "Value");

    switch (grid_type_) {
      case VOLUME_GRID_FLOAT:
        sample_grid<openvdb::FloatGrid>(base_grid_, positions, mask, dst, interpolation_mode_);
        break;
      case VOLUME_GRID_INT:
        sample_grid<openvdb::Int32Grid>(base_grid_, positions, mask, dst, interpolation_mode_);
        break;
      case VOLUME_GRID_BOOLEAN:
        sample_grid<openvdb::BoolGrid>(base_grid_, positions, mask, dst, interpolation_mode_);
        break;
      case VOLUME_GRID_VECTOR_FLOAT:
        sample_grid<openvdb::VectorGrid>(base_grid_, positions, mask, dst, interpolation_mode_);
        break;
      default:
        BLI_assert_unreachable();
        break;
    }
  }
};

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams /*params*/)
{
#ifdef WITH_OPENVDB
  // GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");
  // if (!geometry_set.has_volume()) {
  //   params.set_default_remaining_outputs();
  //   return;
  // }
  // const NodeGeometrySampleVolume &storage = node_storage(params.node());
  // const eCustomDataType output_field_type = eCustomDataType(storage.grid_type);
  // auto interpolation_mode =
  // GeometryNodeSampleVolumeInterpolationMode(storage.interpolation_mode);

  // GField grid_field = get_input_attribute_field(params, output_field_type);
  // const StringRefNull grid_name = get_grid_name(grid_field);
  // if (grid_name == "") {
  //   params.error_message_add(NodeWarningType::Error, TIP_("Grid name needs to be specified"));
  //   params.set_default_remaining_outputs();
  //   return;
  // }

  // const VolumeComponent *component = geometry_set.get_component_for_read<VolumeComponent>();
  // const Volume *volume = component->get_for_read();
  // BKE_volume_load(volume, DEG_get_bmain(params.depsgraph()));
  // const VolumeGrid *volume_grid = BKE_volume_grid_find_for_read(volume, grid_name.c_str());
  // if (volume_grid == nullptr) {
  //   params.set_default_remaining_outputs();
  //   return;
  // }
  // openvdb::GridBase::ConstPtr base_grid = BKE_volume_grid_openvdb_for_read(volume, volume_grid);
  // const VolumeGridType grid_type = BKE_volume_grid_type_openvdb(*base_grid);

  ///* Check that the grid type is supported. */
  // const CPPType *grid_cpp_type = vdb_grid_type_to_cpp_type(grid_type);
  // if (grid_cpp_type == nullptr) {
  //   params.set_default_remaining_outputs();
  //   params.error_message_add(NodeWarningType::Error, TIP_("The grid type is unsupported"));
  //   return;
  // }

  ///* Use to the Nearest Neighbor sampler for Bool grids (no interpolation). */
  // if (grid_type == VOLUME_GRID_BOOLEAN &&
  //     interpolation_mode != GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_NEAREST)
  //{
  //   interpolation_mode = GEO_NODE_SAMPLE_VOLUME_INTERPOLATION_MODE_NEAREST;
  // }

  // Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  // auto fn = std::make_shared<SampleVolumeFunction>(std::move(base_grid), interpolation_mode);
  // auto op = FieldOperation::Create(std::move(fn), {position_field});
  // GField output_field = GField(std::move(op));

  // output_field = bke::get_implicit_type_conversions().try_convert(
  //     output_field, *bke::custom_data_type_to_cpp_type(output_field_type));

  // output_attribute_field(params, std::move(output_field));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_register()
{
  namespace file_ns = blender::nodes::node_geo_volume_gradient_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_VOLUME_GRADIENT, "Gradient", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.gather_link_search_ops = file_ns::search_link_ops;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_volume_gradient_cc
