/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_customdata.hh"
#include "BKE_type_conversions.hh"
#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_index_mask.hh"
#include "BLI_virtual_array.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/Interpolation.h>
#endif

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sample_grid_cc {

enum class InterpolationMode {
  Nearest = 0,
  TriLinear = 1,
  TriQuadratic = 2,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const eCustomDataType data_type = eCustomDataType(node->custom1);

  b.add_input(data_type, "Grid").hide_value();
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);

  b.add_output(data_type, "Value").dependent_field({1});
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_volume_nodes) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "interpolation_mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
  node->custom2 = int16_t(InterpolationMode::TriLinear);
}

#ifdef WITH_OPENVDB

template<typename T>
void sample_grid(const bke::OpenvdbGridType<T> &grid,
                 const InterpolationMode interpolation,
                 const Span<float3> positions,
                 const IndexMask &mask,
                 MutableSpan<T> dst)
{
  using GridType = bke::OpenvdbGridType<T>;
  using GridValueT = typename GridType::ValueType;
  using AccessorT = typename GridType::ConstAccessor;
  using TraitsT = typename bke::VolumeGridTraits<T>;
  AccessorT accessor = grid.getConstAccessor();

  auto sample_data = [&](auto sampler) {
    mask.foreach_index([&](const int64_t i) {
      const float3 &pos = positions[i];
      GridValueT value = sampler.wsSample(openvdb::Vec3R(pos.x, pos.y, pos.z));
      dst[i] = TraitsT::to_blender(value);
    });
  };

  /* Use to the Nearest Neighbor sampler for Bool grids (no interpolation). */
  InterpolationMode real_interpolation = interpolation;
  if constexpr (std::is_same_v<T, bool>) {
    real_interpolation = InterpolationMode::Nearest;
  }
  switch (real_interpolation) {
    case InterpolationMode::TriLinear: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::BoxSampler> sampler(accessor,
                                                                                 grid.transform());
      sample_data(sampler);
      break;
    }
    case InterpolationMode::TriQuadratic: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::QuadraticSampler> sampler(
          accessor, grid.transform());
      sample_data(sampler);
      break;
    }
    case InterpolationMode::Nearest: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::PointSampler> sampler(
          accessor, grid.transform());
      sample_data(sampler);
      break;
    }
  }
}

template<typename Fn> void convert_to_static_type(const VolumeGridType type, const Fn &fn)
{
  switch (type) {
    case VOLUME_GRID_BOOLEAN:
      fn(bool());
      break;
    case VOLUME_GRID_FLOAT:
      fn(float());
      break;
    case VOLUME_GRID_INT:
      fn(int());
      break;
    case VOLUME_GRID_MASK:
      fn(bool());
      break;
    case VOLUME_GRID_VECTOR_FLOAT:
      fn(float3());
      break;
    default:
      break;
  }
}

class SampleGridFunction : public mf::MultiFunction {
  bke::GVolumeGrid grid_;
  InterpolationMode interpolation_;
  mf::Signature signature_;

 public:
  SampleGridFunction(bke::GVolumeGrid grid, InterpolationMode interpolation)
      : grid_(std::move(grid)), interpolation_(interpolation)
  {
    BLI_assert(grid_);

    const std::optional<eCustomDataType> cd_type = bke::volume_grid_type_to_custom_data_type(
        grid_->grid_type());
    const CPPType *cpp_type = bke::custom_data_type_to_cpp_type(*cd_type);
    mf::SignatureBuilder builder{"Sample Volume", signature_};
    builder.single_input<float3>("Position");
    builder.single_output("Value", *cpp_type);
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArraySpan<float3> positions = params.readonly_single_input<float3>(0, "Position");
    GMutableSpan dst = params.uninitialized_single_output(1, "Value");

    bke::VolumeTreeAccessToken tree_token;
    convert_to_static_type(grid_->grid_type(), [&](auto dummy) {
      using T = decltype(dummy);
      sample_grid<T>(
          grid_.typed<T>().grid(tree_token), interpolation_, positions, mask, dst.typed<T>());
    });
  }
};

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const bNode &node = params.node();
  const eCustomDataType data_type = eCustomDataType(node.custom1);
  const InterpolationMode interpolation = InterpolationMode(node.custom2);

  Field<float3> position = params.extract_input<Field<float3>>("Position");
  bke::GVolumeGrid grid = params.extract_input<bke::GVolumeGrid>("Grid");
  if (!grid) {
    params.set_default_remaining_outputs();
    return;
  }

  auto fn = std::make_shared<SampleGridFunction>(std::move(grid), interpolation);
  auto op = FieldOperation::Create(std::move(fn), {std::move(position)});

  const bke::DataTypeConversions &conversions = bke::get_implicit_type_conversions();
  const CPPType &output_type = *bke::custom_data_type_to_cpp_type(data_type);
  const GField output_field = conversions.try_convert(fn::GField(std::move(op)), output_type);
  params.set_output("Value", std::move(output_field));

#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem interpolation_mode_items[] = {
      {int(InterpolationMode::Nearest), "NEAREST", 0, "Nearest Neighbor", ""},
      {int(InterpolationMode::TriLinear), "TRILINEAR", 0, "Trilinear", ""},
      {int(InterpolationMode::TriQuadratic), "TRIQUADRATIC", 0, "Triquadratic", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem grid_type_items[] = {
      {CD_PROP_FLOAT, "FLOAT", 0, "Float", "Floating-point value"},
      {CD_PROP_FLOAT3, "FLOAT_VECTOR", 0, "Vector", "3D vector with floating-point values"},
      {CD_PROP_INT32, "INT", 0, "Integer", "32-bit integer"},
      {CD_PROP_BOOL, "BOOLEAN", 0, "Boolean", "True or false"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of grid to sample data from",
                    grid_type_items,
                    NOD_inline_enum_accessors(custom1),
                    CD_PROP_FLOAT);

  RNA_def_node_enum(srna,
                    "interpolation_mode",
                    "Interpolation Mode",
                    "How to interpolate the values between neighboring voxels",
                    interpolation_mode_items,
                    NOD_inline_enum_accessors(custom2),
                    int(InterpolationMode::TriLinear));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_GRID, "Sample Grid", NODE_CLASS_CONVERTER);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sample_grid_cc
