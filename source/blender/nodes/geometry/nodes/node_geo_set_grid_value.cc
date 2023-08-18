/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"

#include "BKE_type_conversions.hh"
#include "BKE_volume_attribute.hh"
#include "BKE_volume_geometry.hh"
#include "BKE_volume_openvdb.hh"

#include "BLI_virtual_array.hh"

#include "DNA_volume_types.h"

#include "NOD_add_node_search.hh"
#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

/* XXX bad include, don't care, just make it work. */
#include "intern/volume_grids.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Interpolation.h>
#endif

namespace blender::nodes::node_geo_set_grid_value_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(CTX_N_(BLT_I18NCONTEXT_ID_ID, "Volume"))
      .translation_context(BLT_I18NCONTEXT_ID_ID)
      .supported_type(GeometryComponent::Type::Volume);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Vector>(N_("Value"), "Value_Vector").field_on_all();
  b.add_input<decl::Float>(N_("Value"), "Value_Float").field_on_all();
  b.add_input<decl::Bool>(N_("Value"), "Value_Bool").field_on_all();
  b.add_input<decl::Int>(N_("Value"), "Value_Int").field_on_all();

  b.add_output<decl::Geometry>(CTX_N_(BLT_I18NCONTEXT_ID_ID, "Volume"))
      .translation_context(BLT_I18NCONTEXT_ID_ID)
      .supported_type(GeometryComponent::Type::Volume)
      .propagate_all();
}

static void search_node_add_ops(GatherAddNodeSearchParams &params)
{
  if (!U.experimental.use_new_volume_nodes) {
    return;
  }
  blender::nodes::search_node_add_ops_for_basic_node(params);
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (!U.experimental.use_new_volume_nodes) {
    return;
  }
  blender::nodes::search_link_ops_for_basic_node(params);
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", eUI_Item_Flag(0), "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const eCustomDataType data_type = eCustomDataType(node->custom1);

  bNodeSocket *socket_geometry = static_cast<bNodeSocket *>(node->inputs.first);
  bNodeSocket *socket_selection = socket_geometry->next;
  bNodeSocket *socket_value_vector = socket_selection->next;
  bNodeSocket *socket_value_float = socket_value_vector->next;
  bNodeSocket *socket_value_boolean = socket_value_float->next;
  bNodeSocket *socket_value_int32 = socket_value_boolean->next;

  bke::nodeSetSocketAvailability(ntree, socket_value_vector, data_type == CD_PROP_FLOAT3);
  bke::nodeSetSocketAvailability(ntree, socket_value_float, data_type == CD_PROP_FLOAT);
  bke::nodeSetSocketAvailability(ntree, socket_value_boolean, data_type == CD_PROP_BOOL);
  bke::nodeSetSocketAvailability(ntree, socket_value_int32, data_type == CD_PROP_INT32);
}

#ifdef WITH_OPENVDB

static const StringRefNull get_grid_name(GField &field)
{
  if (const auto *attribute_field_input = dynamic_cast<const AttributeFieldInput *>(&field.node()))
  {
    return attribute_field_input->attribute_name();
  }
  return "";
}

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

static GField get_input_attribute_field(GeoNodeExecParams &params, const eCustomDataType data_type)
{
  switch (data_type) {
    case CD_PROP_FLOAT:
      return params.extract_input<Field<float>>("Value_Float");
    case CD_PROP_FLOAT3:
      return params.extract_input<Field<float3>>("Value_Vector");
    case CD_PROP_BOOL:
      return params.extract_input<Field<bool>>("Value_Bool");
    case CD_PROP_INT32:
      return params.extract_input<Field<int>>("Value_Int");
    default:
      BLI_assert_unreachable();
  }
  return {};
}

static void output_attribute_field(GeoNodeExecParams &params, GField field)
{
  switch (bke::cpp_type_to_custom_data_type(field.cpp_type())) {
    case CD_PROP_FLOAT:
      params.set_output("Value_Float", Field<float>(field));
      break;
    case CD_PROP_FLOAT3:
      params.set_output("Value_Vector", Field<float3>(field));
      break;
    case CD_PROP_BOOL:
      params.set_output("Value_Bool", Field<bool>(field));
      break;
    case CD_PROP_INT32:
      params.set_output("Value_Int", Field<int>(field));
      break;
    default:
      break;
  }
}

// static void set_computed_value(bke::VolumeGeometry &volume,
//                                const GVArray &in_values,
//                                const IndexMask &selection)
//{
//   MutableAttributeAccessor attributes = volume.attributes_for_write();
//
//   // const CPPType &cpptype = bke::volume_grid_type_to_cpp_type(volume.grid->type());
//   // values.varray.type().to_static_type_tag([&](auto tag) {
//   bke::volume_grid_to_static_type_tag(volume.grid->type(), [&](auto tag) {
//     using GridType = typename decltype(tag)::type;
//     using Converter = typename bke::template GridValueConverter<typename GridType::ValueType>;
//     using AttributeType = typename Converter::AttributeType;
//
//     AttributeWriter<AttributeType> writer = attributes.lookup_for_write("value");
//     // VMutableArray<AttributeType> values_typed = writer.varray.typed<AttributeType>();
//     //VArray<AttributeType> in_values_typed = in_values.typed<AttributeType>();
//     //in_values.try_assign_GVMutableArray(writer.varray);
//     //AttributeWriter<int> id_attribute = attributes.lookup_or_add_for_write<int>("id", domain);
//     evaluator.add_with_destination(id_field, id_attribute.varray);
//     evaluator.evaluate();
//     id_attribute.finish();
//   });
//   writer.finish();
// }

// static void set_value_in_volume(GeometrySet &geometry,
//                                 const Field<bool> &selection_field,
//                                 const GField &value_field)
//{
//   VolumeGridVector &grids = *geometry.get_volume_for_write()->runtime.grids;
//   if (grids.empty()) {
//     return;
//   }
//   openvdb::GridBase &grid = *grids.front().grid();
//
//   const eAttrDomain domain = ATTR_DOMAIN_VOXEL;
//   const int domain_size = grid.activeVoxelCount();
//   if (domain_size == 0) {
//     return;
//   }
//
//   bke::volume_grid_to_static_type_tag(BKE_volume_grid_type_openvdb(grid), [&](auto tag) {
//     using GridType = typename decltype(tag)::type;
//     using Converter = typename bke::template GridValueConverter<typename GridType::ValueType>;
//     using AttributeType = typename Converter::AttributeType;
//
//     /* Type mismatch (XXX this will be solved when  grids become generic attributes) */
//     if (value_field.cpp_type() != CPPType::get<AttributeType>()) {
//       return;
//     }
//
//     MutableAttributeAccessor attributes = grids.attributes_for_write();
//     AttributeWriter<AttributeType> value_attribute = attributes.lookup_for_write<AttributeType>(
//         "value");
//     bke::GeometryFieldContext field_context{*geometry.get_component_for_read<VolumeComponent>(),
//                                             domain};
//     fn::FieldEvaluator evaluator{field_context, domain_size};
//
//     evaluator.set_selection(selection_field);
//     evaluator.add_with_destination(value_field, value_attribute.varray);
//     evaluator.evaluate();
//
//     // const IndexMask selection = evaluator.get_evaluated_selection_as_mask();
//     // if (selection.is_empty()) {
//     //   return;
//     // }
//
//     value_attribute.finish();
//   });
// }

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  GeometrySet geometry = params.extract_input<GeometrySet>("Volume");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  GField value_field = get_input_attribute_field(params, data_type);

  if (geometry.has_volume()) {
    // set_value_in_volume(geometry, selection_field, value_field);
  }

  params.set_output("Volume", std::move(geometry));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void register_node()
{
  namespace file_ns = blender::nodes::node_geo_set_grid_value_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SET_GRID_VALUE, "Set Grid Value", NODE_CLASS_GEOMETRY);
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.gather_add_node_search_ops = file_ns::search_node_add_ops;
  ntype.gather_link_search_ops = file_ns::search_link_ops;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(register_node)

}  // namespace blender::nodes::node_geo_set_grid_value_cc
