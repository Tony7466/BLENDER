/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"

#include "BKE_volume.h"
#include "node_geometry_util.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Interpolation.h>
#endif

namespace blender::nodes::node_geo_sample_volume_cc {

NODE_STORAGE_FUNCS(NodeGeometrySampleVolume)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(CTX_N_(BLT_I18NCONTEXT_ID_ID, "Volume"))
      .translation_context(BLT_I18NCONTEXT_ID_ID)
      .supported_type(GEO_COMPONENT_TYPE_VOLUME);

  b.add_input<decl::Vector>(N_("Grid")).field_on_all().hide_value();
  b.add_input<decl::Float>(N_("Grid"), "Grid_001").field_on_all().hide_value();
  b.add_input<decl::Bool>(N_("Grid"), "Grid_002").field_on_all().hide_value();
  b.add_input<decl::Int>(N_("Grid"), "Grid_003").field_on_all().hide_value();

  b.add_input<decl::Vector>(N_("Position"));

  b.add_output<decl::Vector>(N_("Value"));
  b.add_output<decl::Float>(N_("Value"), "Value_001");
  b.add_output<decl::Bool>(N_("Value"), "Value_002");
  b.add_output<decl::Int>(N_("Value"), "Value_003");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "grid_type", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySampleVolume *data = MEM_cnew<NodeGeometrySampleVolume>(__func__);
  data->grid_type = CD_PROP_FLOAT;
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const NodeGeometrySampleVolume &storage = node_storage(*node);
  const eCustomDataType grid_type = eCustomDataType(storage.grid_type);

  bNodeSocket *socket_value_geometry = static_cast<bNodeSocket *>(node->inputs.first);
  bNodeSocket *socket_value_vector = socket_value_geometry->next;
  bNodeSocket *socket_value_float = socket_value_vector->next;
  bNodeSocket *socket_value_boolean = socket_value_float->next;
  bNodeSocket *socket_value_int32 = socket_value_boolean->next;

  nodeSetSocketAvailability(ntree, socket_value_vector, grid_type == CD_PROP_FLOAT3);
  nodeSetSocketAvailability(ntree, socket_value_float, grid_type == CD_PROP_FLOAT);
  nodeSetSocketAvailability(ntree, socket_value_boolean, grid_type == CD_PROP_BOOL);
  nodeSetSocketAvailability(ntree, socket_value_int32, grid_type == CD_PROP_INT32);

  bNodeSocket *out_socket_value_vector = static_cast<bNodeSocket *>(node->outputs.first);
  bNodeSocket *out_socket_value_float = out_socket_value_vector->next;
  bNodeSocket *out_socket_value_boolean = out_socket_value_float->next;
  bNodeSocket *out_socket_value_int32 = out_socket_value_boolean->next;

  nodeSetSocketAvailability(ntree, out_socket_value_vector, grid_type == CD_PROP_FLOAT3);
  nodeSetSocketAvailability(ntree, out_socket_value_float, grid_type == CD_PROP_FLOAT);
  nodeSetSocketAvailability(ntree, out_socket_value_boolean, grid_type == CD_PROP_BOOL);
  nodeSetSocketAvailability(ntree, out_socket_value_int32, grid_type == CD_PROP_INT32);
}

#ifdef WITH_OPENVDB

static const StringRefNull get_grid_name(GField &field)
{
  StringRefNull grid_name;

  if (field.node().field_inputs() != nullptr) {
    const std::shared_ptr<const fn::FieldInputs> &field_inputs = field.node().field_inputs();

    for (const fn::FieldInput &field_input : field_inputs->deduplicated_nodes) {
      if (const auto *attribute_field_input = dynamic_cast<const AttributeFieldInput *>(
              &field_input))
      {
        grid_name = attribute_field_input->attribute_name();
        break;
      }
    }
  }
  return grid_name;
}

static StringRefNull identifier_suffix(eCustomDataType data_type)
{
  switch (data_type) {
    case CD_PROP_FLOAT:
      return "_001";
    case CD_PROP_BOOL:
      return "_002";
    case CD_PROP_INT32:
      return "_003";
    case CD_PROP_FLOAT3:
      return "";
    default:
      BLI_assert_unreachable();
      return "";
  }
}

template<typename GridT>
typename GridT::ValueType sample_grid(openvdb::GridBase::ConstPtr &base_grid, float3 &position)
{
  using ValueT = typename GridT::ValueType;
  const GridT::ConstPtr grid = openvdb::gridConstPtrCast<GridT>(base_grid);

  GridT::ConstAccessor accessor = grid->getConstAccessor();
  openvdb::tools::GridSampler<GridT::ConstAccessor, openvdb::tools::BoxSampler> sampler(
      accessor, grid->transform());

  return sampler.wsSample(openvdb::Vec3R(position.x, position.y, position.z));
}

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Volume");
  float3 position = params.extract_input<float3>("Position");

  const NodeGeometrySampleVolume &storage = node_storage(params.node());
  const eCustomDataType grid_type = eCustomDataType(storage.grid_type);
  const std::string output_identifier = "Value" + identifier_suffix(grid_type);
  const std::string input_identifier = "Grid" + identifier_suffix(grid_type);
  GField field;

  switch (grid_type) {
    case CD_PROP_FLOAT:
      field = params.get_input<Field<float>>(input_identifier);
      break;
    case CD_PROP_FLOAT3:
      field = params.get_input<Field<float3>>(input_identifier);
      break;
    case CD_PROP_BOOL:
      field = params.get_input<Field<bool>>(input_identifier);
      break;
    case CD_PROP_INT32:
      field = params.get_input<Field<int>>(input_identifier);
      break;
    default:
      break;
  }

  /* Get the name of the AttributeFieldInput. */
  const StringRefNull grid_name = get_grid_name(field);

  if (grid_name == nullptr) {
    params.error_message_add(NodeWarningType::Error, TIP_("Grid name needs to be specified"));
    params.set_default_remaining_outputs();
    return;
  }
  const VolumeComponent *component = geometry_set.get_component_for_read<VolumeComponent>();
  const Volume *volume = component->get_for_read();
  BKE_volume_load(volume, DEG_get_bmain(params.depsgraph()));

  const VolumeGrid *volume_grid = BKE_volume_grid_find_for_read(volume, grid_name.c_str());
  if (volume_grid == nullptr) {
    params.error_message_add(NodeWarningType::Error, TIP_("Grid not found in the volume"));
    params.set_default_remaining_outputs();
    return;
  }
  openvdb::GridBase::ConstPtr base_grid = BKE_volume_grid_openvdb_for_read(volume, volume_grid);

  if (base_grid->isType<openvdb::FloatGrid>() && grid_type == CD_PROP_FLOAT) {
    params.set_output(output_identifier, sample_grid<openvdb::FloatGrid>(base_grid, position));
    return;
  }
  if (base_grid->isType<openvdb::Int32Grid>() && grid_type == CD_PROP_INT32) {
    params.set_output(output_identifier, sample_grid<openvdb::Int32Grid>(base_grid, position));
    return;
  }
  if (base_grid->isType<openvdb::BoolGrid>() && grid_type == CD_PROP_BOOL) {
    params.set_output(output_identifier, sample_grid<openvdb::BoolGrid>(base_grid, position));
    return;
  }
  if (base_grid->isType<openvdb::VectorGrid>() && grid_type == CD_PROP_FLOAT3) {
    openvdb::Vec3f vector = sample_grid<openvdb::VectorGrid>(base_grid, position);
    params.set_output(output_identifier, float3(vector.asV()));
    return;
  }
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Wrong grid type selected or type not supported"));
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

}  // namespace blender::nodes::node_geo_sample_volume_cc

void register_node_type_geo_sample_volume()
{
  namespace file_ns = blender::nodes::node_geo_sample_volume_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_VOLUME, "Sample Volume", NODE_CLASS_CONVERTER);
  node_type_storage(
      &ntype, "NodeGeometrySampleVolume", node_free_standard_storage, node_copy_standard_storage);
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
