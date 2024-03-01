/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Composite.h>
#  include <openvdb/tools/GridTransformer.h>
#endif

#include "BKE_volume_grid.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sdf_grid_boolean_cc {

enum class Operation {
  Union = 0,
  Difference = 1,
  Intersection = 2,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("SDF Grid 1").hide_value();
  b.add_input<decl::Float>("SDF Grid 2").hide_value();
  b.add_output<decl::Float>("SDF Grid").hide_value();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int16_t(Operation::Union);
}

static openvdb::FloatGrid &get_resampled_grid(bke::VolumeGrid<float> &volume_grid,
                                              bke::VolumeTreeAccessToken &tree_token,
                                              const openvdb::math::Transform::Ptr &transform,
                                              openvdb::FloatGrid::Ptr &storage)
{
  const openvdb::FloatGrid &grid = volume_grid.grid(tree_token);
  if (grid.transform() == *transform) {
    return volume_grid.grid_for_write(tree_token);
  }

  storage = openvdb::FloatGrid::create();
  storage->setTransform(transform);

  /* TODO: #doResampleToMatch when the transform is affine and non-scaled may be faster. */
  openvdb::tools::resampleToMatch<openvdb::tools::BoxSampler>(grid, *storage);
  openvdb::tools::pruneLevelSet(storage->tree());

  return *storage;
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const Operation operation = Operation(params.node().custom1);
  bke::VolumeGrid<float> grid_a = params.extract_input<bke::VolumeGrid<float>>("SDF Grid 1");
  bke::VolumeGrid<float> grid_b = params.extract_input<bke::VolumeGrid<float>>("SDF Grid 2");
  if (!grid_a || !grid_b) {
    switch (operation) {
      case Operation::Union:
        params.set_output("SDF Grid", grid_a ? std::move(grid_a) : std::move(grid_b));
        break;
      case Operation::Difference:
        if (grid_a) {
          params.set_output("SDF Grid", std::move(grid_a));
        }
        else {
          params.set_default_remaining_outputs();
        }
        break;
      case Operation::Intersection:
        params.set_default_remaining_outputs();
    }
    return;
  }

  bke::VolumeTreeAccessToken tree_token_a;
  bke::VolumeTreeAccessToken tree_token_b;
  openvdb::FloatGrid &vdb_a = grid_a.grid_for_write(tree_token_a);

  openvdb::FloatGrid::Ptr resampled_storage;
  openvdb::FloatGrid &vdb_b = get_resampled_grid(
      grid_b, tree_token_b, vdb_a.transformPtr(), resampled_storage);

  switch (operation) {
    case Operation::Union:
      openvdb::tools::csgUnion(vdb_a, vdb_b);
      break;
    case Operation::Difference:
      openvdb::tools::csgDifference(vdb_a, vdb_b);
      break;
    case Operation::Intersection:
      openvdb::tools::csgIntersection(vdb_a, vdb_b);
      break;
  }

  params.set_output("SDF Grid", std::move(grid_a));
#else
  node_geo_exec_with_missing_openvdb(params);
#endif
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem items[] = {
      {int(Operation::Union), "UNION", 0, "Union", ""},
      {int(Operation::Difference), "DIFFERENCE", 0, "Difference", ""},
      {int(Operation::Intersection), "INTERSECT", 0, "Intersect", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "operation",
                    "Operation",
                    "",
                    items,
                    NOD_inline_enum_accessors(custom1),
                    int(Operation::Union));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SDF_GRID_BOOLEAN, "SDF Grid Boolean", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sdf_grid_boolean_cc
