#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_import_stl {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("Path");

  b.add_output<decl::Geometry>("Mesh");
}

static void node_geo_exec(GeoNodeExecParams params)
{

}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IMPORT_STL, "Import STL", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}
