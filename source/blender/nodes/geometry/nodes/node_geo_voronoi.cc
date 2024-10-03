#include "BLI_array_utils.hh"
#include "BLI_task.hh"
#include "BLI_vector_set.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_attribute_math.hh"
#include "BKE_customdata.hh"
#include "BKE_deform.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"
#include "BKE_mesh_runtime.hh"

#include "GEO_mesh_selection.hh"
#include "GEO_randomize.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

#include "voro++.hh"

namespace blender::nodes::node_geo_voronoi {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(("Sites"));
  b.add_input<decl::Geometry>(("Domain"));
  // b.add_input<decl::Int>(N_("Group ID")).field_source();
  b.add_output<decl::Geometry>("Voronoi");
  // b.add_output<decl::Int>(N_("Group ID")).field_source();
}

static void node_init(bNodeTree */*tree*/, bNode *node)
{
  NodeGeometryVoronoi *data = MEM_cnew<NodeGeometryVoronoi>(__func__);
  data->sites = 100;
  node->storage = data;
}

NODE_STORAGE_FUNCS(NodeGeometryVoronoi)

static void node_update(bNodeTree *ntree, bNode *node)
{
  const NodeGeometryVoronoi &storage = node_storage(*node);

  bNodeSocket *out_socket_geometry = (bNodeSocket *)node->outputs.first;
  bNodeSocket *out_socket_group_id = out_socket_geometry->next;

  // Stupid feature for the sake of the example: When there are too many
  // olives, we no longer output the fields!
  // nodeSetSocketAvailability(ntree, out_socket_base, storage.olive_count < 25);
  // nodeSetSocketAvailability(ntree, out_socket_olives, storage.olive_count < 25);
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "sites", UI_ITEM_NONE, "", ICON_NONE);
}

static Mesh *compute_voronoi(const GeometrySet& sites, const GeometrySet& domain){
  // Very simple pizza: the base is a disc and olives are little quads

  // (i) compute element counts
  // int olive_count = 3;
  
  // int edge_count = 32 + olive_count * 4;
  // int corner_count = 32 + olive_count * 4;
  // int face_count = 32 + olive_count;

  // (ii) allocate memory
  const MeshComponent *mesh_comp = sites.get_component<MeshComponent>();
  const Mesh *site_mesh = mesh_comp->get();
  const Span<float3> positions = site_mesh->vert_positions();

  // (iii) fill in element buffers
  // MutableSpan<MVert> verts{mesh->mvert, mesh->totvert};
  // MutableSpan<MLoop> loops{mesh->mloop, mesh->totloop};
  // MutableSpan<MEdge> edges{mesh->medge, mesh->totedge};
  // MutableSpan<MPoly> polys{mesh->mpoly, mesh->totpoly};
  const double x_min=-6.5,x_max=6.5;
  const double y_min=-6.5,y_max=6.5;
  const double z_min=0,z_max=18.5;

  // Set the computational grid size
  const int n_x=7,n_y=7,n_z=14;

  // Create a container with the geometry given above, and make it
	// non-periodic in each of the three coordinates. Allocate space for
	// eight particles within each computational block.
	voro::container con(x_min,x_max,y_min,y_max,z_min,z_max,n_x,n_y,n_z,
			false,false,false,8);
  int i = 0;
  for (float3 p : positions){
    con.put(i, p[0], p[1], p[2]);
    i++;
  }

  voro::voronoicell c;
  double *pp;
  voro::c_loop_all vl(con);
  int vert_count = 0;
  if(vl.start()) do if(con.compute_cell(c,vl)) {
    pp=con.p[vl.ijk]+con.ps*vl.q;
    // c.draw_gnuplot(*pp,pp[1],pp[2],con.fp);
    vert_count += c.p;
  } while(vl.inc());


	// Add a cylindrical wall to the container
	voro::wall_cylinder cyl(0,0,0,0,0,1,6);
	con.add_wall(cyl);
  
	// Import the particles from a file
	// con.import("pack_cylinder");

	// // Output the particle positions in POV-Ray format
	// con.draw_particles_pov("cylinder_p.pov");

	// // Output the Voronoi cells in POV-Ray format
	// con.draw_cells_pov("cylinder_v.pov");
  // [...]
  Mesh *mesh = BKE_mesh_new_nomain(vert_count, 0,0,0);

  return mesh;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  // We first retrieve the property (olive count) and the input socket (radius)
  const NodeGeometryVoronoi &storage = node_storage(params.node());
  const int sites = storage.sites;
  const GeometrySet site_geometry = params.extract_input<GeometrySet>("Sites");
  const GeometrySet domain = params.extract_input<GeometrySet>("Domain");
  // Then we create the mesh (let's put it in a separate function)
  Mesh *voronoi = compute_voronoi(site_geometry, domain);

  // We build a geometry set to wrap the mesh and set it as the output value
  params.set_output("Voronoi", GeometrySet::from_mesh(voronoi));
}

static void node_register()
{

  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_VORONOI, "Voronoi", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  ntype.geometry_node_execute = node_geo_exec;

  blender::bke::node_type_storage(
      &ntype, "NodeGeometryVoronoi", node_free_standard_storage, node_copy_standard_storage);
  ntype.draw_buttons = node_layout;
  blender::bke::node_register_type(&ntype);

  // node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}