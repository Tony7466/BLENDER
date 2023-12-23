/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.hh"
#include "BKE_editmesh.hh"
#include "BKE_subdiv.hh"
#include "BKE_subdiv_mesh.hh"
#include "ED_mesh.hh"

#include "UI_resources.hh"

#include "GEO_randomize.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_subdivide_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Int>("Level").default_value(1).min(0).max(6);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

#ifdef WITH_OPENSUBDIV
static Mesh *simple_subdivide_mesh(const Mesh &mesh, const int level, const Field<bool> &selection_field)
{
  const bke::MeshFieldContext context{mesh, AttrDomain::Edge};
  FieldEvaluator evaluator{context, mesh.edges_num};
  evaluator.set_selection(selection_field);
  evaluator.evaluate();
  const IndexMask selection = evaluator.get_evaluated_selection_as_mask();
  if (selection.is_empty()) {
//    return;
  }

  if(selection.size() != mesh.edges_num) {  
    Mesh* copy = BKE_mesh_copy_for_eval(&mesh);
    const BMAllocTemplate allocsize = BMALLOC_TEMPLATE_FROM_ME(copy);
    BMeshCreateParams bm_create_params{};
    bm_create_params.use_toolflags = true;
    BMesh *bm = BM_mesh_create(&allocsize, &bm_create_params);
    BMeshFromMeshParams mesh_to_bm_params{};
    mesh_to_bm_params.calc_face_normal = true;
    mesh_to_bm_params.calc_vert_normal = true;
    BM_mesh_bm_from_me(bm, copy, &mesh_to_bm_params);

    BMEditMesh *em = BKE_editmesh_create(bm);
    copy->edit_mesh = em;

    BM_mesh_elem_table_ensure(em->bm, BM_ALL);
    BM_mesh_elem_toolflags_ensure(em->bm);
    BMO_push(em->bm, nullptr);

    BM_mesh_elem_hflag_disable_all(em->bm, BM_ALL, BM_ELEM_SELECT, false);
    selection.foreach_index([&](int x) {
      BLI_assert(x<em->bm->totedge);
      BM_edge_select_set(em->bm, BM_edge_at_index(em->bm, x), true);
    });
    BM_mesh_esubdivide(em->bm,
                         BM_ELEM_SELECT,
                         1,
                         SUBD_FALLOFF_LIN,
                         false,
                         0.0/*fractal*/,
                         0.0/*along_normal*/,
                         (1<<level)-1,
                         SUBDIV_SELECT_ORIG,
                         SUBD_CORNER_STRAIGHT_CUT,
                         false,
                         true,
                         false,
                         0);
    EDBMUpdate_Params params{};
    params.calc_looptris = true;
    params.calc_normals = false;
    params.is_destructive = true;
    EDBM_update(copy, &params);

    BMeshToMeshParams bm_to_mesh_params{};
    bm_to_mesh_params.calc_object_remap = false;
    copy = BKE_mesh_from_bmesh_nomain(bm, &bm_to_mesh_params, &mesh);
    BMO_pop(em->bm);
    BKE_editmesh_free_data(em);
    MEM_freeN(em);

    return copy;
  }

  /* Initialize mesh settings. */
  SubdivToMeshSettings mesh_settings;
  mesh_settings.resolution = (1 << level) + 1;
  mesh_settings.use_optimal_display = false;

  /* Initialize subdivision settings. */
  SubdivSettings subdiv_settings;
  subdiv_settings.is_simple = true;
  subdiv_settings.is_adaptive = false;
  subdiv_settings.use_creases = false;
  subdiv_settings.level = 1;
  subdiv_settings.vtx_boundary_interpolation = BKE_subdiv_vtx_boundary_interpolation_from_subsurf(
      0);
  subdiv_settings.fvar_linear_interpolation = BKE_subdiv_fvar_interpolation_from_uv_smooth(0);

  /* Apply subdivision from mesh. */
  Subdiv *subdiv = BKE_subdiv_new_from_mesh(&subdiv_settings, &mesh);
  if (!subdiv) {
    return nullptr;
  }

  Mesh *result = BKE_subdiv_to_mesh(subdiv, &mesh_settings, &mesh);

  BKE_subdiv_free(subdiv);

  geometry::debug_randomize_mesh_order(result);
  return result;
}
#endif /* WITH_OPENSUBDIV */

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
#ifdef WITH_OPENSUBDIV
  /* See CCGSUBSURF_LEVEL_MAX for max limit. */
  const int level = clamp_i(params.extract_input<int>("Level"), 0, 11);
  if (level == 0) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      geometry_set.replace_mesh(simple_subdivide_mesh(*mesh, level, selection_field));
    }
  });
#else
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenSubdiv"));
#endif
  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SUBDIVIDE_MESH, "Subdivide Mesh", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_subdivide_cc
