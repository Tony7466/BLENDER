/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright Blender Foundation */

/** \file
 * \ingroup edrend
 *
 * This file implements shader node previews which rely on a structure owned by each SpaceNode.
 * We take advantage of the RenderResult available as ImBuf images to store a Render for every
 * viewed nested node tree present in a SpaceNode. The computation is initiated at the moment of
 * drawing nodes overlays. One render is started for the current nodetree, having a ViewLayer
 * associated with each previewed node.
 *
 * We separate the previewed nodes in two categories: the shader ones and the non-shader ones.
 * - for non-shader nodes, we use AOVs(Arbitrary Output Variable) which highly speed up the
 * rendering process by rendering every non-shader nodes at the same time. They are rendered in the
 * first ViewLayer.
 * - for shader nodes, we render them each in a different ViewLayer, by routing the node to the
 * output of the material in the preview scene.
 *
 * At the moment of drawing, we take the Render of the viewed node tree and extract the ImBuf of
 * the wanted viewlayer/pass for each previewed node.
 */

#include "BLO_readfile.h"

#include "DNA_camera_types.h"
#include "DNA_collection_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_world_types.h"

#include "BKE_colortools.h"
#include "BKE_compute_contexts.hh"
#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_icons.h"
#include "BKE_idprop.h"
#include "BKE_image.h"
#include "BKE_layer.h"
#include "BKE_lib_id.h"
#include "BKE_light.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.h"
#include "BKE_scene.h"

#include "BIF_glutil.h"
#include "IMB_imbuf_types.h"

#include "RE_engine.h"
#include "RE_pipeline.h"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_datafiles.h"
#include "ED_node_preview.hh"
#include "ED_render.h"
#include "ED_screen.h"
#include "node_intern.hh"

#ifndef NDEBUG
/* Used for database init assert(). */
#  include "BLI_threads.h"
#endif

/* -------------------------------------------------------------------- */
/** \name Local Structs
 * \{ */
using NodeSocketPair = std::pair<bNode *, bNodeSocket *>;

struct ShaderNodesPreviewJob {
  NestedTreePreviews *tree_previews;
  Material *mat_orig;
  /* Listbase containing bNodeTreePath * guiding to the viewed nodetree of mat_orig. */
  ListBase *treepath;
  Scene *scene;
  bool *stop;
  bool *do_update;

  Material *mat_copy;
  bNode *mat_output_copy;
  NodeSocketPair mat_displacement_copy;
  ListBase treepath_copy;
  blender::Vector<bNode *> AOV_nodes;
  blender::Vector<bNode *> shader_nodes;

  Main *bmain;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Compute Context functions
 * \{ */

using namespace blender;
static void ensure_nodetree_previews(const bContext *C,
                                     NestedTreePreviews *tree_previews,
                                     Material *material,
                                     ListBase *treepath);

static std::optional<ComputeContextHash> get_compute_context_hash_for_node_editor(
    const SpaceNode &snode)
{
  Vector<const bNodeTreePath *> treepath = snode.treepath;
  if (treepath.is_empty()) {
    return std::nullopt;
  }
  if (treepath.size() == 1) {
    /* Top group. */
    ComputeContextHash hash;
    hash.v1 = hash.v2 = 0;
    return hash;
  }
  ComputeContextBuilder compute_context_builder;
  for (const int i : treepath.index_range().drop_back(1)) {
    /* The tree path contains the name of the node but not its ID. */
    const bNode *node = nodeFindNodebyName(treepath[i]->nodetree, treepath[i + 1]->node_name);
    if (node == nullptr) {
      /* The current tree path is invalid, probably because some parent group node has been
       * deleted. */
      return std::nullopt;
    }
    compute_context_builder.push<bke::NodeGroupComputeContext>(*node);
  }
  return compute_context_builder.hash();
}

/*
 * This function returns the `NestedTreePreviews *` for the nodetree shown in the SpaceNode.
 * This is the first function in charge of the previews by calling `ensure_nodetree_previews`.
 */
NestedTreePreviews *ED_spacenode_get_nested_previews(const bContext *C, SpaceNode *sn)
{
  if (GS(sn->id->name) != ID_MA) {
    return nullptr;
  }
  NestedTreePreviews *tree_previews = nullptr;
  if (auto hash = get_compute_context_hash_for_node_editor(*sn)) {
    tree_previews = sn->runtime->tree_previews_per_context.lookup_or_add_cb(*hash, [&]() {
      tree_previews = static_cast<NestedTreePreviews *>(
          MEM_callocN(sizeof(NestedTreePreviews), __func__));
      tree_previews->pr_size = U.node_preview_res;
      return tree_previews;
    });
    Material *ma = reinterpret_cast<Material *>(sn->id);

    ensure_nodetree_previews(C, tree_previews, ma, &sn->treepath);
  }
  return tree_previews;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Preview scene
 * \{ */

static Material *duplicate_material(const Material *mat)
{
  if (mat == nullptr) {
    return nullptr;
  }

  Material *ma_copy = reinterpret_cast<Material *>(
      BKE_id_copy_ex(nullptr,
                     &mat->id,
                     nullptr,
                     LIB_ID_CREATE_LOCAL | LIB_ID_COPY_LOCALIZE | LIB_ID_COPY_NO_ANIMDATA));
  return ma_copy;
}

static Scene *preview_prepare_scene(const Main *bmain,
                                    const Scene *scene,
                                    Main *pr_main,
                                    Material *matcopy)
{
  Scene *sce;

  memcpy(pr_main->filepath, BKE_main_blendfile_path(bmain), sizeof(pr_main->filepath));

  if (pr_main == nullptr) {
    return nullptr;
  }
  sce = static_cast<Scene *>(pr_main->scenes.first);
  if (sce == nullptr) {
    return nullptr;
  }

  ViewLayer *view_layer = static_cast<ViewLayer *>(sce->view_layers.first);

  /* Only enable the combined render-pass. */
  view_layer->passflag = SCE_PASS_COMBINED;
  view_layer->eevee.render_passes = 0;

  /* This flag tells render to not execute depsgraph or F-Curves etc. */
  sce->r.scemode |= R_BUTS_PREVIEW;
  STRNCPY(sce->r.engine, scene->r.engine);

  sce->r.color_mgt_flag = scene->r.color_mgt_flag;
  BKE_color_managed_display_settings_copy(&sce->display_settings, &scene->display_settings);

  BKE_color_managed_view_settings_free(&sce->view_settings);
  BKE_color_managed_view_settings_copy(&sce->view_settings, &scene->view_settings);

  sce->r.alphamode = R_ADDSKY;

  sce->r.cfra = scene->r.cfra;

  /* Setup the world. */
  sce->world = ED_preview_prepare_world(pr_main, sce, scene->world, ID_MA, PR_BUTS_RENDER);

  BLI_addtail(&pr_main->materials, matcopy);
  sce->world->use_nodes = false;
  sce->world->horr = 0.05f;
  sce->world->horg = 0.05f;
  sce->world->horb = 0.05f;

  ED_preview_set_visibility(
      pr_main, sce, view_layer, static_cast<ePreviewType>(matcopy->pr_type), PR_BUTS_RENDER);

  BKE_view_layer_synced_ensure(sce, view_layer);
  LISTBASE_FOREACH (Base *, base, BKE_view_layer_object_bases_get(view_layer)) {
    if (base->object->id.name[2] == 'p') {
      if (OB_TYPE_SUPPORT_MATERIAL(base->object->type)) {
        /* Don't use BKE_object_material_assign, it changed mat->id.us, which shows in the UI. */
        Material ***matar = BKE_object_material_array_p(base->object);
        int actcol = max_ii(base->object->actcol - 1, 0);

        if (matar && actcol < base->object->totcol) {
          (*matar)[actcol] = matcopy;
        }
      }
      else if (base->object->type == OB_LAMP) {
        base->flag |= BASE_ENABLED_AND_MAYBE_VISIBLE_IN_VIEWPORT;
      }
    }
  }

  return sce;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Preview rendering
 * \{ */

/* Return the socket used for previewing the node (should probably follow more precise rules). */
static bNodeSocket *node_find_preview_socket(const bNode *node)
{
  LISTBASE_FOREACH (bNodeSocket *, socket, &node->outputs) {
    if (socket->is_visible()) {
      return socket;
    }
  }
  return nullptr;
}

static bool node_use_aov(const bNode *node)
{
  bNodeSocket *socket = node_find_preview_socket(node);
  return socket != nullptr && socket->type != SOCK_SHADER;
}

static ImBuf *get_image_from_viewlayer_and_pass(RenderResult *rr,
                                                const char *layer_name,
                                                const char *pass_name)
{
  RenderLayer *rl;
  if (layer_name) {
    rl = RE_GetRenderLayer(rr, layer_name);
  }
  else {
    rl = static_cast<RenderLayer *>(rr->layers.first);
  }
  if (rl == nullptr) {
    return nullptr;
  }
  RenderPass *rp;
  if (pass_name) {
    rp = RE_pass_find_by_name(rl, pass_name, nullptr);
  }
  else {
    rp = static_cast<RenderPass *>(rl->passes.first);
  }
  ImBuf *ibuf = rp ? rp->ibuf : nullptr;
  return ibuf;
}

/* `ED_node_release_preview_ibuf` should be called after this. */
ImBuf *ED_node_preview_acquire_ibuf(bNodeTree *ntree,
                                    NestedTreePreviews *tree_previews,
                                    const bNode *node)
{
  if (tree_previews->previews_render == nullptr) {
    return nullptr;
  }

  RenderResult *rr = RE_AcquireResultRead(tree_previews->previews_render);
  if (rr == nullptr) {
    return nullptr;
  }
  ImBuf *image = nullptr;
  if (node_use_aov(node)) {
    image = get_image_from_viewlayer_and_pass(rr, nullptr, node->name);
  }
  else {
    image = get_image_from_viewlayer_and_pass(rr, node->name, nullptr);
  }
  if (image == nullptr) {
    ntree->preview_refresh_state++;
  }
  return image;
}

void ED_node_release_preview_ibuf(NestedTreePreviews *tree_previews)
{
  if (tree_previews == nullptr || tree_previews->previews_render == nullptr) {
    return;
  }
  RE_ReleaseResult(tree_previews->previews_render);
}

/* Get a link to the node outside the nested nodegroups by creating a new output socket for each
 * nested nodegroup. To do so we cover all nested nodetrees starting from the farthest, and
 * update the `nested_node` pointer to the current nodegroup instance used for linking. We stop
 * before getting to the main nodetree because the output type is different. */
static void connect_nested_node_to_node(const Vector<const bNodeTreePath *> &treepath,
                                        bNode *nested_node,
                                        bNodeSocket *nested_socket,
                                        bNode *final_node,
                                        bNodeSocket *final_socket)
{
  for (bNodeTreePath *path = (bNodeTreePath *)treepath.last(); path->prev != nullptr;
       path = path->prev)
  {
    bNodeTree *nested_nt = path->nodetree;
    bNode *output_node = nullptr;
    for (bNode *iter_node : nested_nt->all_nodes()) {
      if (iter_node->is_group_output() && iter_node->flag & NODE_DO_OUTPUT) {
        output_node = iter_node;
        break;
      }
    }
    if (output_node == nullptr) {
      output_node = nodeAddStaticNode(nullptr, nested_nt, NODE_GROUP_OUTPUT);
      output_node->flag |= NODE_DO_OUTPUT;
    }

    ntreeAddSocketInterface(nested_nt, SOCK_OUT, nested_socket->idname, nested_node->name);
    BKE_ntree_update_main_tree(G.pr_main, nested_nt, nullptr);
    bNodeSocket *out_socket = blender::bke::node_find_enabled_input_socket(*output_node,
                                                                           nested_node->name);

    nodeAddLink(nested_nt, nested_node, nested_socket, output_node, out_socket);

    nodeSetActive(nested_nt, output_node);
    BKE_ntree_update_main_tree(G.pr_main, nested_nt, nullptr);
    BKE_ntree_update_main_tree(G.pr_main, path->prev->nodetree, nullptr);

    /* Change the `nested_node` pointer to the nested nodegroup instance node. The tree path
     * contains the name of the instance node but not its ID. */
    nested_node = nodeFindNodebyName(path->prev->nodetree, path->node_name);

    /* Now use the newly created socket of the nodegroup as previewing socket of the nodegroup
     * instance node. */
    nested_socket = (bNodeSocket *)nested_node->outputs.last;
  }

  nodeAddLink(treepath.first()->nodetree, nested_node, nested_socket, final_node, final_socket);
}

/* Connect the node to the output of the first nodetree from `treepath`. Last element of `treepath`
 * should be the path to the node's nodetree */
static void connect_node_to_surface_output(Vector<const bNodeTreePath *> treepath,
                                           bNode *node,
                                           bNode *output_node)
{
  bNodeSocket *out_surface_socket = nullptr;
  bNodeTree *main_nt = treepath.first()->nodetree;
  bNodeSocket *node_preview_socket = node_find_preview_socket(node);
  if (node_preview_socket == nullptr) {
    return;
  }
  /* Ensure output is usable. */
  out_surface_socket = nodeFindSocket(output_node, SOCK_IN, "Surface");
  if (out_surface_socket->link) {
    /* make sure no node is already wired to the output before wiring */
    nodeRemLink(main_nt, out_surface_socket->link);
  }

  connect_nested_node_to_node(
      treepath, node, node_preview_socket, output_node, out_surface_socket);
  BKE_ntree_update_main_tree(G.pr_main, main_nt, nullptr);
}

/* Connect the nodes to some aov nodes located in the first nodetree from `treepath`. Last element
 * of `treepath` should be the path to the nodes nodetree. */
static void connect_nodes_to_aovs(const Vector<const bNodeTreePath *> &treepath,
                                  const Vector<bNode *> &nodes)
{
  if (nodes.size() == 0) {
    return;
  }
  bNodeTree *main_nt = treepath.first()->nodetree;
  for (bNode *node : nodes) {
    bNodeSocket *node_preview_socket = node_find_preview_socket(node);

    bNode *aov_node = nodeAddStaticNode(nullptr, main_nt, SH_NODE_OUTPUT_AOV);
    strcpy(reinterpret_cast<NodeShaderOutputAOV *>(aov_node->storage)->name, node->name);
    if (node_preview_socket == nullptr) {
      continue;
    }
    bNodeSocket *aov_socket = nodeFindSocket(aov_node, SOCK_IN, "Color");

    connect_nested_node_to_node(treepath, node, node_preview_socket, aov_node, aov_socket);
  }
  BKE_ntree_update_main_tree(G.pr_main, main_nt, nullptr);
}

/* Called by renderer, checks job stops. */
static bool nodetree_previews_break(void *spv)
{
  ShaderNodesPreviewJob *job_data = static_cast<ShaderNodesPreviewJob *>(spv);

  return *(job_data->stop);
}

static bool prepare_viewlayer_update(void *pvl_data, ViewLayer *vl)
{
  bNode *node = nullptr;
  ShaderNodesPreviewJob *job_data = static_cast<ShaderNodesPreviewJob *>(pvl_data);
  for (bNode *node_iter : job_data->shader_nodes) {
    if (STREQ(vl->name, node_iter->name)) {
      node = node_iter;
      break;
    }
  }
  if (node == nullptr) {
    /* The AOV layer is the default `ViewLayer` of the scene(which should be the first one). */
    return job_data->AOV_nodes.size() > 0 && !vl->prev;
  }
  Vector<const bNodeTreePath *> treepath = job_data->treepath_copy;
  bNodeSocket *displacement_socket = nodeFindSocket(
      job_data->mat_output_copy, SOCK_IN, "Displacement");
  if (job_data->mat_displacement_copy.first != nullptr && displacement_socket->link == nullptr) {
    nodeAddLink(treepath.first()->nodetree,
                job_data->mat_displacement_copy.first,
                job_data->mat_displacement_copy.second,
                job_data->mat_output_copy,
                displacement_socket);
  }
  connect_node_to_surface_output(treepath, node, job_data->mat_output_copy);
  return true;
}

/* Called by renderer, refresh the UI. */
static void all_nodes_preview_update(void *npv, RenderResult * /*rr*/, struct rcti * /*rect*/)
{
  ShaderNodesPreviewJob *job_data = static_cast<ShaderNodesPreviewJob *>(npv);
  *job_data->do_update = true;
}

static void preview_render(ShaderNodesPreviewJob &job_data)
{
  /* Get the stuff from the builtin preview dbase. */
  Scene *sce = preview_prepare_scene(job_data.bmain, job_data.scene, G.pr_main, job_data.mat_copy);
  if (sce == nullptr) {
    return;
  }
  Vector<const bNodeTreePath *> treepath = job_data.treepath_copy;

  /* Disconnect shader, volume and displacement from the material output. */
  bNodeSocket *surface_socket = nodeFindSocket(job_data.mat_output_copy, SOCK_IN, "Surface");
  bNodeSocket *volume_socket = nodeFindSocket(job_data.mat_output_copy, SOCK_IN, "Volume");
  bNodeSocket *disp_socket = nodeFindSocket(job_data.mat_output_copy, SOCK_IN, "Displacement");
  if (surface_socket->link != nullptr) {
    nodeRemLink(treepath.first()->nodetree, surface_socket->link);
  }
  if (volume_socket->link != nullptr) {
    nodeRemLink(treepath.first()->nodetree, volume_socket->link);
  }
  if (disp_socket->link != nullptr) {
    job_data.mat_displacement_copy = std::make_pair(disp_socket->link->fromnode,
                                                    disp_socket->link->fromsock);
    nodeRemLink(treepath.first()->nodetree, disp_socket->link);
  }

  /* AOV nodes are rendered in the first RenderLayer so we route them now. */
  connect_nodes_to_aovs(treepath, job_data.AOV_nodes);

  /* Create the AOV passes for the viewlayer. */
  ViewLayer *AOV_layer = static_cast<ViewLayer *>(sce->view_layers.first);
  for (bNode *node : job_data.shader_nodes) {
    ViewLayer *vl = BKE_view_layer_add(sce, node->name, AOV_layer, VIEWLAYER_ADD_COPY);
    strcpy(vl->name, node->name);
  }
  for (bNode *node : job_data.AOV_nodes) {
    ViewLayerAOV *aov = BKE_view_layer_add_aov(AOV_layer);
    strcpy(aov->name, node->name);
  }
  sce->r.xsch = job_data.tree_previews->pr_size;
  sce->r.ysch = job_data.tree_previews->pr_size;
  sce->r.size = 100;

  if (job_data.tree_previews->previews_render == nullptr) {
    char name[32];
    SNPRINTF(name, "Preview %p", &job_data.tree_previews);
    job_data.tree_previews->previews_render = RE_NewRender(name);
  }
  Render *re = job_data.tree_previews->previews_render;

  /* `sce->r` gets copied in RE_InitState. */
  sce->r.scemode &= ~(R_MATNODE_PREVIEW | R_TEXNODE_PREVIEW);
  sce->r.scemode &= ~R_NO_IMAGE_LOAD;

  sce->display.render_aa = SCE_DISPLAY_AA_SAMPLES_8;

  RE_display_update_cb(re, &job_data, all_nodes_preview_update);
  RE_test_break_cb(re, &job_data, nodetree_previews_break);
  RE_prepare_viewlayer_cb(re, &job_data, prepare_viewlayer_update);

  /* Lens adjust. */
  float oldlens = reinterpret_cast<Camera *>(sce->camera->data)->lens;

  RE_ClearResult(re);
  RE_PreviewRender(re, G.pr_main, sce);

  reinterpret_cast<Camera *>(sce->camera->data)->lens = oldlens;

  /* Free the aov layers and the layers generated for each node. */
  BLI_freelistN(&AOV_layer->aovs);
  ViewLayer *vl = AOV_layer->next;
  while (vl) {
    ViewLayer *vl_rem = vl;
    vl = vl->next;
    BLI_remlink(&sce->view_layers, vl_rem);
    BKE_view_layer_free(vl_rem);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Preview job management
 * \{ */

static void update_needed_flag(const bNodeTree *nt, NestedTreePreviews *tree_previews)
{
  if (nt->preview_refresh_state != tree_previews->previews_refresh_state ||
      tree_previews->pr_size != U.node_preview_res)
  {
    tree_previews->restart_needed = true;
  }
}

static void shader_preview_startjob(void *customdata,
                                    bool *stop,
                                    bool *do_update,
                                    float * /*progress*/)
{
  ShaderNodesPreviewJob *job_data = static_cast<ShaderNodesPreviewJob *>(customdata);

  job_data->stop = stop;
  job_data->do_update = do_update;
  *do_update = true;
  bool size_changed = job_data->tree_previews->pr_size != U.node_preview_res;
  if (size_changed) {
    job_data->tree_previews->pr_size = U.node_preview_res;
  }

  /* Duplicate material for each preview and update things related to it: treepath and previewed
   * node. */
  job_data->mat_copy = duplicate_material(job_data->mat_orig);

  /* Update the treepath copied to fit the structure of the nodetree copied. */
  bNodeTreePath *root_path = MEM_cnew<bNodeTreePath>(__func__);
  root_path->nodetree = job_data->mat_copy->nodetree;
  BLI_addtail(&job_data->treepath_copy, root_path);
  for (bNodeTreePath *original_path =
           static_cast<bNodeTreePath *>(job_data->treepath->first)->next;
       original_path;
       original_path = original_path->next)
  {
    bNodeTreePath *new_path = MEM_cnew<bNodeTreePath>(__func__);
    memcpy(new_path, original_path, sizeof(bNodeTreePath));
    bNode *parent = nodeFindNodebyName(
        static_cast<bNodeTreePath *>(job_data->treepath_copy.last)->nodetree,
        original_path->node_name);
    new_path->nodetree = reinterpret_cast<bNodeTree *>(parent->id);
    BLI_addtail(&job_data->treepath_copy, new_path);
  }

  /* Find the shader output node. */
  for (bNode *node_iter : job_data->mat_copy->nodetree->all_nodes()) {
    if (node_iter->type == SH_NODE_OUTPUT_MATERIAL && node_iter->flag & NODE_DO_OUTPUT) {
      job_data->mat_output_copy = node_iter;
      break;
    }
  }
  if (job_data->mat_output_copy == nullptr) {
    job_data->mat_output_copy = nodeAddStaticNode(
        nullptr, root_path->nodetree, SH_NODE_OUTPUT_MATERIAL);
  }

  bNodeTree *active_nodetree =
      static_cast<bNodeTreePath *>(job_data->treepath_copy.last)->nodetree;
  for (bNode *node : active_nodetree->all_nodes()) {
    if (!(node->flag & NODE_PREVIEW)) {
      continue;
    }

    if (node_use_aov(node)) {
      job_data->AOV_nodes.append(node);
    }
    else {
      job_data->shader_nodes.append(node);
    }
  }

  preview_render(*job_data);
}

static void shader_preview_free(void *customdata)
{
  ShaderNodesPreviewJob *job_data = static_cast<ShaderNodesPreviewJob *>(customdata);
  BLI_freelistN(&job_data->treepath_copy);
  job_data->tree_previews->rendering = false;
  if (job_data->mat_copy != nullptr) {
    BLI_remlink(&G.pr_main->materials, job_data->mat_copy);
    BKE_id_free(G.pr_main, &job_data->mat_copy->id);
    job_data->mat_copy = nullptr;
  }
  MEM_delete(job_data);
}

static void ensure_nodetree_previews(const bContext *C,
                                     NestedTreePreviews *tree_previews,
                                     Material *material,
                                     ListBase *treepath)
{
  Scene *scene = CTX_data_scene(C);
  if (!ED_check_engine_supports_preview(scene)) {
    return;
  }

  bNodeTree *displayed_nt = static_cast<bNodeTreePath *>(treepath->last)->nodetree;
  update_needed_flag(displayed_nt, tree_previews);
  if (!(tree_previews->restart_needed)) {
    return;
  }
  if (tree_previews->rendering) {
    WM_jobs_stop(CTX_wm_manager(C),
                 CTX_wm_space_node(C),
                 reinterpret_cast<void *>(shader_preview_startjob));
    return;
  }
  tree_previews->rendering = true;
  tree_previews->restart_needed = false;
  tree_previews->previews_refresh_state = displayed_nt->preview_refresh_state;

  ED_preview_ensure_dbase(false);

  wmJob *wm_job = WM_jobs_get(CTX_wm_manager(C),
                              CTX_wm_window(C),
                              CTX_wm_space_node(C),
                              "Shader Previews",
                              WM_JOB_EXCL_RENDER,
                              WM_JOB_TYPE_RENDER_PREVIEW);
  ShaderNodesPreviewJob *nt_previews = MEM_new<ShaderNodesPreviewJob>("shader previews");

  nt_previews->scene = scene;
  nt_previews->tree_previews = tree_previews;
  nt_previews->mat_orig = material;
  nt_previews->bmain = CTX_data_main(C);
  nt_previews->treepath = treepath;

  WM_jobs_customdata_set(wm_job, nt_previews, shader_preview_free);
  WM_jobs_timer(wm_job, 0.2, NC_NODE, NC_NODE);
  WM_jobs_callbacks(wm_job, shader_preview_startjob, nullptr, nullptr, nullptr);

  WM_jobs_start(CTX_wm_manager(C), wm_job);
}

static void free_previews(wmWindowManager *wm, SpaceNode *snode, NestedTreePreviews *tree_previews)
{
  /* This should not be called from the drawing pass, because it will result in a deadlock. */
  WM_jobs_kill(wm, snode, shader_preview_startjob);
  if (tree_previews->previews_render) {
    RE_FreeRender(tree_previews->previews_render);
    tree_previews->previews_render = nullptr;
  }
  MEM_freeN(tree_previews);
}

void ED_spacenode_free_previews(wmWindowManager *wm, SpaceNode *snode)
{
  snode->runtime->tree_previews_per_context.foreach_item(
      [&](ComputeContextHash /*hash*/, NestedTreePreviews *tree_previews) {
        free_previews(wm, snode, tree_previews);
        tree_previews = nullptr;
      });
  snode->runtime->tree_previews_per_context.clear();
}

/** \} */
