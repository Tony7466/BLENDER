/* SPDX-FileCopyrightText: 2006 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup cmpnodes
 */

#include <cstring>

#include "BLI_assert.h"
#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_string_utf8.h"
#include "BLI_string_utils.h"
#include "BLI_utildefines.h"

#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "BKE_context.h"
#include "BKE_image.h"
#include "BKE_image_format.h"
#include "BKE_main.h"
#include "BKE_scene.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "WM_api.hh"

#include "IMB_colormanagement.h"
#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"
#include "IMB_openexr.h"

#include "GPU_state.h"
#include "GPU_texture.h"

#include "COM_node_operation.hh"

#include "node_composite_util.hh"

/* **************** OUTPUT FILE ******************** */

/* find unique path */
static bool unique_path_unique_check(void *arg, const char *name)
{
  struct Args {
    ListBase *lb;
    bNodeSocket *sock;
  };
  Args *data = (Args *)arg;

  LISTBASE_FOREACH (bNodeSocket *, sock, data->lb) {
    if (sock != data->sock) {
      NodeImageMultiFileSocket *sockdata = (NodeImageMultiFileSocket *)sock->storage;
      if (STREQ(sockdata->path, name)) {
        return true;
      }
    }
  }
  return false;
}
void ntreeCompositOutputFileUniquePath(ListBase *list,
                                       bNodeSocket *sock,
                                       const char defname[],
                                       char delim)
{
  NodeImageMultiFileSocket *sockdata;
  struct {
    ListBase *lb;
    bNodeSocket *sock;
  } data;
  data.lb = list;
  data.sock = sock;

  /* See if we are given an empty string */
  if (ELEM(nullptr, sock, defname)) {
    return;
  }

  sockdata = (NodeImageMultiFileSocket *)sock->storage;
  BLI_uniquename_cb(
      unique_path_unique_check, &data, defname, delim, sockdata->path, sizeof(sockdata->path));
}

/* find unique EXR layer */
static bool unique_layer_unique_check(void *arg, const char *name)
{
  struct Args {
    ListBase *lb;
    bNodeSocket *sock;
  };
  Args *data = (Args *)arg;

  LISTBASE_FOREACH (bNodeSocket *, sock, data->lb) {
    if (sock != data->sock) {
      NodeImageMultiFileSocket *sockdata = (NodeImageMultiFileSocket *)sock->storage;
      if (STREQ(sockdata->layer, name)) {
        return true;
      }
    }
  }
  return false;
}
void ntreeCompositOutputFileUniqueLayer(ListBase *list,
                                        bNodeSocket *sock,
                                        const char defname[],
                                        char delim)
{
  struct {
    ListBase *lb;
    bNodeSocket *sock;
  } data;
  data.lb = list;
  data.sock = sock;

  /* See if we are given an empty string */
  if (ELEM(nullptr, sock, defname)) {
    return;
  }

  NodeImageMultiFileSocket *sockdata = (NodeImageMultiFileSocket *)sock->storage;
  BLI_uniquename_cb(
      unique_layer_unique_check, &data, defname, delim, sockdata->layer, sizeof(sockdata->layer));
}

bNodeSocket *ntreeCompositOutputFileAddSocket(bNodeTree *ntree,
                                              bNode *node,
                                              const char *name,
                                              const ImageFormatData *im_format)
{
  NodeImageMultiFile *nimf = (NodeImageMultiFile *)node->storage;
  bNodeSocket *sock = nodeAddStaticSocket(
      ntree, node, SOCK_IN, SOCK_RGBA, PROP_NONE, nullptr, name);

  /* create format data for the input socket */
  NodeImageMultiFileSocket *sockdata = MEM_cnew<NodeImageMultiFileSocket>(__func__);
  sock->storage = sockdata;

  STRNCPY_UTF8(sockdata->path, name);
  ntreeCompositOutputFileUniquePath(&node->inputs, sock, name, '_');
  STRNCPY_UTF8(sockdata->layer, name);
  ntreeCompositOutputFileUniqueLayer(&node->inputs, sock, name, '_');

  if (im_format) {
    BKE_image_format_copy(&sockdata->format, im_format);
    sockdata->format.color_management = R_IMF_COLOR_MANAGEMENT_FOLLOW_SCENE;
    if (BKE_imtype_is_movie(sockdata->format.imtype)) {
      sockdata->format.imtype = R_IMF_IMTYPE_OPENEXR;
    }
  }
  else {
    BKE_image_format_init(&sockdata->format, false);
  }
  /* use node data format by default */
  sockdata->use_node_format = true;
  sockdata->save_as_render = true;

  nimf->active_input = BLI_findindex(&node->inputs, sock);

  return sock;
}

int ntreeCompositOutputFileRemoveActiveSocket(bNodeTree *ntree, bNode *node)
{
  NodeImageMultiFile *nimf = (NodeImageMultiFile *)node->storage;
  bNodeSocket *sock = (bNodeSocket *)BLI_findlink(&node->inputs, nimf->active_input);
  int totinputs = BLI_listbase_count(&node->inputs);

  if (!sock) {
    return 0;
  }

  if (nimf->active_input == totinputs - 1) {
    --nimf->active_input;
  }

  /* free format data */
  MEM_freeN(sock->storage);

  nodeRemoveSocket(ntree, node, sock);
  return 1;
}

void ntreeCompositOutputFileSetPath(bNode *node, bNodeSocket *sock, const char *name)
{
  NodeImageMultiFileSocket *sockdata = (NodeImageMultiFileSocket *)sock->storage;
  STRNCPY_UTF8(sockdata->path, name);
  ntreeCompositOutputFileUniquePath(&node->inputs, sock, name, '_');
}

void ntreeCompositOutputFileSetLayer(bNode *node, bNodeSocket *sock, const char *name)
{
  NodeImageMultiFileSocket *sockdata = (NodeImageMultiFileSocket *)sock->storage;
  STRNCPY_UTF8(sockdata->layer, name);
  ntreeCompositOutputFileUniqueLayer(&node->inputs, sock, name, '_');
}

namespace blender::nodes::node_composite_output_file_cc {

NODE_STORAGE_FUNCS(NodeImageMultiFile)

/* XXX uses initfunc_api callback, regular initfunc does not support context yet */
static void init_output_file(const bContext *C, PointerRNA *ptr)
{
  Scene *scene = CTX_data_scene(C);
  bNodeTree *ntree = (bNodeTree *)ptr->owner_id;
  bNode *node = (bNode *)ptr->data;
  NodeImageMultiFile *nimf = MEM_cnew<NodeImageMultiFile>(__func__);
  ImageFormatData *format = nullptr;
  node->storage = nimf;

  if (scene) {
    RenderData *rd = &scene->r;

    STRNCPY(nimf->base_path, rd->pic);
    BKE_image_format_copy(&nimf->format, &rd->im_format);
    nimf->format.color_management = R_IMF_COLOR_MANAGEMENT_FOLLOW_SCENE;
    if (BKE_imtype_is_movie(nimf->format.imtype)) {
      nimf->format.imtype = R_IMF_IMTYPE_OPENEXR;
    }

    format = &nimf->format;
  }
  else {
    BKE_image_format_init(&nimf->format, false);
  }

  /* add one socket by default */
  ntreeCompositOutputFileAddSocket(ntree, node, "Image", format);
}

static void free_output_file(bNode *node)
{
  /* free storage data in sockets */
  LISTBASE_FOREACH (bNodeSocket *, sock, &node->inputs) {
    NodeImageMultiFileSocket *sockdata = (NodeImageMultiFileSocket *)sock->storage;
    BKE_image_format_free(&sockdata->format);
    MEM_freeN(sock->storage);
  }

  NodeImageMultiFile *nimf = (NodeImageMultiFile *)node->storage;
  BKE_image_format_free(&nimf->format);
  MEM_freeN(node->storage);
}

static void copy_output_file(bNodeTree * /*dst_ntree*/, bNode *dest_node, const bNode *src_node)
{
  bNodeSocket *src_sock, *dest_sock;

  dest_node->storage = MEM_dupallocN(src_node->storage);
  NodeImageMultiFile *dest_nimf = (NodeImageMultiFile *)dest_node->storage;
  NodeImageMultiFile *src_nimf = (NodeImageMultiFile *)src_node->storage;
  BKE_image_format_copy(&dest_nimf->format, &src_nimf->format);

  /* duplicate storage data in sockets */
  for (src_sock = (bNodeSocket *)src_node->inputs.first,
      dest_sock = (bNodeSocket *)dest_node->inputs.first;
       src_sock && dest_sock;
       src_sock = src_sock->next, dest_sock = (bNodeSocket *)dest_sock->next)
  {
    dest_sock->storage = MEM_dupallocN(src_sock->storage);
    NodeImageMultiFileSocket *dest_sockdata = (NodeImageMultiFileSocket *)dest_sock->storage;
    NodeImageMultiFileSocket *src_sockdata = (NodeImageMultiFileSocket *)src_sock->storage;
    BKE_image_format_copy(&dest_sockdata->format, &src_sockdata->format);
  }
}

static void update_output_file(bNodeTree *ntree, bNode *node)
{
  /* XXX fix for #36706: remove invalid sockets added with bpy API.
   * This is not ideal, but prevents crashes from missing storage.
   * FileOutput node needs a redesign to support this properly.
   */
  LISTBASE_FOREACH (bNodeSocket *, sock, &node->inputs) {
    if (sock->storage == nullptr) {
      nodeRemoveSocket(ntree, node, sock);
    }
  }
  LISTBASE_FOREACH (bNodeSocket *, sock, &node->outputs) {
    nodeRemoveSocket(ntree, node, sock);
  }

  cmp_node_update_default(ntree, node);

  /* automatically update the socket type based on linked input */
  LISTBASE_FOREACH (bNodeSocket *, sock, &node->inputs) {
    if (sock->link) {
      PointerRNA ptr = RNA_pointer_create((ID *)ntree, &RNA_NodeSocket, sock);
      RNA_enum_set(&ptr, "type", sock->link->fromsock->type);
    }
  }
}

static void node_composit_buts_file_output(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  PointerRNA imfptr = RNA_pointer_get(ptr, "format");
  const bool multilayer = RNA_enum_get(&imfptr, "file_format") == R_IMF_IMTYPE_MULTILAYER;

  if (multilayer) {
    uiItemL(layout, IFACE_("Path:"), ICON_NONE);
  }
  else {
    uiItemL(layout, IFACE_("Base Path:"), ICON_NONE);
  }
  uiItemR(layout, ptr, "base_path", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_composit_buts_file_output_ex(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  Scene *scene = CTX_data_scene(C);
  PointerRNA imfptr = RNA_pointer_get(ptr, "format");
  PointerRNA active_input_ptr, op_ptr;
  uiLayout *row, *col;
  const bool multilayer = RNA_enum_get(&imfptr, "file_format") == R_IMF_IMTYPE_MULTILAYER;
  const bool is_exr = RNA_enum_get(&imfptr, "file_format") == R_IMF_IMTYPE_OPENEXR;
  const bool is_multiview = (scene->r.scemode & R_MULTIVIEW) != 0;

  node_composit_buts_file_output(layout, C, ptr);
  uiTemplateImageSettings(layout, &imfptr, true);

  /* disable stereo output for multilayer, too much work for something that no one will use */
  /* if someone asks for that we can implement it */
  if (is_multiview) {
    uiTemplateImageFormatViews(layout, &imfptr, nullptr);
  }

  uiItemS(layout);

  uiItemO(layout, IFACE_("Add Input"), ICON_ADD, "NODE_OT_output_file_add_socket");

  row = uiLayoutRow(layout, false);
  col = uiLayoutColumn(row, true);

  const int active_index = RNA_int_get(ptr, "active_input_index");
  /* using different collection properties if multilayer format is enabled */
  if (multilayer) {
    uiTemplateList(col,
                   C,
                   "UI_UL_list",
                   "file_output_node",
                   ptr,
                   "layer_slots",
                   ptr,
                   "active_input_index",
                   nullptr,
                   0,
                   0,
                   0,
                   0,
                   UI_TEMPLATE_LIST_FLAG_NONE);
    RNA_property_collection_lookup_int(
        ptr, RNA_struct_find_property(ptr, "layer_slots"), active_index, &active_input_ptr);
  }
  else {
    uiTemplateList(col,
                   C,
                   "UI_UL_list",
                   "file_output_node",
                   ptr,
                   "file_slots",
                   ptr,
                   "active_input_index",
                   nullptr,
                   0,
                   0,
                   0,
                   0,
                   UI_TEMPLATE_LIST_FLAG_NONE);
    RNA_property_collection_lookup_int(
        ptr, RNA_struct_find_property(ptr, "file_slots"), active_index, &active_input_ptr);
  }
  /* XXX collection lookup does not return the ID part of the pointer,
   * setting this manually here */
  active_input_ptr.owner_id = ptr->owner_id;

  col = uiLayoutColumn(row, true);
  wmOperatorType *ot = WM_operatortype_find("NODE_OT_output_file_move_active_socket", false);
  uiItemFullO_ptr(col, ot, "", ICON_TRIA_UP, nullptr, WM_OP_INVOKE_DEFAULT, UI_ITEM_NONE, &op_ptr);
  RNA_enum_set(&op_ptr, "direction", 1);
  uiItemFullO_ptr(
      col, ot, "", ICON_TRIA_DOWN, nullptr, WM_OP_INVOKE_DEFAULT, UI_ITEM_NONE, &op_ptr);
  RNA_enum_set(&op_ptr, "direction", 2);

  if (active_input_ptr.data) {
    if (multilayer) {
      col = uiLayoutColumn(layout, true);

      uiItemL(col, IFACE_("Layer:"), ICON_NONE);
      row = uiLayoutRow(col, false);
      uiItemR(row, &active_input_ptr, "name", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
      uiItemFullO(row,
                  "NODE_OT_output_file_remove_active_socket",
                  "",
                  ICON_X,
                  nullptr,
                  WM_OP_EXEC_DEFAULT,
                  UI_ITEM_R_ICON_ONLY,
                  nullptr);
    }
    else {
      col = uiLayoutColumn(layout, true);

      uiItemL(col, IFACE_("File Subpath:"), ICON_NONE);
      row = uiLayoutRow(col, false);
      uiItemR(row, &active_input_ptr, "path", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
      uiItemFullO(row,
                  "NODE_OT_output_file_remove_active_socket",
                  "",
                  ICON_X,
                  nullptr,
                  WM_OP_EXEC_DEFAULT,
                  UI_ITEM_R_ICON_ONLY,
                  nullptr);

      /* format details for individual files */
      imfptr = RNA_pointer_get(&active_input_ptr, "format");

      col = uiLayoutColumn(layout, true);
      uiItemL(col, IFACE_("Format:"), ICON_NONE);
      uiItemR(col,
              &active_input_ptr,
              "use_node_format",
              UI_ITEM_R_SPLIT_EMPTY_NAME,
              nullptr,
              ICON_NONE);

      const bool is_socket_exr = RNA_enum_get(&imfptr, "file_format") == R_IMF_IMTYPE_OPENEXR;
      const bool use_node_format = RNA_boolean_get(&active_input_ptr, "use_node_format");

      if ((!is_exr && use_node_format) || (!is_socket_exr && !use_node_format)) {
        uiItemR(col,
                &active_input_ptr,
                "save_as_render",
                UI_ITEM_R_SPLIT_EMPTY_NAME,
                nullptr,
                ICON_NONE);
      }

      if (!use_node_format) {
        const bool use_color_management = RNA_boolean_get(&active_input_ptr, "save_as_render");

        col = uiLayoutColumn(layout, false);
        uiTemplateImageSettings(col, &imfptr, use_color_management);

        if (is_multiview) {
          col = uiLayoutColumn(layout, false);
          uiTemplateImageFormatViews(col, &imfptr, nullptr);
        }
      }
    }
  }
}

using namespace blender::realtime_compositor;

class OutputFileOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    if (is_multi_layer_exr()) {
      execute_multi_layer_exr();
    }
    else {
      execute_single_layer();
    }
  }

  /* -----------------------
   * Multi-Layer EXR Images.
   * ----------------------- */

  bool is_multi_layer_exr()
  {
    return node_storage(bnode()).format.imtype == R_IMF_IMTYPE_MULTILAYER;
  }

  void execute_multi_layer_exr()
  {
    const Domain domain = compute_domain();
    const int width = domain.size.x;
    const int height = domain.size.y;

    const bool store_views_in_single_file = is_multi_view_exr();
    const char *view = context().get_view_name().data();

    /* If we are saving all views in a single multi-layer file, we supply an empty view to make
     * sure the file name does not contain a view suffix. */
    char image_path[FILE_MAX];
    get_multi_layer_exr_image_path(store_views_in_single_file ? "" : view, image_path);

    /* This function creates a global EXR image whose lifetime extends outside this operation and
     * even this compositor evaluation. This is useful for multi-view images, since the compositor
     * is dispatched multiple times for each of the views, so we need a global resource to add all
     * views to the same EXR image. The EXR is then only written once the last view was added. */
    void *exr_handle = IMB_exr_get_handle_name(image_path);

    /* No need to add a view if we are saving each view in its own file. */
    if (store_views_in_single_file) {
      IMB_exr_add_view(exr_handle, view);
    }

    for (const bNodeSocket *input : this->node()->input_sockets()) {
      const Result &input_result = get_input(input->identifier);
      if (input_result.is_single_value()) {
        continue;
      }

      GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
      float *buffer = static_cast<float *>(
          GPU_texture_read(input_result.texture(), GPU_DATA_FLOAT, 0));

      const char *layer = (static_cast<NodeImageMultiFileSocket *>(input->storage))->layer;

      /* If we are saving views in separate files, we needn't store the view in the channels
       * themselves, so we supply an empty view name. */
      const char *channel_view = store_views_in_single_file ? view : "";

      switch (input_result.type()) {
        case ResultType::Color:
          add_exr_channels(exr_handle, layer, channel_view, "RGBA", 4, 4, width, buffer);
          break;
        case ResultType::Vector:
          /* The last component of the 4D vector is ignored, hence the stride of 3. */
          add_exr_channels(exr_handle, layer, channel_view, "XYZ", 3, 4, width, buffer);
          break;
        case ResultType::Float:
          add_exr_channels(exr_handle, layer, channel_view, "V", 1, 1, width, buffer);
          break;
        default:
          /* Other types are internal and needn't be handled by operations. */
          BLI_assert_unreachable();
          break;
      }
    }

    /* We only write the EXR image if we added all views already, that is, we are in the last view.
     * Moreover, if we are saving views in separate files, we can write the EXR image directly as
     * there are not any more views to add. */
    if (BKE_scene_multiview_is_render_view_last(&context().get_render_data(), view) ||
        !store_views_in_single_file)
    {
      StampData *stamp_data = BKE_stamp_info_from_scene_static(&context().get_scene());
      IMB_exr_begin_write(exr_handle, image_path, width, height, get_exr_codec(), stamp_data);
      IMB_exr_write_channels(exr_handle);

      /* We free the buffers allocated by the GPU_texture_read function, however, we note that
       * multiple channels could share the same buffer, but with a different offset and stride, so
       * we only free the first channel of each image by inspecting its name, since that would be
       * the buffer with no offset. 'R' for RGBA, 'X' for XYZ, and 'V' is just V. */
      IMB_exr_free_channels_buffer(exr_handle, [](const char *name, float *buffer) {
        if (ELEM(std::string(name).back(), 'R', 'X', 'V')) {
          MEM_freeN(buffer);
        }
      });

      IMB_exr_close(exr_handle);
      BKE_stamp_data_free(stamp_data);
    }
  }

  /* Add the EXR channels representing an image of the given number of channels, width, and pixel
   * buffer. The name of the EXR channel will encode the given layer, view, and each of the pass
   * names corresponding to each of the characters in the given channels string. The channels are
   * assumed to be contagious in the buffer and have the given pixel stride. */
  void add_exr_channels(void *exr_handle,
                        const char *layer,
                        const char *view,
                        const char *channels,
                        int channels_count,
                        int stride,
                        int width,
                        float *buffer)
  {
    for (int i = 0; i < channels_count; i++) {
      const std::string pass = std::string(1, channels[i]);
      IMB_exr_add_channel(exr_handle,
                          layer,
                          pass.c_str(),
                          view,
                          stride,
                          width * stride,
                          buffer + i,
                          use_half_float());
    }
  }

  /* Get the path of the image to be saved and ensure its directory tree exists. If the given view
   * is not empty, its corresponding file suffix will be appended to the name. */
  void get_multi_layer_exr_image_path(const char *view, char *image_path)
  {
    const char *suffix = BKE_scene_multiview_view_suffix_get(&context().get_render_data(), view);
    BKE_image_path_from_imtype(image_path,
                               get_base_path(),
                               BKE_main_blendfile_path_from_global(),
                               context().get_frame_number(),
                               R_IMF_IMTYPE_MULTILAYER,
                               use_file_extension(),
                               true,
                               suffix);
    BLI_file_ensure_parent_dir_exists(image_path);
  }

  /* --------------------
   * Single Layer Images.
   * -------------------- */

  void execute_single_layer()
  {
    for (const bNodeSocket *input : this->node()->input_sockets()) {
      const Result &input_result = get_input(input->identifier);
      if (input_result.is_single_value()) {
        continue;
      }

      NodeImageMultiFileSocket *socket = static_cast<NodeImageMultiFileSocket *>(input->storage);
      const ImageFormatData &format = socket->use_node_format ? node_storage(bnode()).format :
                                                                socket->format;

      char base_path[FILE_MAX];
      if (socket->path[0]) {
        BLI_path_join(base_path, FILE_MAX, get_base_path(), socket->path);
      }
      else {
        STRNCPY(base_path, get_base_path());
        BLI_path_slash_ensure(base_path, FILE_MAX);
      }

      switch (format.views_format) {
        case R_IMF_VIEWS_INDIVIDUAL:
          execute_single_layer_single_view(input_result, *socket, format, base_path);
          break;
        case R_IMF_VIEWS_STEREO_3D:
          execute_single_layer_stereo(input_result, *socket, format, base_path);
          break;
        case R_IMF_VIEWS_MULTIVIEW:
          if (is_multi_view_scene()) {
            execute_single_layer_multi_view(input_result, *socket, format, base_path);
          }
          else {
            execute_single_layer_single_view(input_result, *socket, format, base_path);
          }
          break;
      }
    }
  }

  /* -------------------------
   * Single Layer Single View.
   * ------------------------- */

  void execute_single_layer_single_view(const Result &input_result,
                                        NodeImageMultiFileSocket &socket,
                                        const ImageFormatData &format,
                                        const char *base_path)
  {
    ImageFormatData write_format;
    BKE_image_format_init_for_write(&write_format, &context().get_scene(), &format);

    /* Avoid applying color conversion during writing if not saving as a render. */
    if (!socket.save_as_render) {
      write_format.linear_colorspace_settings.name[0] = '\0';
    }

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
    float *pixel_buffer = static_cast<float *>(
        GPU_texture_read(input_result.texture(), GPU_DATA_FLOAT, 0));

    const Domain domain = input_result.domain();
    const int width = domain.size.x;
    const int height = domain.size.y;

    ImBuf *image_buffer = IMB_allocImBuf(width, height, write_format.planes, 0);

    image_buffer->channels = get_result_channels_count(input_result);
    image_buffer->dither = context().get_render_data().dither_intensity;

    IMB_assign_float_buffer(image_buffer, pixel_buffer, IB_TAKE_OWNERSHIP);

    IMB_colormanagement_imbuf_for_write(image_buffer, socket.save_as_render, false, &write_format);

    char image_path[FILE_MAX];
    const char *suffix = BKE_scene_multiview_view_suffix_get(&context().get_render_data(),
                                                             context().get_view_name().data());
    BKE_image_path_from_imformat(image_path,
                                 base_path,
                                 BKE_main_blendfile_path_from_global(),
                                 context().get_frame_number(),
                                 &write_format,
                                 use_file_extension(),
                                 true,
                                 suffix);

    BKE_imbuf_write(image_buffer, image_path, &write_format);
    IMB_freeImBuf(image_buffer);
  }

  /* --------------------
   * Single Layer Stereo.
   * -------------------- */

  void execute_single_layer_stereo(const Result &input_result,
                                   NodeImageMultiFileSocket &socket,
                                   const ImageFormatData &format,
                                   const char *base_path)
  {
    ImageFormatData write_format;
    BKE_image_format_init_for_write(&write_format, &context().get_scene(), &format);

    /* Avoid applying color conversion during writing if not saving as a render. */
    if (!socket.save_as_render) {
      write_format.linear_colorspace_settings.name[0] = '\0';
    }

    const Domain domain = compute_domain();
    const int width = domain.size.x;
    const int height = domain.size.y;

    /* We are not necessarily saving an EXR image, however, we use the global EXR image structure
     * to store both the left and right views at the same time, which we then extract from the EXR
     * image and save using the standard IMB API. See the comments in execute_multi_layer_exr for
     * more information. */
    void *exr_handle = IMB_exr_get_handle_name(base_path);

    const char *view = context().get_view_name().data();
    IMB_exr_add_view(exr_handle, view);

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
    float *buffer = static_cast<float *>(
        GPU_texture_read(input_result.texture(), GPU_DATA_FLOAT, 0));

    /* The entire pixel buffer is encoded in a float channel, but is it is just a dummy channel for
     * storage as explained above. */
    IMB_exr_add_channel(exr_handle,
                        nullptr,
                        socket.layer,
                        view,
                        1,
                        get_result_channels_count(input_result) * width * height,
                        buffer,
                        write_format.depth == R_IMF_CHAN_DEPTH_16);

    /* This is the last view, so both views are now added and we can write our image. */
    if (BKE_scene_multiview_is_render_view_last(&context().get_render_data(), view)) {
      ImBuf *stereo_image_buffers[2];
      const char *stereo_view_names[2] = {STEREO_LEFT_NAME, STEREO_RIGHT_NAME};

      /* Extract and steal the pixel buffers of both views from the EXR and construct image buffers
       * from them. */
      for (int i = 0; i < 2; i++) {
        float *rectf = IMB_exr_channel_rect(
            exr_handle, nullptr, socket.layer, stereo_view_names[i]);

        stereo_image_buffers[i] = IMB_allocImBuf(width, height, write_format.planes, 0);
        stereo_image_buffers[i]->channels = get_result_channels_count(input_result);
        stereo_image_buffers[i]->dither = context().get_render_data().dither_intensity;

        IMB_assign_float_buffer(stereo_image_buffers[i], rectf, IB_TAKE_OWNERSHIP);

        IMB_colormanagement_imbuf_for_write(stereo_image_buffers[i], true, false, &write_format);
      }

      ImBuf *stereo_image_buffer = IMB_stereo3d_ImBuf(
          &write_format, stereo_image_buffers[0], stereo_image_buffers[1]);

      char image_path[FILE_MAX];
      BKE_image_path_from_imformat(image_path,
                                   base_path,
                                   BKE_main_blendfile_path_from_global(),
                                   context().get_frame_number(),
                                   &write_format,
                                   use_file_extension(),
                                   true,
                                   nullptr);

      BKE_imbuf_write(stereo_image_buffer, image_path, &write_format);

      IMB_freeImBuf(stereo_image_buffers[0]);
      IMB_freeImBuf(stereo_image_buffers[1]);
      IMB_freeImBuf(stereo_image_buffer);

      IMB_exr_close(exr_handle);
    }
  }

  /* ------------------------
   * Single Layer Multi-View.
   * ------------------------ */

  void execute_single_layer_multi_view(const Result &input_result,
                                       NodeImageMultiFileSocket &socket,
                                       const ImageFormatData &format,
                                       const char *base_path)
  {
  }

  /* Get the number of components in the type of the given result. Note that vectors are 4D. */
  int get_result_channels_count(const Result &result)
  {
    switch (result.type()) {
      case ResultType::Color:
      case ResultType::Vector:
        return 4;
      case ResultType::Float:
        return 1;
      default:
        /* Other types are internal and needn't be handled by operations. */
        break;
    }

    BLI_assert_unreachable();
    return 0;
  }

  bool use_half_float()
  {
    return node_storage(bnode()).format.depth == R_IMF_CHAN_DEPTH_16;
  }

  const char *get_base_path()
  {
    return node_storage(bnode()).base_path;
  }

  /* The EXR compression method. */
  char get_exr_codec()
  {
    return node_storage(bnode()).format.exr_codec;
  }

  /* Add the file format extensions to the rendered file name. */
  bool use_file_extension()
  {
    return context().get_render_data().scemode & R_EXTENSION;
  }

  /* If true, save views in a multi-view EXR file, otherwise, save each view in its own file. */
  bool is_multi_view_exr()
  {
    if (!is_multi_view_scene()) {
      return false;
    }

    return node_storage(bnode()).format.views_format == R_IMF_VIEWS_MULTIVIEW;
  }

  bool is_multi_view_scene()
  {
    return context().get_render_data().scemode & R_MULTIVIEW;
  }
};

static NodeOperation *get_compositor_operation(Context &context, DNode node)
{
  return new OutputFileOperation(context, node);
}

}  // namespace blender::nodes::node_composite_output_file_cc

void register_node_type_cmp_output_file()
{
  namespace file_ns = blender::nodes::node_composite_output_file_cc;

  static bNodeType ntype;

  cmp_node_type_base(&ntype, CMP_NODE_OUTPUT_FILE, "File Output", NODE_CLASS_OUTPUT);
  ntype.draw_buttons = file_ns::node_composit_buts_file_output;
  ntype.draw_buttons_ex = file_ns::node_composit_buts_file_output_ex;
  ntype.initfunc_api = file_ns::init_output_file;
  ntype.flag |= NODE_PREVIEW;
  node_type_storage(
      &ntype, "NodeImageMultiFile", file_ns::free_output_file, file_ns::copy_output_file);
  ntype.updatefunc = file_ns::update_output_file;
  ntype.get_compositor_operation = file_ns::get_compositor_operation;

  nodeRegisterType(&ntype);
}
