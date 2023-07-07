/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup cmpnodes
 */

#include "RNA_access.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "IMB_colormanagement.h"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "COM_algorithm_summed_area_table.hh"

#include "node_composite_util.hh"

/* **************** Kuwahara ******************** */

namespace blender::nodes::node_composite_kuwahara_cc {

NODE_STORAGE_FUNCS(NodeKuwaharaData)

static void cmp_node_kuwahara_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>(N_("Image"))
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .compositor_domain_priority(0);
  b.add_output<decl::Color>(N_("Image"));
}

static void node_composit_init_kuwahara(bNodeTree * /*ntree*/, bNode *node)
{
  NodeKuwaharaData *data = MEM_cnew<NodeKuwaharaData>(__func__);
  node->storage = data;

  /* Set defaults. */
  data->size = 4;
  data->smoothing = 2;
}

static void node_composit_buts_kuwahara(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayout *col;

  col = uiLayoutColumn(layout, false);

  uiItemR(col, ptr, "variation", 0, nullptr, ICON_NONE);
  uiItemR(col, ptr, "size", 0, nullptr, ICON_NONE);

  const int variation = RNA_enum_get(ptr, "variation");

  if (variation == CMP_NODE_KUWAHARA_ANISOTROPIC) {
    uiItemR(col, ptr, "smoothing", 0, nullptr, ICON_NONE);
  }
}

using namespace blender::realtime_compositor;

class ConvertKuwaharaOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    if (node_storage(bnode()).variation == CMP_NODE_KUWAHARA_ANISOTROPIC) {
      execute_anisotropic();
    }
    else {
      execute_classic();
    }
  }

  void execute_classic()
  {
    summed_area_table(context(), get_input("Image"), get_result("Image"));
    return;

    GPUShader *shader = shader_manager().get("compositor_kuwahara_classic");
    GPU_shader_bind(shader);

    float luminance_coefficients[3];
    IMB_colormanagement_get_luminance_coefficients(luminance_coefficients);
    GPU_shader_uniform_3fv(shader, "luminance_coefficients", luminance_coefficients);

    GPU_shader_uniform_1i(shader, "radius", node_storage(bnode()).size);

    const Result &input_image = get_input("Image");
    input_image.bind_as_texture(shader, "input_tx");

    const Domain domain = compute_domain();
    Result &output_image = get_result("Image");
    output_image.allocate_texture(domain);
    output_image.bind_as_image(shader, "output_img");

    compute_dispatch_threads_at_least(shader, domain.size);

    input_image.unbind_as_texture();
    output_image.unbind_as_image();
    GPU_shader_unbind();
  }

  void execute_anisotropic()
  {
    get_input("Image").pass_through(get_result("Image"));
    context().set_info_message("Viewport compositor setup not fully supported");
  }
};

static NodeOperation *get_compositor_operation(Context &context, DNode node)
{
  return new ConvertKuwaharaOperation(context, node);
}

}  // namespace blender::nodes::node_composite_kuwahara_cc

void register_node_type_cmp_kuwahara()
{
  namespace file_ns = blender::nodes::node_composite_kuwahara_cc;

  static bNodeType ntype;

  cmp_node_type_base(&ntype, CMP_NODE_KUWAHARA, "Kuwahara", NODE_CLASS_OP_FILTER);
  ntype.declare = file_ns::cmp_node_kuwahara_declare;
  ntype.draw_buttons = file_ns::node_composit_buts_kuwahara;
  ntype.initfunc = file_ns::node_composit_init_kuwahara;
  node_type_storage(
      &ntype, "NodeKuwaharaData", node_free_standard_storage, node_copy_standard_storage);
  ntype.get_compositor_operation = file_ns::get_compositor_operation;

  nodeRegisterType(&ntype);
}
