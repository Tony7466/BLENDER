/* SPDX-FileCopyrightText: 2006 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup cmpnodes
 */

#include "BKE_context.h"
#include "BKE_lib_id.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "COM_node_operation.hh"

#include "node_composite_util.hh"

/* **************** SWITCH VIEW ******************** */

namespace blender::nodes::node_composite_switchview_cc {

static void node_declare_dynamic(const bNodeTree & /*ntree*/,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  Scene *scene = reinterpret_cast<Scene *>(node.id);
  NodeDeclarationBuilder builder(r_declaration);
  builder.add_output<decl::Color>(N_("Image"));

  if (scene != nullptr) {
    /* add the new views */
    LISTBASE_FOREACH (SceneRenderView *, srv, &scene->r.views) {
      if (srv->viewflag & SCE_VIEW_DISABLE) {
        continue;
      }
      builder.add_input<decl::Color>(N_(srv->name)).default_value({0.0f, 0.0f, 0.0f, 1.0f});
    }
  }
}

static void init_switch_view(const bContext *C, PointerRNA *ptr)
{
  Scene *scene = CTX_data_scene(C);
  bNode *node = (bNode *)ptr->data;

  /* store scene for dynamic declaration */
  node->id = (ID *)scene;
  id_us_plus(node->id);
}

static void node_composit_buts_switch_view_ex(uiLayout *layout,
                                              bContext * /*C*/,
                                              PointerRNA * /*ptr*/)
{
  uiItemFullO(layout,
              "NODE_OT_switch_view_update",
              "Update Views",
              ICON_FILE_REFRESH,
              nullptr,
              WM_OP_INVOKE_DEFAULT,
              0,
              nullptr);
}

using namespace blender::realtime_compositor;

class SwitchViewOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &input = get_input(context().get_view_name());
    Result &result = get_result("Image");
    input.pass_through(result);
  }
};

static NodeOperation *get_compositor_operation(Context &context, DNode node)
{
  return new SwitchViewOperation(context, node);
}

}  // namespace blender::nodes::node_composite_switchview_cc

void register_node_type_cmp_switch_view()
{
  namespace file_ns = blender::nodes::node_composite_switchview_cc;

  static bNodeType ntype;

  cmp_node_type_base(&ntype, CMP_NODE_SWITCH_VIEW, "Switch View", NODE_CLASS_CONVERTER);
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.draw_buttons_ex = file_ns::node_composit_buts_switch_view_ex;
  ntype.initfunc_api = file_ns::init_switch_view;
  ntype.get_compositor_operation = file_ns::get_compositor_operation;

  nodeRegisterType(&ntype);
}
