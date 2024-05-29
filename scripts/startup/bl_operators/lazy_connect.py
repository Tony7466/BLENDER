# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Operator, Menu
from bpy.props import (
    IntProperty,
    StringProperty,
    BoolProperty,
)
from bpy_extras.node_utils import connect_sockets
from .node_editor.node_functions import (
    NodeEditorBase,
    NodeEditorMenuBase,
    force_update,
    node_editor_poll,
    node_under_cursor,
    autolink,
)
from .node_editor.draw import draw_callback_node_outline


class NODE_MT_lazy_connect_outputs(Menu, NodeEditorMenuBase):
    bl_idname = "NODE_MT_lazy_connect_outputs"
    bl_label = "From:"

    def draw(self, context):
        layout = self.layout
        nodes = context.space_data.edit_tree.nodes

        n1 = nodes[context.window_manager.nodes_lazy_source]
        for index, output in enumerate(n1.outputs):
            # Only show sockets that are exposed.
            if output.enabled:
                layout.operator(
                    NODE_OT_call_inputs_menu.bl_idname,
                    text=output.name,
                    icon="RADIOBUT_OFF").from_socket = index


class NODE_MT_lazy_connect_inputs(Menu, NodeEditorMenuBase):
    bl_idname = "NODE_MT_lazy_connect_inputs"
    bl_label = "To:"

    def draw(self, context):
        layout = self.layout
        nodes = context.space_data.edit_tree.nodes

        n2 = nodes[context.window_manager.nodes_lazy_target]

        for index, input in enumerate(n2.inputs):
            # Only show sockets that are exposed. This prevents, for example, the scale value socket
            # of the vector math node being added to the list when the mode is not 'SCALE'.
            if input.enabled:
                op = layout.operator(NODE_OT_make_link.bl_idname, text=input.name, icon="FORWARD")
                op.from_socket = context.window_manager.nodes_lazy_source_socket
                op.to_socket = index


class NODE_OT_call_inputs_menu(Operator, NodeEditorBase):
    """Link from this output"""
    bl_idname = 'node.call_inputs_menu'
    bl_label = 'Make Link'
    bl_options = {'REGISTER', 'UNDO'}

    from_socket: IntProperty(
    )

    def execute(self, context):
        nodes = context.space_data.edit_tree.nodes

        context.window_manager.nodes_lazy_source_socket = self.from_socket

        n1 = nodes[context.window_manager.nodes_lazy_source]
        n2 = nodes[context.window_manager.nodes_lazy_target]
        if len(n2.inputs) > 1:
            bpy.ops.wm.call_menu("INVOKE_DEFAULT", name=NODE_MT_lazy_connect_inputs.bl_idname)
        elif len(n2.inputs) == 1:
            connect_sockets(n1.outputs[self.from_socket], n2.inputs[0])
        return {'FINISHED'}


class NODE_OT_make_link(Operator, NodeEditorBase):
    """Make a link from one socket to another"""
    bl_idname = 'node.make_link'
    bl_label = 'Make Link'
    bl_options = {'REGISTER', 'UNDO'}

    from_socket: IntProperty(
    )
    to_socket: IntProperty(
    )

    def execute(self, context):
        nodes = context.space_data.edit_tree.nodes

        n1 = nodes[context.window_manager.nodes_lazy_source]
        n2 = nodes[context.window_manager.nodes_lazy_target]

        connect_sockets(n1.outputs[self.from_socket], n2.inputs[self.to_socket])

        force_update(context)

        return {'FINISHED'}


class NODE_OT_lazy_connect(Operator, NodeEditorBase):
    """Connect two nodes without clicking a specific socket (automatically determined)"""
    bl_idname = "node.lazy_connect"
    bl_label = "Lazy Connect"
    bl_options = {'REGISTER', 'UNDO'}

    with_menu: BoolProperty(
    )

    @classmethod
    def poll(cls, context):
        return node_editor_poll(cls, context)

    def modal(self, context, event):
        context.area.tag_redraw()
        nodes = context.space_data.edit_tree.nodes
        cont = True

        node1 = None
        if not context.window_manager.nodes_lazy_drawing:
            node1 = node_under_cursor(nodes, context, event)
            if node1:
                context.window_manager.nodes_lazy_drawing = node1.name
        else:
            if context.window_manager.nodes_lazy_drawing != 'STOP':
                node1 = nodes[context.window_manager.nodes_lazy_drawing]

        context.window_manager.nodes_lazy_source = node1.name
        context.window_manager.nodes_lazy_target = node_under_cursor(nodes, context, event).name

        if event.type == 'MOUSEMOVE':
            self.mouse_path.append((event.mouse_region_x, event.mouse_region_y))

        elif event.type == 'RIGHTMOUSE' and event.value == 'RELEASE':
            bpy.types.SpaceNodeEditor.draw_handler_remove(self._handle, 'WINDOW')

            node2 = None
            node2 = node_under_cursor(nodes, context, event)
            if node2:
                context.window_manager.nodes_lazy_drawing = node2.name

            if node1 == node2:
                cont = False

            link_success = False
            if cont:
                if node1 and node2:
                    original_sel = []
                    original_unsel = []
                    for node in nodes:
                        if node.select:
                            node.select = False
                            original_sel.append(node)
                        else:
                            original_unsel.append(node)
                    node1.select = True
                    node2.select = True

                    if self.with_menu:
                        if len(node1.outputs) > 1 and node2.inputs:
                            bpy.ops.wm.call_menu('INVOKE_DEFAULT', name=NODE_MT_lazy_connect_outputs.bl_idname)
                        elif len(node1.outputs) == 1:
                            bpy.ops.node.call_inputs_menu(from_socket=0)
                    else:
                        link_success = autolink(node1, node2)

                    for node in original_sel:
                        node.select = True
                    for node in original_unsel:
                        node.select = False

            if link_success:
                force_update(context)
            context.window_manager.nodes_lazy_drawing = ""
            return {'FINISHED'}

        elif event.type == 'ESC':
            bpy.types.SpaceNodeEditor.draw_handler_remove(self._handle, 'WINDOW')
            return {'CANCELLED'}

        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        if context.area.type == 'NODE_EDITOR':
            nodes = context.space_data.edit_tree.nodes
            node = node_under_cursor(nodes, context, event)
            if node:
                context.window_manager.nodes_lazy_drawing = node.name

            # The arguments we pass the the callback.
            mode = "LINK"
            if self.with_menu:
                mode = "LINKMENU"
            args = (self, context, mode)

            # Add the region OpenGL drawing callback.
            # Draw in view space with 'POST_VIEW' and 'PRE_VIEW'.
            self._handle = bpy.types.SpaceNodeEditor.draw_handler_add(
                draw_callback_node_outline, args, 'WINDOW', 'POST_PIXEL')

            self.mouse_path = []

            context.window_manager.modal_handler_add(self)
            return {'RUNNING_MODAL'}
        else:
            self.report({'WARNING'}, "Node editor could not be found. Cannot run the operator")
            return {'CANCELLED'}


classes = (
    NODE_OT_lazy_connect,
    NODE_OT_make_link,
    NODE_OT_call_inputs_menu,
    NODE_MT_lazy_connect_outputs,
    NODE_MT_lazy_connect_inputs,
)

def register():
    # properties
    bpy.types.WindowManager.nodes_lazy_drawing = StringProperty(
        name="Busy Drawing!",
        default="",
        description="An internal property used to store only the first mouse position")
    bpy.types.WindowManager.nodes_lazy_source = StringProperty(
        name="Lazy Source!",
        default="x",
        description="An internal property used to store the first node in a Lazy Connect operation")
    bpy.types.WindowManager.nodes_lazy_target = StringProperty(
        name="Lazy Target!",
        default="x",
        description="An internal property used to store the last node in a Lazy Connect operation")
    bpy.types.WindowManager.nodes_lazy_source_socket = IntProperty(
        name="Source Socket!",
        default=0,
        description="An internal property used to store the source socket in a Lazy Connect operation")

def unregister():
    # properties
    del bpy.types.Scene.nodes_lazy_drawing
    del bpy.types.Scene.nodes_lazy_source
    del bpy.types.Scene.nodes_lazy_target
    del bpy.types.Scene.nodes_lazy_source_socket
