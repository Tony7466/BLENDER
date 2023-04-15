# SPDX-License-Identifier: GPL-2.0-or-later
from __future__ import annotations

import bpy
from bpy.types import (
    Operator,
    PropertyGroup,
)
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    StringProperty,
)

from bpy.app.translations import pgettext_tip as tip_


class NodeSetting(PropertyGroup):
    value: StringProperty(
        name="Value",
        description="Python expression to be evaluated "
        "as the initial node setting",
        default="",
    )


# Base class for node "Add" operators.
class NodeAddOperator:

    type: StringProperty(
        name="Node Type",
        description="Node type",
    )
    use_transform: BoolProperty(
        name="Use Transform",
        description="Start transform operator after inserting the node",
        default=False,
    )
    settings: CollectionProperty(
        name="Settings",
        description="Settings to be applied on the newly created node",
        type=NodeSetting,
        options={'SKIP_SAVE'},
    )

    @staticmethod
    def store_mouse_cursor(context, event):
        space = context.space_data
        tree = space.edit_tree

        # convert mouse position to the View2D for later node placement
        if context.region.type == 'WINDOW':
            # convert mouse position to the View2D for later node placement
            space.cursor_location_from_region(
                event.mouse_region_x, event.mouse_region_y)
        else:
            space.cursor_location = tree.view_center

    # XXX explicit node_type argument is usually not necessary,
    # but required to make search operator work:
    # add_search has to override the 'type' property
    # since it's hardcoded in bpy_operator_wrap.c ...
    def create_node(self, context, node_type=None):
        space = context.space_data
        tree = space.edit_tree

        if node_type is None:
            node_type = self.type

        # select only the new node
        for n in tree.nodes:
            n.select = False

        try:
            node = tree.nodes.new(type=node_type)
        except RuntimeError as e:
            self.report({'ERROR'}, str(e))
            return None

        for setting in self.settings:
            # XXX catch exceptions here?
            value = eval(setting.value)
            node_data = node
            node_attr_name = setting.name

            # Support path to nested data.
            if '.' in node_attr_name:
                node_data_path, node_attr_name = node_attr_name.rsplit(".", 1)
                node_data = node.path_resolve(node_data_path)

            try:
                setattr(node_data, node_attr_name, value)
            except AttributeError as e:
                self.report(
                    {'ERROR_INVALID_INPUT'},
                    "Node has no attribute " + setting.name)
                print(str(e))
                # Continue despite invalid attribute

        node.select = True
        tree.nodes.active = node
        node.location = space.cursor_location
        return node

    @classmethod
    def poll(cls, context):
        space = context.space_data
        # needs active node editor and a tree to add nodes to
        return (space and (space.type == 'NODE_EDITOR') and
                space.edit_tree and not space.edit_tree.library)

    # Default execute simply adds a node
    def execute(self, context):
        if self.properties.is_property_set("type"):
            self.create_node(context)
            return {'FINISHED'}
        else:
            return {'CANCELLED'}

    # Default invoke stores the mouse position to place the node correctly
    # and optionally invokes the transform operator
    def invoke(self, context, event):
        self.store_mouse_cursor(context, event)
        result = self.execute(context)

        if self.use_transform and ('FINISHED' in result):
            # removes the node again if transform is canceled
            bpy.ops.node.translate_attach_remove_on_cancel('INVOKE_DEFAULT')

        return result

    @classmethod
    def description(cls, _context, properties):
        nodetype = properties["type"]
        bl_rna = bpy.types.Node.bl_rna_get_subclass(nodetype)
        if bl_rna is not None:
            return tip_(bl_rna.description)
        else:
            return ""


# Simple basic operator for adding a node
class NODE_OT_add_node(NodeAddOperator, Operator):
    """Add a node to the active tree"""
    bl_idname = "node.add_node"
    bl_label = "Add Node"
    bl_options = {'REGISTER', 'UNDO'}


class NODE_OT_collapse_hide_unused_toggle(Operator):
    """Toggle collapsed nodes and hide unused sockets"""
    bl_idname = "node.collapse_hide_unused_toggle"
    bl_label = "Collapse and Hide Unused Sockets"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        space = context.space_data
        # needs active node editor and a tree
        return (space and (space.type == 'NODE_EDITOR') and
                (space.edit_tree and not space.edit_tree.library))

    def execute(self, context):
        space = context.space_data
        tree = space.edit_tree

        for node in tree.nodes:
            if node.select:
                hide = (not node.hide)

                node.hide = hide
                # Note: connected sockets are ignored internally
                for socket in node.inputs:
                    socket.hide = hide
                for socket in node.outputs:
                    socket.hide = hide

        return {'FINISHED'}


class NODE_OT_tree_path_parent(Operator):
    """Go to parent node tree"""
    bl_idname = "node.tree_path_parent"
    bl_label = "Parent Node Tree"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        space = context.space_data
        # needs active node editor and a tree
        return (space and (space.type == 'NODE_EDITOR') and len(space.path) > 1)

    def execute(self, context):
        space = context.space_data

        space.path.pop()

        return {'FINISHED'}


class NodeFunctionSignatureOperator():
    # Dictionary of node types with methods to get the signature
    signature_nodes = {
        'GeometryNodeEvaluateFunction' : lambda node: node.signature,
    }

    param_type: EnumProperty(
        name = "Parameter Type",
        items = [
            ('INPUT', "Input", ""),
            ('OUTPUT', "Output", ""),
        ]
    )

    def params_get(self, signature):
        return signature.inputs if self.param_type == 'INPUT' else signature.outputs 

    @classmethod
    def poll(cls, context):
        snode = context.space_data
        if snode is None:
            return False
        node = context.active_node
        if node is None or node.bl_idname not in cls.signature_nodes:
            return False
        return True


class NODE_OT_function_parameter_add(NodeFunctionSignatureOperator, Operator):
    '''Add a parameter to the function signature'''
    bl_idname = "node.function_parameter_add"
    bl_label = "Add Parameter"
    bl_options = {'REGISTER', 'UNDO'}

    default_socket_type = 'FLOAT'

    def execute(self, context):
        node = context.active_node
        signature = self.signature_nodes[node.bl_idname](node)
        params = self.params_get(signature)

        # Remember index to move the item
        dst_index = min(params.active_index + 1, len(params))
        # Empty name so it is based on the type only
        params.new(self.default_socket_type, "")
        params.move(len(params) - 1, dst_index)
        params.active_index = dst_index

        return {'FINISHED'}


class NODE_OT_function_parameter_remove(NodeFunctionSignatureOperator, Operator):
    '''Remove a parameter from the function signature'''
    bl_idname = "node.function_parameter_remove"
    bl_label = "Remove Parameter"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        node = context.active_node
        signature = self.signature_nodes[node.bl_idname](node)
        params = self.params_get(signature)

        if params.active:
            params.remove(params.active)
            params.active_index = min(params.active_index, len(params) - 1)

        return {'FINISHED'}


class NODE_OT_function_parameter_move(NodeFunctionSignatureOperator, Operator):
    '''Move a parameter item up or down in the signature'''
    bl_idname = "node.function_parameter_move"
    bl_label = "Move Parameter"
    bl_options = {'REGISTER', 'UNDO'}

    direction: EnumProperty(
        name="Direction",
        items=[('UP', "Up", ""), ('DOWN', "Down", "")],
        default = 'UP',
    )

    def execute(self, context):
        node = context.active_node
        signature = self.signature_nodes[node.bl_idname](node)
        params = self.params_get(signature)

        if self.direction == 'UP' and params.active_index > 0:
            params.move(params.active_index, params.active_index - 1)
            params.active_index -= 1
        elif self.direction == 'DOWN' and params.active_index < len(params) - 1:
            params.move(params.active_index, params.active_index + 1)
            params.active_index += 1

        return {'FINISHED'}


classes = (
    NodeSetting,

    NODE_OT_add_node,
    NODE_OT_collapse_hide_unused_toggle,
    NODE_OT_tree_path_parent,
    NODE_OT_function_parameter_add,
    NODE_OT_function_parameter_remove,
    NODE_OT_function_parameter_move,
)
