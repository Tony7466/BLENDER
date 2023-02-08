# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Operator

from bpy.app.translations import pgettext_data as data_


def geometry_node_group_empty_new(add_link=True):
    group = bpy.data.node_groups.new(data_("Geometry Nodes"), 'GeometryNodeTree')
    group.inputs.new('NodeSocketGeometry', data_("Geometry"))
    group.outputs.new('NodeSocketGeometry', data_("Geometry"))
    input_node = group.nodes.new('NodeGroupInput')
    output_node = group.nodes.new('NodeGroupOutput')
    output_node.is_active_output = True

    input_node.select = False
    output_node.select = False

    input_node.location.x = -200 - input_node.width
    output_node.location.x = 200

    if add_link:
        group.links.new(output_node.inputs[0], input_node.outputs[0])

    return group


def geometry_modifier_poll(context):
    ob = context.object

    # Test object support for geometry node modifier
    if not ob or ob.type not in {'MESH', 'POINTCLOUD', 'VOLUME', 'CURVE', 'FONT', 'CURVES'}:
        return False

    return True


class NewGeometryNodesModifier(Operator):
    """Create a new modifier with a new geometry node group"""

    bl_idname = "node.new_geometry_nodes_modifier"
    bl_label = "New Geometry Node Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return geometry_modifier_poll(context)

    def execute(self, context):
        modifier = context.object.modifiers.new(data_("GeometryNodes"), "NODES")

        if not modifier:
            return {'CANCELLED'}

        group = geometry_node_group_empty_new()
        modifier.node_group = group

        return {'FINISHED'}


class NewGeometryNodeTreeAssign(Operator):
    """Create a new geometry node group and assign it to the active modifier"""

    bl_idname = "node.new_geometry_node_group_assign"
    bl_label = "Assign New Geometry Node Group"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return geometry_modifier_poll(context)

    def execute(self, context):
        if context.area.type == 'PROPERTIES':
            modifier = context.modifier
        else:
            modifier = context.object.modifiers.active

        if not modifier:
            return {'CANCELLED'}

        group = geometry_node_group_empty_new()
        modifier.node_group = group

        return {'FINISHED'}


class CreateModifierWrapperGroup(Operator):
    """Create a new node group wrapping the modifier's group"""

    bl_idname = "node.new_geometry_node_group_wrapper"
    bl_label = "Create Wrapper Group"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return geometry_modifier_poll(context)

    def execute(self, context):
        if context.area.type == 'PROPERTIES':
            modifier = context.modifier
        else:
            modifier = context.object.modifiers.active

        if not modifier:
            return {'CANCELLED'}
        old_group = modifier.node_group
        if not old_group:
            return {'CANCELLED'}

        group = geometry_node_group_empty_new(add_link=False)
        new_group_node = group.nodes.new("GeometryNodeGroup")
        new_group_node.node_tree = old_group

        group_input_node = group.nodes["Group Input"]
        group_output_node = group.nodes["Group Output"]
        
        for input in old_group.inputs:
            group.inputs.new()
            if hasattr(input, "default_value"):
                new_group_node.inputs[input.identifier].default_value = modifier[input.identifier]
            else:
                group.links.new(group_input_node.outputs[input.identifier], new_group_node.inputs[input.identifier])
        
        for output in old_group.outputs:
            group.links.new(new_group_node.outputs[input.identifier], group_output_node.inputs[input.identifier])

        modifier.node_group = group

        return {'FINISHED'}


classes = (
    NewGeometryNodesModifier,
    NewGeometryNodeTreeAssign,
    CreateModifierWrapperGroup,
)
