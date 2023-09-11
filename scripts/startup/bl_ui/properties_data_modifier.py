# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel, Menu, Operator
from bpy.app.translations import contexts as i18n_contexts


class ModifierButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "modifier"
    bl_options = {'HIDE_HEADER'}


class DATA_PT_modifiers(ModifierButtonsPanel, Panel):
    bl_label = "Modifiers"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type != 'GPENCIL'

    def draw(self, _context):
        layout = self.layout
        layout.operator("wm.call_menu", text="Add Modifier", icon='ADD').name = "OBJECT_MT_modifier_add"
        layout.template_modifiers()


class OBJECT_MT_modifier_add(Menu):
    bl_label = "Add Modifier"
    bl_options = {'SEARCH_ON_KEY_PRESS'}

    def draw(self, context):
        layout = self.layout
        ob_type = context.object.type
        geometry_nodes_supported = ob_type in {'MESH', 'CURVE', 'CURVES', 'FONT', 'SURFACE', 'VOLUME', 'POINTCLOUD'}
        if geometry_nodes_supported:
            layout.operator(
                "object.modifier_add",
                icon='GEOMETRY_NODES',
                text="Geometry Nodes",
                text_ctxt=i18n_contexts.default,
            ).type = 'NODES'
            layout.separator()
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.menu("OBJECT_MT_modifier_add_edit")
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'VOLUME'}:
            layout.menu("OBJECT_MT_modifier_add_generate")
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE', 'VOLUME'}:
            layout.menu("OBJECT_MT_modifier_add_deform")
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.menu("OBJECT_MT_modifier_add_physics")

        if geometry_nodes_supported:
            layout.menu_contents("OBJECT_MT_modifier_add_root_catalogs")


class OBJECT_MT_modifier_add_edit(Menu):
    bl_label = "Edit"

    def draw(self, context):
        layout = self.layout
        ob_type = context.object.type
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Data Transfer",
                icon='MOD_DATA_TRANSFER',
                text_ctxt=i18n_contexts.default,
            ).type = 'DATA_TRANSFER'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.operator(
                "object.modifier_add",
                text="Mesh Cache",
                icon='MOD_MESHDEFORM',
                text_ctxt=i18n_contexts.default,
            ).type = 'MESH_CACHE'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Mesh Sequence Cache",
                icon='MOD_MESHDEFORM',
                text_ctxt=i18n_contexts.default,
            ).type = 'MESH_SEQUENCE_CACHE'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Normal Edit",
                icon='MOD_NORMALEDIT',
                text_ctxt=i18n_contexts.default,
            ).type = 'NORMAL_EDIT'
            layout.operator(
                "object.modifier_add",
                text="Weighted Normal",
                icon='MOD_NORMALEDIT',
                text_ctxt=i18n_contexts.default,
            ).type = 'WEIGHTED_NORMAL'
            layout.operator(
                "object.modifier_add",
                text="UV Project",
                icon='MOD_UVPROJECT',
                text_ctxt=i18n_contexts.default,
            ).type = 'UV_PROJECT'
            layout.operator(
                "object.modifier_add",
                text="UV Warp",
                icon='MOD_UVPROJECT',
                text_ctxt=i18n_contexts.default,
            ).type = 'UV_WARP'
            layout.operator(
                "object.modifier_add",
                text="Vertex Weight Edit",
                icon='MOD_VERTEX_WEIGHT',
                text_ctxt=i18n_contexts.default,
            ).type = 'VERTEX_WEIGHT_EDIT'
            layout.operator(
                "object.modifier_add",
                text="Vertex Weight Mix",
                icon='MOD_VERTEX_WEIGHT',
                text_ctxt=i18n_contexts.default,
            ).type = 'VERTEX_WEIGHT_MIX'
            layout.operator(
                "object.modifier_add",
                text="Vertex Weight Proximity",
                icon='MOD_VERTEX_WEIGHT',
                text_ctxt=i18n_contexts.default,
            ).type = 'VERTEX_WEIGHT_PROXIMITY'
        layout.template_modifier_asset_menu_items(catalog_path=self.bl_label)


class OBJECT_MT_modifier_add_generate(Menu):
    bl_label = "Generate"

    def draw(self, context):
        layout = self.layout
        ob_type = context.object.type
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Array",
                icon='MOD_ARRAY',
                text_ctxt=i18n_contexts.default,
            ).type = 'ARRAY'
            layout.operator(
                "object.modifier_add",
                text="Bevel",
                icon='MOD_BEVEL',
                text_ctxt=i18n_contexts.default,
            ).type = 'BEVEL'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Boolean",
                icon='MOD_BOOLEAN',
                text_ctxt=i18n_contexts.default,
            ).type = 'BOOLEAN'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Build",
                icon='MOD_BUILD',
                text_ctxt=i18n_contexts.default,
            ).type = 'BUILD'
            layout.operator(
                "object.modifier_add",
                text="Decimate",
                icon='MOD_DECIM',
                text_ctxt=i18n_contexts.default,
            ).type = 'DECIMATE'
            layout.operator(
                "object.modifier_add",
                text="Edge Split",
                icon='MOD_EDGESPLIT',
                text_ctxt=i18n_contexts.default,
            ).type = 'EDGE_SPLIT'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Mask",
                icon='MOD_MASK',
                text_ctxt=i18n_contexts.default,
            ).type = 'MASK'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Mirror",
                icon='MOD_MIRROR',
                text_ctxt=i18n_contexts.default,
            ).type = 'MIRROR'
        if ob_type == 'VOLUME':
            layout.operator(
                "object.modifier_add",
                text="Mesh to Volume",
                icon='VOLUME_DATA',
                text_ctxt=i18n_contexts.default,
            ).type = 'MESH_TO_VOLUME'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Multiresolution",
                icon='MOD_MULTIRES',
                text_ctxt=i18n_contexts.default,
            ).type = 'MULTIRES'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Remesh",
                icon='MOD_REMESH',
                text_ctxt=i18n_contexts.default,
            ).type = 'REMESH'
            layout.operator(
                "object.modifier_add",
                text="Screw",
                icon='MOD_SCREW',
                text_ctxt=i18n_contexts.default,
            ).type = 'SCREW'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Skin",
                icon='MOD_SKIN',
                text_ctxt=i18n_contexts.default,
            ).type = 'SKIN'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Solidify",
                icon='MOD_SOLIDIFY',
                text_ctxt=i18n_contexts.default,
            ).type = 'SOLIDIFY'
            layout.operator(
                "object.modifier_add",
                text="Subdivision Surface",
                icon='MOD_SUBSURF',
                text_ctxt=i18n_contexts.default,
            ).type = 'SUBSURF'
            layout.operator(
                "object.modifier_add",
                text="Triangulate",
                icon='MOD_TRIANGULATE',
                text_ctxt=i18n_contexts.default,
            ).type = 'TRIANGULATE'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Volume to Mesh",
                icon='VOLUME_DATA',
                text_ctxt=i18n_contexts.default,
            ).type = 'VOLUME_TO_MESH'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Weld",
                icon='AUTOMERGE_OFF',
                text_ctxt=i18n_contexts.default,
            ).type = 'WELD'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Wireframe",
                icon='MOD_WIREFRAME',
                text_ctxt=i18n_contexts.default,
            ).type = 'WIREFRAME'
        layout.template_modifier_asset_menu_items(catalog_path=self.bl_label)


class OBJECT_MT_modifier_add_deform(Menu):
    bl_label = "Deform"

    def draw(self, context):
        layout = self.layout
        ob_type = context.object.type
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.operator(
                "object.modifier_add",
                text="Armature",
                icon='MOD_ARMATURE',
                text_ctxt=i18n_contexts.default,
            ).type = 'ARMATURE'
            layout.operator(
                "object.modifier_add",
                text="Cast",
                icon='MOD_CAST',
                text_ctxt=i18n_contexts.default,
            ).type = 'CAST'
            layout.operator(
                "object.modifier_add",
                text="Curve",
                icon='MOD_CURVE',
                text_ctxt=i18n_contexts.default,
            ).type = 'CURVE'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Displace",
                icon='MOD_DISPLACE',
                text_ctxt=i18n_contexts.default,
            ).type = 'DISPLACE'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.operator(
                "object.modifier_add",
                text="Hook",
                icon='HOOK',
                text_ctxt=i18n_contexts.default,
            ).type = 'HOOK'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Laplacian Deform",
                icon='MOD_MESHDEFORM',
                text_ctxt=i18n_contexts.default,
            ).type = 'LAPLACIANDEFORM'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.operator(
                "object.modifier_add",
                text="Lattice",
                icon='MOD_LATTICE',
                text_ctxt=i18n_contexts.default,
            ).type = 'LATTICE'
            layout.operator(
                "object.modifier_add",
                text="Mesh Deform",
                icon='MOD_MESHDEFORM',
                text_ctxt=i18n_contexts.default,
            ).type = 'MESH_DEFORM'
            layout.operator(
                "object.modifier_add",
                text="Shrinkwrap",
                icon='MOD_SHRINKWRAP',
                text_ctxt=i18n_contexts.default,
            ).type = 'SHRINKWRAP'
            layout.operator(
                "object.modifier_add",
                text="Simple Deform",
                icon='MOD_SIMPLEDEFORM',
                text_ctxt=i18n_contexts.default,
            ).type = 'SIMPLE_DEFORM'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE'}:
            layout.operator(
                "object.modifier_add",
                text="Smooth",
                icon='MOD_SMOOTH',
                text_ctxt=i18n_contexts.default,
            ).type = 'SMOOTH'
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Smooth Corrective",
                icon='MOD_SMOOTH',
                text_ctxt=i18n_contexts.default,
            ).type = 'CORRECTIVE_SMOOTH'
            layout.operator(
                "object.modifier_add",
                text="Smooth Laplacian",
                icon='MOD_SMOOTH',
                text_ctxt=i18n_contexts.default,
            ).type = 'LAPLACIANSMOOTH'
            layout.operator(
                "object.modifier_add",
                text="Surface Deform",
                icon='MOD_MESHDEFORM',
                text_ctxt=i18n_contexts.default,
            ).type = 'SURFACE_DEFORM'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.operator(
                "object.modifier_add",
                text="Warp",
                icon='MOD_WARP',
                text_ctxt=i18n_contexts.default,
            ).type = 'WARP'
            layout.operator(
                "object.modifier_add",
                text="Wave",
                icon='MOD_WAVE',
                text_ctxt=i18n_contexts.default,
            ).type = 'WAVE'
        if ob_type == 'VOLUME':
            layout.operator(
                "object.modifier_add",
                text="Volume Displace",
                icon='VOLUME_DATA',
                text_ctxt=i18n_contexts.default,
            ).type = 'VOLUME_DISPLACE'
        layout.template_modifier_asset_menu_items(catalog_path=self.bl_label)


class OBJECT_MT_modifier_add_physics(Menu):
    bl_label = "Physics"

    def draw(self, context):
        layout = self.layout
        ob_type = context.object.type
        if ob_type == 'MESH':
            layout.operator(
                "object.modifier_add",
                text="Cloth",
                icon='MOD_CLOTH',
                text_ctxt=i18n_contexts.default,
            ).type = 'CLOTH'
            layout.operator(
                "object.modifier_add",
                text="Collision",
                icon='MOD_PHYSICS',
                text_ctxt=i18n_contexts.default,
            ).type = 'COLLISION'
            layout.operator(
                "object.modifier_add",
                text="Dynamic Paint",
                icon='MOD_DYNAMICPAINT',
                text_ctxt=i18n_contexts.default,
            ).type = 'DYNAMIC_PAINT'
            layout.operator(
                "object.modifier_add",
                text="Explode",
                icon='MOD_EXPLODE',
                text_ctxt=i18n_contexts.default,
            ).type = 'EXPLODE'
            layout.operator(
                "object.modifier_add",
                text="Fluid",
                icon='MOD_FLUIDSIM',
                text_ctxt=i18n_contexts.default,
            ).type = 'FLUID'
            layout.operator(
                "object.modifier_add",
                text="Ocean",
                icon='MOD_OCEAN',
                text_ctxt=i18n_contexts.default,
            ).type = 'OCEAN'
            layout.operator(
                "object.modifier_add",
                text="Particle Instance",
                icon='MOD_PARTICLE_INSTANCE',
                text_ctxt=i18n_contexts.default,
            ).type = 'PARTICLE_INSTANCE'
            layout.operator(
                "object.modifier_add",
                text="Particle System",
                icon='MOD_PARTICLES',
                text_ctxt=i18n_contexts.default,
            ).type = 'PARTICLE_SYSTEM'
        if ob_type in {'MESH', 'CURVE', 'FONT', 'SURFACE', 'LATTICE'}:
            layout.operator(
                "object.modifier_add",
                text="Soft Body",
                icon='MOD_SOFT',
                text_ctxt=i18n_contexts.default,
            ).type = 'SOFT_BODY'
        layout.template_modifier_asset_menu_items(catalog_path=self.bl_label)


class DATA_PT_gpencil_modifiers(ModifierButtonsPanel, Panel):
    bl_label = "Modifiers"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type == 'GPENCIL'

    def draw(self, _context):
        layout = self.layout
        layout.operator_menu_enum("object.gpencil_modifier_add", "type")
        layout.template_grease_pencil_modifiers()


class AddModifierMenu(Operator):
    bl_idname = "object.add_modifier_menu"
    bl_label = "Add Modifier"

    @classmethod
    def poll(cls, context):
        # NOTE: This operator only exists to add a poll to the add modifier shortcut in the property editor.
        space = context.space_data
        return space and space.type == 'PROPERTIES' and space.context == "MODIFIER"

    def invoke(self, context, event):
        return bpy.ops.wm.call_menu(name="OBJECT_MT_modifier_add")


classes = (
    DATA_PT_modifiers,
    OBJECT_MT_modifier_add,
    OBJECT_MT_modifier_add_edit,
    OBJECT_MT_modifier_add_generate,
    OBJECT_MT_modifier_add_deform,
    OBJECT_MT_modifier_add_physics,
    DATA_PT_gpencil_modifiers,
    AddModifierMenu,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)
