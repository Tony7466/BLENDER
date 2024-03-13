# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
import bpy
from bpy.types import Panel, Menu, UIList
from rna_prop_ui import PropertyPanel


class DataButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return hasattr(context, "grease_pencil") and context.grease_pencil


class LayerDataButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        grease_pencil = context.grease_pencil
        return grease_pencil and grease_pencil.layers.active


class GREASE_PENCIL_UL_masks(UIList):
    def draw_item(self, _context, layout, _data, item, icon, _active_data, _active_propname, _index):
        mask = item
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            row = layout.row(align=True)
            row.prop(mask, "name", text="", emboss=False, icon_value=icon)
            row.prop(mask, "invert", text="", emboss=False)
            row.prop(mask, "hide", text="", emboss=False)
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.prop(mask, "name", text="", emboss=False, icon_value=icon)

class DATA_PT_context_grease_pencil(DataButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout

        ob = context.object
        grease_pencil = context.grease_pencil
        space = context.space_data

        if ob:
            layout.template_ID(ob, "data")
        elif grease_pencil:
            layout.template_ID(space, "pin_id")


class GREASE_PENCIL_MT_grease_pencil_add_layer_extra(Menu):
    bl_label = "Add Extra"

    def draw(self, context):
        layout = self.layout
        grease_pencil = context.object.data
        space = context.space_data

        if space.type == 'PROPERTIES':
            layout.operator("grease_pencil.layer_group_add", text="Add Group")

        layout.separator()
        layout.operator("grease_pencil.layer_duplicate", text="Duplicate", icon='DUPLICATE')
        layout.operator("grease_pencil.layer_duplicate", text="Duplicate Empty Keyframes").empty_keyframes = True

        layout.separator()
        layout.operator("grease_pencil.layer_reveal", icon='RESTRICT_VIEW_OFF', text="Show All")
        layout.operator("grease_pencil.layer_hide", icon='RESTRICT_VIEW_ON', text="Hide Others").unselected = True

        layout.separator()
        layout.operator("grease_pencil.layer_lock_all", icon='LOCKED', text="Lock All")
        layout.operator("grease_pencil.layer_lock_all", icon='UNLOCKED', text="Unlock All").lock = False

        layout.separator()
        layout.prop(grease_pencil, "use_autolock_layers", text="Autolock Inactive Layers")


class DATA_PT_grease_pencil_layers(DataButtonsPanel, Panel):
    bl_label = "Layers"

    def draw(self, context):
        layout = self.layout

        grease_pencil = context.grease_pencil
        layer = grease_pencil.layers.active

        row = layout.row()
        row.template_grease_pencil_layer_tree()

        col = row.column()
        sub = col.column(align=True)
        sub.operator_context = 'EXEC_DEFAULT'
        sub.operator("grease_pencil.layer_add", icon='ADD', text="")
        sub.menu("GREASE_PENCIL_MT_grease_pencil_add_layer_extra", icon='DOWNARROW_HLT', text="")

        col.operator("grease_pencil.layer_remove", icon='REMOVE', text="")

        # Layer main properties
        if layer:
            layout.use_property_split = True
            layout.use_property_decorate = True
            col = layout.column(align=True)

            row = layout.row(align=True)
            row.prop(layer, "opacity", text="Opacity", slider=True)


class DATA_PT_grease_pencil_layer_masks(LayerDataButtonsPanel, Panel):
    bl_label = "Masks"
    bl_parent_id = "DATA_PT_grease_pencil_layers"
    bl_options = {'DEFAULT_CLOSED'}

    def draw_header(self, context):
        grease_pencil = context.grease_pencil
        layer = grease_pencil.layers.active

        self.layout.prop(layer, "use_masks", text="")

    def draw(self, context):
        layout = self.layout
        grease_pencil = context.grease_pencil
        layer = grease_pencil.layers.active

        layout = self.layout
        layout.enabled = layer.use_masks

        if not layer:
            return
        
        rows = 4
        row = layout.row()
        col = row.column()
        col.template_list("GREASE_PENCIL_UL_masks", "", layer, "mask_layers", layer.mask_layers,
                          "active_mask_index", rows=rows, sort_lock=True)

        # TODO:
        # col2 = row.column(align=True)
        # col2.menu("GPENCIL_MT_layer_mask_menu", icon='ADD', text="")
        # col2.operator("gpencil.layer_mask_remove", icon='REMOVE', text="")

        # col2.separator()

        # sub = col2.column(align=True)
        # sub.operator("gpencil.layer_mask_move", icon='TRIA_UP', text="").type = 'UP'
        # sub.operator("gpencil.layer_mask_move", icon='TRIA_DOWN', text="").type = 'DOWN'


class DATA_PT_grease_pencil_layer_transform(LayerDataButtonsPanel, Panel):
    bl_label = "Transform"
    bl_parent_id = "DATA_PT_grease_pencil_layers"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        grease_pencil = context.grease_pencil
        layer = grease_pencil.layers.active
        layout.active = not layer.lock

        row = layout.row(align=True)
        row.prop(layer, "translation")

        row = layout.row(align=True)
        row.prop(layer, "rotation")

        row = layout.row(align=True)
        row.prop(layer, "scale")


class DATA_PT_grease_pencil_layer_relations(LayerDataButtonsPanel, Panel):
    bl_label = "Relations"
    bl_parent_id = "DATA_PT_grease_pencil_layers"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        grease_pencil = context.grease_pencil
        layer = grease_pencil.layers.active
        layout.active = not layer.lock

        row = layout.row(align=True)
        row.prop(layer, "parent", text="Parent")

        if layer.parent and layer.parent.type == 'ARMATURE':
            row = layout.row(align=True)
            row.prop_search(layer, "parent_bone", layer.parent.data, "bones", text="Bone")

        layout.separator()

        col = layout.row(align=True)
        col.prop(layer, "pass_index")


class DATA_PT_grease_pencil_custom_props(DataButtonsPanel, PropertyPanel, Panel):
    _context_path = "object.data"
    _property_type = bpy.types.GreasePencilv3


classes = (
    GREASE_PENCIL_UL_masks,
    DATA_PT_context_grease_pencil,
    DATA_PT_grease_pencil_layers,
    DATA_PT_grease_pencil_layer_masks,
    DATA_PT_grease_pencil_layer_transform,
    DATA_PT_grease_pencil_layer_relations,
    DATA_PT_grease_pencil_custom_props,
    GREASE_PENCIL_MT_grease_pencil_add_layer_extra,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
