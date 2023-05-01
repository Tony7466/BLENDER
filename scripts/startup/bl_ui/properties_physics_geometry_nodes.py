# SPDX-License-Identifier: GPL-2.0-or-later

from bpy.types import (
    Panel,
)
from bpy.app.translations import pgettext_iface as iface_


class PHYSICS_PT_geometry_nodes(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"
    bl_label = "Geometry Nodes"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def geometry_nodes_objects(cls, context):
        for ob in context.selected_editable_objects:
            if any([modifier.type == 'NODES' for modifier in ob.modifiers]):
                yield ob

    @classmethod
    def poll(cls, context):
        return any(cls.geometry_nodes_objects(context))

    def draw(self, context):
        layout = self.layout

        if len(context.selected_editable_objects) > 1:
            bake_text = iface_("Bake Selected")
        else:
            bake_text = iface_("Bake")

        row = layout.row(align=True)
        row.operator("object.simulation_nodes_cache_bake", text=bake_text).selected = True
        row.operator("object.simulation_nodes_cache_delete", text="", icon='TRASH').selected = True


class PHYSICS_PT_geometry_nodes_cache(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"
    bl_label = ""
    bl_options = {'DEFAULT_CLOSED'}
    bl_parent_id = "PHYSICS_PT_geometry_nodes"

    def draw_header(self, context):
        layout = self.layout
        layout.prop(context.object, "use_simulation_cache", text="Cache")

    def draw(self, context):
        layout = self.layout
        layout.label(text="Button not functional yet")


classes = (
    PHYSICS_PT_geometry_nodes,
    PHYSICS_PT_geometry_nodes_cache,
)


if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
