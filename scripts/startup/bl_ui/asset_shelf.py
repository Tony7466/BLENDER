# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import (
    Panel,
)


class ASSETSHELF_PT_display(Panel):
    bl_label = "Display Settings"
    # Doesn't actually matter. Panel is instanced through popover only.
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'HEADER'

    def draw(self, context):
        layout = self.layout

        shelf = context.asset_shelf

        layout.prop(shelf, "show_names", text="Names")
        layout.prop(shelf, "preview_size")

    @classmethod
    def poll(cls, context):
        return context.asset_shelf is not None


class Utils:
    """Class with public utilities."""
    @staticmethod
    def has_active_asset_shelf(context):
        for region in context.area.regions:
            if region.type != 'ASSET_SHELF':
                continue

            with context.temp_override(region=region):
                return hasattr(context, "asset_shelf") and context.asset_shelf is not None
        return False


classes = (
    ASSETSHELF_PT_display,
)


if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)
