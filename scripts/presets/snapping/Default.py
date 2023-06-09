
attributes = (
    "snap_target",
    "snap_elements",
    "snap_filter_active",
    "snap_filter_edit",
    "snap_filter_nonedit",
    "snap_filter_nonselectable",
    "use_snap_grid_absolute",
    "use_snap_peel_object",
    "use_snap_to_same_target",
    "snap_face_nearest_steps",
    "use_snap_align_rotation",
    "use_snap_backface_culling",
    "use_snap_translate",
    "use_snap_rotate",
    "use_snap_scale",
    "show_snap_filter",
)

import bpy
toolsettings = bpy.context.scene.tool_settings
props = bpy.types.ToolSettings.bl_rna.properties
for attr in attributes:
    prop = props[attr]
    if prop.is_enum_flag:
        setattr(toolsettings, attr, prop.default_flag)
    else:
        setattr(toolsettings, attr, prop.default)
