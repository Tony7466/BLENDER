import bpy
toolsettings = bpy.context.scene.tool_settings

toolsettings.snap_target = 'MEDIAN'
toolsettings.snap_elements = {'FACE_PROJECT', 'FACE_NEAREST', 'VERTEX'}
toolsettings.snap_filter_active = {'FACE', 'EDGE_MIDPOINT', 'VOLUME', 'EDGE', 'EDGE_PERPENDICULAR', 'VERTEX'}
toolsettings.snap_filter_edit = {'FACE', 'EDGE_MIDPOINT', 'VOLUME', 'FACE_PROJECT', 'EDGE', 'EDGE_PERPENDICULAR', 'FACE_NEAREST', 'VERTEX'}
toolsettings.snap_filter_nonedit = {'FACE_PROJECT', 'FACE_NEAREST'}
toolsettings.snap_filter_nonselectable = {'FACE', 'EDGE_MIDPOINT', 'VOLUME', 'FACE_PROJECT', 'EDGE', 'EDGE_PERPENDICULAR', 'FACE_NEAREST', 'VERTEX'}
toolsettings.use_snap_grid_absolute = False
toolsettings.use_snap_peel_object = False
toolsettings.use_snap_to_same_target = False
toolsettings.snap_face_nearest_steps = 1
toolsettings.use_snap_align_rotation = False
toolsettings.use_snap_backface_culling = False
toolsettings.use_snap_translate = True
toolsettings.use_snap_rotate = False
toolsettings.use_snap_scale = False
toolsettings.show_snap_filter = {'NONEDITED', 'ACTIVE'}
