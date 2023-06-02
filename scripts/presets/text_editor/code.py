import bpy
import platform

bpy.context.preferences.filepaths.text_editor_args = "-g $file:$line:$column"

match platform.system():
    case 'Windows':
        bpy.context.preferences.filepaths.text_editor = "code.cmd"
    case other:
        bpy.context.preferences.filepaths.text_editor = "code"
