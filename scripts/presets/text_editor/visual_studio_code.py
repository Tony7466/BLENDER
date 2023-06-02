import bpy
import platform

bpy.context.preferences.filepaths.text_editor_args = "-g $file:$line:$column"

match platform.system():
    case 'Windows':
        bpy.context.preferences.filepaths.text_editor = "code.cmd"
    case _:
        bpy.context.preferences.filepaths.text_editor = "code"
