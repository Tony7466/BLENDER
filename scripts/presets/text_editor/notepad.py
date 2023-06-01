import bpy
import platform

bpy.context.preferences.filepaths.text_editor = "notepad.exe"
bpy.context.preferences.filepaths.text_editor_args = "{file}"
