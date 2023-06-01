import bpy
import platform

bpy.context.preferences.filepaths.text_editor = "code.cmd" if platform.system() == 'Windows' else "code"
bpy.context.preferences.filepaths.text_editor_args = "-g {file}:{line}:{column}"
