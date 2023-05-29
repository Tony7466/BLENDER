from bpy.types import Operator
from bpy.props import (IntProperty, StringProperty)


class TEXT_OT_jump_to_file_at_line(Operator):
    bl_idname = "text.jump_to_file_at_line"
    bl_label = "Open Text File at line"
    bl_description = "Edit text file in external text editor"

    filepath: StringProperty(name="filepath")
    line: IntProperty(name="line")

    def execute(self, context):
        if not self.properties.is_property_set("filepath"):
            text = context.space_data.text
            if not text:
                return {'CANCELLED'}
            self.filepath = text.filepath
            self.line = text.current_line_index + 1

        preset = context.preferences.filepaths.text_editor_preset
        args = []
        import platform
        if preset == 'VSCODE':
            args = ["code.cmd" if platform.system() == 'Windows' else "code", "-g", f"{self.filepath}:{self.line}"]
        if preset == 'CUSTOM':
            args = [context.preferences.filepaths.text_editor, f"{self.filepath}"]
        try:
            import subprocess
            process = subprocess.run(args)
            if process.returncode != 0:
                return {'CANCELLED'}
            return {'FINISHED'}
        except:
            return {'CANCELLED'}


classes = (TEXT_OT_jump_to_file_at_line,)
