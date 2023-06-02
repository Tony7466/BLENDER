# SPDX-License-Identifier: GPL-2.0-or-later

from bpy.types import Operator
from bpy.props import (IntProperty, StringProperty)


class TEXT_OT_jump_to_file_at_point(Operator):
    bl_idname = "text.jump_to_file_at_point"
    bl_label = "Open Text File at point"
    bl_description = "Edit text file in external text editor"

    filepath: StringProperty(name="filepath")
    line: IntProperty(name="line")
    column: IntProperty(name="column")

    def execute(self, context):
        from string import Template
        import shlex
        import subprocess
        
        if not self.properties.is_property_set("filepath"):
            text = context.space_data.text
            if not text:
                return {'CANCELLED'}
            self.filepath = text.filepath
            self.line = text.current_line_index + 1
            self.column = text.current_character + 1

        text_editor = context.preferences.filepaths.text_editor
        text_editor_args = context.preferences.filepaths.text_editor_args

        if not text_editor_args:
            self.report(
                {'ERROR_INVALID_INPUT'},
                "Provide Text Editor Args Format in File Paths/Applications, see input field tooltip for more information.")
            return {'CANCELLED'}

        if "$file" not in text_editor_args:
            self.report({'ERROR_INVALID_INPUT'}, "Text Editor Args Format must contain $file expansion specification")
            return {'CANCELLED'}

        args = [text_editor]
        template_vars = {"file": self.filepath, "line": self.line, "column": self.column}

        try:
            args.extend([Template(arg).substitute(**template_vars) for arg in shlex.split(text_editor_args)])
        except Exception as ex:
            self.report({'ERROR'}, "Exception parsing template: %r" % ex)
            return {'CANCELLED'}

        try:
            # whit check=True if process.returncode !=0 a exception will be raised
            process = subprocess.run(args, check=True)
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, "Exception running external editor: %r" % ex)
            return {'CANCELLED'}


classes = (TEXT_OT_jump_to_file_at_point,)
