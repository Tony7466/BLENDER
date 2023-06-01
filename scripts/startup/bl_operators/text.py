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
        if not self.properties.is_property_set("filepath"):
            text = context.space_data.text
            if not text:
                return {'CANCELLED'}
            self.filepath = text.filepath
            self.line = text.current_line_index + 1
            self.column = text.current_character + 1

        text_editor = context.preferences.filepaths.text_editor
        text_editor_args = context.preferences.filepaths.text_editor_args

        args = [text_editor]

        if not text_editor_args:
            text_editor_args = "{file} {line} {column}"

        text_editor_args = text_editor_args.format(file=self.filepath, line=self.line, column=self.column)
        args.extend(text_editor_args.split(" "))

        try:
            import subprocess
            process = subprocess.run(args)
            if process.returncode != 0:
                return {'CANCELLED'}
            return {'FINISHED'}
        except:
            return {'CANCELLED'}


classes = (TEXT_OT_jump_to_file_at_point,)
