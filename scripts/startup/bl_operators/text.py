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
            self.report(
                {'ERROR_INVALID_INPUT'},
                """Please provide the specific format of the arguments with which the text editor opens files. The supported expansions are as follows:

$file Specifies the absolute path of the file.
$line Specifies the line where the cursor will be placed on. (Optional)
$column Specifies the character position within the $line where the cursor will be placed. (Optional)

Ex: -f $file -l $line -c $column""")
            return {'CANCELLED'}

        if "$file" not in text_editor_args:
            self.report({'ERROR_INVALID_INPUT'}, "Text Editor Args Format must contain $file expansion specification")
            return {'CANCELLED'}

        from string import Template
        import shlex
        for plain_arg in shlex.split(text_editor_args):
            arg = Template(plain_arg).substitute(file=self.filepath, line=self.line, column=self.column)
            args.append(arg)

        try:
            import subprocess
            # whit check=True if process.returncode !=0 a exception will be raised
            process = subprocess.run(args, check=True)
            return {'FINISHED'}
        except subprocess.CalledProcessError as exception:
            self.report({'ERROR'}, exception.output)
            return {'CANCELLED'}


classes = (TEXT_OT_jump_to_file_at_point,)
