"""
File Handler: Operator that imports multiple files
+++++++++++++++
Todo
"""

import bpy
from mathutils import Vector


class ShaderScriptImport(bpy.types.Operator):
    """Test importer that creates scripts nodes from .txt files"""
    bl_idname = "shader.script_import"
    bl_label = "Import a text file as a script node"

    """
    This Operator can import multiple .txt files, we need following directory and files
    properties that the file handler will use to set files path data
    """
    directory: bpy.props.StringProperty(subtype='FILE_PATH', options={'SKIP_SAVE'})
    files: bpy.props.CollectionProperty(type=bpy.types.OperatorFileListElement, options={'SKIP_SAVE'})

    @classmethod
    def poll(cls, context):
        return (context.region and context.region.type == 'WINDOW'
                and context.area and context.area.ui_type == 'ShaderNodeTree'
                and context.object and context.object.type == 'MESH'
                and context.material)

    def execute(self, context):
        """ The directory property need to be set. """
        if not self.directory:
            return {'CANCELLED'}
        x = 0.0
        y = 0.0
        for file in self.files:
            """
            Calls to the operator can send unfiltered file names,
            ensure the file extension is .txt
            """
            if file.name.endswith(".txt"):
                node_tree = context.material.node_tree
                text_node = node_tree.nodes.new(type="ShaderNodeScript")
                text_node.mode = 'EXTERNAL'
                import os
                filepath = os.path.join(self.directory, file.name)
                text_node.filepath = filepath
                text_node.location = Vector((x, y))
                x += 20.0
                y -= 20.0
        return {'FINISHED'}

    """
    By default the file handler invokes the operator with the directory and files properties set,
    if this properties are set we can show a menu or just execute the operator.
    This specific behavior requires to set 'options={'SKIP_SAVE'}' in the properties options so we can avoid
    reused data from previous operator calls and check if the operator is called with new drag&drop path data or not.
    """

    def invoke(self, context, event):
        if self.directory:
            return self.execute(context)
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class Shader_FH_script_import(bpy.types.FileHandler):
    bl_idname = "Shader_FH_script_import"
    bl_label = "File handler for shader script node import"
    bl_import_operator = "shader.script_import"
    bl_file_extensions = ".txt"

    @classmethod
    def poll_drop(cls, context):
        return (context.region and context.region.type == 'WINDOW'
                and context.area and context.area.ui_type == 'ShaderNodeTree')


bpy.utils.register_class(ShaderScriptImport)
bpy.utils.register_class(Shader_FH_script_import)
