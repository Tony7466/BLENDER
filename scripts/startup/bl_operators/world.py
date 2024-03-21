import bpy
import bmesh


class WORLD_OT_convert_volume_to_mesh(bpy.types.Operator):
    """Convert the volume of a world to a mesh"""
    bl_label = "Convert World Volume To Mesh"
    bl_options = {'REGISTER', 'UNDO'}
    bl_idname = "world.convert_volume_to_mesh"

    @classmethod
    def poll(cls, context):
        world = cls._world_get(context)
        if not world or not world.use_nodes:
            return False

        camera = context.scene.camera
        if not camera:
            return False

        ntree = world.node_tree
        node = ntree.get_output_node('EEVEE')
        return bool(node.inputs['Volume'].links)

    def execute(self, context):
        cls = self.__class__
        world = cls._world_get(context)
        camera = context.scene.camera
        collection = context.collection
        view_layer = context.view_layer

        world_tree = world.node_tree
        world_output = world_tree.get_output_node('EEVEE')

        # Add World Volume Mesh object to scene
        name = f"{world.name}Volume"
        mesh = bpy.data.meshes.new(name)
        volume = bpy.data.objects.new(name, mesh)
        volume.parent = camera

        bm = bmesh.new()
        bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=camera.data.clip_end)
        bm.to_mesh(mesh)
        bm.free()

        material = bpy.data.materials.new(name)
        mesh.materials.append(material)
        material.use_nodes = True
        volume_tree = material.node_tree
        for node in volume_tree.nodes:
            if node.type != 'OUTPUT_MATERIAL':
                volume_tree.nodes.remove(node)
        volume_output = volume_tree.get_output_node('EEVEE')

        links_to_add = []
        volume_output_name = volume_output.name
        self.__sync_rna_properties(volume_output, world_output)
        self.__sync_node_input(
            volume_tree,
            volume_output,
            volume_output.inputs['Volume'],
            world_output,
            world_output.inputs['Volume'],
            links_to_add)
        self.__sync_links(volume_tree, links_to_add)
        volume_output.name = volume_output.name

        # Remove all volume links from the world node tree.
        for link in world_output.inputs['Volume'].links:
            world_tree.links.remove(link)

        collection.objects.link(volume)
        volume.select_set(True)
        view_layer.objects.active = volume

        return {"FINISHED"}

    @staticmethod
    def _world_get(context):
        if hasattr(context, 'world'):
            return context.world
        return context.scene.world

    def __sync_node_input(
            self,
            dst_tree: bpy.types.NodeTree,
            dst_node: bpy.types.Node,
            dst_socket: bpy.types.NodeSocket,
            src_node: bpy.types.Node,
            src_socket: bpy.types.NodeSocket,
            links_to_add) -> None:
        self.__sync_rna_properties(dst_socket, src_socket)
        for src_link in src_socket.links:
            src_linked_node = src_link.from_node
            dst_linked_node = self.__sync_node(dst_tree, src_linked_node, links_to_add)

            from_socket_index = src_node.outputs.find(src_link.from_socket.name)
            dst_tree.links.new(
                dst_linked_node.outputs[from_socket_index],
                dst_socket
            )

    def __sync_node(self, dst_tree: bpy.types.NodeTree, src_node: bpy.types.Node, links_to_add) -> bpy.types.Node:
        """
        Find the counter part of the src_node in dst_tree. When found return the counter part. When not found
        create the counter part, sync it and return the created node.
        """
        if src_node.name in dst_tree.nodes:
            return dst_tree.nodes[src_node.name]

        dst_node = dst_tree.nodes.new(src_node.bl_idname)

        self.__sync_rna_properties(dst_node, src_node)
        self.__sync_node_inputs(dst_tree, dst_node, src_node, links_to_add)
        return dst_node

    def __sync_rna_properties(self, dst, src) -> None:
        for rna_prop in src.bl_rna.properties:
            if rna_prop.is_readonly:
                continue

            attr_name = rna_prop.identifier
            setattr(dst, attr_name, getattr(src, attr_name))

    def __sync_node_inputs(
            self,
            dst_tree: bpy.types.NodeTree,
            dst_node: bpy.types.Node,
            src_node: bpy.types.Node,
            links_to_add) -> None:
        for index in range(len(src_node.inputs)):
            src_socket = src_node.inputs[index]
            dst_socket = dst_node.inputs[index]
            self.__sync_node_input(dst_tree, dst_node, dst_socket, src_node, src_socket, links_to_add)

    def __sync_links(self, dst_tree: bpy.types.NodeTree, links_to_add) -> None:
        pass


classes = (
    WORLD_OT_convert_volume_to_mesh,
)
