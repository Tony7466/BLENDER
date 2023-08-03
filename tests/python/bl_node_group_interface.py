# SPDX-FileCopyrightText: 2021-2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

import pathlib
import sys
import unittest
import tempfile

import bpy

args = None


# Utility functions for comparing default values.
# Not all socket value types are trivially comparable, e.g. colors.

def cmp_default(test, a, b):
    test.assertEqual(a, b)

def cmp_array(test, a, b):
    test.assertSequenceEqual(a[:], b[:])



class AbstractNodeGroupInterfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.testdir = args.testdir
        cls._tempdir = tempfile.TemporaryDirectory()
        cls.tempdir = pathlib.Path(cls._tempdir.name)

    def setUp(self):
        self.assertTrue(self.testdir.exists(),
                        'Test dir {0} should exist'.format(self.testdir))

        # Make sure we always start with a known-empty file.
        bpy.ops.wm.open_mainfile(filepath=str(self.testdir / "empty.blend"))

        # XXX Will fail when rotation sockets are no longer experimental.
        # Once that happens just remove this line.
        bpy.context.preferences.experimental.use_rotation_socket = True

    def tearDown(self):
        self._tempdir.cleanup()


class NodeGroupInterfaceTests:
    tree_type = None
    group_node_type = None
    # Tree instance where node groups can be added
    main_tree = None

    def make_group_and_instance(self):
        tree = bpy.data.node_groups.new("test", self.tree_type)
        group_node = self.main_tree.nodes.new(self.group_node_type)
        group_node.node_tree = tree
        return tree, group_node

    def test_empty_nodegroup(self):
        tree, group_node = self.make_group_and_instance()

        self.assertFalse(tree.interface.ui_items, "Interface not empty")
        self.assertFalse(group_node.inputs)
        self.assertFalse(group_node.outputs)

    def do_test_invalid_socket_type(self, socket_type):
        tree, group_node = self.make_group_and_instance()

        with self.assertRaises(TypeError):
            in0 = tree.interface.new_socket("Input 0", socket_type=socket_type, is_input=True)
            self.assertIsNone(in0, f"Socket created for invalid type {socket_type}")
        with self.assertRaises(TypeError):
            out0 = tree.interface.new_socket("Output 0", socket_type=socket_type, is_output=True)
            self.assertIsNone(out0, f"Socket created for invalid type {socket_type}")

    def do_test_sockets_in_out(self, socket_type):
        tree, group_node = self.make_group_and_instance()

        out0 = tree.interface.new_socket("Output 0", socket_type=socket_type, is_output=True)
        self.assertIsNotNone(out0, f"Could not create socket of type {socket_type}")

        in0 = tree.interface.new_socket("Input 0", socket_type=socket_type, is_input=True)
        self.assertIsNotNone(in0, f"Could not create socket of type {socket_type}")

        in1 = tree.interface.new_socket("Input 1", socket_type=socket_type, is_input=True)
        self.assertIsNotNone(in1, f"Could not create socket of type {socket_type}")

        out1 = tree.interface.new_socket("Output 1", socket_type=socket_type, is_output=True)
        self.assertIsNotNone(out1, f"Could not create socket of type {socket_type}")

        inout0 = tree.interface.new_socket("Input/Output 0", socket_type=socket_type, is_output=True, is_input=True)
        self.assertIsNotNone(inout0, f"Could not create socket of type {socket_type}")

        self.assertSequenceEqual([(s.name, s.bl_idname) for s in group_node.inputs], [
            ("Input 0", socket_type),
            ("Input 1", socket_type),
            ("Input/Output 0", socket_type),
            ])
        self.assertSequenceEqual([(s.name, s.bl_idname) for s in group_node.outputs], [
            ("Output 0", socket_type),
            ("Output 1", socket_type),
            ("Input/Output 0", socket_type),
            ])

    def do_test_socket_type(self, socket_type, compare_value=None):
        tree, group_node = self.make_group_and_instance()

        in0 = tree.interface.new_socket("Input 0", socket_type=socket_type, is_input=True)
        out0 = tree.interface.new_socket("Output 0", socket_type=socket_type, is_output=True)
        self.assertIsNotNone(in0, f"Could not create socket of type {socket_type}")
        self.assertIsNotNone(out0, f"Could not create socket of type {socket_type}")

        if compare_value:
            compare_value(self, in0.default_value, group_node.inputs[0].default_value)


class GeometryNodeGroupInterfaceTest(AbstractNodeGroupInterfaceTest, NodeGroupInterfaceTests):
    tree_type = "GeometryNodeTree"
    group_node_type = "GeometryNodeGroup"

    def setUp(self):
        super().setUp()
        self.main_tree = bpy.data.node_groups.new("main", self.tree_type)

    def test_sockets_in_out(self):
        self.do_test_sockets_in_out("NodeSocketFloat")

    def test_all_socket_types(self):
        self.do_test_invalid_socket_type("INVALID_SOCKET_TYPE_11!1")
        self.do_test_socket_type("NodeSocketBool", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketCollection")
        self.do_test_socket_type("NodeSocketColor", compare_value=cmp_array)
        self.do_test_socket_type("NodeSocketFloat", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketGeometry")
        self.do_test_socket_type("NodeSocketImage", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketInt", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketMaterial", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketObject", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketRotation", compare_value=cmp_default)
        self.do_test_invalid_socket_type("NodeSocketShader")
        self.do_test_socket_type("NodeSocketString", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketTexture", compare_value=cmp_default)
        self.do_test_socket_type("NodeSocketVector", compare_value=cmp_array)
        self.do_test_invalid_socket_type("NodeSocketVirtual")

class ShaderNodeGroupInterfaceTest(AbstractNodeGroupInterfaceTest, NodeGroupInterfaceTests):
    tree_type = "ShaderNodeTree"
    group_node_type = "ShaderNodeGroup"

    def setUp(self):
        super().setUp()
        self.material = bpy.data.materials.new("test")
        self.material.use_nodes = True
        self.main_tree = self.material.node_tree

    def test_invalid_socket_type(self):
        self.do_test_invalid_socket_type("INVALID_SOCKET_TYPE_11!1")

    def test_sockets_in_out(self):
        self.do_test_sockets_in_out("NodeSocketFloat")

    def test_all_socket_types(self):
        self.do_test_invalid_socket_type("NodeSocketBool")
        self.do_test_invalid_socket_type("NodeSocketCollection")
        self.do_test_socket_type("NodeSocketColor", compare_value=cmp_array)
        self.do_test_socket_type("NodeSocketFloat", compare_value=cmp_default)
        self.do_test_invalid_socket_type("NodeSocketGeometry")
        self.do_test_invalid_socket_type("NodeSocketImage")
        self.do_test_invalid_socket_type("NodeSocketInt")
        self.do_test_invalid_socket_type("NodeSocketMaterial")
        self.do_test_invalid_socket_type("NodeSocketObject")
        self.do_test_invalid_socket_type("NodeSocketRotation")
        self.do_test_socket_type("NodeSocketShader")
        self.do_test_invalid_socket_type("NodeSocketString")
        self.do_test_invalid_socket_type("NodeSocketTexture")
        self.do_test_socket_type("NodeSocketVector", compare_value=cmp_array)
        self.do_test_invalid_socket_type("NodeSocketVirtual")


class CompositorNodeGroupInterfaceTest(AbstractNodeGroupInterfaceTest, NodeGroupInterfaceTests):
    tree_type = "CompositorNodeTree"
    group_node_type = "CompositorNodeGroup"

    def setUp(self):
        super().setUp()
        self.scene = bpy.data.scenes.new("test")
        self.scene.use_nodes = True
        self.main_tree = self.scene.node_tree

    def test_invalid_socket_type(self):
        self.do_test_invalid_socket_type("INVALID_SOCKET_TYPE_11!1")

    def test_sockets_in_out(self):
        self.do_test_sockets_in_out("NodeSocketFloat")

    def test_all_socket_types(self):
        self.do_test_invalid_socket_type("NodeSocketBool")
        self.do_test_invalid_socket_type("NodeSocketCollection")
        self.do_test_socket_type("NodeSocketColor", compare_value=cmp_array)
        self.do_test_socket_type("NodeSocketFloat", compare_value=cmp_default)
        self.do_test_invalid_socket_type("NodeSocketGeometry")
        self.do_test_invalid_socket_type("NodeSocketImage")
        self.do_test_invalid_socket_type("NodeSocketInt")
        self.do_test_invalid_socket_type("NodeSocketMaterial")
        self.do_test_invalid_socket_type("NodeSocketObject")
        self.do_test_invalid_socket_type("NodeSocketRotation")
        self.do_test_invalid_socket_type("NodeSocketShader")
        self.do_test_invalid_socket_type("NodeSocketString")
        self.do_test_invalid_socket_type("NodeSocketTexture")
        self.do_test_socket_type("NodeSocketVector", compare_value=cmp_array)
        self.do_test_invalid_socket_type("NodeSocketVirtual")


def main():
    global args
    import argparse

    if '--' in sys.argv:
        argv = [sys.argv[0]] + sys.argv[sys.argv.index('--') + 1:]
    else:
        argv = sys.argv

    parser = argparse.ArgumentParser()
    parser.add_argument('--testdir', required=True, type=pathlib.Path)
    args, remaining = parser.parse_known_args(argv)

    unittest.main(argv=remaining)


if __name__ == "__main__":
    main()
