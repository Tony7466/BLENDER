# SPDX-FileCopyrightText: 2021-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import pathlib
import sys
import unittest
import tempfile
import math
from dataclasses import dataclass

import bpy

args = None


class FieldTypeInferenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.testdir = args.testdir
        cls._tempdir = tempfile.TemporaryDirectory()
        cls.tempdir = pathlib.Path(cls._tempdir.name)

    def setUp(self):
        self.assertTrue(self.testdir.exists(),
                        'Test dir {0} should exist'.format(self.testdir))

    def tearDown(self):
        self._tempdir.cleanup()

    # Note: field types are not exposed in RNA yet, so we have to use the
    # socket display shape as a proxy for testing.

    def assert_value_socket(self, socket):
        self.assertEqual(socket.display_shape, 'CIRCLE')

    def assert_field_socket(self, socket):
        self.assertEqual(socket.display_shape, 'DIAMOND')

    def assert_value_or_field_socket(self, socket):
        self.assertEqual(socket.display_shape, 'DIAMOND_DOT')

    def test_field_inferencing(self):
        bpy.ops.wm.open_mainfile(filepath=str(self.testdir / "field_type_inferencing.blend"))
        self.assertEqual(bpy.data.version, (4, 1, 0))

        tree = bpy.data.node_groups['Geometry Nodes']

        # Unconnected nodes

        node = tree.nodes['Group Input']
        self.assertEqual(node.bl_idname, "NodeGroupInput")
        self.assert_value_socket(node.outputs['Geometry'])
        # Field-capable group inputs are resolved as fields for maximum compatibility.
        self.assert_field_socket(node.outputs['Socket'])

        node = tree.nodes['Group Output']
        self.assertEqual(node.bl_idname, "NodeGroupOutput")
        self.assert_value_socket(node.inputs['Geometry'])
        self.assert_value_or_field_socket(node.inputs['Socket'])

        node = tree.nodes['Mix']
        self.assertEqual(node.bl_idname, "ShaderNodeMix")

        node = tree.nodes['Cube']
        self.assertEqual(node.bl_idname, "GeometryNodeMeshCube")

        node = tree.nodes['Capture Attribute']
        self.assertEqual(node.bl_idname, "GeometryNodeCaptureAttribute")
        self.assert_value_socket(node.inputs['Geometry'])
        self.assert_value_or_field_socket(node.inputs['Value'])
        # Capture Attribute output is always a field.
        self.assert_field_socket(node.outputs['Attribute'])


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
