# SPDX-FileCopyrightText: 2021-2023 Blender Foundation
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


type_info = {
    ("VALUE", "NONE"): "NodeSocketFloat",
    ("VALUE", "UNSIGNED"): "NodeSocketFloatUnsigned",
    ("VALUE", "PERCENTAGE"): "NodeSocketFloatPercentage",
    ("VALUE", "FACTOR"): "NodeSocketFloatFactor",
    ("VALUE", "ANGLE"): "NodeSocketFloatAngle",
    ("VALUE", "TIME"): "NodeSocketFloatTime",
    ("VALUE", "TIME_ABSOLUTE"): "NodeSocketFloatTimeAbsolute",
    ("VALUE", "DISTANCE"): "NodeSocketFloatDistance",
    ("INT", "NONE"): "NodeSocketInt",
    ("INT", "UNSIGNED"): "NodeSocketIntUnsigned",
    ("INT", "PERCENTAGE"): "NodeSocketIntPercentage",
    ("INT", "FACTOR"): "NodeSocketIntFactor",
    ("BOOLEAN", "NONE"): "NodeSocketBool",
    ("ROTATION", "NONE"): "NodeSocketRotation",
    ("VECTOR", "NONE"): "NodeSocketVector",
    ("VECTOR", "TRANSLATION"): "NodeSocketVectorTranslation",
    ("VECTOR", "DIRECTION"): "NodeSocketVectorDirection",
    ("VECTOR", "VELOCITY"): "NodeSocketVectorVelocity",
    ("VECTOR", "ACCELERATION"): "NodeSocketVectorAcceleration",
    ("VECTOR", "EULER"): "NodeSocketVectorEuler",
    ("VECTOR", "XYZ"): "NodeSocketVectorXYZ",
    ("RGBA", "NONE"): "NodeSocketColor",
    ("STRING", "NONE"): "NodeSocketString",
    ("SHADER", "NONE"): "NodeSocketShader",
    ("OBJECT", "NONE"): "NodeSocketObject",
    ("IMAGE", "NONE"): "NodeSocketImage",
    ("GEOMETRY", "NONE"): "NodeSocketGeometry",
    ("COLLECTION", "NONE"): "NodeSocketCollection",
    ("TEXTURE", "NONE"): "NodeSocketTexture",
    ("MATERIAL", "NONE"): "NodeSocketMaterial",
}


@dataclass
class SocketSpec():
    name: str
    identifier: str
    type: str
    subtype: str = 'NONE'
    hide_value: bool = False
    hide_in_modifier: bool = False
    default_value: object = None
    min_value: object = None
    max_value: object = None
    links: int = 1

    @property
    def idname(self):
        global type_info
        return type_info[(self.type, self.subtype)]


class AbstractNodeGroupInterfaceTest(unittest.TestCase):
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

    def subtype_compare(self, value, expected, subtype):
        if subtype in {'ANGLE', 'EULER'}:
            # Angle values are shown in degrees in the UI, but stored as radians.
            # Conversion is not exactly the same, so use a fuzzy comparison.
            self.assertAlmostEqual(value, math.radians(expected))
        else:
            self.assertEqual(value, expected)

    def compare_spec(self, item, socket, spec: SocketSpec):
        # Examine the interface item
        self.assertEqual(item.name, spec.name)
        self.assertEqual(item.bl_socket_idname, spec.idname)
        self.assertEqual(item.identifier, spec.identifier)

        # Types that have subtypes
        if spec.type in {'VALUE', 'INT', 'VECTOR'}:
            self.assertEqual(item.subtype, spec.subtype)

        self.assertEqual(item.hide_value, spec.hide_value)
        self.assertEqual(item.hide_in_modifier, spec.hide_in_modifier)

        if spec.type in {'VALUE', 'INT'}:
            self.subtype_compare(item.default_value, spec.default_value, spec.subtype)
            self.assertEqual(item.min_value, spec.min_value)
            self.assertEqual(item.max_value, spec.max_value)
        elif spec.type == 'VECTOR':
            self.subtype_compare(item.default_value[0], spec.default_value[0], spec.subtype)
            self.subtype_compare(item.default_value[1], spec.default_value[1], spec.subtype)
            self.subtype_compare(item.default_value[2], spec.default_value[2], spec.subtype)
            self.assertEqual(item.min_value, spec.min_value)
            self.assertEqual(item.max_value, spec.max_value)
        elif spec.type == 'RGBA':
            # Colors stored as int8 internally, enough rounding error to require fuzzy test.
            self.assertAlmostEqual(item.default_value[0], spec.default_value[0])
            self.assertAlmostEqual(item.default_value[1], spec.default_value[1])
            self.assertAlmostEqual(item.default_value[2], spec.default_value[2])
            self.assertAlmostEqual(item.default_value[3], spec.default_value[3])
        elif spec.type in {'STRING', 'BOOLEAN', 'MATERIAL', 'TEXTURE', 'OBJECT', 'COLLECTION', 'IMAGE'}:
            self.assertEqual(item.default_value, spec.default_value)
        elif spec.type in {'SHADER', 'GEOMETRY'}:
            pass
        else:
            # Add socket type testing above if this happens.
            self.fail("Socket type not supported by test")

        # Examine the node socket
        self.assertEqual(socket.name, spec.name)
        self.assertEqual(socket.bl_idname, spec.idname)
        self.assertEqual(socket.type, spec.type)
        self.assertEqual(socket.identifier, spec.identifier)
        self.assertEqual(socket.hide_value, spec.hide_value)
        # There should be exactly one link connecting to the socket
        self.assertEqual(len(socket.links), spec.links, f"Socket should have exactly {spec.links} connections")


class NodeGroupVersioningTest(AbstractNodeGroupInterfaceTest):
    def open_file(self):
        bpy.ops.wm.open_mainfile(filepath=str(self.testdir/"nodegroup36.blend"))

    def test_load_compositor_nodes_36(self):
        self.open_file()

        tree = bpy.data.scenes['Scene'].node_tree
        group = bpy.data.node_groups.get('NodeGroup')
        self.assertIsNotNone(group, "Compositor node group not found")
        node = tree.nodes['Group']
        self.assertEqual(node.node_tree, group, "Node group must use compositor node tree")

        self.compare_spec(
            group.interface.ui_items[0],
            node.outputs[0],
            SocketSpec("Output Float", "Output_9", "VALUE", hide_value=True, default_value=3.0, min_value=1.0, max_value=1.0))
        self.compare_spec(
            group.interface.ui_items[1],
            node.outputs[1],
            SocketSpec("Output Vector", "Output_10", "VECTOR", subtype="EULER", default_value=(10, 20, 30), min_value=-10.0, max_value=10.0))
        self.compare_spec(
            group.interface.ui_items[2],
            node.outputs[2],
            SocketSpec("Output Color", "Output_11", "RGBA", default_value=(0, 1, 1, 1)))

        self.compare_spec(
            group.interface.ui_items[3],
            node.inputs[0],
            SocketSpec("Input Float", "Input_6", "VALUE", subtype="ANGLE", default_value=-20.0, min_value=5.0, max_value=6.0))
        self.compare_spec(
            group.interface.ui_items[4],
            node.inputs[1],
            SocketSpec("Input Vector", "Input_7", "VECTOR", hide_value=True, default_value=(2, 4, 6), min_value=-4.0, max_value=100.0))
        self.compare_spec(
            group.interface.ui_items[5],
            node.inputs[2],
            SocketSpec("Input Color", "Input_8", "RGBA", default_value=(0.5, 0.4, 0.3, 0.2)))

    def test_load_shader_nodes_36(self):
        self.open_file()

        tree = bpy.data.materials['Material'].node_tree
        group = bpy.data.node_groups.get('NodeGroup.003')
        self.assertIsNotNone(group, "Shader node group not found")
        node = tree.nodes['Group']
        self.assertEqual(node.node_tree, group, "Node group must use shader node tree")

        self.compare_spec(
            group.interface.ui_items[0],
            node.outputs[0],
            SocketSpec("Output Float", "Output_30", "VALUE", hide_value=True, default_value=3.0, min_value=1.0, max_value=1.0))
        self.compare_spec(
            group.interface.ui_items[1],
            node.outputs[1],
            SocketSpec("Output Vector", "Output_31", "VECTOR", subtype="EULER", default_value=(10, 20, 30), min_value=-10.0, max_value=10.0))
        self.compare_spec(
            group.interface.ui_items[2],
            node.outputs[2],
            SocketSpec("Output Color", "Output_32", "RGBA", default_value=(0, 1, 1, 1)))
        self.compare_spec(
            group.interface.ui_items[3],
            node.outputs[3],
            SocketSpec("Output Shader", "Output_33", "SHADER"))

        self.compare_spec(
            group.interface.ui_items[4],
            node.inputs[0],
            SocketSpec("Input Float", "Input_26", "VALUE", subtype="ANGLE", default_value=-20.0, min_value=5.0, max_value=6.0))
        self.compare_spec(
            group.interface.ui_items[5],
            node.inputs[1],
            SocketSpec("Input Vector", "Input_27", "VECTOR", hide_value=True, default_value=(2, 4, 6), min_value=-4.0, max_value=100.0))
        self.compare_spec(
            group.interface.ui_items[6],
            node.inputs[2],
            SocketSpec("Input Color", "Input_28", "RGBA", default_value=(0.5, 0.4, 0.3, 0.2)))
        self.compare_spec(
            group.interface.ui_items[7],
            node.inputs[3],
            SocketSpec("Input Shader", "Input_29", "SHADER"))

    def test_load_geometry_nodes_36(self):
        self.open_file()

        tree = bpy.data.node_groups['Geometry Nodes']
        group = bpy.data.node_groups.get('NodeGroup.002')
        self.assertIsNotNone(group, "Geometry node group not found")
        node = tree.nodes['Group']
        self.assertEqual(node.node_tree, group, "Node group must use geometry node tree")

        self.compare_spec(
            group.interface.ui_items[0],
            node.outputs[0],
            SocketSpec("Output Float", "Output_7", "VALUE", hide_value=True, default_value=3.0, min_value=1.0, max_value=1.0))
        self.compare_spec(
            group.interface.ui_items[1],
            node.outputs[1],
            SocketSpec("Output Vector", "Output_8", "VECTOR", subtype="EULER", default_value=(10, 20, 30), min_value=-10.0, max_value=10.0))
        self.compare_spec(
            group.interface.ui_items[2],
            node.outputs[2],
            SocketSpec("Output Color", "Output_9", "RGBA", default_value=(0, 1, 1, 1)))
        self.compare_spec(
            group.interface.ui_items[3],
            node.outputs[3],
            SocketSpec("Output String", "Output_19", "STRING", default_value=""))
        self.compare_spec(
            group.interface.ui_items[4],
            node.outputs[4],
            SocketSpec("Output Bool", "Output_20", "BOOLEAN", default_value=False))
        self.compare_spec(
            group.interface.ui_items[5],
            node.outputs[5],
            SocketSpec("Output Material", "Output_21", "MATERIAL", default_value=bpy.data.materials['TestMaterial']))
        self.compare_spec(
            group.interface.ui_items[6],
            node.outputs[6],
            SocketSpec("Output Int", "Output_22", "INT", default_value=0, min_value=-2147483648, max_value=2147483647))
        self.compare_spec(
            group.interface.ui_items[7],
            node.outputs[7],
            SocketSpec("Output Geometry", "Output_23", "GEOMETRY"))
        self.compare_spec(
            group.interface.ui_items[8],
            node.outputs[8],
            SocketSpec("Output Collection", "Output_24", "COLLECTION", default_value=bpy.data.collections['TestCollection']))
        self.compare_spec(
            group.interface.ui_items[9],
            node.outputs[9],
            SocketSpec("Output Texture", "Output_25", "TEXTURE", default_value=bpy.data.textures['TestTexture']))
        self.compare_spec(
            group.interface.ui_items[10],
            node.outputs[10],
            SocketSpec("Output Object", "Output_26", "OBJECT", default_value=bpy.data.objects['TestObject']))
        self.compare_spec(
            group.interface.ui_items[11],
            node.outputs[11],
            SocketSpec("Output Image", "Output_27", "IMAGE", default_value=bpy.data.images['TestImage']))

        self.compare_spec(
            group.interface.ui_items[12],
            node.inputs[0],
            SocketSpec("Input Float", "Input_4", "VALUE", subtype="ANGLE", default_value=-20.0, min_value=5.0, max_value=6.0))
        self.compare_spec(
            group.interface.ui_items[13],
            node.inputs[1],
            SocketSpec("Input Vector", "Input_5", "VECTOR", hide_value=True, default_value=(2, 4, 6), min_value=-4.0, max_value=100.0))
        self.compare_spec(
            group.interface.ui_items[14],
            node.inputs[2],
            SocketSpec("Input Color", "Input_6", "RGBA", default_value=(0.5, 0.4, 0.3, 0.2)))
        self.compare_spec(
            group.interface.ui_items[15],
            node.inputs[3],
            SocketSpec("Input String", "Input_10", "STRING", default_value="hello world!"))
        self.compare_spec(
            group.interface.ui_items[16],
            node.inputs[4],
            SocketSpec("Input Bool", "Input_11", "BOOLEAN", default_value=True, hide_in_modifier=True))
        self.compare_spec(
            group.interface.ui_items[17],
            node.inputs[5],
            SocketSpec("Input Material", "Input_12", "MATERIAL", default_value=bpy.data.materials['TestMaterial']))
        self.compare_spec(
            group.interface.ui_items[18],
            node.inputs[6],
            SocketSpec("Input Int", "Input_13", "INT", default_value=500, min_value=200, max_value=1000))
        self.compare_spec(
            group.interface.ui_items[19],
            node.inputs[7],
            SocketSpec("Input Geometry", "Input_14", "GEOMETRY"))
        self.compare_spec(
            group.interface.ui_items[20],
            node.inputs[8],
            SocketSpec("Input Collection", "Input_15", "COLLECTION", default_value=bpy.data.collections['TestCollection']))
        self.compare_spec(
            group.interface.ui_items[21],
            node.inputs[9],
            SocketSpec("Input Texture", "Input_16", "TEXTURE", default_value=bpy.data.textures['TestTexture']))
        self.compare_spec(
            group.interface.ui_items[22],
            node.inputs[10],
            SocketSpec("Input Object", "Input_17", "OBJECT", default_value=bpy.data.objects['TestObject']))
        self.compare_spec(
            group.interface.ui_items[23],
            node.inputs[11],
            SocketSpec("Input Image", "Input_18", "IMAGE", default_value=bpy.data.images['TestImage']))


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
