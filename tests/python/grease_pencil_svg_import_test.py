# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import pathlib
import sys
import unittest
import tempfile

import bpy

args = None


class AbstractSVGTest(unittest.TestCase):
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

    def tearDown(self):
        self._tempdir.cleanup()


class SVGImportTest(AbstractSVGTest):
    def test_import(self):
        infile = str(self.testdir / "rocket.svg")
        res = bpy.ops.wm.grease_pencil_import_svg(filepath=infile)
        self.assertEqual({'FINISHED'}, res)

        objects = bpy.context.scene.collection.objects
        self.assertEqual(1, len(objects))
        ob = objects[0]
        self.assertEqual("rocket.svg", ob.name)

        # Materials
        self.assertEqual(1, len(ob.material_slots))
        # Note: Color import from SVG is currently limited.
        # Per-element colors get converted to vertex colors, but most newer software exports style classes, which are not supported by NanoSVG.
        # https://github.com/memononen/nanosvg/issues/55
        # For now test for the single placeholder material and update this test when style classes are supported.
        material = ob.material_slots[0].material
        self.assertEqual("Fill", material.name)
        self.assertEqual(False, material.grease_pencil.show_stroke)
        self.assertEqual(True, material.grease_pencil.show_fill)
        self.assertEqual((0, 0, 0, 1), material.grease_pencil.color[:])
        self.assertEqual((0.5, 0.5, 0.5, 1), material.grease_pencil.fill_color[:])

        # Layers and frames
        self.assertEqual('GREASEPENCIL', ob.type)
        gp = ob.data
        self.assertEqual(2, len(gp.layers))
        self.assertEqual(1, len(gp.layers[0].frames))
        self.assertEqual(1, len(gp.layers[1].frames))

        # Paths
        drawing = gp.layers[0].frames[0].drawing
        curve_type = frame.attributes['position']
        cyclic = frame.attributes['position']
        positions = frame.attributes['position']
        self.assertEqual(1, len(frame.frame_number))



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
