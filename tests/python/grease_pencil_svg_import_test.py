# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import math
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
        object = objects[0]

        self.assertEqual(object.name, "rocket.svg")

        # Materials
        self.assertEqual(len(object.material_slots), 1)
        material = object.material_slots[0].material
        

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
