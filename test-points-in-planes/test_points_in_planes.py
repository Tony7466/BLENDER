#!/usr/bin/env -S blender --background --python
import sys
import unittest

import bpy
import mathutils
import mathutils.geometry

fixture_cube_aa2m = [
        mathutils.Vector((-1, 0, 0, -1)),
        mathutils.Vector((1, 0, 0, -1)),
        mathutils.Vector((0, -1, 0, -1)),
        mathutils.Vector((0, 1, 0, -1)),
        mathutils.Vector((0, 0, -1, -1)),
        mathutils.Vector((0, 0, 1, -1)),
]

fixture_cube_aa2m_expected_points_planes = (
    [
         mathutils.Vector((-1.0, -1.0, -1.0)),
         mathutils.Vector((-1.0, -1.0, 1.0)),
         mathutils.Vector((-1.0, 1.0, -1.0)),
         mathutils.Vector((-1.0, 1.0, 1.0)),
         mathutils.Vector((1.0, -1.0, -1.0)),
         mathutils.Vector((1.0, -1.0, 1.0)),
         mathutils.Vector((1.0, 1.0, -1.0)),
         mathutils.Vector((1.0, 1.0, 1.0))
    ], [0, 1, 2, 3, 4, 5]
)

fixture_cube_aa2m_expected_points_of_planes = [
    (mathutils.Vector((-1.0, -1.0, -1.0)), 0, 2, 4),
    (mathutils.Vector((-1.0, -1.0, 1.0)), 0, 2, 5),
    (mathutils.Vector((-1.0, 1.0, -1.0)), 0, 3, 4),
    (mathutils.Vector((-1.0, 1.0, 1.0)), 0, 3, 5),
    (mathutils.Vector((1.0, -1.0, -1.0)), 1, 2, 4),
    (mathutils.Vector((1.0, -1.0, 1.0)), 1, 2, 5),
    (mathutils.Vector((1.0, 1.0, -1.0)), 1, 3, 4),
    (mathutils.Vector((1.0, 1.0, 1.0)), 1, 3, 5)
]

class TestCase(unittest.TestCase):
    def test_before(self):
        self.assertEqual(
            mathutils.geometry.points_in_planes(fixture_cube_aa2m),
            fixture_cube_aa2m_expected_points_planes,
        )

    def test_after(self):
        accum = []
        def accum_append(*args):
            accum.append(args)

        self.assertEqual(
            mathutils.geometry.points_in_planes(fixture_cube_aa2m, 1e-5, 1e-6, accum_append),
            fixture_cube_aa2m_expected_points_planes,
        )

        self.assertEqual(accum, fixture_cube_aa2m_expected_points_of_planes)


if __name__ == '__main__':
    if '--' not in sys.argv: sys.argv.append('--')
    sys.argv = [sys.argv[0]] + sys.argv[sys.argv.index('--')+1:]
    unittest.util._MAX_LENGTH=2000
    unittest.main()
