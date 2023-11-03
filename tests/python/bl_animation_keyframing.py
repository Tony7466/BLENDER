# SPDX-FileCopyrightText: 2020-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import unittest
import bpy
import pathlib
import sys

"""
blender -b -noaudio --factory-startup --python tests/python/bl_animation_keyframing.py -- --testdir /path/to/lib/tests/animation
"""


def _fcurve_paths_match(fcurves: list, expected_paths: list) -> bool:
    data_paths = list(set([fcurve.data_path for fcurve in fcurves]))
    # print(sorted(data_paths), sorted(expected_paths))
    return sorted(data_paths) == sorted(expected_paths)


def _get_view3d_context():
    ctx = bpy.context.copy()

    for area in bpy.context.window.screen.areas:
        if area.type != 'VIEW_3D':
            continue

        ctx['area'] = area
        ctx['space'] = area.spaces.active
        break

    return ctx


def _insert_by_name_test(insert_key: str, expected_paths: list):
    bpy.ops.mesh.primitive_monkey_add()
    keyed_object = bpy.context.active_object
    # Ensure that the rotation mode is correct so we can check against rotation_euler
    keyed_object.rotation_mode = "XYZ"
    with bpy.context.temp_override(**_get_view3d_context()):
        bpy.ops.anim.keyframe_insert_by_name(type=insert_key)
    match = _fcurve_paths_match(keyed_object.animation_data.action.fcurves, expected_paths)
    bpy.ops.object.delete(use_global=False)
    return match


def _get_keying_set(scene, name: str):
    return scene.keying_sets_all[scene.keying_sets_all.find(name)]


def _insert_with_keying_set_test(keying_set_name: str, expected_paths: list):
    scene = bpy.context.scene
    keying_set = _get_keying_set(scene, keying_set_name)
    scene.keying_sets.active = keying_set
    bpy.ops.mesh.primitive_monkey_add()
    keyed_object = bpy.context.active_object
    # Ensure that the rotation mode is correct so we can check against rotation_euler
    keyed_object.rotation_mode = "XYZ"
    with bpy.context.temp_override(**_get_view3d_context()):
        bpy.ops.anim.keyframe_insert()
    match = _fcurve_paths_match(keyed_object.animation_data.action.fcurves, expected_paths)
    bpy.ops.object.delete(use_global=False)
    return match


class AbstractKeyframingTest:
    def setUp(self):
        super().setUp()
        bpy.ops.wm.read_homefile(use_factory_startup=True)


class InsertKeyTest(AbstractKeyframingTest, unittest.TestCase):

    def test_insert_by_name(self):
        self.assertTrue(_insert_by_name_test("Location", ["location"]))
        self.assertTrue(_insert_by_name_test("Rotation", ["rotation_euler"]))
        self.assertTrue(_insert_by_name_test("Scaling", ["scale"]))
        self.assertTrue(_insert_by_name_test("LocRotScale", ["location", "rotation_euler", "scale"]))

    def test_insert_with_keying_set(self):
        self.assertTrue(_insert_with_keying_set_test("Location", ["location"]))
        self.assertTrue(_insert_with_keying_set_test("Rotation", ["rotation_euler"]))
        self.assertTrue(_insert_with_keying_set_test("Scale", ["scale"]))
        self.assertTrue(_insert_with_keying_set_test("Location, Rotation & Scale", ["location", "rotation_euler", "scale"]))
        

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
