# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""
This test calls into the platforms class of ``blender_ext`` directly.
"""
import unittest
import os

from typing import (
    Any,
    Dict,
    List,
)


CURRENT_DIR = os.path.abspath(os.path.dirname(__file__))
BASE_DIR = os.path.normpath(os.path.join(CURRENT_DIR, ".."))


# Don't import as module, instead load the class.
def execfile(filepath: str, *, name: str = "__main__") -> Dict[str, Any]:
    global_namespace = {"__file__": filepath, "__name__": name}
    with open(filepath, encoding="utf-8") as file_handle:
        exec(compile(file_handle.read(), filepath, 'exec'), global_namespace)
    return global_namespace


Platforms = execfile(os.path.join(BASE_DIR, "cli", "blender_ext.py"), name="blender_ext")["Platforms"]
assert isinstance(Platforms, type)


class TestPlatform(unittest.TestCase):
    @staticmethod
    def _get_manifest(arguments: List) -> dict:
        return {
            "platforms": arguments,
        }

    def test_platforms_all(self) -> None:
        platforms_all = self._get_manifest(list(Platforms._all_platforms))
        self.assertEqual(Platforms({}), Platforms(platforms_all))

    def test_platforms_mac_vs_linux(self) -> None:
        platforms_linux = self._get_manifest(["linux-x86_64"])
        platforms_macos = self._get_manifest(["macos-arm64", "macos-x86_64"])
        self.assertNotEqual(Platforms(platforms_linux), Platforms(platforms_macos))

    def test_platforms_overlap(self) -> None:
        platforms_macos = self._get_manifest(["macos-arm64", "macos-x86_64"])
        platforms_popular = self._get_manifest(["macos-arm64", "win-amd64", "linux-x86_64"])
        self.assertEqual(Platforms(platforms_macos), Platforms(platforms_popular))


if __name__ == "__main__":
    unittest.main()
