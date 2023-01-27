# SPDX-License-Identifier: GPL-2.0-or-later
import enum
import pathlib
import pprint
import sys
import tempfile
import unittest
from pxr import UsdUtils

import bpy

args = None


class Result(str, enum.Enum):
    finished = "FINISHED"
    cancelled = "CANCELLED"


class AbstractUSDTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tempdir = tempfile.TemporaryDirectory()
        cls.testdir = args.testdir
        cls.tempdir = pathlib.Path(cls._tempdir.name)

        return cls

    def setUp(self):
        self.assertTrue(
            self.testdir.exists(), "Test dir {0} should exist".format(self.testdir)
        )

    def tearDown(self):
        self._tempdir.cleanup()


class USDExportTest(AbstractUSDTest):
    def test_export_usdchecker(self):
        """Test exporting a scene and verifying it passes the usdchecker test suite"""
        bpy.ops.wm.open_mainfile(
            filepath=str(self.testdir / "usd_materials_export.blend")
        )
        export_path = self.tempdir / "usdchecker.usda"
        res = bpy.ops.wm.usd_export(
            filepath=str(export_path),
            export_materials=True,
            evaluation_mode="RENDER",
        )
        self.assertEqual({Result.finished}, res, f"Unable to export to {export_path}")

        checker = UsdUtils.ComplianceChecker(
            arkit=False,
            skipARKitRootLayerCheck=False,
            rootPackageOnly=False,
            skipVariants=False,
            verbose=False,
        )
        checker.CheckCompliance(str(export_path))

        collection = {}

        to_skip = ("MissingReferenceChecker",)
        for rule in checker._rules:
            name = rule.__class__.__name__
            if name in to_skip:
                continue

            issues = rule.GetFailedChecks() + rule.GetWarnings() + rule.GetErrors()
            if not issues:
                continue

            collection[name] = issues

        self.assertFalse(collection, pprint.pformat(collection))


def main():
    global args
    import argparse

    if "--" in sys.argv:
        argv = [sys.argv[0]] + sys.argv[sys.argv.index("--") + 1 :]
    else:
        argv = sys.argv

    parser = argparse.ArgumentParser()
    parser.add_argument("--testdir", required=True, type=pathlib.Path)
    args, remaining = parser.parse_known_args(argv)

    unittest.main(argv=remaining)


if __name__ == "__main__":
    main()
