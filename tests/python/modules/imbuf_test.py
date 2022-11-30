# SPDX-License-Identifier: GPL-2.0-or-later

import os
import pathlib
import shutil
import subprocess
import sys
import unittest

args = None

class COLORS_ANSI:
    RED = '\033[00;31m'
    GREEN = '\033[00;32m'
    ENDC = '\033[0m'


class COLORS_DUMMY:
    RED = ''
    GREEN = ''
    ENDC = ''


COLORS = COLORS_DUMMY


def print_message(message, type=None, status=''):
    if type == 'SUCCESS':
        print(COLORS.GREEN, end="")
    elif type == 'FAILURE':
        print(COLORS.RED, end="")
    status_text = ...
    if status == 'RUN':
        status_text = " RUN      "
    elif status == 'OK':
        status_text = "       OK "
    elif status == 'PASSED':
        status_text = "  PASSED  "
    elif status == 'FAILED':
        status_text = "  FAILED  "
    else:
        status_text = status
    if status_text:
        print("[{}]" . format(status_text), end="")
    print(COLORS.ENDC, end="")
    print(" {}" . format(message))
    sys.stdout.flush()

class AbstractImBufTest(unittest.TestCase):
    @classmethod
    def init(cls, args):
        cls.test_dir = pathlib.Path(args.test_dir)
        cls.reference_dir = pathlib.Path(args.test_dir).joinpath("reference")
        cls.reference_load_dir = pathlib.Path(args.test_dir).joinpath("reference_load")
        cls.output_dir = pathlib.Path(args.output_dir)
        cls.diff_dir = pathlib.Path(args.output_dir).joinpath("diff")
        cls.idiff = pathlib.Path(args.idiff)
        cls.optional_formats = args.optional_formats

        os.makedirs(cls.diff_dir, exist_ok=True)

        cls.errors = 0
        cls.fail_threshold = 0.001
        cls.fail_percent = 1
        cls.verbose = os.environ.get("BLENDER_VERBOSE") is not None
        cls.update = os.getenv('BLENDER_TEST_UPDATE') is not None
        if os.environ.get("BLENDER_TEST_COLOR") is not None:
            global COLORS, COLORS_ANSI
            COLORS = COLORS_ANSI

    def setUp(self):
        self.errors = 0
        print_message("")

    def tearDown(self):
        if self.errors > 0:
            self.fail("{} errors encountered" . format(self.errors))

    def skip_if_format_missing(self, format):
        if self.optional_formats.find(format) < 0:
            self.skipTest("format not available")

    def call_idiff(self, ref_path, out_path):
        ref_filepath = str(ref_path)
        out_filepath = str(out_path)
        out_name = out_path.name
        if os.path.exists(ref_filepath):
            # Diff images test with threshold.
            command = (
                str(self.idiff),
                "-fail", str(self.fail_threshold),
                "-failpercent", str(self.fail_percent),
                ref_filepath,
                out_filepath,
            )
            try:
                subprocess.check_output(command)
                failed = False
            except subprocess.CalledProcessError as e:
                if self.verbose:
                    print_message(e.output.decode("utf-8", 'ignore'))
                failed = e.returncode != 1
        else:
            if not self.update:
                return False

            failed = True

        if failed and self.update:
            # Update reference image if requested.
            shutil.copy(out_filepath, ref_filepath)
            failed = False

        # Generate diff image.
        diff_img = str(self.diff_dir.joinpath(out_name + ".diff.png"))
        command = (
            str(self.idiff),
            "-o", diff_img,
            "-abs", "-scale", "16",
            ref_filepath,
            out_filepath
        )

        try:
            subprocess.check_output(command)
        except subprocess.CalledProcessError as e:
            if self.verbose:
                print_message(e.output.decode("utf-8", 'ignore'))

        return not failed
