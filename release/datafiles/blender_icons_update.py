#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2014-2022 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# This script updates icons from the SVG file
import os
import subprocess
import sys


def run(cmd, *, env=None):
    print("   ", " ".join(cmd))
    subprocess.check_call(cmd, env=env)


BASEDIR = os.path.abspath(os.path.dirname(__file__))

env = {}
# Developers may have ASAN enabled, avoid non-zero exit codes.
env["ASAN_OPTIONS"] = "exitcode=0:" + os.environ.get("ASAN_OPTIONS", "")

# These NEED to be set on windows for python to initialize properly.
if sys.platform[:3] == "win":
    env["PATHEXT"] = os.environ.get("PATHEXT", "")
    env["SystemDrive"] = os.environ.get("SystemDrive", "")
    env["SystemRoot"] = os.environ.get("SystemRoot", "")

inkscape_bin = "inkscape"
blender_bin = "blender"

if sys.platform == 'darwin':
    inkscape_app_path = '/Applications/Inkscape.app/Contents/MacOS/inkscape'
    if os.path.exists(inkscape_app_path):
        inkscape_bin = inkscape_app_path
    blender_app_path = '/Applications/Blender.app/Contents/MacOS/Blender'
    if os.path.exists(blender_app_path):
        blender_bin = blender_app_path
    else:
        blender_bin = "Blender"

inkscape_bin = os.environ.get("INKSCAPE_BIN", inkscape_bin)
blender_bin = os.environ.get("BLENDER_BIN", blender_bin)

cmd = (
    inkscape_bin,
    os.path.join(BASEDIR, "blender_icons.svg"),
    "--export-width=2408",
    "--export-height=2560",
    "--export-type=png",
    "--export-filename=" + os.path.join(BASEDIR, "blender_icons64.png"),
)
run(cmd, env=env)
