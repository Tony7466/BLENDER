#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Keep this script in line with /scripts/modules/bl_ui_utils/bug_report_url.py

import os
import re
import struct
import platform
import urllib.parse
import subprocess

query_params = {"type": "bug_report"}

query_params["project"] = "blender"

query_params["os"] = "{:s} {:d} Bits".format(
    platform.platform(),
    struct.calcsize("P") * 8,
)

# There is no easy way to collect GPU information in Python
# if Blender isn't opening and we can't import the GPU module.
# TODO: Investigate how to work around this
query_params["gpu"] = "Unsure"

# TODO: Adjust this?
os_type = platform.system()
script_directory = os.path.dirname(__file__)
if os_type == "Darwin": #macOS
    blender_dir = os.path.join(script_directory, "../../MacOS/Blender")
elif os_type == "Windows":
    blender_dir = os.path.join(script_directory, "../Blender.exe")
else: # Linux
    blender_dir = os.path.join(script_directory, "../blender")

command = [blender_dir, "--version"]

output = subprocess.run(command, stdout = subprocess.PIPE)
text = output.stdout.decode("utf-8")
# Gather version number and type (Alpha, Beta, etc)
version_match = re.search(r"Blender (\d+\.\d+\.\d+\s[A-Za-z]+)", text)
if not version_match:
    # Gather just version number (Previous version_match doesn't work on final release builds)
    version_match = re.search(r"Blender (\d+\.\d+\.\d+)", text)
branch_match = re.search(r"build branch: (.*)", text)
commit_date_match = re.search(r"build commit date: (\d+-\d+-\d+)", text)
commit_time_match = re.search(r"build commit time: (\d+:\d+)", text)
build_hash_match = re.search(r"build hash: (\w+)", text)

# TODO: Replace with something else?
failed = "Script failed"

query_params["broken_version"] = "{:s}, branch: {:s}, commit date: {:s} {:s}, hash `{:s}`".format(
    version_match.group(1) if version_match else failed,
    branch_match.group(1).strip() if branch_match else failed,
    commit_date_match.group(1) if commit_date_match else failed,
    commit_time_match.group(1) if commit_time_match else failed,
    build_hash_match.group(1) if build_hash_match else failed,
)

# TODO: Since we can't collect GPU info, do we open the URL? Or just print system information
# and a guide on how to manually collect GPU info?
query_str = urllib.parse.urlencode(query_params)
print(f"https://redirect.blender.org/?{query_str}")
