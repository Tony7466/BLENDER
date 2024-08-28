#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Keep the information collected in this script up to date with
# /scripts/modules/bl_ui_utils/bug_report_url.py

def prefill_bug_report_info():
    import re
    import struct
    import platform
    import subprocess
    import webbrowser
    import urllib.parse
    from pathlib import Path

    print("Collecting system information...")

    query_params = {"type": "bug_report", "project": "blender"}

    query_params["os"] = "{:s} {:d} Bits".format(
        platform.platform(),
        struct.calcsize("P") * 8,
    )

    # There doesn't appear to be a easy way to collect GPU information in Python
    # if Blender isn't opening and we can't import the GPU module.
    # So just tell users to follow a written guide.
    query_params["gpu"] = """Follow our guide to collect this information:
https://developer.blender.org/docs/handbook/bug_reports/making_good_bug_reports/collect_system_information/"""

    os_type = platform.system()
    script_directory = Path(__file__).parent.resolve()
    if os_type == "Darwin":  # macOS
        blender_dir = script_directory.joinpath("../../../../MacOS/Blender")
    elif os_type == "Windows":
        blender_dir = script_directory.joinpath("../../../Blender.exe")
    else:  # Linux & other Unix systems
        blender_dir = script_directory.joinpath("../../../blender")

    try:
        blender_output = subprocess.run([blender_dir, "--version"], stdout=subprocess.PIPE, errors="surrogateescape")
    except Exception as ex:
        sys.stderr.write("{:s}\n".format(str(ex)))
        return 1

    text = blender_output.stdout

    # Gather version number and type (Alpha, Beta, etc)
    version_match = re.search(r"Blender (.*)", text)
    branch_match = re.search(r"build branch: (.*)", text)
    commit_date_match = re.search(r"build commit date: (.*)", text)
    commit_time_match = re.search(r"build commit time: (.*)", text)
    build_hash_match = re.search(r"build hash: (.*)", text)

    if not (version_match and branch_match and commit_date_match and commit_time_match and build_hash_match):
        # No valid Blender info could be found.
        print("Blender did not provide any build information. Blender may be corrupt or blocked from running.")
        print("Please try reinstalling Blender and double check your anti-virus isn't blocking it from running.")
        return 1

    missing_string = "<unknown>"

    query_params["broken_version"] = "{:s}, branch: {:s}, commit date: {:s} {:s}, hash `{:s}`".format(
        version_match.group(1).strip() if version_match else missing_string,
        branch_match.group(1).strip() if branch_match else missing_string,
        commit_date_match.group(1).strip() if commit_date_match else missing_string,
        commit_time_match.group(1).strip() if commit_time_match else missing_string,
        build_hash_match.group(1).strip() if build_hash_match else missing_string,
    )

    query_str = urllib.parse.urlencode(query_params)
    webbrowser.open(f"https://redirect.blender.org/?{query_str}")

    return 0

if __name__ == "__main__":
    import sys
    sys.exit(prefill_bug_report_info())
