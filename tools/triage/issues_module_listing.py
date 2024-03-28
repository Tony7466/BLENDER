#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""
# This script prints the numbers of open issues per module.

Example usage:

    python ./issues_module_listing.py --severity High
"""

import argparse
import dataclasses
from datetime import date
from gitea_utils import gitea_json_issues_search

@dataclasses.dataclass
class ModuleInfo:
    name: str
    labelid: str
    buglist: list[str] = dataclasses.field(default_factory=list)

modules = {
    "Module/Animation & Rigging" : ModuleInfo(name="Animation & Rigging", labelid="268"),
    "Module/Core" : ModuleInfo(name="Core", labelid="269"),
    "Module/EEVEE & Viewport" : ModuleInfo(name="EEVEE & Viewport", labelid="272"),
    "Module/Grease Pencil" : ModuleInfo(name="Grease Pencil", labelid="273"),
    "Module/Modeling" : ModuleInfo(name="Modeling", labelid="274"),
    "Module/Nodes & Physics" : ModuleInfo(name="Nodes & Physics", labelid="275"),
    "Module/Pipeline, Assets & IO" : ModuleInfo(name="Pipeline, Assets & I/O", labelid="276"),
    "Module/Platforms, Builds, Test & Devices" : ModuleInfo(name="Platforms, Builds, Test & Devices", labelid="278"),
    "Module/Python API" : ModuleInfo(name="Python API", labelid="279"),
    "Module/Render & Cycles" : ModuleInfo(name="Render & Cycles", labelid="280"),
    "Module/Sculpt, Paint & Texture" : ModuleInfo(name="Sculpt, Paint & Texture", labelid="281"),
    "Module/User Interface" : ModuleInfo(name="User Interface", labelid="283"),
    "Module/VFX & Video" : ModuleInfo(name="VFX & Video", labelid="284"),
}

base_url = "https://projects.blender.org/blender/blender/issues?q=&type=all&sort=&state=open&labels="

severity_labelid = {
    "Low": "286",
    "Normal": "287",
    "High": "285",
    "Unbreak Now!": "288"
}

uncategorized_reports = list()

def compile_list(severity: str) -> None:

    label = f"Priority/{severity}"
    issues_json = gitea_json_issues_search(
        type="issues",
        state="open",
        labels=label,
        verbose=True,
    )

    for issue in issues_json:
        html_url = issue["html_url"]
        number = issue["number"]

        # Check reports module assignement and fill in data
        for label_iter in issue["labels"]:
            label = label_iter["name"]
            if label not in modules:
                continue

            modules[label].buglist.append(f"[#{number}]({html_url})")
            break
    else:
        uncategorized_reports.append(f"[#{number}]({html_url})")

    # Calculate total
    total = 0
    for module in modules:
        total += len(modules[module].buglist)

    # Print statistics
    print(f"Open {severity} Priority bugs as of {date.today()}:\n")

    uncategorized_list = (', '.join(uncategorized_reports))
    for module in modules.values():
        str_list = (', '.join(module.buglist))
        full_url = base_url + severity_labelid[severity] + "%2c" + module.labelid
        if not module.buglist or severity != 'High':
            print(f"- [{module.name}]({full_url}): *{str(len(module.buglist))}*")
        else:
            print(f"- [{module.name}]({full_url}): *{str(len(module.buglist))}* _{str_list}_")

    print()
    print("Total: " + str(total))

    print()
    print(f"Uncategorized: {uncategorized_list}")


def main() -> None:

    parser = argparse.ArgumentParser(
        description="Print statistics on open bug reports per module",
        epilog="This script is used to help module teams")

    parser.add_argument(
        "--severity",
        dest="severity",
        default="High",
        type=str,
        required=False,
        help="Severity of reports (Low, Normal, High, Unbreak Now!")

    args = parser.parse_args()
        
    compile_list(args.severity)


if __name__ == "__main__":
    main()
