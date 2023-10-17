#!/usr/bin/env python3
# 
# Mechanical Blender

"""
"make mblender" for applying mblender patches on current source.
"""

import argparse
import sys
import os

import make_utils
from make_utils import call


MB_0001 = "https://projects.blender.org/JaumeBellet/mblender/raw/branch/mb-0001-operator-repeat/diff/mb-0001-operator-repeat.diff"

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--git-command", default="git")
    parser.add_argument("--wget-command", default="wget")
    return parser.parse_args()


args = parse_arguments()
git_command = args.git_command
wget_command = args.wget_command

tmp_file = "/tmp/mblender.patch"

if make_utils.command_missing(git_command):
        sys.stderr.write("git not found, can't checkout test files\n")
        sys.exit(1)

if make_utils.command_missing(wget_command):
        sys.stderr.write("wget not found, used for downloading patches\n")
        sys.exit(1)


if __name__ == "__main__":
	call([wget_command, MB_0001, "-O", tmp_file])
	call([git_command, "apply", tmp_file])
