#!/usr/bin/env python3

import argparse
import textwrap
import sys
import os
from coverage_report import parse, report_as_html
from pathlib import Path


def main():
    usage = textwrap.dedent("""\
        coverage --build-directory <build-dir>
        """)

    parser = argparse.ArgumentParser(
        description="Blender test coverage",
        usage=usage)

    parser.add_argument("--build-directory", type=str, default=os.getcwd())
    args = parser.parse_args(sys.argv[1:])

    build_dir = Path(args.build_directory).absolute()
    if not is_blender_build_directory(build_dir):
        print("Directory does not seem to be a Blender build directory.")
        sys.exit(1)

    coverage_dir = build_dir / "coverage"
    analysis_dir = coverage_dir / "analysis"
    report_dir = coverage_dir / "report"

    parse(build_dir, analysis_dir)
    report_as_html(analysis_dir, report_dir)

def is_blender_build_directory(build_dir):
    return (Path(build_dir) / "bin" / "blender").exists() or (Path(build_dir) / "bin" / "blender.exe").exists()

main()
