#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import os
from os.path import join

PATHS = (
    "source",

    # files
    "intern/opencolorio/gpu_shader_display_transform_frag.glsl",
    "intern/opencolorio/gpu_shader_display_transform_vert.glsl",
)


SOURCE_DIR = os.path.normpath(os.path.abspath(os.path.normpath(os.path.join(os.path.dirname(__file__)))))

PATHS = tuple(
    os.path.join(SOURCE_DIR, p.replace("/", os.sep))
    for p in PATHS
)


def files(path, test_fn):
    for dirpath, dirnames, filenames in os.walk(path):
        # skip '.git'
        dirnames[:] = [d for d in dirnames if not d.startswith(".")]
        for filename in filenames:
            if test_fn(filename):
                filepath = os.path.join(dirpath, filename)
                yield filepath


SOURCE_EXT = (
    # GLSL
    ".glsl",
)


def is_source(filename):
    return filename.endswith(SOURCE_EXT)


PATHS = PATHS + tuple(files(os.path.join(SOURCE_DIR), is_source))


def year_range_from_filepath(filepath):
    # Ref: https://stackoverflow.com/a/25633731/432509
    import subprocess
    cmd_beg = (
        "git", "log", "--diff-filter=A", "--follow", "--format=%aI", "--",
        filepath,
    )
    cmd_end = (
        "git", "log", "-n1", "--format=%aI", "--",
        filepath,
    )

    # Output will look something like this: `2022-04-27T12:34:57+02:00`
    output_beg = subprocess.check_output(cmd_beg)
    output_end = subprocess.check_output(cmd_end)

    result = (
        output_beg.split(b'-', 1)[0].decode('ascii'),
        output_end.split(b'-', 1)[0].decode('ascii'),
    )

    assert result[0].isdigit()
    assert result[1].isdigit()

    return result


def path_iter(path, filename_check=None):
    for dirpath, dirnames, filenames in os.walk(path):
        # skip ".git"
        dirnames[:] = [d for d in dirnames if not d.startswith(".")]

        for filename in filenames:
            if filename.startswith("."):
                continue
            filepath = join(dirpath, filename)
            if filename_check is None or filename_check(filepath):
                yield filepath


def path_expand(paths, filename_check=None):
    for f in paths:
        if not os.path.exists(f):
            print("Missing:", f)
        elif os.path.isdir(f):
            yield from path_iter(f, filename_check)
        else:
            yield f


def license_file(filepath):
    with open(filepath, "r", encoding="utf-8") as fh:
        data_src = fh.read()

    # Ignore files that have already been handled.
    if "SPDX-FileCopyrightText" in data_src:
        return

    # Strip leading newlines.
    data_dst = []
    for line in data_src.rstrip().splitlines(True):
        data_dst.append(line.rstrip() + "\n")

    while data_dst and data_dst[0] == "\n":
        del data_dst[0]

    data_dst = "".join(data_dst)

    year_beg, year_end = year_range_from_filepath(filepath)

    with open(filepath, "w", encoding="utf-8") as fh:
        header = '''\
/* SPDX-FileCopyrightText: {:s} Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */'''.format(
            year_beg if (year_beg == year_end) else
            "{:s}-{:s}".format(year_beg, year_end)
        )

        fh.write(header)
        fh.write("\n\n")
        fh.write(data_dst)


def main():
    for f in path_expand(PATHS, is_source):
        print(f)
        license_file(f)


if __name__ == "__main__":
    main()
