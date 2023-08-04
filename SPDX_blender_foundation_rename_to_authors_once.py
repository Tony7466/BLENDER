#!/usr/bin/env python3

# Run from Blender's root DIR
# HINT: to test the results run:
#
#    git ls-files -z | xargs -0 pcregrep -i "Blender.*Foundation"
#

import os
import re

SOURCE_DIRS = (
    "build_files",
    "doc",
    "intern",
    "locale",
    "tests",
    "release",
    "scripts",
    "source",
    "tools",
)

SOURCE_FILES_INDIVIDUAL = [
    "CMakeLists.txt",
    "GNUmakefile",
]

# A single string from a mising comma will search "/" ! - bad ju-ju.
assert (type(SOURCE_DIRS) is tuple)

# Naming: from -> to.
replace_all = (
    ("Blender Foundation", "Blender Authors"),
)


replace_tables = (
    replace_all,
)

replace_tables_re = [
    [(r"\b" + src + r"\b", dst) for src, dst in table]
    for table in replace_tables
]


def replace_all(fn, data_src):
    data_dst = data_src
    # Only manipulate files containing SPDX copyright.
    if "SPDX-FileCopyrightText" not in data_src:
        return None
    for table in replace_tables_re:
        for src_re, dst in table:
            data_dst = re.sub(src_re, dst, data_dst)

    if data_dst != data_src:
        print(fn)
        return data_dst
    return None


operation = replace_all


def source_files(path):
    for dirpath, dirnames, filenames in os.walk(path):
        dirnames[:] = [d for d in dirnames if not d.startswith(".")]
        for filename in filenames:
            if filename.startswith("."):
                continue
            ext = os.path.splitext(filename)[1]
            if ext.lower() in {
                    ".c", ".cc", ".cxx", ".cpp", ".m", ".mm",
                    ".h", ".hxx", ".hpp", ".hh", ".inl",
                    ".py",
                    ".glsl",
                    ".cl",
                    # CUDA.
                    ".cu",
                    # OSL.
                    ".osl",
                    # CMake.
                    ".txt",
                    ".cmake",
                    ".rst",
                    # CUDA.
                    ".cy",
                    # Metal.
                    ".metal",
                    # Python.
                    ".py",
                    # Shell.
                    ".sh",
                    # Cycles defines.
                    ".tables",
            }:
                print(os.path.join(dirpath, filename))
                yield os.path.join(dirpath, filename)


def operation_wrap(fn):
    with open(fn, "r", encoding="utf-8") as f:
        try:
            data_src = f.read()
        except Exception as ex:
            print("Failed to read", fn, "with", repr(ex))
            return
        data_dst = operation(fn, data_src)

    if data_dst is None or (data_src == data_dst):
        return

    with open(fn, "w", encoding="utf-8") as f:
        f.write(data_dst)


def main():
    for fn in ([fn for DIR in SOURCE_DIRS for fn in source_files(DIR)] + SOURCE_FILES_INDIVIDUAL):
        operation_wrap(fn)


if __name__ == "__main__":
    main()
