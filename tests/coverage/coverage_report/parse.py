# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import concurrent.futures
import json
import multiprocessing
import os
import random
import shutil
import subprocess
import sys
import textwrap
import time
import zipfile

from collections import defaultdict
from pathlib import Path
from pprint import pprint

from .util import print_updateable_line


def parse(build_dir, analysis_dir, gcov_binary="gcov"):
    """
    Parses coverage data generated in the given directory, merges it, and stores
    result in the analysis directory.
    """

    build_dir = Path(build_dir).absolute()
    analysis_dir = Path(analysis_dir).absolute()

    if not Path(gcov_binary).is_file():
        gcov_binary = shutil.which(gcov_binary)
    gcov_binary = Path(gcov_binary).absolute()

    print("Gather .gcda files...")
    gcda_paths = []
    for gcda_path in build_dir.glob("**/*.gcda"):
        gcda_paths.append(gcda_path)
        print_updateable_line("[{}]: {}".format(len(gcda_paths), gcda_path))
    print()

    if len(gcda_paths) == 0:
        raise RuntimeError(
            textwrap.dedent(
                """\
            No .gcda files found. Make sure to run the tests in a debug build that has
            been compiled with GCC with --coverage.
            """
            )
        )

    # Shuffle to make chunks more similar in size.
    random.shuffle(gcda_paths)
    chunk_size = 10
    gcda_path_chunks = [gcda_paths[i: i + chunk_size] for i in range(0, len(gcda_paths), chunk_size)]

    def parse_with_gcov(file_paths):
        return subprocess.check_output([gcov_binary, "--stdout", "--json-format", *file_paths])

    print("Parse files...")
    print_updateable_line("[0/{}] parsed.".format(len(gcda_paths)))
    gathered_gcov_outputs = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() * 2) as executor:
        futures = {executor.submit(parse_with_gcov, file_paths): file_paths for file_paths in gcda_path_chunks}

        done_count = 0
        for future in concurrent.futures.as_completed(futures):
            file_paths = futures[future]
            done_count += len(file_paths)
            try:
                for line in future.result().splitlines():
                    gathered_gcov_outputs.append(json.loads(line))
            except Exception as e:
                print("Error:", e)
            print_updateable_line("[{}/{}] parsed.".format(done_count, len(gcda_paths)))
    print()

    print("Merge coverage data...")
    data_by_source_file = defaultdict(list)
    for data in gathered_gcov_outputs:
        for file_data in data["files"]:
            data_by_source_file[file_data["file"]].append(file_data)

    if len(data_by_source_file) == 0:
        raise RuntimeError("No coverage data found.")

    # Sort files to make the progress report more useful.
    source_file_order = list(sorted(list(data_by_source_file.keys())))

    new_data_by_source_file = {}
    for i, file_path in enumerate(source_file_order):
        raw_data_for_file = data_by_source_file[file_path]
        print_updateable_line("[{}/{}] merged: {}".format(i + 1, len(data_by_source_file), file_path))
        new_fdata_by_key = {}
        loose_lines = defaultdict(int)

        key_by_function_name = {}

        for data in raw_data_for_file:
            for fdata in data["functions"]:
                start_line = gcov_line_number_to_index(fdata["start_line"])
                end_line = gcov_line_number_to_index(fdata["end_line"])
                start_column = fdata["start_column"]
                end_column = fdata["end_column"]

                function_key = "{}:{}-{}:{}".format(start_line, start_column, end_line, end_column)
                if function_key not in new_fdata_by_key:
                    new_fdata_by_key[function_key] = {
                        "start_line": start_line,
                        "end_line": end_line,
                        "start_column": start_column,
                        "end_column": end_column,
                        "execution_count": 0,
                        "instantiations": {},
                        "lines": defaultdict(int),
                    }

                name = fdata["name"]
                demanged_name = fdata["demangled_name"]
                execution_count = fdata["execution_count"]

                key_by_function_name[name] = function_key

                new_fdata = new_fdata_by_key[function_key]
                new_fdata["execution_count"] += execution_count
                if name not in new_fdata["instantiations"]:
                    new_fdata["instantiations"][name] = {
                        "demangled": demanged_name,
                        "execution_count": 0,
                        "lines": defaultdict(int),
                    }
                new_fdata["instantiations"][name]["execution_count"] += execution_count

            for ldata in data["lines"]:
                line_index = gcov_line_number_to_index(ldata["line_number"])
                count = ldata["count"]
                function_name = ldata.get("function_name")
                if function_name is None:
                    loose_lines[line_index] += ldata["count"]
                else:
                    function_key = key_by_function_name[function_name]
                    new_fdata = new_fdata_by_key[function_key]
                    new_instantiation_fdata = new_fdata["instantiations"][function_name]
                    new_fdata["lines"][line_index] += count
                    new_instantiation_fdata["lines"][line_index] += count

        new_data_by_source_file[file_path] = {
            "file": file_path,
            "functions": new_fdata_by_key,
            "loose_lines": loose_lines,
        }
    print()

    print("Compute summaries...")
    summary_by_source_file = {}
    for i, file_path in enumerate(source_file_order):
        data = new_data_by_source_file[file_path]
        print_updateable_line("[{}/{}] written: {}".format(i + 1, len(new_data_by_source_file), file_path))

        num_instantiated_lines = 0
        num_instantiated_lines_run = 0
        num_instantiated_functions = 0
        num_instantiated_functions_run = 0

        all_lines = set()
        run_lines = set()
        all_function_keys = set()
        run_function_keys = set()

        for function_key, fdata in data["functions"].items():
            all_function_keys.add(function_key)
            if fdata["execution_count"] > 0:
                run_function_keys.add(function_key)

            for line_index, execution_count in fdata["lines"].items():
                all_lines.add(line_index)
                if execution_count > 0:
                    run_lines.add(line_index)

            for function_name, instantiation_fdata in fdata["instantiations"].items():
                num_instantiated_functions += 1
                if instantiation_fdata["execution_count"] > 0:
                    num_instantiated_functions_run += 1
                for line_index, execution_count in instantiation_fdata["lines"].items():
                    num_instantiated_lines += 1
                    if execution_count > 0:
                        num_instantiated_lines_run += 1

        for line_index, execution_count in data["loose_lines"].items():
            num_instantiated_lines += 1
            all_lines.add(line_index)
            if execution_count > 0:
                num_instantiated_lines_run += 1
                run_lines.add(line_index)

        summary_by_source_file[file_path] = {
            "num_instantiated_lines": num_instantiated_lines,
            "num_instantiated_lines_run": num_instantiated_lines_run,
            "num_instantiated_functions": num_instantiated_functions,
            "num_instantiated_functions_run": num_instantiated_functions_run,
            "num_lines": len(all_lines),
            "num_lines_run": len(run_lines),
            "num_functions": len(all_function_keys),
            "num_functions_run": len(run_function_keys),
        }

    summary = {
        "files": summary_by_source_file,
    }

    print()

    print("Clear old analysis...")
    try:
        shutil.rmtree(analysis_dir)
    except:
        pass

    print("Write summary...")
    write_dict_to_zip_file(analysis_dir / "summary.json.zip", summary)

    print("Write per file analysis...")
    for i, file_path in enumerate(source_file_order):
        analysis_file_path = analysis_dir / "files" / Path(file_path).relative_to("/")
        analysis_file_path = str(analysis_file_path) + ".json.zip"

        data = new_data_by_source_file[file_path]
        print_updateable_line("[{}/{}] written: {}".format(i + 1, len(new_data_by_source_file), analysis_file_path))
        write_dict_to_zip_file(analysis_file_path, data)
    print()
    print("Parsed data written to {}.".format(analysis_dir))


def gcov_line_number_to_index(line_number):
    return line_number - 1


def write_dict_to_zip_file(zip_file_path, data):
    zip_file_path = Path(zip_file_path)
    zip_file_path.parent.mkdir(parents=True, exist_ok=True)
    # Was way faster to serialize first before writing to the file instead of using json.dump.
    data_str = json.dumps(data)

    name = zip_file_path.with_suffix("").name
    with zipfile.ZipFile(zip_file_path, "w", compression=zipfile.ZIP_DEFLATED) as f:
        f.writestr(name, data_str)
