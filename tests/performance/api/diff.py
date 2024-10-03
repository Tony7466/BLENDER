# SPDX-FileCopyrightText: 2021-2022 Blender Authors
#
# SPDX-License-Identifier: Apache-2.0

from collections import defaultdict
from dataclasses import dataclass

from . import TestQueue, TestEntry

import json
import pathlib
from typing import Self


@dataclass(frozen=True, order=True)
class RevPair:
    date: str
    revision: str

    @classmethod
    def from_queue(cls, queue: TestQueue) -> Self:
        latest_date = 0
        latest_revision = None
        for entry in queue.entries:
            if entry.date > latest_date:
                latest_date = entry.date
                latest_revision = entry.revision
        return cls(latest_date, latest_revision)

    @classmethod
    def from_entry(cls, entry: TestEntry) -> Self:
        return cls(entry.date, entry.revision)

    def __contains__(self, entry: TestEntry) -> bool:
        return entry.date == self.date and entry.revision == self.revision


class TestDiff:
    def __init__(self, base_file: pathlib.Path, diff_file: pathlib.Path):
        # Initialize graph from JSON file. Note that this is implemented without
        # accessing any benchmark environment or configuration. This ways benchmarks
        # run on various machines can be aggregated and the graph generated on another
        # machine.

        base_queue = TestQueue(base_file)
        diff_queue = TestQueue(diff_file)

        latest_base = RevPair.from_queue(base_queue)
        latest_diff = RevPair.from_queue(diff_queue)

        base_data = self._get_data(base_queue, latest_base)
        diff_data = self._get_data(diff_queue, None)

        data = []
        for key in sorted(set(diff_data.keys()) | set(base_data.keys())):
            device_name, category, output = key
            chart_name = f"{category} ({output})"

            data.append(self.chart_diff(device_name, chart_name, base_data[key], diff_data[key], latest_diff, output))

        self.json = json.dumps(data, indent=2)

    def _get_data(self, queue: TestQueue, filter: RevPair | None):
        # Gather entries for each (device, category, output_key) tuple.
        data = defaultdict(list)
        for entry in queue.entries:
            if filter is not None and entry not in filter:
                continue
            if entry.status in {'done', 'outdated'}:
                device_name = entry.device_name + " (" + entry.device_type + ")"
                for output in entry.output:
                    key = (device_name, entry.category, output)
                    data[key].append(entry)
        return data

    def chart_diff(self, device_name: str, chart_name: str, base_entries: list[TestEntry], diff_entries: list[TestEntry], latest_diff: RevPair, output: str) -> dict:
        # Only list tests for which we have a baseline and at least one diff value
        tests = set(entry.test for entry in base_entries) & set(entry.test for entry in diff_entries)

        # Gather baseline values for all tests.
        baselines = {entry.test: entry.output[output] for entry in base_entries}

        # Gather all revisions that we have data for.
        revisions = sorted(list(set(RevPair.from_entry(entry) for entry in diff_entries)))

        # Gather the values for all test x revision combinations
        values = {test: dict() for test in tests}
        for entry in diff_entries:
            if entry.test not in tests:
                continue
            if output not in entry.output:
                continue
            value = entry.output[output] / baselines[entry.test] - 1
            values[entry.test][RevPair.from_entry(entry)] = value

        # Sort by the difference in the latest revision
        tests = sorted(list(tests), key=lambda test: values[test][latest_diff])

        data = []
        # First row contains the column headers
        header = [' ']
        for revision in revisions:
            header.append(revision.revision)
        data.append(header)
        # Following rows contain one test and all its values across the revisions
        for test in tests:
            row = [test]
            for revision in revisions:
                value = values[test].get(revision, -1.0)
                row.append(value)
            data.append(row)

        return {'device': device_name, 'name': chart_name, 'data': data}

    def write(self, filepath: pathlib.Path) -> None:
        # Write HTML page with JSON diff data embedded.
        template_dir = pathlib.Path(__file__).parent
        with open(template_dir / 'diff.template.html', 'r') as f:
            template = f.read()

        contents = template.replace('%JSON_DATA%', self.json)
        with open(filepath, "w") as f:
            f.write(contents)
