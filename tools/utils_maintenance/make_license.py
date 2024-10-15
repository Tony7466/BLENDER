# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

from pathlib import Path
import os


CURRENT_DIR = Path(os.path.dirname(__file__))
ROOT = Path(os.path.abspath(CURRENT_DIR / "../../"))


class Filepaths:
    LICENSES_FOLDER = ROOT / "release/license/"
    VERSIONS_CMAKE = ROOT / "build_files/build_environment/cmake/versions.cmake"
    EXTERN_LIBRARIES = ROOT / "extern"
    LICENSES_INDEX = LICENSES_FOLDER / "licenses.json"  # List of licenses and definitions
    LICENSE_GENERATED = LICENSES_FOLDER / "license.md"  # Generated licenses file


INTRODUCTION = """<!--

This document is auto-generated with `make license`.
To update it, edit:

 * For external libraries: versions.cmake
 * For internal libraries: the corresponding Blender.README
 * For fonts: make_license.py
 * To add new licenses: licenses.json

Then run `make license` and commit `license.md`.

-->
# Blender Third-Party Licenses

While Blender itself is released under [GPU-GPL 3.0 or later](https://spdx.org/licenses/GPL-3.0-or-later.html),
it contains dependencies which have different licenses.

<SPDX:GPL-3.0-or-later>

## Fonts

| Font     | License | Copyright |
| -------  | --------- | ------- |
| Arev   | Arev Fonts | `Copyright (c) 2006 by Tavmjong Bah. All Rights Reserved.` |
| Bistream | Bistream Vera Fonts | `Copyright (c) 2003 by Bitstream, Inc. All Rights Reserved. Bitstream Vera is a trademark of Bitstream, Inc.`
| DejaVu-Lite | Public Domain | |

<Arev-Fonts>

<SPDX:Bitstream-Vera>

"""


import json
import os
import re
import sys

NO_LICENSE = "No License Set"

LICENSES_NOT_NEEDED = {
    None,
    'Public Domain',
}


class Library:
    name: str
    version: str
    homepage: str
    copyright: str

    def __init__(self, name: str, version: str, homepage: str, copyright: str):
        self.name = name
        self.version = version
        self.homepage = homepage
        self.copyright = copyright

    def __lt__(self, other):
        return self.name.lower() < other.name.lower()

    def __str__(self) -> str:
        raw_data = f'| {self.name} | {self.version} | {self.homepage} | `{self.copyright}` |\n'
        raw_data = raw_data.replace("`None`", "-")
        raw_data = raw_data.replace("``", "-")
        return raw_data


class LicenseDict(dict):
    """Dictionary class that can be merged by the main keys."""
    def __or__(self, other):
        # Create a new LicenseDict object
        new_data = self.copy()
        for key, value in other.items():
            if key in new_data and isinstance(new_data[key], dict) and isinstance(value, dict):
                # Merge dictionaries if both are dicts
                new_data[key] = LicenseDict(new_data[key]) | LicenseDict(value)
            else:
                new_data[key] = value
        return LicenseDict(new_data)

    def __ior__(self, other):
        for key, value in other.items():
            if key in self and isinstance(self[key], dict) and isinstance(value, dict):
                # Merge dictionaries if both are dicts
                self[key] = LicenseDict(self[key]) | LicenseDict(value)
            else:
                self[key] = value
        return self

    def __repr__(self):
        return f"{super().__repr__()}"  # Use the built-in repr of the dict


class License:
    identifier: str
    name: str
    url: str
    libraries: list[Library]

    def __init__(self, identifier, **kwargs):
        self.identifier = identifier
        self.name = kwargs["name"]
        self.url = kwargs.get("url")
        self.libraries = list()

    @property
    def filepath(self):
        if self.identifier.startswith("SPDX"):
            filepath = os.path.join(Filepaths.LICENSES_FOLDER, "spdx", f"{self.identifier[5:]}.txt")
        else:
            filepath = os.path.join(Filepaths.LICENSES_FOLDER, "others", f"{self.identifier}.txt")
        return filepath

    def dump(self) -> str:
        """Read the complete license file from disk and return as string
        """
        # Make sure we only throw the error if we actually need the file.
        # If there are no libraries using this license, there is no need to complain.
        # The json could even have all the licenses from SPDX and only include the ones
        # Blender needs.
        if not os.path.exists(self.filepath):
            print(f'Error: Could not find license file for {self.identifier}: "{self.filepath}"')
            sys.exit(1)

        with open(self.filepath, 'r') as f:
            license_raw = f.read()

        # license_raw = "# TODO Debug" # useful if you want a human-inspectionable document without the licenses.
        license = f'<details>\n<summary>{self.name}</summary>\n' \
            f'\n{license_raw}\n' \
            "</details>"

        return license

    def __lt__(self, other):
        return self.name.lower() < other.name.lower()

    def __repr__(self):
        as_dict = {
            self.identifier: {
                "name": self.name,
                "url": self.url,
                "filepath": self.filepath,
                "libraries": len(self.libraries),
            }
        }
        return json.dumps(as_dict, indent=2)


def initialize_licenses() -> set[License]:
    index_filepath = Filepaths.LICENSES_INDEX

    with open(index_filepath, 'r') as f:
        licenses_json = json.load(f)

    licenses = {key: License(identifier=key, **values) for key, values in licenses_json.items()}
    return licenses


def group_libraries_per_license(libraries_raw: dict) -> LicenseDict:
    grouped_by_license = {}
    for library, details in libraries_raw.items():
        name = details.get("name")
        license = details.get("license", NO_LICENSE)
        if license not in grouped_by_license:
            grouped_by_license[license] = {}
        grouped_by_license[license][name] = details

    return LicenseDict(grouped_by_license)


def process_versions_cmake() -> LicenseDict:
    """
    Parse versions.cmake

        Return a dictionary grouped by license.
    """
    libraries_raw = {}
    with open(Filepaths.VERSIONS_CMAKE, 'r') as versions_cmake:
        # Split the input data into lines
        for line in versions_cmake:
            # Use regex to capture the key and value from each set() statement
            match = re.match(r'set\((\w+)\s+(.+)\)', line)
            if match:
                key, value = match.groups()

                # Determine the library name from the prefix (minus the suffix)
                library_name = '_'.join(key.split('_')[:1])

                # Initialize the library entry if it doesn't exist
                if library_name not in libraries_raw:
                    libraries_raw[library_name] = {
                        "name": library_name.replace("_", " ").title(),
                        # "homepage": None,
                        # "version": None,
                        # "license": None,
                        # "copyright": None,
                        "hash": None,
                    }

                # Populate the relevant fields based on the key
                if key.endswith('NAME'):
                    libraries_raw[library_name]["name"] = value.strip('"')
                elif key.endswith('HOMEPAGE'):
                    libraries_raw[library_name]["homepage"] = value.strip('"')
                elif key.endswith('VERSION'):
                    libraries_raw[library_name]["version"] = value
                elif key.endswith('LICENSE'):
                    libraries_raw[library_name]["license"] = value
                elif key.endswith('COPYRIGHT'):
                    libraries_raw[library_name]["copyright"] = value.strip('"')
                elif key.endswith('HASH'):
                    libraries_raw[library_name]["hash"] = value

    # Cleanup: if there is no hash we assume there is not a real library but some other information on the file.
    libraries_clean = {key: data for key, data in libraries_raw.items() if data["hash"]}
    libraries = group_libraries_per_license(libraries_clean)
    return libraries


def iterate_readme_files(base_folder: str):
    base_path = Path(base_folder)

    # Iterate over all subdirectories
    for subfolder in base_path.iterdir():
        if not subfolder.is_dir():
            continue

        readme_path = subfolder / "README.blender"
        if not readme_path.exists():
            print(f'Warning: Missing file "{readme_path}"')
            continue

        with readme_path.open('r') as readme_file:
            contents = readme_file.read()
            yield contents  # Yield the contents instead of printing


def process_readme_blender() -> LicenseDict:
    """"Handle the README.blender files"""
    keys = {
        "Project": "name",
        "URL": "homepage",
        "License": "license",
        "Upstream version": "version",
        "Copyright": "copyright"
    }

    libraries_raw = {}
    for readme in iterate_readme_files(Filepaths.EXTERN_LIBRARIES):
        lines = readme.strip().split("\n")

        # Initialize an empty dictionary to store the project data
        project_data = {}

        # Temporary storage for project fields
        project_fields = {}

        for line in lines:
            line_split = line.split(":", 1)

            # Ignore comments and empty lines.
            if len(line_split) != 2:
                continue

            key, value = line_split
            key = key.strip()
            value = value.strip().strip('"')  # Strip surrounding quotes, if any

            # Check if the current line matches one of the provided keys
            if key in keys:
                project_fields[keys[key]] = value

        # Assign the fields to the project name
        project_name = project_fields.get("name", "Unknown Project")
        project_data = {
            "name": project_name,
            "version": project_fields.get("version"),
            "homepage": project_fields.get("homepage"),
            "license": project_fields.get("license"),
            "copyright": project_fields.get("copyright")
        }

        libraries_raw[project_name] = project_data

    libraries = group_libraries_per_license(libraries_raw)
    return libraries


def fetch_libraries_licenses(licenses: set[License]) -> None:
    """Populate the licenses dict with its corresponding libraries and copyrights"""

    licenses_data = LicenseDict()
    # Get data from versions.cmake.
    licenses_data |= process_versions_cmake()
    # Get data from README.blender files.
    licenses_data |= process_readme_blender()

    # Populate licenses with the corresponding libraries
    for license_key, libraries_data in licenses_data.items():
        if license_key in licenses:
            license_obj = licenses[license_key]

            for lib_name, lib_info in libraries_data.items():
                library = Library(
                    name=lib_name,
                    version=lib_info["version"],
                    homepage=lib_info.get("homepage", ""),
                    copyright=lib_info.get("copyright", "")
                )
                license_obj.libraries.append(library)
        elif license_key == NO_LICENSE:
            print('Warning: The following libraries have no license:')
            for lib_name, _lib_info in libraries_data.items():
                print(f' * {lib_name}')
        elif license_key in LICENSES_NOT_NEEDED:
            # Do nothing about these licenses.
            pass
        else:
            print(f'Error: {license_key} license not found in: "{Filepaths.LICENSES_INDEX}"')
            # sys.exit(1) # TODO DEBUG


def get_introduction(licenses: set[License]) -> str:
    introduction = INTRODUCTION

    license_lookups = {
        'SPDX:GPL-3.0-or-later',
        'SPDX:Bitstream-Vera',
        'Arev-Fonts',
    }

    for license_lookup in license_lookups:
        if license_lookup not in licenses:
            print(f'Error: {license_lookup} license not found in: "{Filepaths.LICENSES_INDEX}"')
            # sys.exit(1) # TODO DEBUG
            continue

        introduction = introduction.replace(
            f'<{license_lookup}>',
            licenses[license_lookup].dump()
        )

    return introduction


def generate_license_file(licenses: set[License]) -> None:
    filepath = Filepaths.LICENSE_GENERATED

    with open(filepath, 'w') as f:
        f.write(get_introduction(licenses))

        for license in sorted(licenses.values()):
            if not len(license.libraries):
                continue

            if license.url:
                f.write(f'\n\n## [{license.name}]({license.url})\n\n')
            else:
                f.write(f'\n\n## {license.name}\n\n')

            f.write("| Library | version | URL | Copyright |\n")
            f.write("| ------- | ------- | --- | --------- |\n")

            for library in sorted(license.libraries):
                f.write(str(library))

            f.write(license.dump())

    print(f'\nLicense file successfully generated: "{filepath}"')
    print("Remember to commit the file to the Blender repository.\n")


def main():
    licenses = initialize_licenses()

    libraries = fetch_libraries_licenses(licenses)

    generate_license_file(licenses)


if __name__ == "__main__":
    main()
