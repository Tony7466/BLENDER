# SPDX-FileCopyrightText: 2017-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# TODO: file-type icons are currently not setup.
# Currently `xdg-icon-resource` doesn't support SVG's, so we would need to generate PNG's.
# Or wait until SVG's are supported, see: https://gitlab.freedesktop.org/xdg/xdg-utils/-/merge_requests/41

__all__ = (
    "register",
    "unregister",
)

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

from typing import (
    Optional,
)

VERBOSE = True

# Initialize by `bpy` or command line arguments.
BLENDER_BIN = ""
# Set to `os.path.dirname(BLENDER_BIN)`.
BLENDER_DIR = ""

BLENDER_DESKTOP = "blender.desktop"
HOME_DIR = os.path.normpath(os.path.expanduser("~"))

# https://wiki.archlinux.org/title/XDG_Base_Directory
# Typically: `~/.local/share`.
XDG_DATA_HOME = os.environ.get("XDG_DATA_HOME") or os.path.join(HOME_DIR, ".local", "share")

HOMEDIR_LOCAL_BIN = os.path.join(HOME_DIR, ".local", "bin")

# Use `/usr/local` because this is not managed by the systems package manager.
SYSTEM_PREFIX = "/usr/local"

# Programs.
XDG_MIME_PROG = shutil.which("xdg-mime") or ""


# -----------------------------------------------------------------------------
# Utility Functions

def filepath_repr(filepath: str) -> str:
    if filepath.startswith(HOME_DIR):
        return "~" + filepath[len(HOME_DIR):]
    return filepath


# -----------------------------------------------------------------------------
# Handle Associations

def handle_bin(*, do_unregister: bool, all_users: bool) -> bool:
    if all_users:
        dirpath_dst = os.path.join(SYSTEM_PREFIX, "bin")
    else:
        dirpath_dst = HOMEDIR_LOCAL_BIN

    if VERBOSE:
        sys.stdout.write("- {:s} symbolic-links in: {:s}\n".format(
            ("Setup", "Remove")[do_unregister],
            filepath_repr(dirpath_dst),
        ))

    if not do_unregister:
        found = False
        for path in os.environ.get("PATH", "").split(os.pathsep):
            # `$PATH` can include relative locations.
            path = os.path.normpath(os.path.abspath(path))
            if path == dirpath_dst:
                found = True
                break
        if not found:
            sys.stdout.write(
                "The PATH environment variable doesn't contain \"{:s}\", not creating symlinks\n".format(dirpath_dst)
            )
            # NOTE: this is not an error, don't consider it a failure.
            return True

        os.makedirs(dirpath_dst, exist_ok=True)

    # Full path, then name to create at the destination.
    files_to_link = [
        (BLENDER_BIN, "blender", False),
        # Unfortunately the thumbnailer must be copied for `bwrap` to find it.
        (os.path.join(BLENDER_DIR, "blender-thumbnailer"), "blender-thumbnailer", True),
    ]

    for filepath_src, filename, do_full_copy in files_to_link:
        filepath_dst = os.path.join(dirpath_dst, filename)
        if os.path.exists(filepath_dst):
            os.remove(filepath_dst)
        if do_unregister:
            continue

        if not os.path.exists(filepath_src):
            sys.stderr.write("File not found, skipping link: \"{:s}\" -> \"{:s}\"\n".format(
                filepath_src, filepath_dst,
            ))
        if do_full_copy:
            shutil.copyfile(filepath_src, filepath_dst)
            os.chmod(filepath_dst, 0o755)
        else:
            os.symlink(filepath_src, filepath_dst)
    return True


def handle_desktop_file(*, do_unregister: bool, all_users: bool) -> bool:
    # `cp ./blender.desktop ~/.local/share/applications/`

    filename = BLENDER_DESKTOP

    if all_users:
        base_dir = os.path.join(SYSTEM_PREFIX, "share")
    else:
        base_dir = XDG_DATA_HOME

    dirpath_dst = os.path.join(base_dir, "applications")

    filepath_desktop_src = os.path.join(BLENDER_DIR, filename)
    filepath_desktop_dst = os.path.join(dirpath_dst, filename)

    if VERBOSE:
        sys.stdout.write("- {:s} desktop-file: {:s}\n".format(
            ("Setup", "Remove")[do_unregister],
            filepath_repr(filepath_desktop_dst),
        ))

    if do_unregister:
        if os.path.exists(filepath_desktop_dst):
            os.remove(filepath_desktop_dst)
        return True

    if not os.path.exists(filepath_desktop_src):
        sys.stderr.write("Desktop file not found, skipping: {:s}\n".format(filepath_desktop_src))
        return False

    os.makedirs(dirpath_dst, exist_ok=True)

    with open(filepath_desktop_src, "r", encoding="utf-8") as fh:
        data = fh.read()

    data = data.replace("\nExec=blender %f\n", "\nExec={:s} %f\n".format(BLENDER_BIN))

    with open(filepath_desktop_dst, "w", encoding="utf-8") as fh:
        fh.write(data)
    return True


def handle_thumbnailer(*, do_unregister: bool, all_users: bool) -> bool:
    filename = "blender.thumbnailer"

    if all_users:
        base_dir = os.path.join(SYSTEM_PREFIX, "share")
    else:
        base_dir = XDG_DATA_HOME

    dirpath_dst = os.path.join(base_dir, "thumbnailers")
    filepath_thumbnailer_dst = os.path.join(dirpath_dst, filename)

    if VERBOSE:
        sys.stdout.write("- {:s} thumbnailer: {:s}\n".format(
            ("Setup", "Remove")[do_unregister],
            filepath_repr(filepath_thumbnailer_dst),
        ))

    if do_unregister:
        if os.path.exists(filepath_thumbnailer_dst):
            os.remove(filepath_thumbnailer_dst)
        return True

    blender_thumbnailer_bin = os.path.join(BLENDER_DIR, "blender-thumbnailer")
    if not os.path.exists(blender_thumbnailer_bin):
        sys.stderr.write("Thumbnailer not found, this may not be a portable installation: {:s}\n".format(
            blender_thumbnailer_bin,
        ))
        return True

    os.makedirs(dirpath_dst, exist_ok=True)

    # NOTE: unfortunately this can't be `blender_thumbnailer_bin` because GNOME calls the command
    # with wrapper that means the command *must* be in the users `$PATH`.
    # and it cannot be a SYMLINK.
    if shutil.which("bwrap") is not None:
        command = "blender-thumbnailer"
    else:
        command = blender_thumbnailer_bin

    with open(filepath_thumbnailer_dst, "w", encoding="utf-8") as fh:
        fh.write("[Thumbnailer Entry]\n")
        fh.write("TryExec={:s}\n".format(command))
        fh.write("Exec={:s} %i %o\n".format(command))
        fh.write("MimeType=application/x-blender;\n")
    return True


def handle_mime_association_xml(*, do_unregister: bool, all_users: bool) -> bool:
    # `xdg-mime install x-blender.xml`
    filename = "x-blender.xml"

    if all_users:
        base_dir = os.path.join(SYSTEM_PREFIX, "share")
    else:
        base_dir = XDG_DATA_HOME

    # Unfortunately there doesn't seem to be a way to know the installed location.
    # Use hard-coded location.
    package_xml_dst = os.path.join(base_dir, "mime", "application", filename)

    if VERBOSE:
        sys.stdout.write("- {:s} mime type: {:s}\n".format(
            ("Setup", "Remove")[do_unregister],
            filepath_repr(package_xml_dst),
        ))

    if do_unregister:
        if not os.path.exists(package_xml_dst):
            return True
        # NOTE: `xdg-mime query default application/x-blender` could be used to check
        # if the XML is installed, however there is some slim chance the XML is installed
        # but the default doesn't point to Blender, just uninstall as it's harmless.
        cmd = (
            XDG_MIME_PROG,
            "uninstall",
            "--mode", "system" if all_users else "user",
            package_xml_dst,
        )
        subprocess.check_output(cmd)
        return True

    with tempfile.TemporaryDirectory() as tempdir:
        package_xml_src = os.path.join(tempdir, "x-blender.xml")
        with open(package_xml_src, mode="w", encoding="utf-8") as fh:
            fh.write("""<?xml version="1.0" encoding="UTF-8"?>\n""")
            fh.write("""<mime-info xmlns="http://www.freedesktop.org/standards/shared-mime-info">\n""")
            fh.write("""  <mime-type type="application/x-blender">\n""")
            fh.write("""    <comment>Blender scene</comment>\n""")
            fh.write("""    <glob pattern="*.blend"/>\n""")
            # TODO: this doesn't seem to work, GNOME's Nautilus & KDE's Dolphin
            # already have a file-type icon for this so we might consider this low priority.
            if False:
                fh.write("""    <icon name="application-x-blender"/>\n""")
            fh.write("""  </mime-type>\n""")
            fh.write("""</mime-info>\n""")

        cmd = (
            XDG_MIME_PROG,
            "install",
            "--mode", "system" if all_users else "user",
            package_xml_src,
        )
        subprocess.check_output(cmd)
    return True


def handle_mime_association_default(*, do_unregister: bool, all_users: bool) -> bool:
    # `xdg-mime default blender.desktop application/x-blender`

    if VERBOSE:
        sys.stdout.write("- {:s} mime type as default\n".format(
            ("Setup", "Remove")[do_unregister],
        ))

    # NOTE: there doesn't seem to be a way to reverse this action.
    if do_unregister:
        return True

    cmd = (
        XDG_MIME_PROG,
        "default",
        BLENDER_DESKTOP,
        "application/x-blender",
    )
    subprocess.check_output(cmd)
    return True


def handle_icon(*, do_unregister: bool, all_users: bool) -> bool:
    # `cp ~/`
    filename = "blender.svg"
    if all_users:
        base_dir = os.path.join(SYSTEM_PREFIX, "share")
    else:
        base_dir = XDG_DATA_HOME

    dirpath_dst = os.path.join(base_dir, "icons", "hicolor", "scalable", "apps")

    filepath_desktop_src = os.path.join(BLENDER_DIR, filename)
    filepath_desktop_dst = os.path.join(dirpath_dst, filename)

    if VERBOSE:
        sys.stdout.write("- {:s} icon: {:s}\n".format(
            ("Setup", "Remove")[do_unregister],
            filepath_repr(filepath_desktop_dst),
        ))

    if do_unregister:
        if os.path.exists(filepath_desktop_dst):
            os.remove(filepath_desktop_dst)
        return True

    if not os.path.exists(filepath_desktop_src):
        sys.stderr.write("Icon file not found, skipping: \"{:s}\"\n".format(filepath_desktop_src))
        return False

    os.makedirs(dirpath_dst, exist_ok=True)

    with open(filepath_desktop_src, "rb") as fh:
        data = fh.read()

    with open(filepath_desktop_dst, "wb") as fh:
        fh.write(data)

    return True


# -----------------------------------------------------------------------------
# Main Registration Functions

def register_impl(do_unregister: bool, all_users: bool) -> bool:
    global BLENDER_BIN
    global BLENDER_DIR

    if BLENDER_BIN == "":
        # Only use of `bpy`.
        BLENDER_BIN = os.path.normpath(__import__("bpy").app.binary_path)

    BLENDER_DIR = os.path.dirname(BLENDER_BIN)

    if all_users:
        # Not an admin, run this script with escalated privileges.
        if os.geteuid() != 0:
            prog: Optional[str] = shutil.which("pkexec")
            if prog is None:
                sys.stderr.write("Error: \"pkexec\" not found\n")
                return False
            proc = subprocess.run(
                [
                    prog,
                    sys.executable,
                    __file__,
                    BLENDER_BIN,
                    ("--register-allusers", "--unregister-allusers")[do_unregister],
                ]
            )
            return proc.returncode == 0

    if all_users:
        if not os.access(SYSTEM_PREFIX, os.W_OK):
            sys.stderr.write("Error: {:s} not writable, this command may need to run as a superuser!\n".format(
                SYSTEM_PREFIX,
            ))
            return False

    if VERBOSE:
        print(("Register:", "Unregister:")[do_unregister], BLENDER_BIN)

    if XDG_MIME_PROG == "":
        sys.stderr.write("Could not find \"xdg-mime\", unable to associate mime-types\n")
        return False

    ok = True
    ok &= handle_bin(do_unregister=do_unregister, all_users=all_users)
    ok &= handle_desktop_file(do_unregister=do_unregister, all_users=all_users)
    ok &= handle_mime_association_xml(do_unregister=do_unregister, all_users=all_users)
    ok &= handle_mime_association_default(do_unregister=do_unregister, all_users=all_users)
    ok &= handle_icon(do_unregister=do_unregister, all_users=all_users)

    # The thumbnailer only works when installed for all users.
    if all_users:
        ok &= handle_thumbnailer(do_unregister=do_unregister, all_users=all_users)
    return ok


def register(all_users: bool = False) -> bool:
    return register_impl(False, all_users)


def unregister(all_users: bool = False) -> bool:
    return register_impl(True, all_users)


# -----------------------------------------------------------------------------
# Running directly
#
# Needed when running as an administer.

def argparse_create() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "blender_bin",
        metavar='INSTALL_DIR',
        type=str,
        help="The location of Blender's binary",
    )
    # Match blender's args.
    parser.add_argument(
        "--register",
        dest="register",
        default=False,
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--unregister",
        dest="unregister",
        default=False,
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--register-allusers",
        dest="register_all_users",
        default=False,
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "--unregister-allusers",
        dest="unregister_all_users",
        default=False,
        action="store_true",
        required=False,
    )

    return parser


def main() -> int:
    global BLENDER_BIN
    assert BLENDER_BIN == ""
    args = argparse_create().parse_args()
    BLENDER_BIN = args.blender_bin
    if args.register:
        do_unregister, all_users = False, False
    elif args.unregister:
        do_unregister, all_users = True, False
    elif args.register_all_users:
        do_unregister, all_users = False, True
    elif args.unregister_all_users:
        do_unregister, all_users = True, True

    if not do_unregister:
        result = register(all_users=all_users)
    else:
        result = unregister(all_users=all_users)

    return 0 if result else 1


if __name__ == "__main__":
    sys.exit(main())
