# SPDX-License-Identifier: GPL-2.0-or-later

__all__ = (
    "export_mtlx",
)

def export_mtlx(material):
    """ Exports material to .mtlx file. It is called from Blender source code. """
    import materialx.utils as mx_utils

    doc = mx_utils.export(material, None)
    if not doc:
        return ""

    mtlx_file = mx_utils.get_temp_file(".mtlx", f"mat_{material.as_pointer():016x}")
    mx_utils.export_to_file(doc, mtlx_file, export_deps=True, copy_deps=False)
    return str(mtlx_file)
