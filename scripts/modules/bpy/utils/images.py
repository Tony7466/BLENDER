# SPDX-FileCopyrightText: 2015-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""
This module contains utility functions to handle custom images.
"""

__all__ = (
    "load",
    "release"
    "list",
)

from _bpy import _utils_images


list = []


def load(name, path):
    r =  _utils_images.load(name, path)
    if r != None:
        data = {'id' : r , 'name' : name, 'path' : path}
        list.append(data)
        return data;
    else:
        return None;

load.__doc__ = _utils_images.load.__doc__;

def release(image_id):
    r = _utils_images.release(image_id)
    if r == True:
        for data in list:
            if data.get('id') == image_id:
                list.remove(data)

    return r;
release.__doc__ = _utils_images.release.__doc__

import atexit

def exit_clear():
    while len(list):
        release(list[0].get('id'))

atexit.register(exit_clear)
del atexit, exit_clear
