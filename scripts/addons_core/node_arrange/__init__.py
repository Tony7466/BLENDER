# SPDX-License-Identifier: GPL-2.0-or-later

bl_info = {
  "name": "Node Arrange",
  "description": "Node Tree Arrangement Tools",
  "author": "Leonardo Pike-Excell",
  "version": (0, 9, 0),
  "blender": (4, 1, 0),
  "location": "Node Editor > Arrange",
  "category": "Node"}

from importlib import reload

should_reload = 'operators' in locals()
from . import (
  ui,
  operators,
  properties,
)

if should_reload:
    properties = reload(properties)
    operators = reload(operators)
    ui = reload(ui)


def register():
    ui.register()
    operators.register()
    properties.register()


def unregister():
    properties.unregister()
    operators.unregister()
    ui.unregister()
