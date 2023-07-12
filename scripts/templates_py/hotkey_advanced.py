# Usage: Run this script, then in the 3D viewport press Ctrl+Shift+Q
# to run the To Sphere operator in object mode. 
# (Have multiple objects in different positions selected)

from bpy_extras.keyconfig_utils import PyKeyMapItem, KeyMapException

# Register a shortcut in multiple KeyMaps.
# Instead of using the ``addon_hotkey_register`` function, we can get more control
# by instantiating a PyKeyMapItem directly.
# The only difference here is that the ``keymap_name`` gets passed later.
py_kmi = PyKeyMapItem(
    op_idname='transform.tosphere',
    key_id='Q',
    ctrl=True,
    shift=True,
    op_kwargs={},
)
try:
    py_kmi.register(keymap_name='Pose')
    py_kmi.register(keymap_name='Armature')
    py_kmi.register(keymap_name='Weight Paint')
    py_kmi.register(keymap_name='Object Mode')

except KeyMapException as e:
    # Registering a hotkey that would result in a key conflict will result in an error.
    print(str(e))

# Unregister all versions of the hotkey.
py_kmi.unregister()

# Register one back again!
py_kmi.register(keymap_name='Object Mode')
