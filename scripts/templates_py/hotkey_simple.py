# Usage: Run this script, then in the 3D viewport press Ctrl+Alt+A
# to activate the new hotkey for the Add menu (normally on Shift+A)

from bpy_extras.keyconfig_utils import addon_hotkey_register, KeyMapException

try:
    addon_hotkey_register(
        keymap_name='3D View',
        op_idname='wm.call_menu',
        key_id='A',
        ctrl=True,
        alt=True,
        op_kwargs={'name' : 'VIEW3D_MT_add'},
    )
except KeyMapException as e:
    # Registering a hotkey that would result in a key conflict will result in an error.
    print(str(e))
