#!/bin/sh

# Directory to this SH file.
BASE_DIR=$(dirname "$0")

PYTHON_EXECUTABLE=$(basename "@PYTHON_EXECUTABLE@")
PYTHON_BIN="$BASE_DIR/@BLENDER_VERSION@/python/bin/$PYTHON_EXECUTABLE"
SYSTEM_INFO_STARTUP_PY="$BASE_DIR/@BLENDER_VERSION@/scripts/modules/_bpy_internal/system_info/startup.py"
if test -f "$PYTHON_BIN"; then
  exec "$PYTHON_BIN" -I "$SYSTEM_INFO_STARTUP_PY"
fi

echo "ERROR: Failed to find python executable. Possible causes include:"
echo "- Your Blender installation is corrupt or missing python."
echo "- You're a developer using a debug build of Blender."
echo "- The location or name of python of the python executable has changed."
exit 1
