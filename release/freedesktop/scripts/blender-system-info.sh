#!/bin/sh

script_dir=$(dirname "$0") # Directory to this sh file

for directory in "$script_dir"/*; do
    if test -d "$directory"; then
        if test -d "$directory/python/bin"; then # If Python bin directory exists
            for file in "$directory/python/bin"/*; do
                if grep -i "python" "$file" > /dev/null; then # Search for Python executable
                    "$file" "$directory/scripts/system_info/system_info.py" # Run Python script
                    exit 0
                fi
            done
        fi
    fi
done

echo "ERROR: Failed to find python executable. Possible causes include:"
echo "- Your Blender installation is corrupt or missing python."
echo "- You're a developer using a debug build of Blender."
echo "- The location or name of python of the python executable has changed."
