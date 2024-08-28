#!/bin/sh

script_dir=$(dirname "$0") # Directory to this sh file

for directory in "$script_dir"/*; do
    if test -d "$directory"; then
        if test -d "$directory/python/bin"; then # If Python bin directory exists
            for file in "$directory/python/bin"/*; do
                if grep -i "python" "$file" > /dev/null; then # Search for Python executable
                    "$file" "$directory/scripts/system_info/system_info.py" # Run Python script
                fi
            done
        fi
    fi
done
