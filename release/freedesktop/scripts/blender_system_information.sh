#!/bin/sh

script_dir=$(dirname "$0") # Directory to this sh file

for folder in "$script_dir"/*; do
    if test -d "$folder"; then
        if test -d "$folder/python/bin"; then # If Python bin folder exists
            for file in "$folder/python/bin"/*; do
                filename=$(basename "$file")
                if grep -i "python" "$file" > /dev/null; then # Search for Python executable
                    "$file" "$folder/scripts/system_info/system_info.py" # Run Python script
                fi
            done
        fi
    fi
done
