#!/bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

for folder in "$script_dir"/*; do
    if [[ -d "$folder" ]]; then
        if [[ -d "$folder/python/bin" ]]; then
            for file in "$folder/python/bin"/*; do
                filename=$(basename "$file")
                if [[ $filename =~ ^python+ ]] ; then
                    "$file" "$folder/scripts/system_info/system_info.py"
                fi
            done
        fi
    fi
done
