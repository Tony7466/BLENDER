script=$(readlink -f "$0") # Path to this sh file
script_dir=$(dirname "$script")

for folder in "$script_dir"/*; do
    if [[ -d "$folder" ]]; then
        if [[ -d "$folder/python/bin" ]]; then # If Python bin folder exists
            for file in "$folder/python/bin"/*; do
                filename=$(basename "$file")
                if [[ $filename =~ ^python+ ]] ; then # Search for Python executable
                    "$file" "$folder/scripts/system_info/system_info.py" # Run Python script
                fi
            done
        fi
    fi
done
