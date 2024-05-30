if NOT EXIST %PYTHON% (
    echo python not found, required for this operation
    exit /b 1
)

set FORMAT_PATHS=%BLENDER_DIR%\tools\utils_maintenance\autopep8_format_paths.py

for %%a in (%PYTHON%) do (
    set PEP8_LOCATION=%%~dpa\..\lib\site-packages\autopep8.py
)

REM Use -B to avoid writing __pycache__ in lib directory and causing update conflicts.
%PYTHON% -B %FORMAT_PATHS%  --autopep8-command "%PEP8_LOCATION%" --no-subprocess %FORMAT_ARGS%

:EOF
