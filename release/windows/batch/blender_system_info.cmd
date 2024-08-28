@echo off

For /D %%d in (*) do (
  if exist "%%d\python\bin\python.exe" (
    "%%d\python\bin\python.exe" "%%d\scripts\system_info\system_info.py"
    exit /b
  )
)

echo ERROR: Failed to find python.exe. Possible causes include:
echo - Your Blender installation is corrupt or missing python.exe.
echo - You're a developer using a debug build of Blender.
echo - The location or name of python.exe has changed.
pause
