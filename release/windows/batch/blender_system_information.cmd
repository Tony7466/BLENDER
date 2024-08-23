@echo off

For /D %%d in (*) do (
  if exist "%%d\python\bin\python.exe" (
    "%%d\python\bin\python.exe" "%%d\scripts\system_info\system_info.py"
  )
)
