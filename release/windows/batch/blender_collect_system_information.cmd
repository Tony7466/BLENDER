@echo off

For /D %%d in (*) do (
  if exist "%%d\python" (
    %%d\python\bin\python.exe %%d\collect_bug_report_info.py
  )
)

pause
