if "%GIT%" == "" (
	echo Git not found, cannot apply patches.
	goto ERR
)

if "%CURL%" == "" (
	echo Curl not found, cannot download patches
	goto ERR
)

set MB_0001=https://projects.blender.org/JaumeBellet/mblender/raw/branch/mb-0001-operator-repeat/diff/MB-0001-operator-repeat.diff
set MB_0005=https://projects.blender.org/JaumeBellet/mblender/raw/branch/mb-0005-splash-changes/diff/MB-0005-splash-changes.diff
set MB_0006=https://projects.blender.org/JaumeBellet/mblender/raw/branch/mb-0006-allow-no-modal-transform/diff/MB-0006-allow-no-modal-transform.diff
set MB_0007=https://projects.blender.org/JaumeBellet/mblender/raw/branch/mb-0007-transform-flags/diff/MB-0007-transform-flags.diff

echo Apply MB_0005
"%CURL%" "%MB_0005%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

echo Apply MB_0007
"%CURL%" "%MB_0007%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

rem Depends on MB_007
echo Apply MB_0001
"%CURL%" "%MB_0001%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

rem Depends on MB_007
echo Apply MB_0006
"%CURL%" "%MB_0006%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

echo Now build blender as usually.

:EOF
exit /b 0

:ERR
echo Something went wrong!
exit /b 1
