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

"%CURL%" "%MB_0001%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

"%CURL%" "%MB_0005%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

"%CURL%" "%MB_0006%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

echo Now build blender as usually.

:EOF
exit /b 0

:ERR
echo Something went wrong!
exit /b 1
