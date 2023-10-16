if "%GIT%" == "" (
	echo Git not found, cannot apply patches.
	goto ERR
)

if "%CURL%" == "" (
	echo Curl not found, cannot download patches
	goto ERR
)

set MB_001=https://projects.blender.org/JaumeBellet/mblender/raw/branch/mb-0001-operator-repeat/diff/mb-0001-operator-repeat.diff

echo "%MB_001%"

"%CURL%" "%MB_001%" --ssl-no-revoke | "%GIT%" apply 
if errorlevel 1 goto ERR

echo Now build blender as usually.

:EOF
exit /b 0

:ERR
echo Something went wrong!
exit /b 1
