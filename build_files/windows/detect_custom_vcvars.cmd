if NOT "%verbose%" == "" (
	echo Detecting custom vcvars env
)

rem See if MSBuild is on the path
if NOT "%verbose%" == "" (
	echo Testing for MSBuild 
)
msbuild /version > NUL 
if errorlevel 1 (
	if NOT "%verbose%" == "" (
		echo Custom env msbuild not found
	)
	goto FAIL
)

if NOT "%verbose%" == "" (
		echo Custom env msbuild found 
)

rem See if CL is on the path
if NOT "%verbose%" == "" (
	echo Testing for the C/C++ Compiler
)
cl 2> NUL 1>&2
if errorlevel 1 (
	if NOT "%verbose%" == "" (
		echo Custom env C/C++ Compiler not found
	)
	goto FAIL
)

rem We now know we are in a vcvars env of some sort (msbuild/cl work),
rem so we try and detect which one it is

rem Extract the first 2 chars of the VS version in the env
set BUILD_VS_VER=%VisualStudioVersion:~0,2%

if "%BUILD_VS_VER%" == "17" (
	set BUILD_VS_YEAR=2022
) else if "%BUILD_VS_VER%" == "16" (
	set BUILD_VS_YEAR=2019
) else (
	if NOT "%verbose%" == "" (
		echo Custom env year unknown
	)
	goto FAIL
)

if NOT "%verbose%" == "" (
	echo Custom env year detected as VS%BUILD_VS_YEAR%
)
goto EOF

:FAIL
exit /b 1 

:EOF
