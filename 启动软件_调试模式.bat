@echo off
setlocal EnableExtensions
cd /d "%~dp0"

call "%~dp0tools\migrate_runtime_data_to_workdir.bat" /quiet

set "PYTHONNOUSERSITE=1"
set "PYTHONUTF8=1"
set "PYTHON=%~dp0runtime\python.exe"

echo ========================================
echo  Debug launcher
echo ========================================
echo Project : %CD%
echo Python  : %PYTHON%
echo.

if not exist "%PYTHON%" (
  echo [ERROR] Portable Python runtime not found:
  echo   %PYTHON%
  echo.
  echo Please run on a build PC first:
  echo   tools\build_runtime.bat
  pause
  exit /b 1
)

if not exist "%~dp0main.py" (
  echo [ERROR] main.py not found.
  pause
  exit /b 1
)

"%PYTHON%" -c "import sys; print('sys.executable =', sys.executable); print('sys.version  =', sys.version)"
echo.
echo Starting main.py ...
echo ----------------------------------------
"%PYTHON%" "%~dp0main.py" %*
set "APP_EXIT_CODE=%ERRORLEVEL%"
echo ----------------------------------------
echo Exit code: %APP_EXIT_CODE%
if not "%APP_EXIT_CODE%"=="0" (
  echo.
  echo Check logs\startup.log and logs\startup_fault.log
  echo Or run: runtime\python.exe tools\check_runtime.py
)
echo.
pause
endlocal
exit /b %APP_EXIT_CODE%
