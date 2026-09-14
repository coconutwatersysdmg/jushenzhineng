@echo off
setlocal EnableExtensions
cd /d "%~dp0"

call "%~dp0tools\migrate_runtime_data_to_workdir.bat" /quiet

set "PYTHONNOUSERSITE=1"
set "PYTHONUTF8=1"
set "PYTHONW=%~dp0runtime\pythonw.exe"
set "PYTHON=%~dp0runtime\python.exe"

if not exist "%PYTHON%" (
  echo [ERROR] Portable Python runtime not found:
  echo   %PYTHON%
  echo.
  echo Please run on a build PC first:
  echo   tools\build_runtime.bat
  echo Then copy the whole project folder to the target PC.
  pause
  exit /b 1
)

if not exist "%~dp0main.py" (
  echo [ERROR] main.py not found.
  pause
  exit /b 1
)

if exist "%PYTHONW%" (
  start "" "%PYTHONW%" "%~dp0main.py" %*
) else (
  start "" "%PYTHON%" "%~dp0main.py" %*
)

endlocal
exit /b 0
