@echo off
setlocal
pushd "%~dp0"

if not exist "venv\Scripts\python.exe" (
  echo [ERROR] Project Python was not found: %CD%\venv\Scripts\python.exe
  echo Install dependencies first, then run this launcher again.
  pause
  popd
  exit /b 1
)

"venv\Scripts\python.exe" main.py
set "APP_EXIT_CODE=%ERRORLEVEL%"
if not "%APP_EXIT_CODE%"=="0" (
  echo.
  echo Startup failed. See runtime\startup.log for details.
  pause
)

popd
exit /b %APP_EXIT_CODE%
