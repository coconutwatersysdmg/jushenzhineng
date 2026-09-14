@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0.."

set "DO_REBUILD=0"
set "NO_PAUSE=0"
if /I "%~1"=="/rebuild" set "DO_REBUILD=1"
if /I "%~2"=="/rebuild" set "DO_REBUILD=1"
if /I "%~1"=="/nopause" set "NO_PAUSE=1"
if /I "%~2"=="/nopause" set "NO_PAUSE=1"

echo ========================================
echo  Build portable Python runtime
echo ========================================
echo Project root: %CD%
echo.

call "%~dp0migrate_runtime_data_to_workdir.bat" /quiet
if errorlevel 1 (
  echo [ERROR] Failed to migrate old runtime data.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

set "RUNTIME_DIR=%CD%\runtime"
set "PYTHON_VERSION=3.12.10"
set "INSTALLER_NAME=python-%PYTHON_VERSION%-amd64.exe"
set "CACHE_DIR=%CD%\tools\_cache"
set "INSTALLER=%CACHE_DIR%\%INSTALLER_NAME%"
set "PYTHON_URL=https://www.python.org/ftp/python/%PYTHON_VERSION%/%INSTALLER_NAME%"

if exist "%RUNTIME_DIR%\python.exe" (
  echo [INFO] Found: %RUNTIME_DIR%\python.exe
  if "%DO_REBUILD%"=="0" (
    echo [SKIP] Keep existing runtime; install/update deps only.
    echo        Rebuild with: tools\build_runtime.bat /rebuild
    goto :INSTALL_DEPS
  )
  echo [INFO] /rebuild: removing old runtime ...
  rmdir /S /Q "%RUNTIME_DIR%" 2>nul
  if exist "%RUNTIME_DIR%" (
    echo [ERROR] Cannot delete runtime. Close programs using it and retry.
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
)

if exist "%RUNTIME_DIR%" (
  echo [INFO] Cleaning leftover files under runtime ...
  rmdir /S /Q "%RUNTIME_DIR%" 2>nul
  if exist "%RUNTIME_DIR%" (
    echo [WARN] runtime folder not fully removed; installer will reuse TargetDir.
  )
)

if not exist "%CACHE_DIR%" mkdir "%CACHE_DIR%"

if not exist "%INSTALLER%" (
  echo [INFO] Downloading Python %PYTHON_VERSION% ...
  echo        %PYTHON_URL%
  powershell -NoProfile -ExecutionPolicy Bypass -Command "try { Invoke-WebRequest -Uri '%PYTHON_URL%' -OutFile '%INSTALLER%' -UseBasicParsing } catch { Write-Error $_; exit 1 }"
  if errorlevel 1 (
    echo [ERROR] Download failed. Put the installer manually at:
    echo   %INSTALLER%
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
) else (
  echo [INFO] Using cached installer: %INSTALLER%
)

echo [INFO] Silent-installing Python into runtime\ ...
"%INSTALLER%" /quiet InstallAllUsers=0 PrependPath=0 Include_launcher=0 Include_test=0 Shortcuts=0 AssociateFiles=0 InstallLauncherAllUsers=0 TargetDir="%RUNTIME_DIR%"
if errorlevel 1 (
  echo [ERROR] Python install failed.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

if not exist "%RUNTIME_DIR%\python.exe" (
  echo [ERROR] python.exe not found after install.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo [OK] Python installed: %RUNTIME_DIR%\python.exe
"%RUNTIME_DIR%\python.exe" -c "import sys; print(sys.version)"

:INSTALL_DEPS
echo.
echo [INFO] Upgrading pip / setuptools / wheel ...
"%RUNTIME_DIR%\python.exe" -m ensurepip --upgrade
"%RUNTIME_DIR%\python.exe" -m pip install --upgrade pip setuptools wheel
if errorlevel 1 (
  echo [ERROR] pip bootstrap failed.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo.
echo [INFO] Installing requirements.txt (CPU torch; may take a while) ...
"%RUNTIME_DIR%\python.exe" -m pip install -r "%CD%\requirements.txt"
if errorlevel 1 (
  echo [ERROR] Dependency install failed. Check network/proxy and retry.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo.
echo [INFO] Running tools\check_runtime.py ...
"%RUNTIME_DIR%\python.exe" "%CD%\tools\check_runtime.py"
set "CHECK_RC=%ERRORLEVEL%"

echo.
if not "%CHECK_RC%"=="0" (
  echo [WARN] Self-check reported issues. Exit code=%CHECK_RC%
) else (
  echo [OK] runtime build finished.
)
echo.
echo Start with:
echo   启动软件.bat
echo   启动软件_调试模式.bat
echo.
echo Copy the whole jushenzhineng_v3 folder (including runtime) to the target PC.
echo Target PC does not need Anaconda or a new venv.
echo.
if "%NO_PAUSE%"=="0" pause
endlocal
exit /b %CHECK_RC%
