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
REM Official python-3.12.10-amd64.exe is ~25MB; reject tiny/corrupt downloads.
set "MIN_INSTALLER_BYTES=20000000"

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

for %%A in ("%INSTALLER%") do set "INSTALLER_SIZE=%%~zA"
if not defined INSTALLER_SIZE set "INSTALLER_SIZE=0"
if !INSTALLER_SIZE! LSS %MIN_INSTALLER_BYTES% (
  echo [ERROR] Installer looks corrupt/incomplete: !INSTALLER_SIZE! bytes
  echo         Expected at least %MIN_INSTALLER_BYTES% bytes.
  echo         Delete it and re-run, or download manually:
  echo           %PYTHON_URL%
  echo           -^> %INSTALLER%
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)
echo [INFO] Installer size: !INSTALLER_SIZE! bytes

if not exist "%RUNTIME_DIR%" mkdir "%RUNTIME_DIR%"

echo [INFO] Silent-installing Python into runtime\ ...
echo        (must wait for child setup to finish; do not close this window)
REM start /wait is required: the outer .exe often exits before files appear.
start "" /wait "%INSTALLER%" /quiet InstallAllUsers=0 PrependPath=0 Include_launcher=0 Include_test=0 Shortcuts=0 AssociateFiles=0 InstallLauncherAllUsers=0 TargetDir="%RUNTIME_DIR%"
set "INSTALL_RC=!ERRORLEVEL!"
if not "!INSTALL_RC!"=="0" (
  echo [WARN] Installer exit code=!INSTALL_RC! ; checking TargetDir anyway ...
)

if exist "%RUNTIME_DIR%\python.exe" goto :INSTALL_OK

echo [ERROR] python.exe not found after install: %RUNTIME_DIR%\python.exe
echo.
echo [DIAG] Contents of runtime\ (if any):
dir /b "%RUNTIME_DIR%" 2>nul
echo.
echo Common causes:
echo   1) Same Python %PYTHON_VERSION% already installed on this PC.
echo      Official installer then IGNORES TargetDir and updates the old install.
echo      Fix: Settings -^> Apps -^> uninstall "Python %PYTHON_VERSION%", then re-run:
echo        tools\build_runtime.bat /rebuild
echo   2) Cached installer corrupt — delete and re-download:
echo        del "%INSTALLER%"
echo   3) Antivirus blocked write into runtime\
echo.
set "DEFAULT_PY=%LocalAppData%\Programs\Python\Python312\python.exe"
if exist "%DEFAULT_PY%" (
  echo [DIAG] Found default user install instead:
  echo        %DEFAULT_PY%
  echo        That usually means TargetDir was ignored because 3.12 already exists.
)
echo.
echo To see the installer UI next time, run manually:
echo   "%INSTALLER%" InstallAllUsers=0 PrependPath=0 Include_launcher=0 TargetDir="%RUNTIME_DIR%"
echo.
if "%NO_PAUSE%"=="0" pause
exit /b 1

:INSTALL_OK
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
