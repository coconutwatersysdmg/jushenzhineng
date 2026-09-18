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
set "PY_MAJOR_MINOR=312"
set "CACHE_DIR=%CD%\tools\_cache"
set "EMBED_NAME=python-%PYTHON_VERSION%-embed-amd64.zip"
set "EMBED_ZIP=%CACHE_DIR%\%EMBED_NAME%"
set "EMBED_URL=https://www.python.org/ftp/python/%PYTHON_VERSION%/%EMBED_NAME%"
set "GETPIP=%CACHE_DIR%\get-pip.py"
set "GETPIP_URL=https://bootstrap.pypa.io/get-pip.py"
REM Official embed zip is ~11MB; reject tiny/corrupt downloads.
set "MIN_EMBED_BYTES=8000000"

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
    echo [ERROR] Cannot clear runtime folder. Close programs using it and retry.
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
)

if not exist "%CACHE_DIR%" mkdir "%CACHE_DIR%"

if not exist "%EMBED_ZIP%" (
  echo [INFO] Downloading Python embed %PYTHON_VERSION% ...
  echo        %EMBED_URL%
  powershell -NoProfile -ExecutionPolicy Bypass -Command "try { Invoke-WebRequest -Uri '%EMBED_URL%' -OutFile '%EMBED_ZIP%' -UseBasicParsing } catch { Write-Error $_; exit 1 }"
  if errorlevel 1 (
    echo [ERROR] Download failed. Put the zip manually at:
    echo   %EMBED_ZIP%
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
) else (
  echo [INFO] Using cached embed zip: %EMBED_ZIP%
)

for %%A in ("%EMBED_ZIP%") do set "EMBED_SIZE=%%~zA"
if not defined EMBED_SIZE set "EMBED_SIZE=0"
if !EMBED_SIZE! LSS %MIN_EMBED_BYTES% (
  echo [ERROR] Embed zip looks corrupt/incomplete: !EMBED_SIZE! bytes
  echo         Expected at least %MIN_EMBED_BYTES% bytes.
  echo         Delete it and re-run, or download manually:
  echo           %EMBED_URL%
  echo           -^> %EMBED_ZIP%
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)
echo [INFO] Embed zip size: !EMBED_SIZE! bytes

mkdir "%RUNTIME_DIR%" >nul 2>&1
echo [INFO] Extracting embed package into runtime\ ...
powershell -NoProfile -ExecutionPolicy Bypass -Command "Expand-Archive -LiteralPath '%EMBED_ZIP%' -DestinationPath '%RUNTIME_DIR%' -Force"
if errorlevel 1 (
  echo [ERROR] Expand-Archive failed.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

if not exist "%RUNTIME_DIR%\python.exe" (
  echo [ERROR] python.exe missing after extract: %RUNTIME_DIR%\python.exe
  echo [DIAG] Contents of runtime\:
  dir /b "%RUNTIME_DIR%" 2>nul
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

REM Embed disables site-packages by default; enable pip installs.
set "PTH_FILE=%RUNTIME_DIR%\python%PY_MAJOR_MINOR%._pth"
echo [INFO] Enabling site-packages in %PTH_FILE%
> "%PTH_FILE%" (
  echo python%PY_MAJOR_MINOR%.zip
  echo .
  echo Lib\site-packages
  echo import site
)

if not exist "%GETPIP%" (
  echo [INFO] Downloading get-pip.py ...
  powershell -NoProfile -ExecutionPolicy Bypass -Command "try { Invoke-WebRequest -Uri '%GETPIP_URL%' -OutFile '%GETPIP%' -UseBasicParsing } catch { Write-Error $_; exit 1 }"
  if errorlevel 1 (
    echo [ERROR] get-pip download failed: %GETPIP_URL%
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
) else (
  echo [INFO] Using cached get-pip.py
)

echo [INFO] Bootstrapping pip into embed runtime ...
"%RUNTIME_DIR%\python.exe" "%GETPIP%" --no-warn-script-location
if errorlevel 1 (
  echo [ERROR] get-pip failed.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo [OK] Python ready: %RUNTIME_DIR%\python.exe
"%RUNTIME_DIR%\python.exe" -c "import sys; print(sys.version); print(sys.executable)"

:INSTALL_DEPS
echo.
echo [INFO] Upgrading pip / setuptools / wheel ...
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
echo Copy the whole project folder (including runtime) to the target PC.
echo Target PC does not need Anaconda or a system Python.
echo.
if "%NO_PAUSE%"=="0" pause
endlocal
exit /b %CHECK_RC%
