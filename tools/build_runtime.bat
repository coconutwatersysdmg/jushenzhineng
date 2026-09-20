@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0.."

set "DO_REBUILD=0"
set "NO_PAUSE=0"
if /I "%~1"=="/rebuild" set "DO_REBUILD=1"
if /I "%~2"=="/rebuild" set "DO_REBUILD=1"
if /I "%~1"=="/nopause" set "NO_PAUSE=1"
if /I "%~2"=="/nopause" set "NO_PAUSE=1"

set "RUNTIME_DIR=%CD%\runtime"
set "PYTHON_VERSION=3.12.10"
set "PY_MAJOR_MINOR=312"
set "REQUIRED_MAJOR=3"
set "REQUIRED_MINOR=12"
set "CACHE_DIR=%CD%\tools\_cache"
set "EMBED_NAME=python-%PYTHON_VERSION%-embed-amd64.zip"
set "EMBED_ZIP=%CACHE_DIR%\%EMBED_NAME%"
set "EMBED_URL=https://www.python.org/ftp/python/%PYTHON_VERSION%/%EMBED_NAME%"
set "GETPIP=%CACHE_DIR%\get-pip.py"
set "GETPIP_URL=https://bootstrap.pypa.io/get-pip.py"
REM Official embed zip is ~11MB; reject tiny/corrupt downloads.
set "MIN_EMBED_BYTES=8000000"

echo ========================================
echo  构建项目便携 Python runtime
echo ========================================
echo 项目目录: %CD%
echo.
echo [说明] 本脚本会下载并安装项目内嵌的 Python %PYTHON_VERSION%。
echo        与系统自带的 python / 3.13.x / Anaconda 无关，也不会改系统 Python。
echo        目标机即使是 3.13.9，也请用本脚本；日常改代码无需反复执行。
echo        首次部署或换机: tools\build_runtime.bat
echo        强制整包重建: tools\build_runtime.bat /rebuild
echo.

call "%~dp0migrate_runtime_data_to_workdir.bat" /quiet
if errorlevel 1 (
  echo [ERROR] Failed to migrate old runtime data.
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

REM 提示系统 Python（仅展示，绝不用于安装）
echo [INFO] 探测系统 Python（仅提示，不会使用）...
where python >nul 2>&1
if errorlevel 1 (
  echo        系统 PATH 中未找到 python —— 没关系，本脚本不依赖它。
) else (
  for /f "delims=" %%P in ('where python 2^>nul') do (
    echo        发现: %%P
    goto :AFTER_WHERE_PY
  )
)
:AFTER_WHERE_PY
python -c "import sys; print('       版本:', sys.version.split()[0])" 2>nul
if errorlevel 1 (
  echo        （无法读取系统 python 版本，可忽略）
) else (
  echo        提醒: 即使上面是 3.13.x，依赖也会装进 runtime\ 里的 3.12，不要用系统 python 跑本项目。
)
echo.

if exist "%RUNTIME_DIR%\python.exe" (
  echo [INFO] 已找到: %RUNTIME_DIR%\python.exe
  "%RUNTIME_DIR%\python.exe" -c "import sys; raise SystemExit(0 if sys.version_info[:2]==(%REQUIRED_MAJOR%,%REQUIRED_MINOR%) else 1)" 2>nul
  if errorlevel 1 (
    echo [WARN] 现有 runtime 不是 Python %REQUIRED_MAJOR%.%REQUIRED_MINOR%，将自动整包重建。
    set "DO_REBUILD=1"
  ) else (
    echo [OK]   runtime 版本符合要求: Python %REQUIRED_MAJOR%.%REQUIRED_MINOR%.x
  )
  if "!DO_REBUILD!"=="0" (
    echo [SKIP] 保留现有 runtime，仅安装/更新 requirements.txt。
    echo        若要整包重装请运行: tools\build_runtime.bat /rebuild
    goto :INSTALL_DEPS
  )
  echo [INFO] /rebuild: 删除旧 runtime ...
  rmdir /S /Q "%RUNTIME_DIR%" 2>nul
  if exist "%RUNTIME_DIR%" (
    echo [ERROR] 无法删除 runtime。请关闭正在使用它的程序后重试。
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
)

if exist "%RUNTIME_DIR%" (
  echo [INFO] 清理 runtime 残留文件 ...
  rmdir /S /Q "%RUNTIME_DIR%" 2>nul
  if exist "%RUNTIME_DIR%" (
    echo [ERROR] 无法清空 runtime 目录。请关闭占用后重试。
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
)

if not exist "%CACHE_DIR%" mkdir "%CACHE_DIR%"

if not exist "%EMBED_ZIP%" (
  echo [INFO] 下载 Python embed %PYTHON_VERSION% ...
  echo        %EMBED_URL%
  powershell -NoProfile -ExecutionPolicy Bypass -Command "try { Invoke-WebRequest -Uri '%EMBED_URL%' -OutFile '%EMBED_ZIP%' -UseBasicParsing } catch { Write-Error $_; exit 1 }"
  if errorlevel 1 (
    echo [ERROR] 下载失败。请手动把 zip 放到:
    echo   %EMBED_ZIP%
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
) else (
  echo [INFO] 使用缓存的 embed zip: %EMBED_ZIP%
)

for %%A in ("%EMBED_ZIP%") do set "EMBED_SIZE=%%~zA"
if not defined EMBED_SIZE set "EMBED_SIZE=0"
if !EMBED_SIZE! LSS %MIN_EMBED_BYTES% (
  echo [ERROR] Embed zip 损坏或不完整: !EMBED_SIZE! bytes
  echo         期望至少 %MIN_EMBED_BYTES% bytes。
  echo         请删除后重跑，或手动下载:
  echo           %EMBED_URL%
  echo           -^> %EMBED_ZIP%
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)
echo [INFO] Embed zip 大小: !EMBED_SIZE! bytes

mkdir "%RUNTIME_DIR%" >nul 2>&1
echo [INFO] 解压到 runtime\ ...
powershell -NoProfile -ExecutionPolicy Bypass -Command "Expand-Archive -LiteralPath '%EMBED_ZIP%' -DestinationPath '%RUNTIME_DIR%' -Force"
if errorlevel 1 (
  echo [ERROR] Expand-Archive 失败。
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

if not exist "%RUNTIME_DIR%\python.exe" (
  echo [ERROR] 解压后缺少 python.exe: %RUNTIME_DIR%\python.exe
  echo [DIAG] runtime\ 内容:
  dir /b "%RUNTIME_DIR%" 2>nul
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

REM Embed ._pth is isolated: "." = runtime\, ".." = project root (for ui/ etc).
set "PTH_FILE=%RUNTIME_DIR%\python%PY_MAJOR_MINOR%._pth"
echo [INFO] 配置 site-packages + 项目根目录: %PTH_FILE%
> "%PTH_FILE%" (
  echo python%PY_MAJOR_MINOR%.zip
  echo .
  echo ..
  echo Lib\site-packages
  echo import site
)

if not exist "%GETPIP%" (
  echo [INFO] 下载 get-pip.py ...
  powershell -NoProfile -ExecutionPolicy Bypass -Command "try { Invoke-WebRequest -Uri '%GETPIP_URL%' -OutFile '%GETPIP%' -UseBasicParsing } catch { Write-Error $_; exit 1 }"
  if errorlevel 1 (
    echo [ERROR] get-pip 下载失败: %GETPIP_URL%
    if "%NO_PAUSE%"=="0" pause
    exit /b 1
  )
) else (
  echo [INFO] 使用缓存的 get-pip.py
)

echo [INFO] 向 embed runtime 安装 pip ...
"%RUNTIME_DIR%\python.exe" "%GETPIP%" --no-warn-script-location
if errorlevel 1 (
  echo [ERROR] get-pip 失败。
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo [OK] Python 就绪: %RUNTIME_DIR%\python.exe
"%RUNTIME_DIR%\python.exe" -c "import sys; print(sys.version); print(sys.executable)"
"%RUNTIME_DIR%\python.exe" -c "import sys; raise SystemExit(0 if sys.version_info[:2]==(%REQUIRED_MAJOR%,%REQUIRED_MINOR%) else 1)"
if errorlevel 1 (
  echo [ERROR] runtime 不是 Python %REQUIRED_MAJOR%.%REQUIRED_MINOR%，中止安装依赖。
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

:INSTALL_DEPS
echo.
echo [INFO] 再次确认将使用的解释器（必须是项目 runtime，不是系统 3.13）...
"%RUNTIME_DIR%\python.exe" -c "import sys; print('executable=', sys.executable); print('version=', sys.version.split()[0]); raise SystemExit(0 if sys.version_info[:2]==(%REQUIRED_MAJOR%,%REQUIRED_MINOR%) else 1)"
if errorlevel 1 (
  echo [ERROR] %RUNTIME_DIR%\python.exe 版本不对。请执行: tools\build_runtime.bat /rebuild
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo.
echo [INFO] 升级 pip / setuptools / wheel ...
"%RUNTIME_DIR%\python.exe" -m pip install --upgrade pip setuptools wheel
if errorlevel 1 (
  echo [ERROR] pip bootstrap 失败。
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo.
echo [INFO] 安装 requirements.txt 到 runtime（CPU torch，可能较久）...
echo        命令等价于: runtime\python.exe -m pip install -r requirements.txt
"%RUNTIME_DIR%\python.exe" -m pip install -r "%CD%\requirements.txt"
if errorlevel 1 (
  echo [ERROR] 依赖安装失败。请检查网络/代理后重试。
  if "%NO_PAUSE%"=="0" pause
  exit /b 1
)

echo.
echo [INFO] 运行 tools\check_runtime.py ...
"%RUNTIME_DIR%\python.exe" "%CD%\tools\check_runtime.py"
set "CHECK_RC=%ERRORLEVEL%"

echo.
if not "%CHECK_RC%"=="0" (
  echo [WARN] 自检有问题。Exit code=%CHECK_RC%
) else (
  echo [OK] runtime 构建完成（Python %REQUIRED_MAJOR%.%REQUIRED_MINOR% 便携环境）。
)
echo.
echo 启动方式（不要用系统 python / 3.13）:
echo   启动软件.bat
echo   启动软件_调试模式.bat
echo   或: runtime\python.exe main.py
echo.
echo 换机部署: 拷贝整个项目文件夹（含 runtime\）即可；目标机不必装 Anaconda。
echo 改业务代码后: 直接启动，无需再跑本脚本。
echo 仅当改了 requirements.txt 或 runtime 损坏时: 再运行本 bat（或加 /rebuild）。
echo.
if "%NO_PAUSE%"=="0" pause
endlocal
exit /b %CHECK_RC%
