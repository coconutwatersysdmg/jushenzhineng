@echo off
setlocal

REM 设置 Livox-SDK2 根目录后再编译。不要写死某台电脑的用户路径。
REM 示例：
REM   set SDK_ROOT=D:\deps\Livox-SDK2
if not defined SDK_ROOT (
  echo [ERROR] Please set SDK_ROOT to your Livox-SDK2 directory first.
  echo Example: set SDK_ROOT=D:\deps\Livox-SDK2
  exit /b 1
)

set SRC_DIR=%~dp0
set OUT_EXE=%SRC_DIR%livox_realtime_select_and_move.exe

echo SDK_ROOT=%SDK_ROOT%
echo SRC_DIR=%SRC_DIR%

cl /EHsc /std:c++17 /MD ^
  /I "%SDK_ROOT%\include" ^
  "%SRC_DIR%livox_realtime_select_and_move.cpp" ^
  /link ^
  "%SDK_ROOT%\build\sdk_core\livox_lidar_sdk_static.lib" ^
  ws2_32.lib ^
  /OUT:"%OUT_EXE%"

if errorlevel 1 (
  echo.
  echo Build failed. Please check SDK_ROOT.
  echo If livox_lidar_sdk_static.lib is not found, search it under Livox-SDK2\build.
  echo Then set SDK_ROOT to the real SDK root and retry.
  exit /b 1
)

echo.
echo Build finished: %OUT_EXE%
