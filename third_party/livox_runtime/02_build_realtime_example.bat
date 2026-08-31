@echo off
setlocal

set SDK_ROOT=C:\Users\15316\Desktop\Livox-SDK2
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
  echo Then edit this bat and set the real lib path.
  exit /b 1
)

echo.
echo Build finished: %OUT_EXE%
