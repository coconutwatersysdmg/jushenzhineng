@echo off
cd /d "%~dp0.."
if exist "runtime\python.exe" (
  "runtime\python.exe" -m plc_console
) else (
  python -m plc_console
)
