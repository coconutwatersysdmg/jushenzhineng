@echo off
cd /d "%~dp0.."
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_mecheye_runtime.ps1" %*
exit /b %ERRORLEVEL%
