@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"

REM 兼容旧启动入口：转到便携 runtime 启动脚本
call "%~dp0启动软件.bat" %*
exit /b %ERRORLEVEL%
