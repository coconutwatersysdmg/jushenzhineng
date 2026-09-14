@echo off
REM Migrate old business data from runtime\ to workdir\ and logs\
REM Skip when runtime\python.exe already exists.
setlocal EnableExtensions
cd /d "%~dp0.."

set "QUIET=0"
if /I "%~1"=="/quiet" set "QUIET=1"

if exist "runtime\python.exe" (
  if "%QUIET%"=="0" echo [SKIP] runtime\python.exe exists; no data migrate.
  exit /b 0
)

if not exist "runtime\" (
  if "%QUIET%"=="0" echo [SKIP] no runtime folder.
  exit /b 0
)

if not exist "workdir\" mkdir "workdir" >nul 2>&1
if not exist "logs\" mkdir "logs" >nul 2>&1

if "%QUIET%"=="0" echo [MIGRATE] Moving old runtime data to workdir/logs ...

if exist "runtime\startup.log" if not exist "logs\startup.log" move /Y "runtime\startup.log" "logs\startup.log" >nul
if exist "runtime\startup_fault.log" if not exist "logs\startup_fault.log" move /Y "runtime\startup_fault.log" "logs\startup_fault.log" >nul

for %%D in (
  dynamic_monitoring_results
  recognition_results
  module_call_evidence
  pallet_cargo_offset_results
  point_cloud_results
  camera_captures
  test_v8_inputs
  test_v7_images
  ui_qa
  doc_review_20260824
  mysql83-data
  mysql83-uploads
) do (
  if exist "runtime\%%D" if not exist "workdir\%%D" move /Y "runtime\%%D" "workdir\%%D" >nul
)

for %%F in (
  vehicle_loading.db
  vehicle_loading.db-shm
  vehicle_loading.db-wal
  twin_state.json
  overall_results.json
  module_call_summary.json
  loading_cycle_state.json
  mysql83-project.ini
  mysql83-project.err
  mysql83-project.pid
) do (
  if exist "runtime\%%F" if not exist "workdir\%%F" move /Y "runtime\%%F" "workdir\%%F" >nul
)

for %%I in ("runtime\*") do (
  if exist "%%~I" (
    if /I not "%%~nxI"=="python.exe" if /I not "%%~nxI"=="pythonw.exe" (
      if not exist "workdir\%%~nxI" move /Y "%%~I" "workdir\%%~nxI" >nul 2>&1
    )
  )
)

if "%QUIET%"=="0" echo [MIGRATE] done.
exit /b 0
