@echo off
rem M11 pool on the Work machine: Isaac native on Windows, the physics process inside it on UDP 127.0.0.1:9002.
rem SITL, mediamtx and BlueOS run in WSL2 (setup_blueos_wsl.sh, then docker compose ... --profile blueos up -d);
rem with WSL's mirrored networking their 127.0.0.1 is this machine's, so nothing else needs configuring.
rem Run from anywhere: hil\win\pool\run_pool.bat [--gui] [--view chase] [other uw pool options]
rem Only session variables: nothing system-wide. Log: output\pool\run.log. Stop with Ctrl+C.

setlocal
for %%I in ("%~dp0..\..\..") do set "UW_WS=%%~fI\"
cd /d "%UW_WS%"

set "ISAAC_ENV=F:\envs\isaac"
call "%ISAAC_ENV%\Scripts\activate.bat"
set UW_RUNTIME=native
set "UW_PYTHON=%ISAAC_ENV%\Scripts\python.exe"
set OMNI_KIT_ACCEPT_EULA=YES
set PIP_CONFIG_FILE=nul

if not exist "%UW_WS%output\pool" mkdir "%UW_WS%output\pool"
if exist "%UW_WS%output\pool\run.log" move /y "%UW_WS%output\pool\run.log" "%UW_WS%output\pool\run_prev.log" >nul
echo [run_pool] log: %UW_WS%output\pool\run.log
python -m uw pool --profile workstation --stats-file output\pool\run_stats.json %* > "%UW_WS%output\pool\run.log" 2>&1
set "RC=%ERRORLEVEL%"
echo [run_pool] exit %RC%
endlocal & exit /b %RC%
