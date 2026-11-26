@echo off
rem M11 pool on the Work machine, scenario B (hil\win\pool\README.md): Isaac native on Windows, the physics process
rem inside it on UDP 127.0.0.1:9002 and the emulated DVL A50 on 127.0.0.1:16171/8061; ArduSub SITL, mediamtx, BlueOS
rem and the DVL extension in WSL2 (stack.sh), whose 127.0.0.1 is this machine's with mirrored networking.
rem
rem Before Isaac starts: brings the WSL stack up and restarts SITL (a fresh physics process needs a fresh EKF,
rem docs/pool.md), then starts in a minimised window BlueOS's configuration, which waits for the vehicle
rem (log: output\pool\blueos_setup.log). UW_POOL_WSL=0 skips all of it (BlueOS elsewhere, scenario A).
rem
rem   hil\win\pool\run_pool.bat [--gui] [--preset turbid] [--view chase] [other uw pool options]
rem
rem Session variables only, each overridable from the calling shell: ISAAC_ENV (F:\envs\isaac), UW_WSL_DISTRO
rem (Ubuntu-24.04), UW_POOL_WSL (1). Isaac's log: output\pool\run.log. Stop with Ctrl+C.

setlocal
for %%I in ("%~dp0..\..\..") do set "UW_WS=%%~fI\"
cd /d "%UW_WS%"

if not defined ISAAC_ENV set "ISAAC_ENV=F:\envs\isaac"
if not defined UW_WSL_DISTRO set "UW_WSL_DISTRO=Ubuntu-24.04"
if not defined UW_POOL_WSL set "UW_POOL_WSL=1"
if not exist "%ISAAC_ENV%\Scripts\python.exe" (echo [run_pool] no Python in %ISAAC_ENV%: set ISAAC_ENV & exit /b 1)

call "%ISAAC_ENV%\Scripts\activate.bat"
set UW_RUNTIME=native
set "UW_PYTHON=%ISAAC_ENV%\Scripts\python.exe"
set OMNI_KIT_ACCEPT_EULA=YES
set PIP_CONFIG_FILE=nul

if not exist "%UW_WS%output\pool" mkdir "%UW_WS%output\pool"

if not "%UW_POOL_WSL%"=="1" goto :isaac
rem "%UW_WS%." and not "%UW_WS%": a trailing backslash before the quote would escape it
echo [run_pool] WSL %UW_WSL_DISTRO%: stack up, fresh SITL
wsl -d %UW_WSL_DISTRO% --cd "%UW_WS%." -- bash hil/win/pool/stack.sh up
if errorlevel 1 goto :wslfail
wsl -d %UW_WSL_DISTRO% --cd "%UW_WS%." -- bash hil/win/pool/stack.sh restart-sitl
if errorlevel 1 goto :wslfail
echo [run_pool] BlueOS and DVL setup in the background, once the vehicle is up: output\pool\blueos_setup.log
start "uw pool: BlueOS setup" /min wsl -d %UW_WSL_DISTRO% --cd "%UW_WS%." -- bash hil/win/pool/stack.sh configure

:isaac
if exist "%UW_WS%output\pool\run.log" move /y "%UW_WS%output\pool\run.log" "%UW_WS%output\pool\run_prev.log" >nul
echo [run_pool] Isaac: log in %UW_WS%output\pool\run.log (the first start compiles shaders for minutes)
python -m uw pool --profile workstation --stats-file output\pool\run_stats.json %* > "%UW_WS%output\pool\run.log" 2>&1
set "RC=%ERRORLEVEL%"
echo [run_pool] exit %RC%
endlocal & exit /b %RC%

:wslfail
echo [run_pool] the WSL stack did not come up: wsl -d %UW_WSL_DISTRO% -- bash hil/win/pool/stack.sh status
endlocal & exit /b 1
