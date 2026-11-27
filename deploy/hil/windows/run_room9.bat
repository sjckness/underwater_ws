@echo off
rem room9 + ArduSub physics on UDP 127.0.0.1:9002 (vehicles/common/physics_udp.py), ROS sink on the HIL domain.
rem Run from anywhere: cmd /c output\hil\run_room9.bat
rem Before: stop sim\tools\stub_physics.py (port 9002). After Isaac prints "physics: listening": deploy\docker\ardusub_sitl\run.bat
rem Only session variables: nothing system-wide, no network or firewall change.
rem Log: output\hil\run.log (the previous one is kept as run_prev.log). Close with Ctrl+C or Invio at the end.

setlocal
for %%I in ("%~dp0..\..\..") do set "UW_WS=%%~fI\"
cd /d "%UW_WS%"

rem --- uw.bat: native mode ---
set "ISAAC_ENV=F:\envs\isaac"
call "%ISAAC_ENV%\Scripts\activate.bat"
set UW_RUNTIME=native
set "UW_PYTHON=%ISAAC_ENV%\Scripts\python.exe"
set OMNI_KIT_ACCEPT_EULA=YES
set PIP_CONFIG_FILE=nul

rem --- hil_env.bat, same order: ROS_DISTRO, ROS_DOMAIN_ID, RMW_IMPLEMENTATION, FASTRTPS_DEFAULT_PROFILES_FILE, PATH ---
call "%UW_WS%output\hil\hil_env.bat"
if errorlevel 1 exit /b 1

if exist "%UW_WS%output\hil\run.log" move /y "%UW_WS%output\hil\run.log" "%UW_WS%output\hil\run_prev.log" >nul

rem sim\tools\debug_sim.py instead of "python -m uw sim": same run, plus the uw.* INFO lines (physics: listening ...) in the log
python sim\tools\debug_sim.py --gui --sink ros --preset room9 ^
  --set sonar.pose_source=ardusub --set sonar.static_cache=on_motion ^
  --set sim.physics_port=9002 --set sim.physics_rtf=1.0 ^
  --set sim.gui_fps=10 --set lights.ambient_dome=300 ^
  > "%UW_WS%output\hil\run.log" 2>&1
echo [run_room9] exit %ERRORLEVEL%, log: %UW_WS%output\hil\run.log
endlocal
