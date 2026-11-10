@echo off
rem ArduSub SITL (BlueROV2, "vectored") in Docker, for the Thor HIL. Safe to run from any directory; paths with spaces ok.
rem
rem     run.bat build      build the image uw-ardusub:4.5.7 (log: output\ardusub_sitl\build.log)
rem     run.bat            start the container (same as "up"); detached, name uw-ardusub
rem     run.bat logs       save the container log to output\ardusub_sitl\sitl.log and show its last 15 lines
rem     run.bat stop       save the log, stop and remove the container
rem     run.bat status
rem
rem Overrides (set them in the console before calling, nothing is persisted):
rem     PHYSICS_HOST   where the physics sim listens on UDP 9002   (default host.docker.internal = this machine)
rem     MAVLINK_OUT    ardusub --serial0 device                    (default udpclient:192.168.200.10:14550 = Thor)
rem     HOME_LOC       lat,lon,alt,heading                          (default 45.0703,7.6869,0,0)
rem     SPEEDUP        SITL speedup                                 (default 1)
rem
rem No ports are published, no host networking, no --privileged: SITL only makes outgoing UDP (to the physics sim and
rem to the Thor). Nothing here touches the network configuration or the firewall.

setlocal
set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "WS=%%~fI"
set "OUT=%WS%\output\ardusub_sitl"
set "IMAGE=uw-ardusub:4.5.7"
set "NAME=uw-ardusub"
if not defined PHYSICS_HOST set "PHYSICS_HOST=host.docker.internal"
if not defined MAVLINK_OUT set "MAVLINK_OUT=udpclient:192.168.200.10:14550"
if not exist "%OUT%" mkdir "%OUT%"

set "CMD=%~1"
if "%CMD%"=="" set "CMD=up"
if /i "%CMD%"=="build"  goto :build
if /i "%CMD%"=="up"     goto :up
if /i "%CMD%"=="logs"   goto :logs
if /i "%CMD%"=="stop"   goto :stop
if /i "%CMD%"=="status" goto :status
echo usage: run.bat [build ^| up ^| logs ^| stop ^| status]
exit /b 2

:build
echo [sitl] building %IMAGE%, log: %OUT%\build.log
docker build --progress=plain -t %IMAGE% "%HERE%." > "%OUT%\build.log" 2>&1
echo [sitl] build exit code %ERRORLEVEL%
exit /b %ERRORLEVEL%

:up
docker image inspect %IMAGE% >nul 2>&1
if errorlevel 1 (
  echo [sitl] image %IMAGE% not found: run "run.bat build" first
  exit /b 1
)
docker container inspect %NAME% >nul 2>&1
if not errorlevel 1 (
  echo [sitl] container %NAME% already exists: run "run.bat stop" first
  exit /b 1
)
docker run -d --name %NAME% --restart no --init --cap-drop ALL --security-opt no-new-privileges ^
  -e PHYSICS_HOST -e MAVLINK_OUT -e HOME_LOC -e SPEEDUP ^
  -v "%HERE%params:/params:ro" ^
  %IMAGE% > "%OUT%\container_id.txt" 2>&1
if errorlevel 1 (
  type "%OUT%\container_id.txt"
  exit /b 1
)
echo [sitl] %NAME% started: physics -^> %PHYSICS_HOST%:9002, MAVLink -^> %MAVLINK_OUT%
echo [sitl] logs: run.bat logs      stop: run.bat stop
exit /b 0

:logs
docker logs %NAME% > "%OUT%\sitl.log" 2>&1
echo [sitl] full log: %OUT%\sitl.log
powershell -NoProfile -Command "Get-Content -LiteralPath '%OUT%\sitl.log' -Tail 15"
exit /b 0

:stop
docker logs %NAME% > "%OUT%\sitl.log" 2>&1
docker stop -t 5 %NAME% >nul 2>&1
docker rm %NAME% >nul 2>&1
echo [sitl] stopped and removed %NAME%; log: %OUT%\sitl.log
exit /b 0

:status
docker ps -a --filter name=%NAME% --format "{{.Names}}  {{.Status}}  {{.Image}}"
exit /b 0
