@echo off
rem Firewall rules for the ArduSub SITL <-> Thor MAVLink link. NOT run by Claude: you run it from an ADMINISTRATOR console.
rem
rem     firewall_sitl.bat add       create the rules
rem     firewall_sitl.bat remove    delete them (only the rules named "HIL SITL ...")
rem     firewall_sitl.bat show      read-only: list them (does not need administrator)
rem
rem Scope of every rule: UDP only, local 192.168.200.2 (the direct cable), remote 192.168.200.10 (the Thor), all profiles.
rem
rem   "HIL SITL MAVLink in (UDP)"   Thor -> work.  SITL pushes MAVLink out of the container from an ephemeral source
rem                                 port, and the Thor answers to that port. Windows normally lets such replies in as
rem                                 part of the outgoing UDP flow, but Docker Desktop sends through its own backend
rem                                 process, so the rule is there as a safety net. It is NOT tied to a program or a
rem                                 port (the destination port is ephemeral) and is therefore redundant if
rem                                 "HIL Thor in ingresso (UDP)" from the previous session is still present
rem                                 (same scope): check with "show" first.
rem   "HIL SITL MAVLink out (UDP)"  work -> Thor, remote port 14550 only. Outgoing is allowed by default; this only
rem                                 matters if the machine's policy blocks outgoing by default.
rem
rem The physics link (container -> Isaac/stub on UDP 9002) never touches the cable: it stays on the machine
rem (host.docker.internal), so it needs no rule here. Nothing else is opened: no TCP, no 14550 listener on Windows.

setlocal
set "CMD=%~1"
if /i "%CMD%"=="show" goto :show
if /i "%CMD%"=="add" goto :needadmin
if /i "%CMD%"=="remove" goto :needadmin
echo usage: firewall_sitl.bat add ^| remove ^| show
exit /b 2

:needadmin
net session >nul 2>&1
if errorlevel 1 (
  echo This needs an administrator console. Nothing was changed.
  exit /b 1
)
if /i "%CMD%"=="add" goto :add
goto :remove

:add
netsh advfirewall firewall add rule name="HIL SITL MAVLink in (UDP)" dir=in action=allow protocol=UDP localip=192.168.200.2 remoteip=192.168.200.10 profile=any
netsh advfirewall firewall add rule name="HIL SITL MAVLink out (UDP)" dir=out action=allow protocol=UDP localip=192.168.200.2 remoteip=192.168.200.10 remoteport=14550 profile=any
goto :show

:remove
netsh advfirewall firewall delete rule name="HIL SITL MAVLink in (UDP)"
netsh advfirewall firewall delete rule name="HIL SITL MAVLink out (UDP)"
goto :show

:show
echo.
echo === rules named "HIL ..." ===
netsh advfirewall firewall show rule name=all | findstr /i /c:"Rule Name" /c:"Nome regola" | findstr /i /c:"HIL"
exit /b 0
