@echo off
rem Ambiente ROS 2 per l'HIL con il Thor. Da lanciare nel terminale aperto da uw.bat, dalla radice di underwater_ws:
rem     call output\hil\hil_env.bat
rem Solo variabili di questa sessione: nessuna modifica di sistema, rete o firewall.

set "ISAAC_ENV=F:\envs\isaac"
set "BRIDGE=%ISAAC_ENV%\Lib\site-packages\isaacsim\exts\isaacsim.ros2.bridge"
set "HIL_DIR=%~dp0"

rem Il bridge di Isaac 5.1 su Windows ha le librerie interne di una sola distribuzione (di norma humble).
rem uw/ros_bridge.py usa jazzy come predefinito: qui lo impostiamo in base a cio' che esiste davvero.
set "ROS_DISTRO="
if exist "%BRIDGE%\humble\lib" set "ROS_DISTRO=humble"
if not defined ROS_DISTRO if exist "%BRIDGE%\jazzy\lib" set "ROS_DISTRO=jazzy"
if not defined ROS_DISTRO goto :nodistro

rem Dominio 73 = convenzione del repo (mai quello di F1Tenth, che sul Thor e' 0). Deve coincidere con hil_env.sh.
set ROS_DOMAIN_ID=73
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set "FASTRTPS_DEFAULT_PROFILES_FILE=%HIL_DIR%fastdds_win.xml"
set "PATH=%PATH%;%BRIDGE%\%ROS_DISTRO%\lib"

echo [hil] ROS_DISTRO=%ROS_DISTRO%  ROS_DOMAIN_ID=%ROS_DOMAIN_ID%  RMW=%RMW_IMPLEMENTATION%
echo [hil] profilo DDS: %FASTRTPS_DEFAULT_PROFILES_FILE%
echo [hil] lib bridge:  %BRIDGE%\%ROS_DISTRO%\lib
exit /b 0

:nodistro
echo [hil] Nessuna distribuzione ROS 2 trovata in %BRIDGE%
echo [hil] Controlla il percorso con:  dir "%BRIDGE%"
exit /b 1
