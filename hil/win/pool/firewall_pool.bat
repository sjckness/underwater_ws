@echo off
rem M11 pool: inbound rules for the WSL2 stack on the Work machine (run as administrator, once).
rem Needed only for traffic that comes from ANOTHER machine: SITL on linus sending MAVLink to BlueOS, a browser on
rem another PC opening BlueOS/Cockpit. Everything on this machine (http://localhost) needs no rule.
rem In mirrored mode inbound traffic to WSL goes through the Hyper-V firewall, hence New-NetFirewallHyperVRule
rem (VMCreatorId of WSL). Remove: powershell -Command "Remove-NetFirewallHyperVRule -Name 'uw-pool-*'"

set "WSL={40E0AC32-46A5-438A-A0B2-2B479E8F2E90}"
powershell -NoProfile -Command ^
  "New-NetFirewallHyperVRule -Name 'uw-pool-mavlink' -DisplayName 'uw pool: MAVLink from SITL (UDP 14551)' -Direction Inbound -VMCreatorId '%WSL%' -Protocol UDP -LocalPorts 14551 -Action Allow;" ^
  "New-NetFirewallHyperVRule -Name 'uw-pool-blueos-web' -DisplayName 'uw pool: BlueOS web, camera manager, signalling' -Direction Inbound -VMCreatorId '%WSL%' -Protocol TCP -LocalPorts 80,6020,6021 -Action Allow;" ^
  "New-NetFirewallHyperVRule -Name 'uw-pool-webrtc' -DisplayName 'uw pool: WebRTC media and STUN/TURN' -Direction Inbound -VMCreatorId '%WSL%' -Protocol UDP -LocalPorts 3478,49152-65535 -Action Allow;"
if errorlevel 1 (echo [firewall] failed: run this from an administrator prompt & exit /b 1)
echo [firewall] rules added (uw-pool-*)
