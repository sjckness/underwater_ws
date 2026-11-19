@echo off
rem Three headless recording runs in room9 (circle, updown, combined), one after the other, then output\records\report.md.
rem     cmd /c output\hil\record_all.bat                 5 min simulated per run (default)
rem     cmd /c output\hil\record_all.bat 20              20 s per run (a quick check)
rem Run ids: <tag>_<trajectory>, tag = date-time of the start (set RUN_TAG to choose it). Per run: output\records\<id>\run.log
rem and exit_code.txt; a failed run does not stop the next. Exit code of this script: the number of failed runs.
rem Environment: uw.bat's variables only (no ROS: --sink file, no SITL). Nothing system-wide is changed.

setlocal EnableDelayedExpansion
for %%I in ("%~dp0..\..") do set "UW_WS=%%~fI\"
cd /d "%UW_WS%"

set "ISAAC_ENV=F:\envs\isaac"
call "%ISAAC_ENV%\Scripts\activate.bat"
set UW_RUNTIME=native
set "UW_PYTHON=%ISAAC_ENV%\Scripts\python.exe"
set OMNI_KIT_ACCEPT_EULA=YES
set PIP_CONFIG_FILE=nul

set "DURATION=%~1"
if "%DURATION%"=="" set "DURATION=300"
if not defined RUN_TAG for /f %%T in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmm"') do set "RUN_TAG=%%T"
set "ROOT=%UW_WS%output\records"
set FAILED=0
set "IDS="

for %%R in (circle updown combined) do (
  set "ID=%RUN_TAG%_%%R"
  set "DIR=%ROOT%\!ID!"
  if not exist "!DIR!" mkdir "!DIR!"
  echo [record_all] !ID!: %DURATION% s simulated, log !DIR!\run.log
  python tools\record.py --trajectory %%R --duration %DURATION% --run-id !ID! > "!DIR!\run.log" 2>&1
  set "RC=!ERRORLEVEL!"
  > "!DIR!\exit_code.txt" echo !RC!
  echo [record_all] !ID!: exit !RC!
  if not "!RC!"=="0" set /a FAILED+=1
  set "IDS=!IDS! !ID!"
)

python tools\record_report.py --root "%ROOT%" --runs%IDS%
echo [record_all] failed runs: %FAILED%; report: %ROOT%\report.md
endlocal & exit /b %FAILED%
