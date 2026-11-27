@echo off
rem La vasca prove sulla Work con un comando (M12): controlli, installazione la prima volta, modelli 3-D, Cockpit,
rem poi deploy\hil\windows\pool\run_pool.bat. Doppio clic, oppure da CMD:
rem   pool.bat                  headless (il video va in Cockpit)
rem   pool.bat --gui            con la finestra di Isaac
rem   pool.bat --check          dice solo cosa manca, non cambia niente
rem   pool.bat --preset turbid  ogni altra opzione va a `uw pool`
rem Guida: deploy\hil\windows\pool\README.md. ISAAC_ENV (F:\envs\isaac) e UW_WSL_DISTRO (Ubuntu-24.04) si cambiano
rem dalla shell che lo lancia.
setlocal
if not defined ISAAC_ENV set "ISAAC_ENV=F:\envs\isaac"
if not exist "%ISAAC_ENV%\Scripts\python.exe" (
  echo [pool] nessun Python in %ISAAC_ENV%: imposta ISAAC_ENV ^(l'ambiente Isaac nativo, deploy\hil\windows\README.md^)
  pause
  exit /b 1
)
"%ISAAC_ENV%\Scripts\python.exe" "%~dp0deploy\hil\windows\pool\pool_up.py" %*
set "RC=%ERRORLEVEL%"
if not "%RC%"=="0" pause
endlocal & exit /b %RC%
