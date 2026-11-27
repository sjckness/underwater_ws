
@echo off
rem Apre un terminale pronto per underwater_ws su Windows, in modalita' nativa.
rem Metti questo file nella radice di underwater_ws e lancialo con un doppio clic,
rem oppure da CMD:  uw.bat          (apre la shell)
rem                 uw.bat sim --gui --sink none      (lancia e resta aperto)
rem
rem Tutto resta nel profilo utente: nessuna modifica di sistema, rete o firewall.
 
set "ISAAC_ENV=F:\envs\isaac"
set "UW_WS=%~dp0"
 
if not exist "%ISAAC_ENV%\Scripts\python.exe" (
  echo [uw] Ambiente non trovato: %ISAAC_ENV%
  echo [uw] Modifica ISAAC_ENV in cima a questo file.
  pause
  exit /b 1
)
 
call "%ISAAC_ENV%\Scripts\activate.bat"
cd /d "%UW_WS%"
 
rem --- modalita' nativa: il launcher userebbe docker (Linux) ---
set UW_RUNTIME=native
set UW_PYTHON=%ISAAC_ENV%\Scripts\python.exe
 
rem --- Isaac Sim: evita il prompt della licenza a ogni avvio ---
set OMNI_KIT_ACCEPT_EULA=YES
 
rem --- pip: il file globale C:\ProgramData\pip\pip.ini aggiunge un indice non raggiungibile ---
rem     (minuscolo: su Windows pip confronta con 'nul')
set PIP_CONFIG_FILE=nul
 
echo [uw] ambiente   %ISAAC_ENV%
echo [uw] workspace  %UW_WS%
echo [uw] runtime    native
echo.
 
if not "%~1"=="" goto :run
echo [uw] Pronto. Esempi:
echo        python deploy\hil\windows\setup_native.py --write
echo        python -m uw sim --gui --sink none --set lights.ambient_dome=300
echo        python sim\tools\debug_sim.py
echo.
cmd /k
exit /b

:run
python -m uw %*
set "UW_RC=%ERRORLEVEL%"
echo.
echo [uw] uscita: %UW_RC%
cmd /k