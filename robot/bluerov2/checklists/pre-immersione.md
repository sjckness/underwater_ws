# Pre-immersione — BlueROV2

Copia questo file come nota della sessione (`AAAA-MM-GG_<luogo>.md`, fuori dal repo o in `logs/`) e spunta.

Data: ____ · Luogo: ____ · Pilota: ____ · Assistente al cavo: ____

## A terra, prima di accendere

- [ ] Copia della SD recente (data in `VERSIONS.md`). Se no: **fermati e falla**.
- [ ] O-ring dei tubi: puliti, ingrassati, senza capelli/sabbia; tappi serrati.
- [ ] Penetratori e cavi: nessun taglio; tether senza nodi né schiacciamenti.
- [ ] Batteria carica (tensione: ____ V), fissata, connettore asciutto.
- [ ] Eliche: libere, nessun gioco, verso corretto, viti strette.
- [ ] Zavorre/galleggianti come nell'ultima configurazione (foto se cambiati).
- [ ] Test del vuoto (vent plug): tenuta ____ min a ____ mbar.

## Acceso, sul banco o in superficie

- [ ] Topside su `192.168.2.1`, `ping 192.168.2.2` ok.
- [ ] BlueOS raggiungibile, versione = `VERSIONS.md` (nessun aggiornamento in sospeso da installare oggi).
- [ ] ArduSub: heartbeat presente, **disarmato**, nessun errore di pre-arm inatteso (vedi Cockpit).
- [ ] Origine EKF impostata (ufficio o luogo di oggi: ORIGIN_LAT/LON, `ardusub/params/README.md`).
- [ ] Sensori: profondità ~0 in aria, assetto plausibile, bussola calibrata qui (se luogo nuovo).
- [ ] Camera e luci ok in Cockpit; joystick riconosciuto.
- [ ] Estensione ROS 2 attiva; `uw smoke rov` passa (heartbeat, frequenza dei topic, CPU della Pi, disarmato).
- [ ] Registrazione: bag leggero avviato (profilo `light` di `ros/src/rov_bringup`), spazio libero sulla Pi
      ____ GB; log `.BIN` di ArduSub attivo.
- [ ] Ora del topside corretta (serve ad allineare `.BIN` e bag).

## In acqua, prima di partire

- [ ] Nessuna bolla dai tubi nei primi minuti.
- [ ] Armare solo con il veicolo in acqua e l'area libera; prova breve di ogni asse.
- [ ] Failsafe noti: perdita del pilota/GCS, batteria, profondità massima ____ m.
