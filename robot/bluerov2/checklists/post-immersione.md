# Post-immersione — BlueROV2

Data: ____ · Luogo: ____ · Durata in acqua: ____ min

## Veicolo

- [ ] **Disarmato** prima di toglierlo dall'acqua.
- [ ] Sciacquato in acqua dolce (soprattutto dopo mare); eliche e penetratori controllati.
- [ ] Nessuna goccia d'acqua nei tubi (se sì: foto, nota, non riaccendere prima di asciugare).
- [ ] Batteria scollegata, tensione finale ____ V, messa in carica/stoccaggio.

## Dati

- [ ] Ferma la registrazione del bag (se è ancora attiva).
- [ ] `uw logs pull rov` — scarica `.BIN` e bag nella cartella dei log (fuori da git) e aggiorna
      `robot/bluerov2/logs/INDEX.md`. Controlla che l'ultima riga sia la sessione di oggi.
- [ ] Se un parametro di ArduSub è stato cambiato oggi: export in
      `robot/bluerov2/ardusub/params/AAAA-MM-GG_<motivo>.params` + riga in `VERSIONS.md`.
- [ ] Se BlueOS, ArduSub o l'estensione sono cambiati: riga in `VERSIONS.md`.
- [ ] Nota della sessione (sotto) salvata accanto ai log e riassunta nella colonna *Note* di `logs/INDEX.md`.
- [ ] Commit da linus di indice, parametri e versioni.

## Nota della sessione

- Obiettivo:
- Cosa è stato fatto (manovre, profondità, durata di ogni prova):
- Cosa non ha funzionato:
- Da guardare nei log (ora approssimativa → evento):
- Da sistemare prima della prossima volta:
