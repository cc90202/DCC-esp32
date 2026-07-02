# WiFi Provisioning & Pannello Web Permanente — Design

**Data:** 2026-05-12
**Branch corrente alla scrittura:** `feat/railcom` (il design è separato dal lavoro RailCom; verrà implementato su branch dedicato)
**Status:** Approved

---

## 0. Sommario esecutivo

Questo documento descrive il design di un sottosistema che sostituisce l'attuale configurazione WiFi via file `.env` (compilata nel firmware) con un meccanismo di **provisioning runtime** e di **amministrazione web permanente**. È il primo passo per trasformare il device da prototipo configurato in fabbrica a prodotto commerciale configurabile dall'utente finale.

**Cosa cambia per l'utente.** Al primo boot (o dopo un factory reset), il device accende un hotspot WiFi temporaneo e mostra una pagina di configurazione attraverso un portale captive. L'utente inserisce nome device, password amministratore, e credenziali della rete WiFi a cui far collegare il device. Da quel momento, il device si avvia direttamente in modalità operativa, esponendo un pannello web amministrativo sulla rete di casa per cambiare configurazione, riconfigurare la WiFi, fare factory reset, e (in futuro) caricare nuovo firmware.

**Cosa cambia per il firmware.** Vengono introdotti quattro nuovi moduli (`provisioning/`, `web_panel/`, `storage/`, `reset_button/`) e viene modificato `net/udp_control.rs` per leggere le credenziali da memoria flash invece che da `env!()`. Il layout della memoria flash riserva spazio per configurazione utente e per una futura strategia OTA, ma il dual-slot OTA non viene considerato valido finché non è dimostrato con lo stack `esp-hal`/`no_std` del progetto.

**Scope:** Livello 1 (configurazione di rete e identità). **Fuori scope:** dashboard operativa con stato locomotive, upload firmware (predisposto ma non implementato), programmazione CV via web, mDNS.

**Piano figlio iniziale:** `docs/superpowers/plans/2026-05-12-wifi-setup-softap-spike.md` — spike minimo per validare SoftAP + HTTP su `esp-hal` prima del resto del provisioning.

---

## 1. Visione architetturale

### 1.1 Cosa stiamo costruendo

Un sottosistema integrato nel firmware esistente che permetta all'utente di:

1. Configurare alla prima accensione le credenziali WiFi e la password amministratore tramite un'interfaccia web minimale, senza dover ricompilare nulla.
2. Riconfigurare in seguito le stesse credenziali da un pannello web sempre disponibile sulla rete WiFi del device.
3. Effettuare un factory reset hardware (pressione prolungata di un pulsante fisico) o software (endpoint web autenticato).
4. (In futuro) caricare aggiornamenti firmware via web senza accesso fisico al device.

### 1.2 I tre stati operativi del device

In ogni momento il device si trova in uno di tre stati di rete mutuamente esclusivi:

- **Provisioning** — Il device è un hotspot WiFi. Espone una pagina di configurazione tramite portale captive. La logica DCC è ferma: H-bridge disabilitato, idle packets sospesi, motori non alimentati. Stato deliberatamente "tutto fermo" perché in fase di setup non si sta erogando trazione.

- **Operativo Connesso** — Il device è collegato a una rete WiFi esistente. Logica DCC piena e attiva. Pannello web amministrativo raggiungibile via IP. La Z21 app trova la centralina via UDP broadcast (porta 21105) come fa già oggi.

- **Operativo Offline (degraded)** — Il device era connesso ma ha perso la rete (router spento, fuori portata, hotspot caduto). DCC continua a girare normalmente: le locomotive non si fermano. Il pannello web e la Z21 app sono irraggiungibili finché la rete non torna. Tentativi di riconnessione automatici in background con backoff esponenziale.

### 1.3 Componenti software introdotti

| Modulo | Responsabilità | Nuovo / Modificato |
|---|---|---|
| `src/provisioning/` | Macchina a stati di rete, hotspot, portale captive | Nuovo |
| `src/web_panel/` | Web server HTTP, autenticazione, endpoint amministrativi | Nuovo |
| `src/storage/` | Wrapper sopra la memoria flash chiave-valore per la config persistente | Nuovo |
| `src/reset_button/` | Driver del pulsante GPIO0 con logica multi-livello di pressione | Nuovo |
| `src/net/udp_control.rs` | Lettura credenziali a runtime invece che da `env!()` | Modificato |
| `src/system_status.rs` | Tre nuovi stati: `WifiConnecting`, `Provisioning`, `OfflineDegraded` | Modificato |
| `src/boot.rs` | Riorganizzazione della sequenza di init per inserire i nuovi task | Modificato |

### 1.4 Decisioni chiave già prese (riassunto)

| Decisione | Scelta | Motivazione |
|---|---|---|
| Modalità di rete | Solo client (joins existing) | Caso d'uso: device si sposta tra reti diverse (casa, club, fiera, hotspot) |
| Persistenza credenziali | Storage persistente astratto | Dettagli flash/record rimandati al piano implementativo |
| Trigger provisioning | Automatico (config assente o 5 fallimenti) + tasto GPIO0 hold 5s | Combinazione standard prodotti commerciali |
| Reti ricordate | Una sola (l'ultima configurata) | Scelta minimal; multi-rete rimandata se servirà |
| Pannello web | Permanente (sempre attivo in stato Operativo) | Necessario per cambiare WiFi senza factory reset |
| Scope pannello | Livello 1: stato base + cambio rete + cambio nome | Anti-scope-creep, OTA esplicitamente predisposto ma non implementato |
| Autenticazione | Login form + cookie di sessione (`HttpOnly`, `SameSite=Strict`) | Esperienza più "prodotto", costo accettabile |
| Sicurezza provisioning | SoftAP WPA2 con PIN per-device mostrato su OLED | Evita invio di password admin/WiFi su rete aperta |
| Hardware setup | Tasto su GPIO0 con condensatore 100 nF + pull-up interno | Stesso pattern dei tasti stop/resume esistenti |
| Discovery sulla rete | IP visualizzato sull'OLED del device | Semplicità; mDNS rimandato |
| Lingua pannello | Italiano (con variabile per estensione futura) | Mercato di partenza italiano |
| Display OLED | Mostra stato + IP in operativo, nome hotspot + IP fisso in provisioning | Riusa display esistente, niente nuovo hardware |
| Web server library | `picoserve` (Embassy-native, supporta streaming upload) | Predisposto per OTA |
| HTTPS | No nel Livello 1 | LAN domestica con WPA2/WPA3, costo certificati troppo alto per il valore |
| Layout flash | Zona config + spazio firmware/OTA riservato | Decisione da validare con `esp-hal`, non con assunzioni ESP-IDF |
| OTA security | Firmware OTA solo firmato crittograficamente | Requisito architetturale, anche se upload non è nel Livello 1 |

---

## 2. Macchina a stati di rete

### 2.1 Gli stati

| Stato | Tipo | Cosa è acceso | DCC attivo? |
|---|---|---|---|
| `Boot` | Transiente | Solo init hardware | No |
| `WifiConnecting` | Transiente | Tentativi collegamento iniziali | No |
| `Provisioning` | Stabile | Hotspot + DHCP + DNS + Web server limitato | No (H-bridge OFF) |
| `OperationalConnected` | Stabile | WiFi client + Web server completo + Z21 + DCC | Sì |
| `OperationalOffline` | Stabile | DCC + Z21 (irraggiungibile) + tentativi riconnessione | Sì |
| `FactoryResetting` | Transiente | Cancellazione configurazione persistente in corso | No |

### 2.2 Tabella delle transizioni

| Da → A | Trigger |
|---|---|
| `Boot` → `WifiConnecting` | Init completo **e** configurazione persistente valida |
| `Boot` → `Provisioning` | Init completo **e** configurazione assente |
| `WifiConnecting` → `OperationalConnected` | Collegamento WiFi riuscito entro 5 tentativi |
| `WifiConnecting` → `Provisioning` | 5 fallimenti consecutivi (credenziali obsolete) |
| `OperationalConnected` → `OperationalOffline` | Evento `WifiDisconnected` da `esp-radio` |
| `OperationalOffline` → `OperationalConnected` | Riconnessione in background riuscita |
| **Qualunque stato** → `Provisioning` | GPIO0 premuto 5 secondi continui |
| **Qualunque stato** → `FactoryResetting` | GPIO0 premuto 10 secondi continui **oppure** `POST /api/factory-reset` autenticato + password riconfermata |
| Pannello web → `Provisioning` | `POST /api/reprovision` autenticato + password |
| `Provisioning` → `WifiConnecting` | Utente conferma form portale captive con credenziali valide |
| `FactoryResetting` → `Boot` | Riavvio software completato |

### 2.3 Policy non ovvie (esplicite)

**P1. Da `OperationalOffline` non si va automaticamente in `Provisioning`.**
Indipendentemente da quanto resta giù la WiFi, il device continua a fare DCC e tenta riconnessioni indefinitamente con backoff esponenziale (5s, 10s, 20s, 40s, max 60s). La transizione a `Provisioning` è solo esplicita (tasto fisico o richiesta web). Motivazione: un blackout temporaneo della rete domestica non deve fermare le locomotive sui binari né scatenare un hotspot inatteso.

**P2. L'entrata in `Provisioning` da uno stato operativo ferma DCC in modo ordinato.**
Quando il tasto GPIO0 forza il provisioning, il `provisioning::supervisor` chiama prima una procedura di shutdown ordinato: invia un e-stop packet alle locomotive attive (le ferma in modo controllato), disabilita l'H-bridge, **poi** avvia l'hotspot. Non è uno spegnimento brutale.

**P3. Durante il primo provisioning post-boot (config assente), DCC non parte mai.**
Il device esce dalla fabbrica senza configurazione utente → prima accensione → entra subito in `Provisioning` senza aver mai energizzato i binari. È solo dopo la conferma dell'utente che si passa a `WifiConnecting` → `OperationalConnected` e DCC parte per la prima volta.

### 2.4 Concorrenza

La macchina a stati vive in **un singolo Embassy task** (`provisioning_supervisor_task`) che ne è proprietario esclusivo. Comunica con il resto del sistema via:

- **In ingresso**: `embassy_sync::Channel<ProvisioningEvent, 16>` per eventi (button events, WiFi events, richieste dal web server).
- **In uscita**: eventi al `StatusModel` esistente, comandi a `TrackOutput` per shutdown ordinato DCC.

Single-threaded logicamente. Nessun lock, nessuna race condition.

### 2.5 Ownership dello stack di rete

Il firmware attuale concentra WiFi, `embassy-net`, connection task e UDP Z21 dentro `src/net/udp_control.rs`. Con il pannello web permanente questa responsabilità viene separata:

- `src/net/runtime.rs` diventa il proprietario esclusivo di `esp_radio`, `WifiController`, interfacce STA/AP, `embassy_net::Stack`, runner e transizioni Client/SoftAP.
- `src/net/udp_control.rs` diventa un servizio Z21 sopra uno `Stack` già avviato: apre solo la socket UDP 21105 e traduce pacchetti Z21 in comandi DCC.
- `src/web_panel/server.rs` diventa un servizio HTTP sopra lo stesso `Stack`, senza inizializzare radio o DHCP.
- `src/provisioning/supervisor.rs` decide lo stato operativo e invia comandi al runtime di rete (`StartProvisioningAp`, `StartClient`, `Reconnect`, `StopNetworkForReboot`), ma non possiede direttamente il controller WiFi.

Regola di progetto: **un solo modulo possiede il driver radio e lo stack IP**. Tutto il resto riceve handle, eventi o comandi bounded via channel. Questo evita doppi runner, doppie configurazioni WiFi e ownership ambigua tra Z21, pannello web e provisioning.

---

## 3. Layout della memoria flash

### 3.1 Premessa

L'ESP32-C6 di riferimento è `ESP32-C6-WROOM-1-N8` con **8 MB di flash**. **Da verificare** che il modulo in uso sia effettivamente la variante N8 (lettura con `espflash board-info`). Se il modulo dovesse essere a 4 MB, gli slot firmware si riducono a 1.8 MB ciascuno, sufficiente per la dimensione attuale del firmware (~500 KB) con ampio margine, ma stretti per uno scenario di crescita futuro significativo.

Il progetto usa `esp-hal`/`no_std`, non ESP-IDF. Quindi il piano non assume semantiche ESP-IDF per partition table, `otadata`, boot-and-rollback o selezione automatica dello slot. Nel Livello 1 il requisito concreto è avere una zona persistente per configurazione utente e non bloccare una futura OTA firmata. Il layout dual-slot diventa una **decisione gated**: si abilita solo dopo uno spike hardware che dimostra boot, switch slot e rollback con lo stack effettivamente usato dal progetto.

### 3.2 Layout proposto per il Livello 1

| Area | Offset | Dimensione | Cosa contiene | Stato |
|---|---|---|---|---|
| Boot/app corrente | Da layout `esp-hal`/`espflash` corrente | Da confermare con build reale | Firmware Livello 1 | In scope |
| Config persistente | Da scegliere dopo verifica flash map | Da stimare | Credenziali WiFi, password admin hash, nome device | In scope come capability, non come formato |
| OTA reserved | Da scegliere dopo spike | Resto flash disponibile | Spazio riservato per futura immagine firmata o storage firmware | Riservato, non usato nel Livello 1 |
| Storage esteso | Da scegliere dopo spike | Opzionale | Backup/log/asset futuri | Fuori scope |

Il piano di implementazione deve produrre una flash map concreta per `espflash` e validarla su hardware. Non si introducono nomi `partition-table`, `otadata`, `ota_0` o `ota_1` come requisiti finché non esiste una prova funzionante in `esp-hal`.

### 3.2.1 Spike obbligatorio prima del dual-slot OTA

Prima di approvare qualsiasi implementazione OTA dual-slot:

1. Build firmware `esp-hal` con layout flash esplicito.
2. Flash via `espflash` su slot/app corrente e verifica boot.
3. Scrittura controllata di una seconda immagine in area riservata.
4. Meccanismo verificato per selezionare quale immagine avviare, senza appoggiarsi a semantiche IDF non presenti.
5. Rollback verificato dopo boot fallito o watchdog.
6. Test su modulo reale N8 e, se supportato, variante 4 MB.

Se uno di questi punti fallisce, il Livello 1 resta single-app con config persistente e spazio riservato; OTA viene ripianificata come fase separata.

### 3.3 Persistenza configurazione (rimandata)

Il Livello 1 richiede una capability: caricare, salvare e cancellare credenziali WiFi, password admin hash e nome device. Il formato flash, la strategia erase/write, l'eventuale uso della partizione esistente e la recovery da reset durante scrittura sono dettagli del piano implementativo.

Regola per ora sufficiente: il dominio non conosce il backend. Il resto del firmware vede solo `ConfigRepository` con operazioni ad alto livello (`load`, `save_after_verified_wifi`, `wipe_all`). Niente dettagli NVS/record/checksum in questo design.

### 3.4 Password admin: hash con bcrypt

La password admin **non viene mai salvata in chiaro**. Quando l'utente la sceglie:

1. Il firmware genera un salt casuale di 16 byte usando il generatore hardware ESP32-C6.
2. Calcola `hash = bcrypt(password, salt, cost=8)`. Cost factor 8 è scelto per impiegare ~50ms sull'hardware: abbastanza da rendere impraticabile la forza bruta, abbastanza veloce da non rallentare il login utente. **Il cost factor andrà tarato empiricamente** durante l'implementazione.
3. Salva `salt` e `hash` nello storage persistente.

In login: lo stesso calcolo viene ripetuto sulla password ricevuta e confrontato con l'hash salvato.

Conseguenza: anche un attaccante che smonta fisicamente il device e legge la flash non può ricavare la password — solo provare a indovinarla, e ogni tentativo costa 50 ms di CPU dedicata.

### 3.5 Password WiFi: in chiaro

A differenza della password admin, la password della rete WiFi deve essere conservata in chiaro perché il firmware deve passarla al collegamento WiFi (non è autenticabile via hash). È lo standard di tutti i device WiFi del mondo. Per scenari futuri ad alta sensibilità si valuterà una protezione flash compatibile con `esp-hal` e con il flusso di provisioning scelto. Non in scope per il Livello 1.

### 3.6 Migrazione dal codice attuale

Il firmware attuale gira con un layout single-app senza predisposizione OTA. La transizione richiede:

1. **Riflash una tantum via USB** con il nuovo firmware e la nuova flash map `esp-hal` validata.
2. Le credenziali WiFi che erano in `.env` vengono **perse** in questo passaggio (la configurazione persistente parte vuota). Atteso e voluto.
3. Primo boot post-migrazione: device entra in `Provisioning`, utente completa la configurazione dal portale captive.
4. Dopo la prima programmazione del nuovo firmware, `.env` non serve più. `WIFI_SSID`/`WIFI_PASS` vengono rimossi da `udp_control.rs` e dalle eventuali variabili `.cargo/config.toml`.

---

## 4. Hardware

### 4.1 Componenti aggiunti

Una sola modifica hardware:

- **1 microswitch tattile** (push-button momentaneo, normalmente aperto), stesso modello dei tasti stop/resume esistenti.
- **1 condensatore ceramico 100 nF**, stesso modello degli esistenti.
- **2 jumper wire**.

Nessun resistore di pull-up esterno (si usa il pull-up interno del chip ESP32-C6).

### 4.2 Schema elettrico

- Un terminale del microswitch → **GPIO0** del modulo ESP32-C6.
- Altro terminale → **GND**.
- Condensatore 100 nF in parallelo al microswitch (debounce hardware).

Logica:
- Tasto rilasciato → GPIO0 alto (pull-up interno attivo).
- Tasto premuto → GPIO0 basso (cortocircuito a massa).

### 4.3 Considerazioni di pin assignment

GPIO0 sull'ESP32-C6 è:
- **Non strapping pin** (a differenza dell'ESP32 classico).
- **LP_IO/RTC-capable** — apre la strada a wake-up da deep sleep in futuro.
- General-purpose, nessuna funzione hardware riservata.

Compatibile con futura architettura a booster CAN: il GPIO matrix dell'ESP32-C6 permette di mappare TWAI TX/RX su qualsiasi pin, quindi la scelta di GPIO0 per il tasto setup non vincola la futura aggiunta del CAN bus.

### 4.4 Considerazione meccanica per il prodotto

Per la fase di prodotto commerciale finale, il tasto GPIO0 dovrà essere **incassato sulla scocca** (premibile solo con penna/graffetta), stile pulsante reset dei router consumer. Rende deliberatamente fastidiosa la pressione accidentale. Decisione meccanica, non firmware.

### 4.5 Modifica al boot

In `src/boot.rs`, dopo l'inizializzazione degli altri pulsanti, va aggiunto:

```rust
let setup_btn = new_button_input(peripherals.GPIO0);
spawner.spawn(setup_button_task(setup_btn, provisioning_event_sender)).unwrap();
```

(Pseudocodice. La firma esatta verrà definita nel piano di implementazione.)

---

## 5. Flusso di provisioning

### 5.1 I quattro mini-servizi attivi durante `Provisioning`

Quattro task Embassy che girano contemporaneamente dentro il modulo `provisioning/`:

1. **Hotspot WiFi (modalità access point)** — Nome rete: `DCC-Setup-XXXX` (XXXX = ultime 4 cifre hex del MAC). Rete protetta WPA2-Personal con PIN temporaneo per-device visualizzato sull'OLED (`PIN: 12345678`). Il PIN viene generato dal RNG hardware a ogni ingresso in provisioning, non viene salvato in modo persistente, e scade quando il device esce da `Provisioning`. Se il display non è disponibile, il provisioning resta bloccato e mostra errore via LED: non si apre mai una rete non cifrata che riceve password admin/WiFi.

2. **Server DHCP interno** — Assegna IP nell'intervallo `192.168.4.2-192.168.4.10` ai client che si collegano. Device stesso ha IP fisso `192.168.4.1`, convenzione comune negli esempi ESP SoftAP ma implementata qui con `esp-radio`/`embassy-net`, non con IDF.

3. **Server DNS "trasversale"** — Risponde con `192.168.4.1` a **qualsiasi** query DNS. Il telefono che si collega all'hotspot, quando il suo OS prova a contattare `captive.apple.com` / `connectivitycheck.gstatic.com` / `www.msftconnecttest.com` per testare l'accesso internet, riceve come risposta l'IP del device, e il sistema operativo riconosce la situazione come "portale captive" aprendo automaticamente la pagina di configurazione in vista modale.

4. **Server HTTP** — Risponde sulla porta 80. Endpoint:

| Metodo | Path | Funzione |
|---|---|---|
| `GET` | `/` (e qualsiasi altro path) | Serve la pagina HTML di configurazione |
| `GET` | `/api/scan` | JSON con la lista delle reti WiFi visibili |
| `POST` | `/api/configure` | Riceve i dati del form, valida, prova collegamento, salva |
| `GET` | `/api/provisioning-status` | JSON con lo stato del tentativo di collegamento (per polling del client). **Endpoint specifico del provisioning, distinto da `/api/status` del pannello operativo descritto nella Sezione 6.** |

### 5.2 La pagina di configurazione

Una sola pagina HTML, minimale, no librerie esterne. Quattro campi:

1. **Nome del device** — pre-compilato con `DCC-XXXX`. Vincoli: max 32 char, alfanumerico + trattino.
2. **Password amministratore** — campo password + campo conferma. Vincoli: 8-64 caratteri.
3. **Rete WiFi (SSID)** — campo testo + bottone "Scansiona reti" che popola un menu a tendina via `/api/scan`.
4. **Password della rete WiFi** — campo password. Vincoli: 0 (rete aperta) o 8-64 caratteri (WPA2). Checkbox "rete aperta" sopra che nasconde il campo password.

Unico bottone: **Salva e collega**.

### 5.3 Flusso del salvataggio

Disegnato per non salvare nulla finché non sappiamo che le credenziali WiFi funzionano:

1. **Validazione lato server**: vincoli di lunghezza/caratteri/match password. Se fallisce → pagina riproposta con messaggio inline.
2. **Test del collegamento WiFi**: il firmware usa le credenziali ricevute come configurazione temporanea, riavvia se necessario per resettare radio e stack IP tra SoftAP e Client, poi tenta il collegamento con timeout di 15 secondi e massimo 5 tentativi.
3. **Esito positivo**: solo dopo la verifica salva la configurazione persistente, entra in `OperationalConnected`, aggiorna OLED con il nuovo IP.
4. **Esito negativo**: non salva nulla, rientra in `Provisioning` con nuovo PIN SoftAP e mostra errore sull'OLED.

La forma concreta del salvataggio persistente è fuori da questo design. Il requisito funzionale è semplice: non rendere attiva una configurazione WiFi mai verificata.

### 5.4 Out of scope nel Livello 1

- Configurazione IP statica
- Wizard multi-step
- Recupero password (utente fa factory reset col GPIO0)
- Configurazione fuso orario/locale

---

## 6. Pannello web permanente

### 6.1 Tre pagine HTML

#### `/login` (pubblica)
Form con un solo campo password + bottone "Accedi". Messaggio d'errore inline ("Password errata" o "Troppi tentativi falliti, riprova tra X minuti").

#### `/` (home, autenticato)
Dashboard di sola lettura. Mostra:
- Nome device (es. `DCC-Soggiorno`)
- Versione firmware (da `env!("CARGO_PKG_VERSION")`)
- Stato operativo corrente (dal `StatusModel` esistente)
- Nome rete WiFi
- IP sulla rete
- Forza segnale WiFi (es. "Eccellente (-45 dBm)")
- Uptime
- Numero locomotive in slot (dallo `SlotManager` esistente)

Link a `/system` e `/api/logout`.

#### `/system` (autenticato)
Quattro blocchi:
- **Identità**: cambio nome device.
- **Sicurezza**: cambio password admin (richiede password attuale), logout.
- **Rete WiFi**: visualizzazione rete attuale, bottone "Cambia rete..." → mini-form inline.
- **Avanzate (zona pericolosa)**: bottone "Riconfigura da zero" (cancella solo credenziali WiFi), bottone "Factory reset" (cancella tutto). Entrambi con doppia conferma.

### 6.2 Mappa completa degli endpoint

**Endpoint pubblici:**

| Metodo | Path | Funzione | Note sicurezza |
|---|---|---|---|
| `GET` | `/login` | Pagina login (redirect a `/` se già autenticato) | — |
| `POST` | `/api/login` | Verifica password, crea sessione | Rate limit: 5 tentativi/IP/5min |

**Endpoint autenticati:**

| Metodo | Path | Funzione | Password nel body? |
|---|---|---|---|
| `GET` | `/` | Dashboard | No |
| `GET` | `/system` | Pagina sistema | No |
| `GET` | `/api/status` | JSON info device | No |
| `POST` | `/api/logout` | Invalida sessione | No |
| `POST` | `/api/system/name` | Cambia nome device | No |
| `POST` | `/api/system/password` | Cambia password admin | **Sì** (vecchia + nuova) |
| `POST` | `/api/wifi/change` | Cambia credenziali WiFi e prova | **Sì** |
| `POST` | `/api/reprovision` | Cancella solo WiFi, va in provisioning | **Sì** |
| `POST` | `/api/factory-reset` | Cancella tutto, va in provisioning | **Sì** |
| `GET` | `/api/scan` | Lista reti visibili | No |

**Endpoint riservati per futuro OTA (rispondono `501` nel Livello 1):**

| Metodo | Path | Funzione futura |
|---|---|---|
| `POST` | `/api/firmware/upload` | Upload firmware in streaming |
| `POST` | `/api/firmware/commit` | Marca nuovo slot attivo, riavvia |
| `POST` | `/api/firmware/rollback` | Torna allo slot precedente |
| `GET` | `/api/firmware/info` | Versione, hash, slot attivo |

### 6.3 Modello di sessione

Su login riuscito:
1. Server genera token 32 byte (256 bit di entropia, dal generatore hardware ESP32-C6).
2. Salva `Session { token, last_activity, source_ip }` in tabella in RAM (max 4 sessioni concorrenti).
3. Risponde con cookie `dcc_session=<base64>; HttpOnly; SameSite=Strict; Path=/`.

Per ogni richiesta successiva, il middleware:
- Estrae token dal cookie.
- Cerca la sessione corrispondente.
- Verifica `last_activity` non più vecchio di **30 minuti** (timeout di inattività).
- Aggiorna `last_activity` al timestamp corrente.
- Lascia passare la richiesta o ritorna `401`.

Logout: `POST /api/logout` rimuove la sessione dalla tabella + risponde con cookie scaduto.

Niente flag `Secure` perché non serviamo HTTPS nel Livello 1.

### 6.4 Protezione anti-CSRF

Tutti gli endpoint `POST` autenticati richiedono:

1. Cookie di sessione valido.
2. Token CSRF per-sessione generato al login e inserito nei form o nell'header `X-DCC-CSRF`.

Gli endpoint marcati "password nel body" applicano un terzo controllo: password admin riconfermata nel body JSON. La password è richiesta per cambio password, cambio WiFi, reprovision, factory reset e, in futuro, operazioni OTA.

Esempio:
```
POST /api/factory-reset
Cookie: dcc_session=abc123...
X-DCC-CSRF: def456...
Content-Type: application/json

{ "admin_password": "lamiapassword" }
```

Difende dal caso in cui un sito web malevolo, visitato dall'utente in un'altra tab, tenta richieste verso il device sfruttando i cookie già attivi. Il sito malevolo non conosce il token CSRF, e per le operazioni sensibili non conosce la password admin.

### 6.5 Rate limiting

Tabella in RAM con max 16 IP tracciati: `{ ip, failed_attempts, last_attempt_ts }`. 5 tentativi falliti consecutivi → blocco IP per 5 minuti. Dopo 5 minuti il contatore si resetta.

### 6.6 Flusso del cambio WiFi via pannello

Caso particolare con interazione di rete complessa.

1. Utente compila form `Cambia rete...` e clicca "Cambia".
2. Server valida cookie, token CSRF e password admin, poi avvia un cambio rete controllato senza sovrascrivere subito la configurazione attiva.
3. Server risponde subito con pagina "Cambio rete in corso. Il device si riavvia; leggi il nuovo IP sull'OLED o attendi il ritorno sulla rete precedente."
4. Firmware ferma DCC in modo ordinato, spegne l'H-bridge, esegue shutdown del runtime di rete e riavvia il device. Il reboot è breve e viene annunciato all'utente come operazione amministrativa.
5. Al boot successivo il runtime tenta la nuova rete con timeout 15 secondi e massimo 5 tentativi.
6. **Successo**: salva la nuova configurazione come attiva, OLED mostra nuovo nome rete e nuovo IP.
7. **Fallimento**: ripristina la configurazione precedente e torna su `OperationalConnected` con la rete vecchia. L'OLED mostra "Cambio rete fallito".

Questa scelta evita di dipendere da AP+STA simultaneo e da transizioni live del driver radio. AP+STA potrà essere ottimizzato in futuro solo dopo test hardware esplicito; non è un requisito del Livello 1.

### 6.7 Web server library

**Scelta:** `picoserve` (Embassy-native, supporta streaming upload — predisposizione OTA).

Configurazione: max 4 connessioni TCP concorrenti.

### 6.8 HTTPS: rimandato

HTTPS richiederebbe certificati auto-firmati gestiti dal device + TLS stack (`embedded-tls` o `rustls`) + esperienza utente del "certificato non sicuro" nel browser. Su rete WiFi domestica con WPA2/WPA3, il rischio reale di intercettazione passiva è basso. HTTPS resta nice-to-have per il futuro.

Eccezione importante: durante il provisioning il link WiFi deve essere cifrato tramite WPA2 SoftAP con PIN per-device, perché in quella fase transitano credenziali WiFi e password admin. La decisione "no HTTPS" non autorizza mai un provisioning su rete aperta.

### 6.9 Display OLED durante uso normale

Quattro righe statiche aggiornate via il task OLED esistente:

```
DCC-Soggiorno
Rete: CasaMia
IP: 192.168.1.47
Stato: OK
```

Durante provisioning:

```
SETUP NECESSARIO
DCC-Setup-A4F2
PIN: 12345678
IP: 192.168.4.1
```

Durante errore collegamento (intermedio):

```
WiFi non disponibile
"CasaMia"
Tentativo 2/5
```

Durante boot/init:

```
DCC Station
v0.x.x
Avvio...
```

LED rosso/verde mantengono il loro ruolo (feedback hold del tasto GPIO0, stato operativo). Il display non è coinvolto nel feedback hold.

---

## 7. Predisposizione OTA, testing, integrazione

### 7.1 Cosa blocchiamo adesso per essere OTA-ready

Sette decisioni che pagano ora un costo basso per ripagare in fase OTA:

1. **Flash map esplicita `esp-hal`** (Sezione 3) — config store in scope, spazio OTA solo riservato finché lo spike dual-slot non passa.
2. **Web server con supporto streaming upload** (`picoserve`) — già scelto.
3. **Endpoint riservati `/api/firmware/*`** — dichiarati, rispondono `501`.
4. **Middleware autenticazione condivisa** — sessioni e CSRF già pronti per proteggere upload firmware.
5. **`/api/status` con campo `firmware_version`** — da `env!("CARGO_PKG_VERSION")`. In futuro estenderemo con `active_slot` e `firmware_hash`.
6. **Formato manifesto firmware firmato** — ogni immagine OTA futura dovrà arrivare con manifest contenente versione, dimensione, hash SHA-256 e firma Ed25519/ECDSA.
7. **Chiave pubblica verificatore compilata nel firmware** — il Livello 1 non implementa upload, ma riserva già il punto architetturale in cui la verifica firma diventerà obbligatoria.

### 7.2 Invarianti di sicurezza OTA

Queste regole sono parte del design anche se l'upload firmware è fuori scope nel Livello 1:

- Nessun firmware può essere scritto nello slot inattivo se manifest, hash e firma non sono validi.
- Nessun `/api/firmware/commit` può marcare uno slot attivo se l'immagine non è stata verificata.
- Il rollback è permesso solo verso immagini già verificate e firmate.
- La chiave privata di firma non sta mai nel device; il firmware contiene solo la chiave pubblica di verifica o un suo digest protetto.
- `/api/firmware/info` dovrà esporre `active_slot`, `pending_slot`, `firmware_version`, `firmware_hash` e stato di verifica.
- Le operazioni OTA richiedono sessione valida, token CSRF e password admin riconfermata.

### 7.3 Cosa rimandiamo esplicitamente

Fuori scope Livello 1, riferimento per iterazione futura:

- Implementazione `/api/firmware/upload`, `/commit`, `/rollback`
- Logica di scrittura in area firmware inattiva o riservata
- Boot-and-rollback automatico in caso di crash post-OTA
- Tooling concreto per firmare `.bin` e generare manifest
- Utility desktop per generare/firmare `.bin`

### 7.4 Struttura dei moduli (con Clean Architecture pragmatica — vedi Sezione 8)

```
src/
├── net/
│   ├── runtime.rs       # Owner di radio, WiFi controller, stack, runner
│   ├── events.rs        # Eventi e comandi rete bounded
│   └── udp_control.rs   # Servizio Z21 UDP sopra stack già inizializzato
├── provisioning/
│   ├── mod.rs           # API pubblica + spawn supervisor
│   ├── state.rs         # Macchina a stati pura (domain)
│   ├── events.rs        # Event types (domain)
│   ├── supervisor.rs    # Task Embassy che lega state ai task esterni (application)
│   ├── softap.rs        # Configurazione access point (infrastructure)
│   ├── dhcp.rs          # Server DHCP (infrastructure)
│   └── captive_dns.rs   # Server DNS captive (infrastructure)
├── web_panel/
│   ├── mod.rs
│   ├── auth.rs          # Pure: verify, sessions, rate limit (domain + clock port)
│   ├── clock.rs         # Clock trait + EmbassyClock impl
│   ├── server.rs        # picoserve setup (infrastructure)
│   ├── handlers/        # HTTP → use case adapters
│   │   ├── status.rs
│   │   ├── admin.rs
│   │   └── recovery.rs
│   ├── view_models.rs   # Dati puri per le pagine, testabili su host
│   ├── pages.rs         # Rendering HTML (presentation adapter)
│   └── csrf.rs          # Verifica password nel body (domain)
├── storage/
│   ├── mod.rs           # ConfigRepository trait (port)
│   ├── schema.rs        # Tipi dato persistente (domain)
│   ├── flash.rs         # Adapter produzione, backend da decidere nel piano impl
│   └── password_hash.rs # bcrypt wrapper (domain puro)
└── reset_button/
    ├── mod.rs
    ├── detector.rs      # Logica timing → eventi (domain)
    └── task.rs          # Task Embassy su GPIO0 (infrastructure)
```

### 7.5 Nuove dipendenze

Da aggiungere a `Cargo.toml`:

- `picoserve` (web server)
- `embassy-net` (stack TCP/IP — possibile sia già presente)
- storage flash compatibile `esp-hal` da selezionare nel piano implementativo
- `embedded-storage` (trait comuni)
- `serde` + `serde-json-core` (JSON)
- `heapless-bytes` o equivalente per body bounded
- `bcrypt-no-std` (o equivalente `no_std` compatibile)
- `hmac` + `sha2`
- `ed25519-dalek` o alternativa no_std verificata per firme firmware future
- `getrandom` configurato con il generatore hardware ESP32-C6

Versioni esatte e feature flags da definire nel piano di implementazione.

### 7.6 Strategia di test

**Test host** (`cargo test-host`) per:
- `provisioning::state` — macchina a stati pura, transizioni esaustive
- `storage::password_hash` — bcrypt round-trip, validità con password sbagliate
- `storage::schema` — serializzazione/deserializzazione
- `storage` con fake backend — load/save/wipe e recovery definita dal piano implementativo
- `web_panel::auth` — rate limiter, generazione/scadenza sessioni (con `MockClock`)
- `web_panel::csrf` — confronto password nel body
- `web_panel::view_models` — mapping da `StatusModel` mock a dati pagina
- `web_panel::pages` — generazione HTML con escaping dai view model
- `reset_button::detector` — logica timing da sequenze sintetiche
- `ota::manifest` futuro — parsing manifesto e verifica hash/firma su test vector

**Test su dispositivo** (checklist di accettazione del Livello 1):

- [ ] Primo boot con configurazione assente → `Provisioning`, hotspot WPA2 visibile dal telefono, PIN mostrato su OLED, portale si apre dopo connessione, configurazione si completa.
- [ ] Sniff passivo durante provisioning → non sono leggibili password admin o password WiFi perché il link SoftAP è cifrato.
- [ ] Boot con credenziali valide salvate → `OperationalConnected`, IP sull'OLED, pannello raggiungibile.
- [ ] Boot con credenziali obsolete → 5 fallimenti → `Provisioning`.
- [ ] Login con password corretta → dashboard accessibile.
- [ ] Login con password sbagliata 5 volte → blocco per 5 minuti.
- [ ] POST autenticato senza token CSRF → rifiutato.
- [ ] Cambio nome device → persistito, mostrato sull'OLED dopo riavvio.
- [ ] Cambio password admin → vecchia password rifiutata, nuova accettata.
- [ ] Cambio password admin → sessioni esistenti invalidate.
- [ ] Cambio rete WiFi via pannello → prova nuova rete, salva solo se funziona, OLED aggiornato.
- [ ] Cambio rete WiFi fallito → rete precedente ripristinata, OLED mostra errore.
- [ ] GPIO0 hold 5s → `Provisioning` (LED rosso lampeggia durante hold).
- [ ] GPIO0 hold 10s → factory reset (LED rosso fisso durante hold), riavvio in stato vergine.
- [ ] DCC continua in `OperationalOffline` → spegnere router mentre loco in moto, verifica che continua.
- [ ] Z21 app perde connessione in `OperationalOffline`, la riacquista quando router torna.

### 7.7 Integrazione con codice esistente

**`src/net/runtime.rs`** — nuovo proprietario di radio, WiFi controller, `embassy-net::Stack`, runner e transizioni Client/SoftAP. Espone eventi e comandi bounded al supervisor.

**`src/net/udp_control.rs`** — riceve uno stack già configurato dal runtime rete e credenziali/stato rete come eventi runtime invece che da `env!()`. La responsabilità diventa solo UDP Z21.

**`src/boot.rs`** — sequenza di init riorganizzata:
```
peripherals → RMT → I2C OLED → H-bridge (held off) →
storage::init (legge configurazione persistente) →
provisioning::supervisor (decide stato iniziale) →
  IF first_boot_done == false OR creds_missing:
    net_runtime::start_softap(pin_from_rng)
    spawn provisioning tasks (dns + web_panel limited)
    DCC NON parte
  ELSE:
    net_runtime::start_client (con creds da storage)
    udp_control::start (con stack da net_runtime)
    web_panel::start_full (con stack da net_runtime)
    spawn dcc_engine_task, packet_scheduler_task (come oggi)
    H-bridge enabled
spawn LEDs, OLED, control buttons, short detector, reset button GPIO0
```

**`src/system_status.rs`** — aggiungere stati `WifiConnecting`, `Provisioning`, `OfflineDegraded`. Pattern matching su LED e OLED estesi di conseguenza.

### 7.8 Piano di migrazione

1. Riflash via USB con flash map `esp-hal` validata (operazione una tantum).
2. Credenziali in `.env` vengono perse (atteso).
3. Primo boot post-migrazione → `Provisioning` → configurazione utente.
4. Rimuovere `WIFI_SSID`/`WIFI_PASS` da `udp_control.rs` e da eventuali `.cargo/config.toml`.

---

## 8. Clean Architecture: applicazione pragmatica

### 8.1 La regola che seguiamo

Una sola, leggera: **la logica non sa niente del mondo**.

- I moduli di logica pura (macchine a stati, validazioni, hash, view model, contatori rate limit) non importano `esp-hal`, `esp-radio`, `embassy-net`, `picoserve`. Usano solo `core` e al massimo `heapless`.
- I moduli che parlano col mondo (hotspot, GPIO, flash, server HTTP) sono adapter sottili che chiamano la logica pura.

Quando una dipendenza esterna è un'astrazione che vale la pena testare con mock, introduciamo un **trait** (port). Quando non lo è, evitiamo cerimonia.

### 8.2 Dove applichiamo

**`storage` → trait `ConfigRepository` + adapter flash**
```rust
pub trait ConfigRepository {
    fn load(&self) -> Result<Option<DeviceConfig>, StorageError>;
    fn save_after_verified_wifi(&mut self, cfg: &DeviceConfig) -> Result<(), StorageError>;
    fn wipe_all(&mut self) -> Result<(), StorageError>;
}
```
Impl produzione: adapter flash da scegliere nel piano implementativo. Impl test: `InMemoryConfigRepository`. Il dominio non espone setter separati per WiFi/admin/device name durante il provisioning: salva solo una configurazione completa dopo WiFi verificata.

**`provisioning::state` → macchina a stati pura**
```rust
pub enum ProvisioningState {
    Boot,
    WifiConnecting,
    OperationalConnected,
    OperationalOffline,
    Provisioning,
    FactoryResetting,
}
pub enum ProvisioningEvent {
    InitComplete,
    WifiConnected,
    WifiDisconnected,
    WifiAttemptFailed,        // singolo tentativo fallito (incrementa contatore)
    WifiMaxRetriesExceeded,   // raggiunto 5/5 fallimenti
    ButtonHold5s,
    ButtonHold10s,
    ReprovisionFromWeb,
    FactoryResetFromWeb,
    ConfigSaved,              // utente ha confermato form di provisioning con successo
}
pub fn next_state(current: ProvisioningState, event: ProvisioningEvent) -> ProvisioningState;
```
Funzione pura. Testabile esaustivamente su host. **I nomi degli stati combaciano esattamente con la tabella della Sezione 2.1.**

**`web_panel::auth` → logica pura + `Clock` port**
```rust
pub trait Clock { fn now(&self) -> u64; }
pub struct EmbassyClock;       // produzione
pub struct MockClock(u64);     // test
```
Tutta la logica di sessione e rate limiting riceve `&dyn Clock` come parametro, testabile con time mockato.

**`web_panel::view_models` + `pages` → Humble Object**
- `view_models.rs`: costruisce dati puri e già validati per dashboard/form, testabile senza HTTP.
- `pages.rs`: rendering HTML e escaping. È presentation adapter, non domain.

**`reset_button` → detector logico + task hardware**
- `detector.rs`: riceve `Pressed{ts}` / `Released{ts}` e produce `ShortPress` / `ForceProvisioning` / `FactoryReset` in base ai delta. Testabile su host.
- `task.rs`: task Embassy su GPIO0 che traduce eventi pin in chiamate al detector.

### 8.3 Dove NON applichiamo

1. **Niente directory top-level `domain/` `application/` `infrastructure/`** — separazione vive dentro ogni modulo (es. `state.rs` vs `softap.rs`).
2. **Niente DI containers** — generics + traits di Rust bastano.
3. **Niente CQRS, Event Sourcing, aggregati DDD** — sproporzionati per la scala del firmware.
4. **Niente refactoring del dominio DCC/Z21 esistente** — fuori scope.

### 8.4 Bilancio costo/beneficio

**Costo aggiuntivo rispetto a un design non-CA:**
- 2 trait nuovi (`ConfigRepository`, `Clock`), ~30-40 righe totali
- 2 mock impl (`InMemoryConfigRepository`, `MockClock`), solo in `#[cfg(test)]`
- 1-2 file separati per modulo dove avremmo potuto averne uno (es. `reset_button/detector.rs` + `reset_button/task.rs`)

**Beneficio:**
- Test host su ~70% del codice nuovo
- Confini chiari per code review (un file `domain` non ha hardware dentro per design)
- Refactoring infrastrutturale futuro più facile (cambiare `picoserve` tocca solo `server.rs` + handlers)

---

## 9. Cosa NON facciamo (out of scope, esplicito)

Lista deliberatamente fuori dallo scope del Livello 1, da rivisitare in iterazioni successive:

- **Implementazione effettiva degli aggiornamenti firmware OTA** (predisposto, non implementato).
- **Dashboard operativa** con stato locomotive live, corrente assorbita, log eventi.
- **Programmazione CV via web** (resta solo via Z21 app).
- **mDNS / Bonjour** (`dcc-station.local`) — solo IP sull'OLED.
- **Lista multi-rete** ricordata (una sola rete salvata).
- **Configurazione IP statica** (DHCP only).
- **HTTPS** con certificati gestiti dal device.
- **Wizard multi-step** nel provisioning.
- **Recupero password admin** via email/SMS (solo factory reset hardware).
- **Tooling e implementazione completa della firma firmware**. Il requisito architetturale di firma OTA è invece già in scope e non va rimosso.
- **Protezione flash hardware/software avanzata** (la password WiFi viene salvata recuperabile dal firmware).
- **Internazionalizzazione completa** (`it`/`en` come flag salvato ma testi solo italiani nel Livello 1).
- **Telemetria/log persistenti** nella zona `storage` della flash.

---

## 10. Rischi e questioni aperte

Cose che dal design non si possono blindare completamente e che andranno verificate in implementazione:

**R1. Compatibilità di `picoserve` con `esp-radio` nelle due modalità.**
`picoserve` lavora sopra `embassy-net`, che parla col driver WiFi di `esp-radio`. Da verificare:
- Funzionamento stabile in modalità client.
- Funzionamento stabile in modalità SoftAP.

La transizione SoftAP → Client non è live nel Livello 1: avviene tramite riavvio controllato. AP+STA simultaneo resta una possibile ottimizzazione futura, non un prerequisito.

**R2. Coesistenza WiFi + UDP Z21 + TCP HTTP.**
Aggiungendo TCP HTTP allo stack esistente di UDP Z21, da verificare il budget di socket di `embassy-net` (N socket TCP + M socket UDP, con N+M nei limiti supportati).

**R3. Tempi di hashing della password.**
Bcrypt sull'ESP32-C6 va misurato. Cost factor 8 è un'ipotesi iniziale. Taratura empirica per bilanciare UX di login (deve essere < 500 ms) e difesa anti-bruteforce (deve essere > 10 ms).

**R4. Storage persistente credenziali.**
Il backend flash non è progettato in questo documento. Il piano implementativo deve scegliere l'adapter `esp-hal` compatibile, definire erase/write/recovery, e dimostrare che un reset durante scrittura non lascia il device in uno stato non recuperabile.

**R5. Pinmap GPIO0.**
Da verificare che GPIO0 sia disponibile (non usato da nessun'altra funzione futura) e che il pull-up interno sia sufficiente. Il device verrà programmato/flashato via USB anche dopo l'introduzione di GPIO0 come tasto — verificare nessuna interferenza con la procedura di flash (l'ESP32-C6 usa USB-Serial-JTAG nativo, non condivide pin con GPIO0).

**R6. Dimensione effettiva della flash.**
Confermare con `espflash board-info` che il modulo in uso ha 8 MB. Layout alternativo per 4 MB da preparare in subordine.

**R7. Libreria di firma no_std.**
La scelta Ed25519/ECDSA va validata su `riscv32` no_std per dimensione codice, RAM e tempi di verifica. Se la libreria scelta è troppo pesante, l'invariante resta invariato: cambia l'algoritmo/libreria, non il requisito di verifica firma.

---

## 11. Riferimenti

- NMRA Standards `docs/specs/NMRA-STANDARDS.md` (compatibilità DCC esistente — non impattata da questo design)
- Z21 LAN Protocol `docs/specs/z21-lan-protokoll-en.pdf` (compatibilità Z21 esistente — non impattata)
- `esp-hal`, `esp-radio`, `embassy-net`, `espflash` — documentazione ufficiale dei crate/tool usati dal firmware
- Bcrypt RFC — funzione di hashing scelta per password admin
- RFC 6585 (HTTP 429 Too Many Requests) — per il rate limiting di login
- OWASP Authentication Cheat Sheet — riferimento generale per le scelte di sicurezza

---

**Documento approvato dall'utente?** Sì, confermato il 2026-05-12.
