# WiFi Provisioning & Pannello Web Permanente — Design

**Data:** 2026-05-12
**Branch corrente alla scrittura:** `feat/railcom` (il design è separato dal lavoro RailCom; verrà implementato su branch dedicato)
**Status:** In review

---

## 0. Sommario esecutivo

Questo documento descrive il design di un sottosistema che sostituisce l'attuale configurazione WiFi via file `.env` (compilata nel firmware) con un meccanismo di **provisioning runtime** e di **amministrazione web permanente**. È il primo passo per trasformare il device da prototipo configurato in fabbrica a prodotto commerciale configurabile dall'utente finale.

**Cosa cambia per l'utente.** Al primo boot (o dopo un factory reset), il device accende un hotspot WiFi temporaneo e mostra una pagina di configurazione attraverso un portale captive. L'utente inserisce nome device, password amministratore, e credenziali della rete WiFi a cui far collegare il device. Da quel momento, il device si avvia direttamente in modalità operativa, esponendo un pannello web amministrativo sulla rete di casa per cambiare configurazione, riconfigurare la WiFi, fare factory reset, e (in futuro) caricare nuovo firmware.

**Cosa cambia per il firmware.** Vengono introdotti quattro nuovi moduli (`provisioning/`, `web_panel/`, `storage/`, `reset_button/`) e viene modificato `net/udp_control.rs` per leggere le credenziali da memoria flash invece che da `env!()`. Il layout della memoria flash viene ridefinito con due slot firmware (predisposizione aggiornamenti OTA) e una zona configurazione utente non-volatile.

**Scope:** Livello 1 (configurazione di rete e identità). **Fuori scope:** dashboard operativa con stato locomotive, upload firmware (predisposto ma non implementato), programmazione CV via web, mDNS.

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
| Persistenza credenziali | Memoria flash chiave-valore (NVS) | Standard ESP, supporto transazionale atomico |
| Trigger provisioning | Automatico (NVS vuota o 5 fallimenti) + tasto GPIO0 hold 5s | Combinazione standard prodotti commerciali |
| Reti ricordate | Una sola (l'ultima configurata) | Scelta minimal; multi-rete rimandata se servirà |
| Pannello web | Permanente (sempre attivo in stato Operativo) | Necessario per cambiare WiFi senza factory reset |
| Scope pannello | Livello 1: stato base + cambio rete + cambio nome | Anti-scope-creep, OTA esplicitamente predisposto ma non implementato |
| Autenticazione | Login form + cookie di sessione (`HttpOnly`, `SameSite=Strict`) | Esperienza più "prodotto", costo accettabile |
| Hardware setup | Tasto su GPIO0 con condensatore 100 nF + pull-up interno | Stesso pattern dei tasti stop/resume esistenti |
| Discovery sulla rete | IP visualizzato sull'OLED del device | Semplicità; mDNS rimandato |
| Lingua pannello | Italiano (con variabile per estensione futura) | Mercato di partenza italiano |
| Display OLED | Mostra stato + IP in operativo, nome hotspot + IP fisso in provisioning | Riusa display esistente, niente nuovo hardware |
| Web server library | `picoserve` (Embassy-native, supporta streaming upload) | Predisposto per OTA |
| HTTPS | No nel Livello 1 | LAN domestica con WPA2/WPA3, costo certificati troppo alto per il valore |
| Layout flash | 2 slot firmware da 3 MB (predisposizione OTA) | Decisione irreversibile da prendere ora |

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
| `FactoryResetting` | Transiente | Cancellazione NVS in corso | No |

### 2.2 Tabella delle transizioni

| Da → A | Trigger |
|---|---|
| `Boot` → `WifiConnecting` | Init completo **e** NVS contiene credenziali valide |
| `Boot` → `Provisioning` | Init completo **e** NVS vuota |
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

**P3. Durante il primo provisioning post-boot (NVS vuota), DCC non parte mai.**
Il device esce dalla fabbrica con NVS vuota → prima accensione → entra subito in `Provisioning` senza aver mai energizzato i binari. È solo dopo la conferma dell'utente che si passa a `WifiConnecting` → `OperationalConnected` e DCC parte per la prima volta.

### 2.4 Concorrenza

La macchina a stati vive in **un singolo Embassy task** (`provisioning_supervisor_task`) che ne è proprietario esclusivo. Comunica con il resto del sistema via:

- **In ingresso**: `embassy_sync::Channel<ProvisioningEvent, 16>` per eventi (button events, WiFi events, richieste dal web server).
- **In uscita**: eventi al `StatusModel` esistente, comandi a `TrackOutput` per shutdown ordinato DCC.

Single-threaded logicamente. Nessun lock, nessuna race condition.

---

## 3. Layout della memoria flash

### 3.1 Premessa

L'ESP32-C6 di riferimento è `ESP32-C6-WROOM-1-N8` con **8 MB di flash**. **Da verificare** che il modulo in uso sia effettivamente la variante N8 (lettura con `espflash board-info`). Se il modulo dovesse essere a 4 MB, gli slot firmware si riducono a 1.8 MB ciascuno, sufficiente per la dimensione attuale del firmware (~500 KB) con ampio margine, ma stretti per uno scenario di crescita futuro significativo.

Questo layout viene flashato **una sola volta** alla prima programmazione del nuovo firmware. Successive scritture (incluse quelle via OTA in futuro) toccano solo le partizioni dati e/o gli slot firmware, mai la tabella partizioni stessa.

### 3.2 Le partizioni

| Nome | Tipo | Offset | Dimensione | Cosa contiene |
|---|---|---|---|---|
| `bootloader` | bootloader | 0x0000 | 32 KB | Bootloader ESP-IDF. Decide quale slot firmware caricare in base a `otadata`. |
| `partition-table` | partition | 0x8000 | 4 KB | Tabella che descrive questo layout. |
| `nvs` | data/nvs | 0x9000 | 24 KB | **Zona configurazione utente.** Credenziali WiFi, password admin hash, nome device. |
| `phy_init` | data/phy | 0xF000 | 4 KB | Calibrazione radio WiFi (gestita da `esp-radio`). |
| `otadata` | data/ota | 0x10000 | 8 KB | Indice degli slot OTA (quale è attivo). |
| `ota_0` | app/ota_0 | 0x20000 | 3 MB | **Slot firmware A.** Oggi il firmware gira da qui. |
| `ota_1` | app/ota_1 | 0x320000 | 3 MB | **Slot firmware B.** Oggi vuoto. Destinazione futura per aggiornamenti OTA. |
| `storage` | data/fat | 0x620000 | ~1.9 MB | Zona riservata per dati estesi (backup, log persistenti, asset statici). Non usata nel Livello 1. |

Totale: 8 MB.

### 3.3 Chiavi nella partizione `nvs` (Livello 1)

| Chiave | Tipo | Esempio | Note |
|---|---|---|---|
| `wifi.ssid` | string | `"CasaMia"` | SSID rete |
| `wifi.psk` | string | `"miapassword"` | Password rete. Salvata in chiaro (deve essere ricostruibile per il collegamento). |
| `admin.password_hash` | bytes (32) | `<hash>` | Hash bcrypt della password admin |
| `admin.password_salt` | bytes (16) | `<salt>` | Salt unico per device |
| `device.name` | string | `"DCC-Soggiorno"` | Nome utente-configurabile |
| `device.language` | string | `"it"` | `it` o `en`, per estensione futura del pannello multilingua |
| `device.first_boot_done` | bool | `true` | Flag che indica se il provisioning è già stato completato |

Tutte queste chiavi vengono cancellate da un factory reset, lasciando `nvs` vuota.

### 3.4 Password admin: hash con bcrypt

La password admin **non viene mai salvata in chiaro**. Quando l'utente la sceglie:

1. Il firmware genera un salt casuale di 16 byte usando il generatore hardware ESP32-C6.
2. Calcola `hash = bcrypt(password, salt, cost=8)`. Cost factor 8 è scelto per impiegare ~50ms sull'hardware: abbastanza da rendere impraticabile la forza bruta, abbastanza veloce da non rallentare il login utente. **Il cost factor andrà tarato empiricamente** durante l'implementazione.
3. Salva `salt` e `hash` in NVS.

In login: lo stesso calcolo viene ripetuto sulla password ricevuta e confrontato con l'hash salvato.

Conseguenza: anche un attaccante che smonta fisicamente il device e legge la flash non può ricavare la password — solo provare a indovinarla, e ogni tentativo costa 50 ms di CPU dedicata.

### 3.5 Password WiFi: in chiaro

A differenza della password admin, la password della rete WiFi deve essere conservata in chiaro perché il firmware deve passarla al collegamento WiFi (non è autenticabile via hash). È lo standard di tutti i device WiFi del mondo. Per scenari futuri ad alta sensibilità si può abilitare la **Flash Encryption** di ESP-IDF (cifra tutta la flash con chiave da eFuse non leggibile). Non in scope per il Livello 1.

### 3.6 Migrazione dal codice attuale

Il firmware attuale gira con un layout single-app senza predisposizione OTA. La transizione richiede:

1. **Riflash una tantum via USB** con la nuova tabella partizioni.
2. Le credenziali WiFi che erano in `.env` vengono **perse** in questo passaggio (NVS è vuota dopo il riflash). Atteso e voluto.
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

1. **Hotspot WiFi (modalità access point)** — Nome rete: `DCC-Setup-XXXX` (XXXX = ultime 4 cifre hex del MAC). Rete aperta (no password). L'hotspot è temporaneo (solo durante il setup): la mancanza di password è accettabile per uno scenario di pochi minuti, ed evita il problema della "password generica stampata su etichetta" che è un problema di produzione.

2. **Server DHCP interno** — Assegna IP nell'intervallo `192.168.4.2-192.168.4.10` ai client che si collegano. Device stesso ha IP fisso `192.168.4.1` (standard ESP-IDF SoftAP). Implementazione tramite la libreria DHCP integrata in `esp-radio`/`embassy-net`.

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
2. **Test del collegamento WiFi**: il device tenta il collegamento con timeout di 15 secondi. Durante questo tempo l'hotspot resta acceso e il browser riceve una pagina "in corso" con polling su `/api/provisioning-status`.
3. **Esito positivo**: salvataggio atomico in NVS (`wifi.ssid`, `wifi.psk`, `device.name`, `admin.password_hash`, `admin.password_salt`, `device.first_boot_done = true`). Pagina di successo che mostra "Configurazione salvata. Collegati alla tua rete WiFi e apri http://<IP>". Transizione a `OperationalConnected`. OLED aggiornato con nuovo IP.
4. **Esito negativo**: pagina con messaggio d'errore. Nessun salvataggio. Stato resta `Provisioning`. Utente può ritentare.

Il salvataggio in NVS usa la primitiva transazionale di `esp-storage` per garantire atomicità.

### 5.4 Out of scope nel Livello 1

- Configurazione IP statica
- Wizard multi-step
- Recupero password (utente fa factory reset col GPIO0)
- Configurazione fuso orario/locale (lingua sì, come flag in NVS)

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

### 6.4 Protezione anti-CSRF sugli endpoint distruttivi

Gli endpoint marcati "password nel body" applicano un secondo controllo: cookie di sessione valido **e** password admin riconfermata nel body JSON.

Esempio:
```
POST /api/factory-reset
Cookie: dcc_session=abc123...
Content-Type: application/json

{ "admin_password": "lamiapassword" }
```

Difende dal caso in cui un sito web malevolo, visitato dall'utente in un'altra tab, tenta richieste verso il device sfruttando i cookie già attivi: il sito malevolo non conosce la password admin.

### 6.5 Rate limiting

Tabella in RAM con max 16 IP tracciati: `{ ip, failed_attempts, last_attempt_ts }`. 5 tentativi falliti consecutivi → blocco IP per 5 minuti. Dopo 5 minuti il contatore si resetta.

### 6.6 Flusso del cambio WiFi via pannello

Caso particolare con interazione di rete complessa.

1. Utente compila form `Cambia rete...` e clicca "Cambia".
2. Server valida, risponde subito con pagina "Sto cambiando rete... attendi ~30 secondi e ricarica." (il messaggio mostra 30s — abbondante margine — anche se il timeout server effettivo è 15s; i 15s aggiuntivi servono all'utente per trovare il nuovo IP sull'OLED).
3. Firmware si scollega dalla rete attuale e tenta la nuova con timeout 15s.
4. **Successo**: salva nuove credenziali in NVS. OLED mostra nuovo nome rete e nuovo IP. L'utente attende, ricarica la pagina nel browser (che è morta perché device non più sulla stessa rete), passa al WiFi nuovo dal suo dispositivo, legge nuovo IP dall'OLED, accede al pannello sul nuovo IP.
5. **Fallimento**: dopo 15s di tentativi falliti, firmware torna alle credenziali precedenti e riconnette alla rete vecchia (presumibilmente stesso IP). L'utente che ricarica trova la pagina viva con messaggio "Cambio rete fallito".

Durante i 15s di transizione il device è temporaneamente irraggiungibile. DCC continua a girare (è stato `OperationalOffline` temporaneo).

### 6.7 Web server library

**Scelta:** `picoserve` (Embassy-native, supporta streaming upload — predisposizione OTA).

Configurazione: max 4 connessioni TCP concorrenti.

### 6.8 HTTPS: rimandato

HTTPS richiederebbe certificati auto-firmati gestiti dal device + TLS stack (`embedded-tls` o `rustls`) + esperienza utente del "certificato non sicuro" nel browser. Su rete WiFi domestica con WPA2/WPA3, il rischio reale di intercettazione passiva è basso. HTTPS resta nice-to-have per il futuro.

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
Rete WiFi:
DCC-Setup-A4F2
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

Cinque decisioni che pagano ora un costo basso per ripagare in fase OTA:

1. **Layout flash con 2 slot firmware** (Sezione 3) — già definito, lo slot B oggi è vuoto.
2. **Web server con supporto streaming upload** (`picoserve`) — già scelto.
3. **Endpoint riservati `/api/firmware/*`** — dichiarati, rispondono `501`.
4. **Middleware autenticazione condivisa** — sessioni e CSRF già pronti per proteggere upload firmware.
5. **`/api/status` con campo `firmware_version`** — da `env!("CARGO_PKG_VERSION")`. In futuro estenderemo con `active_slot` e `firmware_hash`.

### 7.2 Cosa rimandiamo esplicitamente

Fuori scope Livello 1, riferimento per iterazione futura:

- Implementazione `/api/firmware/upload`, `/commit`, `/rollback`
- Logica di scrittura nello slot inattivo (`esp-storage` + wrapper)
- Boot-and-rollback automatico in caso di crash post-OTA
- Firma crittografica del firmware
- Utility desktop per generare/firmare `.bin`

### 7.3 Struttura dei moduli (con Clean Architecture pragmatica — vedi Sezione 8)

```
src/
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
│   ├── pages.rs         # Template HTML puri (domain)
│   └── csrf.rs          # Verifica password nel body (domain)
├── storage/
│   ├── mod.rs           # ConfigStore trait (port)
│   ├── schema.rs        # Tipi dato persistente (domain)
│   ├── nvs.rs           # NvsConfigStore: ConfigStore (infrastructure)
│   └── password_hash.rs # bcrypt wrapper (domain puro)
└── reset_button/
    ├── mod.rs
    ├── detector.rs      # Logica timing → eventi (domain)
    └── task.rs          # Task Embassy su GPIO0 (infrastructure)
```

### 7.4 Nuove dipendenze

Da aggiungere a `Cargo.toml`:

- `picoserve` (web server)
- `embassy-net` (stack TCP/IP — possibile sia già presente)
- `esp-storage` (accesso flash a livello partizione)
- `embedded-storage` (trait comuni)
- `serde` + `serde-json-core` (JSON)
- `heapless-bytes` o equivalente per body bounded
- `bcrypt-no-std` (o equivalente `no_std` compatibile)
- `hmac` + `sha2`
- `getrandom` configurato con il generatore hardware ESP32-C6

Versioni esatte e feature flags da definire nel piano di implementazione.

### 7.5 Strategia di test

**Test host** (`cargo test-host`) per:
- `provisioning::state` — macchina a stati pura, transizioni esaustive
- `storage::password_hash` — bcrypt round-trip, validità con password sbagliate
- `storage::schema` — serializzazione/deserializzazione
- `web_panel::auth` — rate limiter, generazione/scadenza sessioni (con `MockClock`)
- `web_panel::csrf` — confronto password nel body
- `web_panel::pages` — generazione HTML da `StatusModel` mock
- `reset_button::detector` — logica timing da sequenze sintetiche

**Test su dispositivo** (checklist di accettazione del Livello 1):

- [ ] Primo boot con NVS vuota → `Provisioning`, hotspot visibile dal telefono, portale si apre da sé, configurazione si completa.
- [ ] Boot con credenziali valide salvate → `OperationalConnected`, IP sull'OLED, pannello raggiungibile.
- [ ] Boot con credenziali obsolete → 5 fallimenti → `Provisioning`.
- [ ] Login con password corretta → dashboard accessibile.
- [ ] Login con password sbagliata 5 volte → blocco per 5 minuti.
- [ ] Cambio nome device → persistito, mostrato sull'OLED dopo riavvio.
- [ ] Cambio password admin → vecchia password rifiutata, nuova accettata.
- [ ] Cambio rete WiFi via pannello → device si scollega, prova nuova, OK riconnette, OLED aggiornato.
- [ ] GPIO0 hold 5s → `Provisioning` (LED rosso lampeggia durante hold).
- [ ] GPIO0 hold 10s → factory reset (LED rosso fisso durante hold), riavvio in stato vergine.
- [ ] DCC continua in `OperationalOffline` → spegnere router mentre loco in moto, verifica che continua.
- [ ] Z21 app perde connessione in `OperationalOffline`, la riacquista quando router torna.

### 7.6 Integrazione con codice esistente

**`src/net/udp_control.rs`** — riceve credenziali come parametri runtime invece che da `env!()`. API esposta (task UDP per Z21) invariata.

**`src/boot.rs`** — sequenza di init riorganizzata:
```
peripherals → RMT → I2C OLED → H-bridge (held off) →
storage::init (legge NVS) →
provisioning::supervisor (decide stato iniziale) →
  IF first_boot_done == false OR creds_missing:
    spawn provisioning tasks (softap + dhcp + dns + web_panel limited)
    DCC NON parte
  ELSE:
    udp_control::start (con creds da storage)
    spawn web_panel full (modalità OperationalConnected)
    spawn dcc_engine_task, packet_scheduler_task (come oggi)
    H-bridge enabled
spawn LEDs, OLED, control buttons, short detector, reset button GPIO0
```

**`src/system_status.rs`** — aggiungere stati `WifiConnecting`, `Provisioning`, `OfflineDegraded`. Pattern matching su LED e OLED estesi di conseguenza.

### 7.7 Piano di migrazione

1. Riflash via USB con nuova tabella partizioni (operazione una tantum).
2. Credenziali in `.env` vengono perse (atteso).
3. Primo boot post-migrazione → `Provisioning` → configurazione utente.
4. Rimuovere `WIFI_SSID`/`WIFI_PASS` da `udp_control.rs` e da eventuali `.cargo/config.toml`.

---

## 8. Clean Architecture: applicazione pragmatica

### 8.1 La regola che seguiamo

Una sola, leggera: **la logica non sa niente del mondo**.

- I moduli di logica pura (macchine a stati, validazioni, hash, generazione HTML, contatori rate limit) non importano `esp-hal`, `esp-radio`, `embassy-net`, `picoserve`. Usano solo `core` e al massimo `heapless`.
- I moduli che parlano col mondo (hotspot, GPIO, flash, server HTTP) sono adapter sottili che chiamano la logica pura.

Quando una dipendenza esterna è un'astrazione che vale la pena testare con mock, introduciamo un **trait** (port). Quando non lo è, evitiamo cerimonia.

### 8.2 Dove applichiamo

**`storage` → trait `ConfigStore` + `NvsConfigStore`**
```rust
pub trait ConfigStore {
    fn load_wifi_creds(&self) -> Result<Option<WifiCreds>, StorageError>;
    fn save_wifi_creds(&mut self, creds: &WifiCreds) -> Result<(), StorageError>;
    fn load_admin(&self) -> Result<Option<AdminCredentials>, StorageError>;
    fn save_admin(&mut self, admin: &AdminCredentials) -> Result<(), StorageError>;
    fn load_device_config(&self) -> Result<DeviceConfig, StorageError>;
    fn save_device_config(&mut self, cfg: &DeviceConfig) -> Result<(), StorageError>;
    fn wipe_all(&mut self) -> Result<(), StorageError>;
}
```
Impl produzione: `NvsConfigStore` (usa `esp-storage`). Impl test: `InMemoryConfigStore` (HashMap).

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

**`reset_button` → detector logico + task hardware**
- `detector.rs`: riceve `Pressed{ts}` / `Released{ts}` e produce `ShortPress` / `ForceProvisioning` / `FactoryReset` in base ai delta. Testabile su host.
- `task.rs`: task Embassy su GPIO0 che traduce eventi pin in chiamate al detector.

### 8.3 Dove NON applichiamo

1. **Niente directory `domain/` `application/` `infrastructure/`** — separazione vive dentro ogni modulo (es. `state.rs` vs `softap.rs`).
2. **Niente DI containers** — generics + traits di Rust bastano.
3. **Niente CQRS, Event Sourcing, aggregati DDD** — sproporzionati per la scala del firmware.
4. **Niente refactoring del dominio DCC/Z21 esistente** — fuori scope.

### 8.4 Bilancio costo/beneficio

**Costo aggiuntivo rispetto a un design non-CA:**
- 2 trait nuovi (`ConfigStore`, `Clock`), ~30-40 righe totali
- 2 mock impl (`InMemoryConfigStore`, `MockClock`), solo in `#[cfg(test)]`
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
- **Lista multi-rete** ricordata (una sola rete in NVS).
- **Configurazione IP statica** (DHCP only).
- **HTTPS** con certificati gestiti dal device.
- **Wizard multi-step** nel provisioning.
- **Recupero password admin** via email/SMS (solo factory reset hardware).
- **Firma crittografica del firmware**.
- **Flash Encryption** di ESP-IDF (la password WiFi viene salvata in chiaro).
- **Internazionalizzazione completa** (`it`/`en` come flag salvato ma testi solo italiani nel Livello 1).
- **Telemetria/log persistenti** nella zona `storage` della flash.

---

## 10. Rischi e questioni aperte

Cose che dal design non si possono blindare completamente e che andranno verificate in implementazione:

**R1. Compatibilità di `picoserve` con `esp-radio` in modalità mista.**
`picoserve` lavora sopra `embassy-net`, che parla col driver WiFi di `esp-radio`. Da verificare:
- Funzionamento stabile sia in modalità client che in modalità SoftAP.
- Transizione pulita tra `Provisioning` (SoftAP) e `OperationalConnected` (client) senza stati inconsistenti.

Mitigazione possibile: riavvio software dopo il provisioning, per resettare lo stack di rete in modo pulito.

**R2. Coesistenza WiFi + UDP Z21 + TCP HTTP.**
Aggiungendo TCP HTTP allo stack esistente di UDP Z21, da verificare il budget di socket di `embassy-net` (N socket TCP + M socket UDP, con N+M nei limiti supportati).

**R3. Tempi di hashing della password.**
Bcrypt sull'ESP32-C6 va misurato. Cost factor 8 è un'ipotesi iniziale. Taratura empirica per bilanciare UX di login (deve essere < 500 ms) e difesa anti-bruteforce (deve essere > 10 ms).

**R4. Limiti di scrittura della flash NVS.**
La flash ha cicli di scrittura finiti (~100k per cella). Per le credenziali WiFi non è un problema (cambi rari). Da considerare se in futuro aggiungeremo logging persistente.

**R5. Pinmap GPIO0.**
Da verificare che GPIO0 sia disponibile (non usato da nessun'altra funzione futura) e che il pull-up interno sia sufficiente. Il device verrà programmato/flashato via USB anche dopo l'introduzione di GPIO0 come tasto — verificare nessuna interferenza con la procedura di flash (l'ESP32-C6 usa USB-Serial-JTAG nativo, non condivide pin con GPIO0).

**R6. Dimensione effettiva della flash.**
Confermare con `espflash board-info` che il modulo in uso ha 8 MB. Layout alternativo per 4 MB da preparare in subordine.

---

## 11. Riferimenti

- NMRA Standards `docs/specs/NMRA-STANDARDS.md` (compatibilità DCC esistente — non impattata da questo design)
- Z21 LAN Protocol `docs/specs/z21-lan-protokoll-en.pdf` (compatibilità Z21 esistente — non impattata)
- ESP-IDF Partition Tables — documentazione ufficiale Espressif
- Bcrypt RFC — funzione di hashing scelta per password admin
- RFC 6585 (HTTP 429 Too Many Requests) — per il rate limiting di login
- OWASP Authentication Cheat Sheet — riferimento generale per le scelte di sicurezza

---

**Documento approvato dall'utente?** *(da segnare quando l'utente conferma la review)*
