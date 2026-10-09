# Aggiornamento firmware via Wi-Fi — design

Stato: revisione 3, convergenza dopo tre round di review avversariale;
implementazione locale verificata con test host/build/browser. Nessuna prova
su dispositivo ancora eseguita, nessuna pubblicazione o installazione.
La convergenza riguarda il design, non una certificazione OTA.

## Decisioni e limiti

- ESP32-C6 con **8 MB**, come dichiarato dal proprietario; verifica con
  `espflash board-info` prima del primo flash. Non aggiungiamo altri layout.
- Firmware **Rust no_std + esp-hal**, esp-radio, Embassy ed esp-storage.
  Nessuna applicazione o build ESP-IDF. Si conserva il bootloader binario
  precompilato di espflash, fissandone versione e checksum.
- Pagina HTML/CSS/JS incorporata nel firmware, accessibile dal browser del
  telefono sulla stessa LAN: `/update`. Nessuna app aggiuntiva.
- File firmato `.dccfw`, scaricato dal telefono e poi **caricato** sulla
  centrale. La Z21 app resta il comando dei treni, non l'updater.
- Due slot A/B. Scrittura esclusivamente nello slot non eseguito. Rollback
  applicativo dopo fallimento dell'health check, panic o reset durante prova.
- Nessun downgrade OTA, aggiornamento automatico, TLS, scrittura OTA di
  bootloader/partition table o modifica eFuse. Secure Boot e Flash Encryption
  sono interventi separati, irreversibili e non autorizzati da questo design.
- **Limite esplicito:** con il bootloader standard senza rollback, un guasto
  prima del controllo applicativo (startup Rust, esp_hal::init, inizializzazione
  heap, costruttore FlashStorage) può richiedere USB. Non possiamo promettere
  rollback di qualunque firmware guasto senza un bootloader che lo gestisca.
- La firma protegge l'autenticità del file, non autorizza l'operatore.
  Un host ostile sulla LAN può fermare i treni via Z21 e causare un DoS o
  installare una release pubblica firmata più recente. Questa limitazione è
  accettata per una LAN fidata, senza pulsante fisico aggiuntivo; limiti di
  tempo e rate limit riducono gli abusi, non costituiscono autenticazione.

## Esperienza utente

1. Scarica la release `.dccfw` sul telefono/PC.
2. Spegne il binario dall'app Z21 o dal pulsante Stop.
3. Apre `/update` all'indirizzo IP mostrato dalla centrale.
4. Sceglie il file: controlli preliminari, versione nuova e problemi in italiano.
5. Tocca **Aggiorna**: upload %, poi **Verifica della memoria**, **Riavvio**,
   **Verifica del nuovo firmware**.
6. Esito: aggiornato e confermato, oppure versione precedente ripristinata.
   Il binario rimane spento anche dopo il riavvio: per ripartire serve una
   nuova azione Resume/Z21, mai una riaccensione ritardata automatica.

`xhr.upload.onprogress` mostra bytes trasmessi dal browser, **non** bytes
persistiti in flash. Il 100% di invio non significa installazione riuscita:
verifica e conferma hanno stati distinti. Il progresso di scrittura, se mostrato,
proviene da `/update/status`, non dal contatore XHR.

## Evidenze che condizionano il progetto

- `Cargo.toml`: esp-hal ~1.0, esp-storage 0.8.1,
  esp-bootloader-esp-idf 0.4.0, esp-radio 0.17, embassy-net 0.8.
- Il crate esp-bootloader-esp-idf è già usato da `esp_app_desc!()` e dal parser
  partizioni: compatibilità di formato, non dipendenza dal runtime IDF.
- espflash 4.3.0 contiene il bootloader C6 derivato da IDF
  `v5.5.1-838-gd66ebb86d2e`, senza `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`.
  Il formato otadata contiene due entry: seq, label[20], stato e CRC(seq).
  Il bootloader usa la seq più alta con CRC corretto e stato diverso da
  INVALID/ABORTED; slot = `(seq - 1) % 2`. Non cambia NEW in PENDING_VERIFY.
- Il bootloader può provare altre immagini se quella scelta non è caricabile,
  **anche se la loro entry era INVALID**. Senza factory e con otadata vuota
  prova ota_0; può poi inizializzare una entry VALID senza tag applicativo.
  Fonte: `bootloader_common_loader.c` e `bootloader_utility.c` a quel commit.
- `OtaUpdater` 0.4.0 non è adatto al contratto: scelta con otadata vuota/no
  factory problematica, stato e selezione scritti separatamente, lettura seq
  senza filtrare stati/CRC. Riutilizziamo parser partizioni, booted_partition,
  formato immagine, enum stati; codec otadata locale minimo e testato.
- `FlashRegion::erase` 0.4.0 rifiuta il bound esclusivo pari alla fine regione
  (`partitions.rs:677`); bug già descritto nella spec Wi-Fi di questo repo.
  OTA usa accessi assoluti a FlashStorage con range validati, non quell'erase.
- `esp-storage/src/hardware.rs`: erase/write dentro critical section. RMT,
  cutout e short detector vengono ritardati. Il tempo va **misurato** sul
  dispositivo; non assumiamo che Wi-Fi e DCC sopravvivano a qualunque erase.
- `src/dcc/engine.rs`: watchdog software ISR 50 ms. Non è dimostrato che ogni
  erase lo attivi: l'ISR può aggiornare heartbeat prima che riparta l'executor.
- `src/boot/readiness.rs`: Net ack consumato/ignorato; non duplicarne receiver.
- `src/fault_manager.rs`: candidate policy viene applicata assumendo successo
  del setter GPIO; rifiuto OTA deve propagarsi alla policy e agli effetti.
- `FlashStorage::new` legge header con `unwrap()`; non è un fallibile init.
  `esp_hal::init` disabilita i watchdog. Il rischio iniziale resta documentato.

## Partizioni e migrazione

```text
# Name,      Type, SubType, Offset,    Size
nvs,         data, nvs,     0x9000,    0x4000
otadata,     data, ota,     0xD000,    0x2000
phy_init,    data, phy,     0xF000,    0x1000
ota_0,       app,  ota_0,   0x10000,   0x300000
ota_1,       app,  ota_1,   0x310000,  0x300000
dcc_cfg,     data, nvs,     0x610000,  0x3000
ota_journal, data, undefined, 0x613000, 0x2000
```

Una volta via USB: verificare 8 MB, flash bootloader fissato + tabella +
firmware, cancellare **otadata, ota_journal e ota_1**, riconfigurare Wi-Fi perché
dcc_cfg cambia indirizzo. nvs attuale non viene usato dai nostri stub radio.
Il primo avvio può entrare nel setup AP: non è un boot OTA pending.
Factory non serve: conserviamo il precedente slot confermato. Il layout non
garantisce recupero da guasti fisici di entrambe le immagini o della flash.

Bootloader copiato dalla release espflash 4.3.0 con provenienza, licenza e
SHA-256 in `bootloader/README.md`, senza ricompilarlo. Ogni runner passa
`--bootloader`, `--partition-table`, `--erase-parts otadata,ota_journal,ota_1`
(etichette separate da virgole, espflash 4.3.0). USB riscrive ota_0 e distrugge
storia e fallback; cancellare ota_1 evita un fallback obsoleto dopo il reset
del journal. Non usare questa cancellazione durante un update OTA.

Correzione verificata in implementazione: espflash 4.3.0 / esp-idf-part 0.6.0
va in panic con subtype data custom 0x40. Si usa `data,undefined` (0x06),
supportato da entrambi i parser e ignorato dal bootloader. Indirizzi, codec
e dimensioni restano uguali; il journal non è NVS né otadata.

## Registro durabile e selezione del boot

Otadata determina **dove** parte il bootloader, non se il firmware abbia
superato i nostri controlli. La fonte di verità applicativa è `ota_journal`.
Serve perché il CRC otadata copre solo seq: scritture interrotte possono
lasciare stato VALID senza verifica e possono cancellare la storia rollback.

Due settori journal, un record per settore, alternati. Record fisso versionato:
magic, schema, generazione u64, fase, immagine corrente confermata
{slot, versione, lunghezza, SHA-256}, eventuale candidata {stessi campi},
eventuale precedente confermata, esito ultimo tentativo e versione rifiutata,
flag persistente hold_track_off, CRC del corpo e parola commit. Campi opzionali
hanno flag canonici. hold_track_off viene impostato in RECEIVING e conservato
anche in conferma/aborto/rollback: ogni boot successivo richiede Resume fresco,
anche se una precedente sessione aveva riavviato i treni. Non lo si cancella
con una scrittura flash mentre il binario è acceso.
Il codec schema 2 usa 172 byte: magic `DCC-JNL2` a 0..8, schema u16 a
8..10, fase/esito/hold/opzioni a 10..14, zero a 14..16, generazione a
16..24; identità di 44 byte a 24, 68 e 112. La versione massima rifiutata
occupa 156..162 (tre u16 LE), zero a 162..164; bit 0x04 delle opzioni
ne indica la presenza. CRC a 164..168, commit a 168..172. Non viene persa
sostituendo la candidata o abortendo un trasferimento: via OTA sono ammesse
solo versioni maggiori sia della corrente sia di questa soglia. Fallimenti
di trasferimento non alzano la soglia; un successivo boot fallito la alza,
mai la abbassa. Lo schema 1 sperimentale non è distribuito: eventuali
schede di prova con quel journal richiedono recupero USB, non una migrazione
che inventi la storia delle release fallite.

Scrittura: erase del settore non più recente, corpo e CRC, read-back del corpo,
poi parola commit `0x00000000` programmata per ultima (4 byte allineati).
Accettare solo commit esatto, schema noto e CRC corretto. Il record precedente
rimane intatto fino al commit; generazione senza wrap (rifiutare overflow).
Un commit parziale non esatto non conta; se risulta esatto, il corpo era già
scritto e letto prima del commit. CRC non protegge da avaria hardware arbitraria.

Fasi:

| Fase | Significato e stato sicuro al riavvio |
|---|---|
| CONFIRMED | Corrente verificata; eventuale precedente è solo fallback |
| RECEIVING | Corrente intatta, candidata non attivabile; aborto al riavvio |
| READY | Candidata integralmente scritta e hash verificato; non ancora provata |
| TRYING | Un boot di prova è iniziato; un altro reset prima di conferma fallisce |
| ROLLED_BACK | Corrente precedente ripristinata, candidata rifiutata |

L'identità è SHA+lunghezza, non solo versione. Un vecchio tag VALID otadata non
può confermare una nuova immagine scritta nello stesso slot.

Codec otadata: CRC e selezione esattamente come bootloader, seq > 0 e non
0xFFFFFFFF, nessun wrap. Scrivere entry per slot richiesto nel settore non
attivo, con seq più alta mappata al target. Corpo/stato prima, CRC(seq) **ultimo**.
Read-back prima del reset. Se non basta una seq disponibile, rifiutare OTA.
Il journal, non un CRC privato in label, determina pending e validità.

## Algoritmo di avvio e rollback

All'ingresso di boot::run: GPIO18 disabilitato, FLASH e LPWR acquisiti,
FlashStorage una volta, lettura journal/otadata, slot **effettivamente booted**.
Hash dell'immagine booted quando serve a confrontarla con un record: letture
in blocchi con yield dopo avvio runtime, mai armare nel frattempo.
Nessuna operazione flash mentre il binario è alimentato.

Ordine e politica totali:

1. Journal entrambi completamente erased: bootstrap USB. Hash/identità del
   booted slot, creare CONFIRMED e selezione otadata coerente. Setup Wi-Fi
   normale consentito. Se uno contiene dati ma nessun record valido: **recovery
   sicura**, non bootstrap; non confermare automaticamente.
2. CONFIRMED/ROLLED_BACK: booted corrisponde alla corrente → boot normale.
   Se il bootloader è caduto sulla precedente, verificarne hash e promuoverla
   come corrente, marcare l'altra non bootable e riparare otadata. Altrimenti
   se la corrente è comunque verificabile, riparare selezione e resettare su
   corrente (caso cut dopo commit rollback ma prima della riparazione otadata);
   se non è verificabile, recovery sicura. Nessuna promozione basata su VALID.
3. RECEIVING: se booted è corrente, cancellare magic del candidato se necessario
   e registrare aborto; altrimenti registrare aborto, selezionare corrente
   verificata e resettare senza toccare l'immagine attualmente eseguita.
4. READY: booted corrente → trasferimento pronto ma attivazione interrotta:
   abortire candidato, restare sulla corrente (non riprovare automaticamente).
   Booted candidata e hash esatto → commit TRYING **prima** di iniziare la
   prova. Qualunque altra immagine → recupero corrente verificata.
5. TRYING al nuovo boot: precedente prova interrotta → rollback alla corrente
   confermata. Mai ripetere automaticamente la prova né invalidare la corrente.

In qualunque fase vietato cancellare il magic o scrivere lo slot booted:
cleanup della candidata eventualmente eseguita si rimanda al boot della
corrente. Le modifiche journal/otadata non toccano app code. hold_track_off
viene applicato anche a bootstrap/boot ordinario secondo il valore del record,
non solo al boot di prova, preservando Stop attraverso un rollback/reset.

Rollback: verificare hash della corrente confermata prima di selezionarla,
commit ROLLED_BACK, riparare otadata puntando corrente e resettare. Se corrente
non è leggibile/verificabile: recovery sicura, non loop di reset. Se booted è
già corrente, riparare senza reset ulteriore. L'altra entry/immagine non è
fallback valido solo perché idf-valid: deve corrispondere a un'identità
confermata nel journal e non essere marcata non bootable.

Recovery sicura: GPIO18 LOW, nessun TrackPowerArmed, niente write/upload OTA,
display/log chiari, HTTP status read-only se rete disponibile, indicare USB.
Errori di lettura, scrittura, schema sconosciuto o booted_partition assente non
diventano VALID e non consentono binario. Se scritture boot-check falliscono,
non fare fail_fast in loop: restare in recovery. Guasti antecedenti all'init
restano il limite USB descritto sopra.

TRYING corrente RAM entra in health check; TRYING letto al boot successivo
entra in rollback. Prima di entrare in provisioning durante prova, rollback:
rete/config non verificabile non va confermata. Durante prova long-press setup
viene ignorato con messaggio. Con journal bootstrap/CONFIRMED il setup resta
come oggi.

## Health check

RWDT esp-hal stage reset 10 s, feeder Embassy ogni 1 s **dopo** avvio runtime,
prima di attese pulsanti/rete; nessuna pretesa di priorità task non supportata.
Confermare prova solo quando:

- boot critico completato senza armare binario;
- link e DHCP attualmente validi, Z21 socket bind riuscito;
- HTTP listener funzionante;
- self-test non distruttivo: range, journal/otadata, lettura slot inattivo,
  validità della chiave, parser/package fixture;
- durante 60 s continuativi, heartbeat recenti dei task critici (DCC,
  scheduler, fault manager, rete/Z21, HTTP). Loop idle usano timer/select per
  aggiornare heartbeat, non serve traffico né app Z21 collegata.

Qualunque perdita requisito azzera la finestra. Limite totale 180 s dal boot;
quindi tutti i requisiti devono essere pronti entro 120 s. Questa soglia è
voluta: una rete troppo lenta produce rollback, come richiesto dal proprietario.
Heartbeat non dimostra correctness di ogni ramo; prove N→N+1→N+2 restano
necessarie. Blocchi totali attivano RWDT; stalli task singoli impediscono conferma.

Conferma: commit CONFIRMED per candidata con precedente conservata, poi
riparare stato otadata VALID. Mantiene binario in E-stop. Arma la possibilità
di Resume con `TrackPowerArmed` solo dopo E-stop applicato/acknowledged;
abilitazione fisica richiede una **nuova** richiesta dell'operatore. RWDT prova
si disabilita dopo commit; eventuale errore otadata → recovery, non alimentare.
Timeout → rollback. Cut durante conferma prima commit → rollback; dopo commit
→ candidata già verificata, boot ripara selezione.

## Concorrenza, ownership e sicurezza binario

Una sola FLASH owner: mutex statico
`embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Option<FlashStorage<'static>>>`,
inizializzato runtime. Boot, store Wi-Fi, provisioning, conferma e upload
prendono async mutex per operazione, mai critical section estesa su un'intera
hash/attesa. Journal, otadata e writer usano accessi NOR a offset assoluti:
geometria/capacità controllate, write allineate a 4 byte, erase a settori di
4096 byte entro i rispettivi intervalli. Non serve un adapter PartitionRange.
NOR write per streaming, non Storage::write che può riscrivere un settore
per ogni chunk. La policy di ammissione e la proiezione JSON dello stato sono
pure e testabili su host; atomiche, ACK e operazioni HAL restano nel runtime.

Lock OTA unico: check/upload acquisiscono solo quando boot è concluso,
journal disponibile, non pending/recovery, track off. Header completo entro
2 s prima di lock; verify firma dopo lock. Mutex flash non è lock OTA.

Fault manager è proprietario dell'interlock funzionale. Prima di modificare
flash: mantenere lock fisico, inviare StopPressed e attendere ack che policy
è E-stop e fence drenata. Resume/network-resume/clear-fault che riattivano sono
rifiutati durante lock **prima** di candidate policy ed effetti scheduler;
Stop e veri fault restano sempre processati. Rifiuto propagato nel setter GPIO
con risultato applied/refused, anche nell'Option di apply_if_current. Store
TRACK_ENABLED e check OTA nella stessa critical section dell'enable GPIO.
Non scartare TrackPowerArmed: acquisizione OTA bloccata fino a completamento
arming; in pending la conferma applica E-stop prima di arming.
Sblocco non fa Resume: policy rimane E-stop.

Runtime provisioning ≥10 s durante upload: rifiutare finché lock attivo,
non scrivere flag/reboot concorrente. Anche le scritture Wi-Fi ordinarie
seguono bridge-off, policy/fence e mutex. Journal mutabile solo dall'upload o
conferma, mai entrambi. Nessun flash guard oltre await rete.

RMT watchdog: durante lock con bridge off, su timeout resta off, evita log
per-poll/loop a deadline zero, yield con timer e rate-limit log; non resetta
per il solo ritardo flash. Fuori lock comportamento attuale invariato.
RWDT durante upload copre un executor bloccato; reset lascia RECEIVING e
ricade sulla corrente. Interruzioni radio durante erase richiedono prova
hardware: TCP può recuperare, non lo assumiamo verificato. Se impraticabile,
si passa a ramo update separato prima di consegnare l'implementazione.

## Upload e scritture

Due buffer statici 4096 byte (settore zero trattenuto, settore corrente),
SHA incrementale e hash flash in chunk con yield. RAM budget verificato dal
linker/build; attualmente heap allocatore 64 KiB. Nessun buffer immagine intera.

1. HTTP framing e header; lock + firma/versione ri-verificati, E-stop ack.
2. Target non booted. Trattenere settore zero e controllare header/app descriptor.
3. Commit RECEIVING con candidata e corrente, **prima** di modificare target.
   Rimuovere qualsiasi entry otadata relativa al target senza toccare selezione
   corrente. L'esclusione target dall'eventuale precedente confermata fa parte
   dello stesso commit RECEIVING, non è una scrittura separata successiva.
4. Erase settore zero target per renderlo non bootable; poi settori successivi
   erase/write, ultimo chunk padding FF solo per allineamento (hash solo len).
5. Hash stream su **tutta immagine incluso settore zero** == hash firmato.
6. Hash read-back usando settore zero ancora in RAM + resto flash == hash.
   Solo adesso scrivere settore zero, leggerlo e confrontarlo integralmente.
7. Commit READY. Se commit fallisce cancellare magic target e recovery.
8. Scrivere/rileggere otadata NEW per candidata, lasciando corrente intatta.
9. Tentare risposta, ma **reset obbligatorio** anche se TCP cade; mantenere
   lock fino al reset. Nessuna gestione errori generica che liberi lock dopo
   activation o consenta un secondo upload.

Prima di READY, errori lasciano corrente selezionata; pulire magic target se
possibile e registrare aborto. Fallimento cleanup/metadata → recovery con
binario off. READY e flash target bootable sono entrambi autenticati dal hash;
un cut prima READY lascia RECEIVING che rifiuta target anche se bootloader lo
scansiona come fallback. Un firmware ostile non firmato non riceve mai settore
zero. Limite: flash che mente su read/write o CRC collisione arbitraria non è
una garanzia coperta senza hardware trusted/Secure Boot.

## Pacchetto firmato

256-byte header LE + app image binaria non merged (`espflash save-image`).

| Offset | Size | Campo |
|---|---|---|
| 0 | 8 | magic DCC-OTA1 |
| 8 | 2 | header_len = 256 |
| 10 | 2 | flags = 0 |
| 12 | 2 | chip_id = 0x000D |
| 14 | 2 | riservato zero |
| 16 | 6 | major/minor/patch u16 |
| 22 | 2 | riservato zero |
| 24 | 4 | image_len |
| 28 | 4 | riservato zero |
| 32 | 32 | SHA-256 immagine completa |
| 64 | 32 | project_name NUL-padded dcc-esp32 |
| 96 | 8 | key_id = SHA-256(pubkey)[0..8] |
| 104 | 88 | riservato zero |
| 192 | 64 | firma Ed25519 su bytes [0,192) |

Ed25519-dalek no_std verify_strict e sha2. I CRC di journal e otadata sono
algoritmi manuali nei codec locali, non una dipendenza dal crate `crc`.
Benchmark e stack host non sostituiscono misure sul dispositivo.
Chiave id, firma, chip, nome, lunghezza 4096..slot_size, versione strettamente
maggiore della corrente. Non riprovare versione registrata come rollback:
correzione = release superiore alla soglia persistente, oppure USB. Content-Length check 256+len solo
su upload, `/check` richiede esattamente 256 bytes. Header immagine magic E9,
chip C6, dimensione flash 8 MB; app descriptor offset 32 magic ABCD5432,
nome/versione uguali al manifest. Tool verifica anche segmenti, checksum e
digest ESP, immagine non merged. Versione solo major.minor.patch canonico.

Firma locale privata fuori repo. Pubkey incorporata in sezione identificabile
con marker, insieme a build_kind release/dev/fault. `ota-pack sign` rifiuta
release signing di dev/fault e controlla pubkey immagine == pubkey firmatario.
Rotazione esplicita `--rotate-to PUBKEY`, controllando nuovo embedded key.
Dev key privata fuori repo, usata solo su dispositivo bench con firmware dev.
CI di verifica key fingerprint e review su keys/; niente firma automatica CI
o pubblicazione senza autorizzazione. Unknown key → messaggio generico, nessuna
versione ponte inventata; release notes documentano percorso rotazione.

## HTTP, tempi e UI

Due socket server port 80, 4 KiB RX + 1.5 KiB TX ciascuno, risorse stack da 3
a 5. Secondo socket può rispondere status/busy durante upload; non promettiamo
capacità illimitata (terzo client può ricevere RST). Keep-alive disabilitato,
favicon data: incorporata, niente asset esterni. Head massimo 1024 bytes,
timeout assoluto head+package-header 2 s. Framing: unico Content-Length decimale
limitato, rifiutare Transfer-Encoding, duplicati, overflow e body oltre len.
Route note soltanto; niente parsing form/multipart: XHR manda Blob grezzo.

POST richiede X-DCC-OTA: 1; OPTIONS rifiutato senza CORS headers. Host unicamente
IP corrente con opzionale :80; Origin, quando presente, canonicamente
`http://<ip>` (default port normalizzata), mai dominio DNS. Questo protegge
browser da CSRF/rebinding, non autentica host LAN. Nessun innerHTML di dati
dispositivo; JSON escaped e UI textContent.

GET /update; GET /update/status; POST /update/check; POST /update.
Status schema 1 stabile tra release: boot_id casuale per avvio, uptime,
versione, slot, pending, ota_available, uploading, received_bytes,
written_bytes, total_bytes, fase, track_enabled, ultimo tentativo
{candidate_digest, candidate_version, result, current_version, reason}.
Esito durabile dal journal; bootstrap = none, conferma = ok, rollback =
rolled_back, errore caricamento = not_bootable, trasferimento troncato = aborted.

Upload: timeout inattività 10 s, limite assoluto 5 min, minimo 1 KiB/s
misurato su finestre 15 s includendo flash (target realistico da verificare).
Rifiuti firma rate-limited (un check/sec), fallimenti trasferimento backoff
30 s. Errori prima erase non tengono lock mentre si drena il body.
Errori upload: dopo cleanup, lock rilasciato, drain al massimo 30 s e body
limitato; se non finisce abort TCP, UI generic network error, non promettere
sempre un messaggio JSON dopo un disconnect.

Codici strutturati: track_on, boot_not_ready, pending_verify, busy,
ota_unavailable, bad_package, unknown_key, bad_signature, same_version,
downgrade, rejected_version, too_large, bad_image, digest_mismatch,
flash_error, timeout, forbidden. Testi italiani con azione riprova/USB/Stop.

La pagina serializza richieste, GET retry limitato. Dopo upload 100% mostra
verifica; dopo 200 **o** errore rete successivo al completamento invio interroga
status. Esito conclusivo solo con boot_id diverso da iniziale, pending=false
e digest tentativo corrispondente al file scelto. Se non c'è match dice esito
non determinabile, non attribuisce un vecchio rollback al file attuale.
Timeout 300 s dopo fine upload → controlla display/IP/router; DHCP può cambiare
IP. Senza risposta non può sapere se ha funzionato. Nessun polling interpretato
come successo prima del reboot.

## Tooling e verifiche richieste

`tools/ota-pack` host crate, dipendente solo dai moduli puri. Root alias deve
specificare **--target host**; config annidata da sola non viene letta da Cargo
invocato alla root. CI build/test/fmt/clippy tool espliciti, fixture bin reale.
Build release + `espflash save-image --chip esp32c6 --flash-size 8mb
--partition-table partitions.csv` senza --merge, tool sign/inspect/verify/extract.
Fault injection feature unica con selector env (default none), dev build_kind;
--all-features deve compilare ma NON produrre release firmabile.

Host: journal codec e power-cut ad ogni erase/write/commit; stati otadata
blank/CRC/torn/invalid e fallback bootloader; ogni riga boot algoritmica;
sessione chunk asimmetrici, hash read-back, erase last sector, cleanup failure,
cut prima/dopo READY/TRYING/CONFIRMED. Invarianti: mai write running slot,
mai conferma per sola otadata, mai target RECEIVING usato come corrente,
mai flash con bridge on, lock fino a reboot, stato policy == stato fisico.
HTTP origine normalizzata, framing/limiti, deadlines, body 256 /check; key
release/dev/fault/rotation; CLI roundtrip; browser upload/errore/reboot/rollback.

Hardware indispensabile prima delivery: artefatto **esatto** `.dccfw` su bench
via OTA (o extract + write-bin raw, non rigenerazione ELF); N→N+1→N+2;
panic/hang/no-wifi/no-z21/stallo HTTP dopo ready; cut durante tutte le fasi;
AP riavvio/ritardo DHCP; Resume/Z21/provisioning concorrenti; erase tempi/radio;
assenza power-on automatico; entry 1/ultimo settore; USB reflash e startup
failure con recupero USB. Test browser HTML locale prima integrazione HW
verifica UI soltanto, non prova scrittura flash né rollback fisico.

Contratti versionati (package, journal, status, dcc_cfg) devono rimanere leggibili
dal precedente firmware di fallback. Incompatibilità formato/layout richiede
USB, non migrazione OTA distruttiva. Implementazione in fasi: prima journal,
boot e interlock con test; poi signing/stream; poi HTTP/UI e prove hardware.

## Esito della review avversariale

Tre round, tre revisori indipendenti per round: boot/flash, sicurezza/protocollo,
integrazione runtime. Nei round 1 e 2 findings verificati hanno portato a:

- non usare OtaUpdater/FlashRegion per le scritture critiche;
- journal durabile anziché dedurre conferma/rollback dal solo stato otadata;
- identità SHA/len e settore zero per ultimo, nessuna promozione di fallback
  mai confermato, recovery sicura quando manca un'immagine nota buona;
- interlock che copre policy, GPIO, provisioning e reboot, con Stop persistente;
- health check continuo con heartbeat, non semplici flag di startup;
- distinzione % invio/esito finale, sessione correlata con digest e boot_id;
- vincoli HTTP, signing release/dev, rotazione esplicita e host CLI corretto.

Il round 3 ha chiuso tutti i findings materiali su contenuto con SHA-256
`00444b419b2ee39fc0a2fdcbb1fc02a4169f1a278b7c3478891d50b72f33910f`
(prima dell'aggiunta di questo verbale e dell'aggiornamento della riga Stato).

| Area | Revisore fresco, round 3 | Verdetto |
|---|---|---|
| Journal/boot/rollback | [Review](https://ampcode.com/threads/T-01a1211c-9d34-7571-a9bb-9ae1a3bea283) | NO MATERIAL FINDINGS |
| Sicurezza/protocollo/UI | [Review](https://ampcode.com/threads/T-01a1211c-a544-709d-b6d5-d6b16e2c503f) | NO MATERIAL FINDINGS |
| Interlock/runtime | [Review](https://ampcode.com/threads/T-01a1211c-aeff-7599-8185-251a0821d432) | NO MATERIAL FINDINGS |

Non tutte le proposte dei revisori sono diventate requisiti: autenticazione
aggiuntiva/pulsante e varianti flash 4 MB sono fuori dallo scope concordato.
Le ipotesi su comportamento radio, tempi erase e watchdog restano prove da
eseguire, non cause dichiarate dimostrate. L'accesso fallibile alla flash prima
dell'inizializzazione non è offerto dal costruttore attuale: è un limite noto,
non una garanzia inventata. Nessun test hardware è sostituito dal consenso.
