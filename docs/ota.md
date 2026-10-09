# OTA ESP32-C6: guida operativa

Implementazione da validare sul dispositivo: **nessuna prova hardware OTA è
qui dichiarata superata**. Contratti e limiti: [design](specs/ota-update.md).
Solo scheda ESP32-C6 con flash fisica **8 MB**, due slot da 3 MB; niente layout
4 MB. OTA modifica lo slot inattivo e i metadati, non bootloader/tabella/eFuse.

Test host/CLI, roundtrip sign → verify → extract e controlli browser verificano
logica e formato, non flash/radio reali. Prima dell'uso operativo completare
la checklist hardware in fondo alla guida sull'artefatto esatto da installare.

## Prima installazione o recupero USB (distruttivo)

Tenere il binario spento, scollegare i carichi se necessario e verificare
l'hardware secondo [wiring](hardware/wiring.md). Installare la versione fissata:

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cargo install espflash --version 4.3.0 --locked
espflash --version
espflash board-info                 # deve riportare flash fisica 8 MB
sha256sum bootloader/esp32c6-bootloader.bin
cargo run --release
```

Il runner fissa bootloader, `partitions.csv`, 8 MB e `ota_0`, e cancella
`--erase-parts otadata,ota_journal,ota_1` (etichette separate da **virgole**).
In espflash 4.3.0 `erase_partitions` cerca l'etichetta senza filtro sul tipo:
anche lo slot **app** `ota_1` è supportato; vedi
[sorgente upstream](https://github.com/esp-rs/espflash/blob/v4.3.0/espflash/src/cli/mod.rs).
Va cancellato per evitare che il bootloader provi un'immagine inattiva obsoleta:
un journal appena cancellato non conosce alcun fallback confermato.
Il subtype del journal è `data,undefined`: il subtype custom `0x40` provoca
panic nel parser di espflash 4.3.0. Non modificare la tabella per aggirarlo.

**Ogni `cargo run` via questo runner è migrazione/recupero USB**, non OTA:
distrugge storia e fallback. Dopo migrazione del layout, `dcc_cfg` cambia
indirizzo: riconfigurare Wi-Fi dall'AP `DCC-Setup-XXXX`, password `dcc-setup`,
pagina `http://192.168.4.1`. Non cancellare journal/otadata durante OTA.
Il primo bootstrap USB non è una prova pending del nuovo firmware.

## Aggiornare dal telefono/PC

1. Scaricare la release `.dccfw` corretta. La versione deve essere maggiore
   della versione corrente e della più alta versione rifiutata dopo rollback.
   Questa soglia resta memorizzata anche dopo upload interrotti: serve una
   release superiore o USB. La baseline iniziale è **0.1.0**: un pacchetto 0.1.0
   non aggiorna una centrale già 0.1.0 (per esempio la prossima sarà 0.1.1).
2. Spegnere il binario con Stop o Z21. Aprire `http://<IP-della-centrale>/update`
   sulla stessa LAN fidata, usando l'IP mostrato dall'OLED, non un nome DNS.
3. Scegliere il file, attendere i controlli e premere **Aggiorna**.
4. La percentuale indica **upload dal browser**, non bytes persistiti in flash.
   Il 100% non significa successo: attendere verifica memoria, riavvio e
   verifica del nuovo firmware. Non chiudere la pagina solo perché l'invio finisce.
5. L'esito deve corrispondere al digest del file e a un nuovo boot, senza pending:
   confermato oppure versione precedente ripristinata. Un errore di rete dopo
   l'invio non prova un fallimento; la pagina controlla lo stato. Se l'esito resta
   indeterminabile, controllare display/router/IP (DHCP può cambiarlo), poi USB
   se indicato. Non attribuire al file scelto un vecchio esito del journal.
6. **Il binario resta spento** dopo aggiornamento, errore o rollback. Per ripartire
   serve un nuovo Resume/Z21; nessuna riaccensione automatica ritardata.

Durante upload/prova Resume e richieste concorrenti che alimentano il binario
sono rifiutate; Stop e fault restano attivi. Anche la riconfigurazione Wi-Fi
non deve riavviare la centrale durante upload; durante prova non entra in setup.
Conferma richiede rete/Z21/HTTP e heartbeat sani per 60 s continuativi, con
limite complessivo 180 s. Una rete troppo lenta può causare rollback.

Il journal attuale è schema 2 (172 byte), con soglia rifiutata indipendente
dall'ultimo trasferimento. Le versioni sperimentali con journal schema 1
richiedono recupero USB; non sono compatibili con questa OTA. In recovery
l'OLED mostra `OTA: ripristino USB` anche se l'avvio della rete non termina.

## Creare una release (operatore autorizzato, locale)

Incrementare prima la versione canonica `major.minor.patch` in `Cargo.toml`.
Non usare `--all-features`: include dev/fault, non una release firmabile.
Usare un nome output nuovo: il tool rifiuta di sovrascrivere file.

```bash
cargo build-esp-release
mkdir -p target/ota
espflash save-image --chip esp32c6 --flash-size 8mb \
  --bootloader bootloader/esp32c6-bootloader.bin \
  --partition-table partitions.csv --target-app-partition ota_0 \
  target/riscv32imac-unknown-none-elf/release/dcc-esp32 target/ota/app.bin
cargo ota-pack sign --key ~/.config/dcc-esp32/ota_release.key \
  --image target/ota/app.bin --out target/ota/release.dccfw
cargo ota-pack inspect target/ota/release.dccfw
cargo ota-pack verify target/ota/release.dccfw --public keys/ota_release.pub
cargo ota-pack extract target/ota/release.dccfw --out target/ota/extracted.bin
cmp target/ota/app.bin target/ota/extracted.bin
```

**Mai `--merge`**: il pacchetto contiene header firmato di 256 byte e solo
l'immagine app ESP, non bootloader/tabella né un dump della flash. `sign`
controlla chip C6, header 8 MB, segmenti/checksum/digest ESP, descriptor nome
`dcc-esp32` e versione, build kind e chiave incorporata. La dimensione 8 MB
nel file è solo un header: né il parser né un roundtrip host dimostrano la
dimensione del chip fisico (`board-info` è necessario).
`inspect` mostra la struttura ma **non autentica**; `verify` autentica firma,
hash e immagine ma non conosce la policy versione della centrale; `extract`
controlla integrità ma richiede `verify` separato per autenticità.

Il comando root `cargo ota-pack` fissa il target host x86_64 Linux, evitando
il target embedded predefinito. Altro host: invocare il manifest del tool con
`--target` esplicito appropriato. [Custodia e rotazione chiavi](../keys/README.md).
La CI controlla fmt/test/clippy tool, fingerprint e bootloader, converte la
release reale e prova sign/inspect/verify/extract con una **chiave temporanea**
e `--rotate-to keys/ota_release.pub`. Quel file è solo fixture, non installabile
con la chiave produzione e non pubblicato. Nessuna firma release automatica.

## Bench e fault injection

Solo dispositivo bench con chiave dev già installata via USB:

```bash
OTA_FAULT=panic cargo build-esp-release --features ota-fault-inject
# Convertire l'ELF come sopra, poi usare seed dev e --development:
cargo ota-pack sign --key ~/.config/dcc-esp32/ota_dev.key --development \
  --image target/ota/app.bin --out target/ota/bench.dccfw
```

Riconvertire sempre l'ELF appena compilato, senza riutilizzare un vecchio app.bin.
`OTA_FAULT` è selezionato **a compilazione**, solo con `ota-fault-inject`:
`none` (default), `panic`, `hang`, `health-timeout`. La feature include
`ota-dev-key`; `ota-dev-key` da sola produce build dev, `ota-fault-inject`
produce build fault anche con selector none. Tornare a build default e seed
release per produzione; `--development` non autorizza una release pubblica.

## Limiti di sicurezza e recupero

Non ci sono TLS, autenticazione dell'operatore LAN, Secure Boot o Flash Encryption.
La firma autentica **il file**, non l'host che lo invia: un host ostile nella LAN
può fare DoS/Stop o installare una release pubblica firmata più recente.
CSRF/rebinding e rate limit non sono autenticazione. Usare una LAN fidata.
Il bootloader stock non garantisce rollback di qualsiasi crash: guasti prima
del controllo OTA (startup Rust, init HAL/heap/FlashStorage o executor non avviato)
possono richiedere USB. Recovery mantiene binario off e può esporre solo status
se la rete è disponibile; non promettere riparazione OTA in recovery.
Corruzione hardware di entrambi gli slot non è recuperabile tramite fallback.

## Checklist hardware ancora da eseguire

- [ ] Installare l'artefatto **esatto** `.dccfw` via OTA, N → N+1 → N+2, alternando
  entrambi gli slot; nessuna rigenerazione ELF per simulare il pacchetto.
- [ ] Power cut a erase/write/commit, RECEIVING/READY/TRYING/CONFIRMED/rollback;
  entry otadata 1, ultimo settore, abort/disconnect e cleanup failure.
- [ ] Panic, hang, health-timeout; perdita Wi-Fi/DHCP, Z21/HTTP/heartbeat,
  riavvio AP e rete lenta; verificare conferma/rollback e identità journal.
- [ ] Resume fisico, Resume Z21, clear-fault e long-press setup concorrenti;
  nessun impulso GPIO/power-on, nessun reboot/config write durante upload.
- [ ] Misurare tempi erase e latenza radio/TCP/watchdog sul dispositivo;
  non assumere che la radio sopravviva a ogni critical section flash.
- [ ] Verificare binario sempre off fino a nuova azione, IP cambiato e UI
  indeterminata, recupero USB con ota_1 obsoleto e guasti startup pre-OTA.

Registrare versione, hash pacchetto, scheda, alimentazione e misure per ogni
prova. Test host/browser e CI non certificano flash, rollback o sicurezza fisica.
