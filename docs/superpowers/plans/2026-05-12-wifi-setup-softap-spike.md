# WiFi Setup SoftAP Spike — Piano Figlio

**Data:** 2026-05-12
**Parent design:** `docs/superpowers/specs/2026-05-12-wifi-provisioning-web-panel-design.md`
**Status:** Draft

---

## Obiettivo

Dimostrare su ESP32-C6, con lo stack reale `esp-hal` / `esp-radio` / `embassy-net`, che il device può entrare in modalità setup:

1. DCC spento.
2. H-bridge disabilitato.
3. Rete WiFi `DCC-Setup-XXXX` visibile.
4. Telefono collegabile alla rete.
5. Pagina HTTP minima raggiungibile da browser.

Questa fase non implementa salvataggio credenziali, login, pannello admin, factory reset software o OTA.

---

## Perché iniziare da qui

Il rischio principale del design non è la pagina web: è verificare che SoftAP, stack IP e HTTP funzionino bene nel firmware `no_std` attuale senza rompere il controllo DCC.

Se questa prova non funziona, tutto il resto del provisioning resta teorico.

---

## Scope

### In scope

- Separazione iniziale tra owner della rete e servizio Z21.
- Modalità setup forzata via stub/fake config assente.
- SoftAP WPA2 con SSID `DCC-Setup-XXXX`.
- PIN temporaneo generato o, per lo spike, costante compilata chiaramente come temporanea.
- IP device `192.168.4.1` se supportato dallo stack.
- HTTP minimale: pagina statica con testo `DCC Setup`.
- DCC e H-bridge non avviati in modalità setup.
- Test host per la state machine minima.
- Check target con `cargo check-esp`.
- Test hardware manuale con telefono.

### Out of scope

- Persistenza reale in flash.
- Form credenziali WiFi.
- Scan reti WiFi.
- Cambio rete.
- Login admin.
- CSRF/sessioni.
- Captive portal completo.
- DNS captive.
- OTA.
- Layout flash dual-slot.

---

## Architettura minima

```text
src/
├── net/
│   ├── runtime.rs        # owner di radio, WiFi controller, embassy-net stack
│   └── mod.rs
├── provisioning/
│   ├── state.rs          # logica pura testabile su host
│   ├── supervisor.rs     # decide modalità setup vs operativa
│   ├── softap.rs         # avvio SoftAP
│   ├── http.rs           # pagina HTTP minimale
│   └── mod.rs
└── storage/
    └── mod.rs            # trait/fake: config presente o assente
```

Regola: un solo modulo possiede radio e stack IP: `net::runtime`.

---

## State Machine Minima

```rust
pub enum ProvisioningState {
    Boot,
    Provisioning,
    Operational,
}

pub enum ProvisioningEvent {
    ConfigMissing,
    ConfigPresent,
    SetupRequested,
}

pub fn next_state(
    state: ProvisioningState,
    event: ProvisioningEvent,
) -> ProvisioningState;
```

Questa logica non importa `esp-hal`, `esp-radio`, `embassy-net`, GPIO, HTTP o flash.

---

## Flusso Di Boot Per Lo Spike

```text
boot
 -> fake storage: config assente
 -> provisioning state = Provisioning
 -> H-bridge OFF
 -> DCC scheduler NON parte
 -> net_runtime avvia SoftAP
 -> http serve pagina minima
```

Modalità operativa con Z21 resta da preservare, ma lo spike può essere eseguito su branch dedicato con modalità setup forzata se serve a ridurre rischio.

---

## Task

1. Estrarre un primo `net::runtime` che può possedere radio, WiFi controller, stack e runner.
2. Lasciare `udp_control` come servizio separabile, senza inizializzare direttamente la rete nel nuovo percorso.
3. Aggiungere `provisioning::state` pura con test host.
4. Aggiungere `provisioning::supervisor` con decisione `config assente -> setup`.
5. Aggiungere `provisioning::softap` per avviare `DCC-Setup-XXXX`.
6. Aggiungere HTTP minimo su `192.168.4.1`.
7. Garantire che in setup DCC scheduler e H-bridge non partano.
8. Eseguire `cargo test-host`.
9. Eseguire `cargo check-esp`.
10. Flashare su hardware e verificare da telefono.

---

## Criteri Di Accettazione

- [ ] `cargo test-host` passa.
- [ ] `cargo check-esp` passa.
- [ ] La rete `DCC-Setup-XXXX` è visibile da telefono.
- [ ] Il telefono si collega alla rete setup.
- [ ] Il telefono riceve un IP.
- [ ] `http://192.168.4.1` mostra una pagina minima.
- [ ] In modalità setup i binari restano spenti.
- [ ] La modalità setup non dipende da ESP-IDF.
- [ ] Il piano principale resta valido: nessun dettaglio storage/OTA viene anticipato qui.

---

## Note Tecniche

- Il repo usa target `riscv32imac-unknown-none-elf`, `esp-hal`, `esp-radio`, `esp-rtos`, `embassy`.
- Non progettare NVS in questa fase.
- Non progettare OTA in questa fase.
- Non assumere semantiche ESP-IDF.
- Se SoftAP WPA2 non è disponibile subito, documentare il blocco e fermare la fase prima di costruire il resto.
