# RailCom v3 — Checklist di costruzione breadboard dedicata

Ultimo aggiornamento: 2026-05-01

Breadboard bianca 30 righe dedicata al rilevatore RailCom v3 (TLV3501 + 2× IRLZ44N + buffer 74HC14). Si collega alla breadboard main DCC tramite 4 fili di segnale + alimentazioni.

Riferimenti incrociati:
- `docs/hardware/railcom-detector-v3-breadboard-layout.md` (layout di riferimento)
- `docs/hardware/main-dcc-checklist.md` (per le modifiche da fare sulla main DCC)

## Lessons from the past

Il rebuild di oggi parte da 6 lezioni concrete del passato:

1. **LM393 troppo lento → falsi positivi su transienti switching**. Risolto con TLV3501 (rise/fall 4.5 ns, 200× più veloce).
2. **CUTOUT_INV scollegato (2026-04-14) ha invalidato giorni di test**. Risolto con verifica continuità OBBLIGATORIA prima di alimentare, doppio check sui critici.
3. **VREF a 33 mV instabile durante DCC** (rumore sui rail). **Aggiunto cap di bypass dedicato sul VREF** (non presente nel doc v3 base).
4. **Ground bounce sul comparatore**. **Aggiunto star ground locale** (non presente nel doc v3 base).
5. **Fronti TLV3501 esposti a EMI sul filo esterno**. Già coperto dal v3 (buffer 74HC14 a doppia inversione).
6. **R_hyst marcata "opzionale" nel doc**. **Resa obbligatoria** in questa checklist (non opzionale).

Le caselle che hanno **➕** indicano miglioramenti rispetto al doc v3 base, motivati da queste lezioni.

---

## A — Pre-requisiti hardware (componenti pronti)

- [ ] Breadboard bianca 30 righe con rail + e − su entrambi i lati
- [ ] **TLV3501AIDR** SOIC-8 saldato sul breakout ProtoAdvantage PA0002 (DIP-8 passivo)
- [ ] 2× MOSFET **IRLZ44N** (logic-level, 55V, package TO-220)
- [ ] Resistore **4.7 Ω** (R_sense, NMRA-compliant burden)
- [ ] Resistore **1 Ω** (R_limit Track B)
- [ ] 2× resistori **220 Ω** (R_gate1, R_gate2 — gate drive MOSFET)
- [ ] 2× resistori **100 KΩ** (R_gate_pd1, R_gate_pd2 — pull-down gate MOSFET)
- [ ] Resistore **100 KΩ** (R_ref_high — top partitore VREF)
- [ ] Resistore **1 KΩ** (R_ref_low — bottom partitore VREF)
- [ ] Resistore **47 KΩ** (R_hyst — isteresi TLV3501) ➕ obbligatorio
- [ ] Cap **100 nF** ceramico (C_bypass — tra V+ e GND TLV3501)
- [ ] Cap **100 nF** ceramico (C_vref — bypass VREF) ➕ aggiunto
- [ ] Filo per le 4 connessioni esterne main↔RailCom
- [ ] Filo grosso (AWG 20-22 rame) per Track A, Track B (corrente alta)

---

## B — Mappa zone breadboard

```
riga  1-3    ZONA A — TRACK_A / TRACK_B input (lato sx)
riga  3      CUTOUT_INV (lato dx, ingresso gate MOSFET)
riga  5-7    ZONA M1 — Q1 (IRLZ44N) — Track A clamp
riga  8-10   corridoio isolamento (vuoto)
riga 11-13   ZONA M2 — Q2 (IRLZ44N) — Track B clamp
riga 14      ★ STAR GROUND POINT del comparatore ➕ aggiunto
riga 15      SENSE NODE (lato sx)
riga 16-19   corridoio isolamento (vuoto, separa zona alta corrente da TLV3501)
riga 20-23   ZONA C — TLV3501 PA0002 (DIP-8, pin 1 verso riga 20)
riga 25      VREF (~33 mV)
riga 26      ★ STAR GROUND ext. (jumper da riga 14 sx, per cap/R vicini al VREF) ➕ aggiunto
riga 27-30   spazio libero
```

**Rationale separazione fisica**: la zona alta corrente (MOSFET, 1-15 A surge al cutout start) sta sulle righe 5-13, fisicamente lontana 7+ righe dal TLV3501 (riga 20-23). Il filo SENSE NODE (riga 15) attraversa il "corridoio di isolamento" verso il TLV3501 con percorso dritto e corto (~5 righe).

---

## C — Modifiche preliminari alla main DCC (per il buffer 74HC14)

Prima di iniziare la breadboard RailCom, fare due tipi di modifiche sulla main DCC, in ordine.

### C.1 — Liberare riga 11 sx del 74HC14 (spostamento cap di bypass)

Sulla main DCC, riga 11 sx era usata come capo GND del cap di bypass del 74HC14. Per il buffer v3, riga 11 sx serve come pin 13 (input gate 6) e deve essere **liberata**.

- [ ] Estrai la gamba del **cap di bypass 100 nF** che è su **riga 11 sx**, infilala su **riga 9 sx** (riga libera sopra il chip). L'altra gamba (riga 10 sx, pin 14 VCC) resta dov'è.
- [ ] Stacca il jumper che va da **riga 11 sx → riga 16 dx (pin 7 GND)**, rimettilo da **riga 9 sx → riga 16 dx (pin 7 GND)**.

Risultato: cap continua a fare bypass tra VCC (riga 10 sx) e GND (riga 9 sx via jumper a pin 7), riga 11 sx è ora libera per il buffer.

Verifica veloce con multimetro buzzer (alimentazione scollegata):
- [ ] Riga 9 sx ↔ riga 16 dx (pin 7) : continuo
- [ ] Riga 11 sx ↔ qualsiasi altro nodo : NON continuo (deve essere libera)

### C.2 — Cablare il buffer Schmitt v3 (gate 5 + gate 6 del 74HC14)

Solo dopo che C.1 è completata. Tre nuovi cablaggi sulla main DCC, sui gate 5 e 6 del 74HC14 (ora che riga 11 sx è libera):

- [ ] Filo da **breadboard RailCom riga 22 dx** (TLV3501 OUT, lo cablerai in Zona C della breadboard RailCom) → **74HC14 pin 11** main DCC = **riga 13 sx**
- [ ] **Ponticello breve riga 14 sx (74HC14 pin 10) → riga 11 sx (74HC14 pin 13)** sulla main DCC
- [ ] Filo da **74HC14 pin 12** main DCC = **riga 12 sx** → **ESP32 GPIO5**

Catena segnale finale:
```
TLV3501 OUT (RailCom, riga 22 dx)
  ↓ filo esterno
riga 13 sx main (pin 11 IN gate 5)
  ↓ Schmitt invert
riga 14 sx main (pin 10 OUT gate 5)
  ↓ ponticello breve
riga 11 sx main (pin 13 IN gate 6)
  ↓ Schmitt invert
riga 12 sx main (pin 12 OUT gate 6)
  ↓ filo
GPIO5 ESP32
```

Doppia inversione = polarità invariata, ma fronti puliti. Il firmware non cambia.

➕ Vedi anche `main-dcc-checklist.md` Sezione C (cap aggiornato) e Sezione I.2 (catena buffer documentata sulla main DCC).

---

## D — Cablaggio breadboard RailCom passo-passo

### D1 — Pianta i componenti fisici

- [ ] **TLV3501 breakout PA0002** a cavallo del canale, righe **20-23**, **pin 1 verso riga 20** (verifica visiva: il puntino o smusso d'angolo deve essere su riga 20 sx)
- [ ] **Q1 (IRLZ44N)** sul lato dx, scritta leggibile verso l'alto, righe **5 dx (G), 6 dx (D), 7 dx (S)**
- [ ] **Q2 (IRLZ44N)** sul lato dx, scritta leggibile verso l'alto, righe **11 dx (G), 12 dx (D), 13 dx (S)**

⚠️ Le placche metalliche dei due MOSFET NON si devono toccare. Le 3 righe di distanza (8-10) garantiscono l'isolamento.

### D2 — Alimentazioni e star ground (zona TLV3501)

- [ ] Jumper **rail + dx → riga 21 dx (TLV3501 pin 7, V+)**
- [ ] Jumper **rail − dx → riga 20 dx (TLV3501 pin 8, SHDN)** — OBBLIGATORIO, altrimenti chip in shutdown casuale
- [ ] Jumper **riga 23 sx (TLV3501 pin 4, V−) → riga 14 sx (★ STAR GROUND POINT)** ➕ NON al rail − direttamente
- [ ] Jumper **★ riga 14 sx (STAR GROUND) → rail − sx** (filo dedicato singolo)

➕ **Motivo dello star ground**: i tre nodi GND critici del comparatore (TLV3501 V−, R_sense bottom, C_vref/R_ref_low) vanno tutti a riga 14 sx, e di lì un singolo filo al rail − sx. Riduce ground bounce indotto dalle correnti del MOSFET.

### D3 — Bypass del TLV3501 (decoupling)

- [ ] **C_bypass 100 nF MLCC** tra **riga 21 dx (V+) e riga 20 dx (SHDN/GND)**, gambe corte (≤ 5 mm), il più vicino possibile ai pin del chip

### D4 — VREF + cap di filtro ➕

Per fare arrivare i componenti del partitore al GND, **estendo lo star ground** fino vicino al VREF tramite un jumper, così il cap e R_ref_low hanno entrambe le gambe a 1 riga di distanza.

- [ ] Jumper **riga 14 sx (★ STAR GROUND) → riga 26 sx** (estensione GND vicino al VREF) ➕
- [ ] **R_ref_high 100 KΩ** tra **rail + sx** e **riga 25 sx (VREF)**
- [ ] **R_ref_low 1 KΩ** tra **riga 25 sx (VREF)** e **riga 26 sx (★ STAR GROUND ext.)**
- [ ] **C_vref 100 nF** tra **riga 25 sx (VREF)** e **riga 26 sx (★ STAR GROUND ext.)** ➕ aggiunto, anti-rumore
- [ ] Jumper **riga 25 sx (VREF) → riga 22 sx (TLV3501 pin 3, IN+)**

Riga 26 sx è elettricamente lo stesso nodo di riga 14 sx (collegate dal jumper), quindi sono entrambe "comparator GND" → il cap e R_ref_low fanno il loro lavoro a soli 2.54 mm di distanza dalle proprie gambe sul VREF.

➕ **Motivo del cap su VREF**: il partitore 100K+1K ha alta impedenza in cima (~990 Ω equivalenti). Qualunque variazione del rail + 3.3 V durante DCC switching si propaga a VREF. Il cap 100 nF cortocircuita la AC verso GND, mantenendo VREF stabile a 33 mV DC.

### D5 — MOSFET cutout (Track A clamp + sense)

- [ ] Filo esterno **CUTOUT_INV (da pin 8 74HC14 main DCC) → riga 3 dx**
- [ ] **R_gate1 220 Ω** tra **riga 3 dx (CUTOUT_INV)** e **riga 5 dx (Q1 gate)**
- [ ] **R_gate_pd1 100 KΩ** tra **riga 5 dx (Q1 gate)** e **rail − dx** (gate non flotta a boot)
- [ ] Jumper **riga 6 dx (Q1 drain) → riga 2 sx (TRACK_A)**
- [ ] Filo **TRACK_A dal binario A (e da BTS M+)** → **riga 2 sx**
- [ ] Jumper **riga 7 dx (Q1 source) → riga 15 sx (SENSE NODE)**
- [ ] **R_sense 4.7 Ω** tra **riga 15 sx (SENSE NODE)** e **riga 14 sx (★ STAR GROUND)** ➕ NON al rail − direttamente
- [ ] Jumper **riga 15 sx (SENSE NODE) → riga 21 sx (TLV3501 pin 2, IN−)**

### D6 — MOSFET cutout (Track B clamp)

- [ ] **R_gate2 220 Ω** tra **riga 3 dx (CUTOUT_INV)** e **riga 11 dx (Q2 gate)**
- [ ] **R_gate_pd2 100 KΩ** tra **riga 11 dx (Q2 gate)** e **rail − dx**
- [ ] Jumper **riga 12 dx (Q2 drain) → riga 3 sx (TRACK_B)**
- [ ] Filo **TRACK_B dal binario B (e da BTS M−)** → **riga 3 sx**
- [ ] **R_limit 1 Ω** tra **riga 13 dx (Q2 source)** e **rail − dx**

### D7 — Isteresi sul TLV3501 ➕ obbligatoria

- [ ] **R_hyst 47 KΩ** tra **riga 22 dx (TLV3501 pin 6, OUT)** e **riga 22 sx (TLV3501 pin 3, IN+)**

➕ **Motivo R_hyst obbligatoria**: aumenta l'isteresi della soglia del comparatore. Quando OUT è LOW, IN+ effettivo cala leggermente sotto VREF, evitando oscillazione su micro-rumore vicino alla soglia. Quando OUT è HIGH, IN+ effettivo sale leggermente sopra VREF. Risultato: zero false transizioni durante il floor di rumore. Questo è il singolo intervento più efficace contro il problema "16% success rate" del LM393 passato.

### D8 — Uscita verso main DCC

- [ ] Filo esterno **riga 22 dx (TLV3501 pin 6, OUT)** → **riga 13 sx main DCC** (74HC14 pin 11, in gate 5)

---

## E — Fili tra main DCC e RailCom (twisted pair) ➕

Tutti i fili lunghi di interconnessione devono essere **attorcigliati a coppia con un GND di accompagnamento**, per ridurre il pickup di EMI.

- [ ] Filo **3.3 V** (rail + main → rail + RailCom) twistato con un filo GND parallelo ➕
- [ ] Filo **GND comune** (rail − main → rail − RailCom) — uno dei due fili del twisted pair sopra
- [ ] Filo **CUTOUT_INV** (74HC14 pin 8 main → riga 3 dx RailCom) twistato con un filo GND parallelo ➕
- [ ] Filo **TLV3501 OUT** (riga 22 dx RailCom → 74HC14 pin 11 main) twistato con un filo GND parallelo ➕

➕ **Motivo twisted pair**: senza twisting, l'area della spira tra segnale e GND è grande, e capta EMI proporzionale a quell'area. Twistando, l'area si riduce a quasi zero. Particolarmente importante per CUTOUT_INV (segnale veloce) e TLV3501 OUT (segnale che entra nel pin 11 del 74HC14 — se rumoroso, lo Schmitt lo "pulisce" ma è meglio evitare di farglielo "sporcare" già dal filo).

I 3 fili GND extra possono essere riassunti in **1 solo filo GND molto più grosso** (es. AWG 18) tra le due breadboard, con i 3 segnali twistati intorno a quel filo unico. Più semplice.

---

## F — Verifica continuità (alimentazione SCOLLEGATA, multimetro buzzer)

➕ **Doppio check sui critici** marcati ⚡: questi sono punti dove un cattivo contatto in passato ha causato giorni di debug inutile.

### F1 — Alimentazioni e star ground

- [ ] Rail + RailCom ↔ riga 21 dx (V+ TLV3501) : continuo
- [ ] Rail − RailCom (lato dx) ↔ riga 20 dx (SHDN) : continuo ⚡
- [ ] Riga 23 sx (V− TLV3501) ↔ riga 14 sx (★ STAR GROUND) : continuo
- [ ] Riga 14 sx (★ STAR GROUND) ↔ rail − sx : continuo

### F2 — Catena cutout (la più critica per "lessons from the past")

- [ ] Filo CUTOUT_INV ↔ riga 3 dx : continuo (capo dal pin 8 main 74HC14) ⚡⚡
- [ ] Riga 3 dx ↔ riga 5 dx : NON continuo (R_gate1 220Ω in mezzo)
- [ ] Ohmmetro riga 3 dx ↔ riga 5 dx : ~220 Ω
- [ ] Riga 3 dx ↔ riga 11 dx : NON continuo (R_gate2 220Ω in mezzo)
- [ ] Ohmmetro riga 3 dx ↔ riga 11 dx : ~220 Ω
- [ ] Riga 5 dx ↔ rail − dx : NON continuo (R_gate_pd1 100KΩ in mezzo)
- [ ] Ohmmetro riga 5 dx ↔ rail − dx : ~100 KΩ
- [ ] Riga 11 dx ↔ rail − dx : NON continuo (R_gate_pd2 100KΩ in mezzo)
- [ ] Ohmmetro riga 11 dx ↔ rail − dx : ~100 KΩ

### F3 — Catena Track A / sense

- [ ] Riga 2 sx (TRACK_A) ↔ riga 6 dx (Q1 drain) : continuo (jumper)
- [ ] Riga 7 dx (Q1 source) ↔ riga 15 sx (SENSE NODE) : continuo (jumper)
- [ ] Riga 15 sx (SENSE NODE) ↔ riga 14 sx (★ STAR GROUND) : NON continuo (R_sense 4.7Ω in mezzo)
- [ ] Ohmmetro riga 15 sx ↔ riga 14 sx : ~4.7 Ω
- [ ] Riga 15 sx (SENSE NODE) ↔ riga 21 sx (TLV3501 IN−) : continuo (jumper) ⚡

### F4 — Catena Track B / R_limit

- [ ] Riga 3 sx (TRACK_B) ↔ riga 12 dx (Q2 drain) : continuo (jumper)
- [ ] Riga 13 dx (Q2 source) ↔ rail − dx : NON continuo (R_limit 1Ω in mezzo)
- [ ] Ohmmetro riga 13 dx ↔ rail − dx : ~1 Ω

### F5 — VREF e cap di filtro ➕

- [ ] Rail + sx ↔ riga 25 sx (VREF) : NON continuo direttamente (R_ref_high 100KΩ in mezzo)
- [ ] Ohmmetro rail + sx ↔ riga 25 sx : ~100 KΩ
- [ ] Riga 14 sx ↔ riga 26 sx : continuo (jumper estensione star ground) ⚡
- [ ] Riga 26 sx ↔ rail − sx : continuo (transitivo via riga 14 sx → rail − sx)
- [ ] Riga 25 sx ↔ riga 26 sx : NON continuo direttamente (C_vref + R_ref_low 1KΩ in parallelo, ohmmetro alto in carica del cap, poi ~1 KΩ)
- [ ] Riga 25 sx ↔ riga 22 sx (TLV3501 IN+) : continuo (jumper) ⚡

### F6 — Isteresi ➕

- [ ] Riga 22 dx (TLV3501 OUT) ↔ riga 22 sx (TLV3501 IN+) : NON continuo (R_hyst 47KΩ in mezzo)
- [ ] Ohmmetro riga 22 dx ↔ riga 22 sx : ~47 KΩ

### F7 — Verifica negativa (per evitare corti)

- [ ] Rail + ↔ rail − (RailCom): NON continuo
- [ ] TRACK_A (riga 2 sx) ↔ TRACK_B (riga 3 sx) : NON continuo
- [ ] V+ TLV3501 (riga 21 dx) ↔ V− TLV3501 (riga 23 sx) : NON continuo direttamente (cap di bypass in carica → ohmmetro alto)

---

## G — Power-up graduale + verifiche scope

### G1 — Solo USB ESP32 (15 V SPENTO)

Riconnetti USB, lancia firmware DCC normale. NON accendere ancora il 15 V dell'alimentatore di banco.

- [ ] Multimetro DC su rail + RailCom : ~3.0 V (uguale al rail + main)
- [ ] Multimetro DC su rail − RailCom : 0 V
- [ ] Multimetro DC su riga 14 sx (★ STAR GROUND) : 0 V
- [ ] Multimetro DC su riga 21 dx (TLV3501 V+) : ~3.0 V
- [ ] Multimetro DC su riga 25 sx (VREF) : **~33 mV** (intorno a 30-35 mV) ⚡
- [ ] Multimetro DC su riga 22 dx (TLV3501 OUT) : ~3.0 V (a riposo, IN+ > IN− perché sense node = 0)

### G2 — Scope sul cutout chain

Senza loco sui binari, con 15 V SPENTO:

- [ ] Sonda su **riga 3 dx (CUTOUT_INV)**, time base 200 µs/div
- [ ] Devi vedere impulsi **HIGH** di ~460 µs ogni ~45 ms (gate 4 inverte GPIO4: GPIO4 LOW → pin 8 HIGH)
- [ ] Sonda su **riga 5 dx (Q1 gate)** o **riga 11 dx (Q2 gate)**: stessa forma, livello atteso ~3 V durante il cutout (R_gate 220Ω + R_gate_pd 100KΩ → divisore quasi 1:1, gate vede ~99% di CUTOUT_INV)

### G3 — Power-on 15 V

Solo dopo che G1 e G2 sono OK:

- [ ] Inserisci fusibile, accendi alimentatore 15 V
- [ ] Multimetro DC su VREF: deve restare **~33 mV** stabile (se fluttua > 5 mV è il segnale che il C_vref non sta filtrando bene → indagare)
- [ ] Multimetro DC su SENSE NODE (riga 15 sx) senza loco sui binari : ~0 V (la corrente di leakage attraverso il binario aperto è trascurabile)
- [ ] Multimetro DC su TLV3501 OUT (riga 22 dx) senza loco : ~3.0 V (idle HIGH)

---

## H — Test funzionale finale

### H1 — Loco sui binari

- [ ] Metti loco DCC-RailCom-capable sui binari (es. ESU LokSound 5)
- [ ] Verifica che si muova con comandi via Z21 / firmware
- [ ] Aspetta che la loco invii pacchetti POM o il decoder broadcast l'indirizzo

### H2 — Log diagnostici

Nel serial monitor cerca le righe `railcom diag:` ogni 10 secondi. **Cosa devi vedere migliorato rispetto al passato (LM393, 16% success rate)**:

- [ ] `started=N` cresce regolarmente (~22/sec con scheduler default)
- [ ] **`rx_windows` cresce E `rx_empty` < `rx_windows`** ⚡ (il decoder sta rispondendo, non tutte vuote!)
- [ ] **`rx_bytes > 0`** ⚡⚡ (byte effettivamente ricevuti dal TLV3501)
- [ ] **`rx_ok / rx_windows > 0.7`** target con TLV3501 (vs 0.16 con LM393)
- [ ] `rx_corrupted`, `rx_glitch`, `rx_framing` bassi rispetto a `rx_ok`
- [ ] `loco_id_ok > 0` quando il decoder invia il proprio indirizzo (ogni qualche secondo)

### H3 — Scope sul TLV3501 OUT (riga 22 dx) durante una loco attiva

- [ ] Time base 100 µs/div, trigger su falling edge ~1.5 V, mode Single
- [ ] Devi vedere un'**onda quadra UART** durante la finestra cutout di 460 µs (bit a 250 kbaud → ~4 µs per bit)
- [ ] Tra cutout, segnale stabile HIGH a 3 V

### H4 — Scope sul SENSE NODE (riga 15 sx) durante cutout

⚠️ Questa misura serve per debug, livelli piccoli (~175 mV).

- [ ] Vertical 50 mV/div (sonda 10X non serve a livelli così bassi, ma se ce l'hai usala lo stesso per protezione)
- [ ] Trigger su CUTOUT_INV (riga 3 dx) edge falling
- [ ] Time base 100 µs/div
- [ ] Devi vedere il segnale del decoder come piccoli impulsi su SENSE NODE durante i 460 µs

---

## I — Se NON funziona — diagnosi rapida

| Sintomo | Causa probabile | Cosa controllare |
|---------|-----------------|------------------|
| `rx_windows = 0` | Cutout non arriva ai MOSFET | Sezione F2 (continuità chain cutout); pin 8 main 74HC14 → riga 3 dx RailCom ⚡⚡ |
| `rx_windows > 0` ma tutte `empty` | Decoder non risponde | Loco RailCom-capable? Acceso? Su entrambi i binari? VREF a ~33 mV stabile? Sense resistor 4.7 Ω corretto? |
| `rx_bytes > 0` ma `rx_ok = 0` | Bit corretti ma framing errato | Verifica polarità: F.5 sense node / F.6 R_hyst — se polarità invertita, scambiare TRACK_A e TRACK_B |
| Spike fantasma su TLV3501 OUT senza loco | Rumore sopra soglia | Verifica C_vref presente e ben saldato; verifica star ground; aumenta R_hyst da 47K a 100K |
| TLV3501 OUT bloccato a HIGH | SHDN flottante | Verifica jumper riga 20 dx → rail − dx (è il problema più comune del TLV3501) |
| TLV3501 OUT bloccato a LOW | Cap di bypass mancante / chip danneggiato | Verifica C_bypass presente tra V+ e GND, gambe corte |

---

## Note finali

Le 6 modifiche anti-rumore aggiunte (➕) rispetto al doc v3 base sono cumulative: nessuna è "magic bullet" da sola, insieme abbattono il floor di rumore di un fattore 5-10×. Stima realistica del success rate target: **70-90%** (vs 16% del LM393 storico).

Quando il system funziona, **NON RIMUOVERE** queste modifiche per "semplificare":
- C_vref senza il cap → VREF instabile → falsi positivi
- R_hyst senza isteresi → oscillazione su soglia → falsi positivi
- Star ground rimosso → ground bounce dal MOSFET sul comparatore → falsi positivi
- Twisted pair sciolti → EMI captata sui fili lunghi → falsi positivi

Quando si farà il PCB definitivo, queste 6 misure diventeranno scelte di layout (ground plane, pour, trace shielding) e non saranno più "componenti aggiunti". Ma su breadboard sono indispensabili.
