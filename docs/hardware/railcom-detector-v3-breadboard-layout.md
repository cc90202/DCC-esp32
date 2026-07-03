# Detector RailCom v3 — Layout Breadboard Bianca (30 righe, rail +/−)

Ultimo aggiornamento: 2026-04-22

Breadboard dedicata al detector RailCom v3. Collegata alla breadboard principale (ESP32 + 74HC14) tramite 4 fili di segnale + 2 di alimentazione.

**Differenza rispetto alla v2:** l'uscita del comparatore TLV3501 non va più direttamente a GPIO5. Prima passa per due inverter Schmitt-trigger del 74HC14 (gate 5 e gate 6, prima "Libero") come stadio di buffer. Doppia inversione = polarità invariata; il firmware non cambia.

Motivi dell'aggiunta:
- fronti rimessi "in squadra" dallo Schmitt trigger (ulteriore isteresi sul percorso, oltre a quella interna del TLV3501);
- isolamento dell'uscita del comparatore dal pin GPIO dell'ESP32 (protezione + driver 74HC14);
- utilizzo di due gate del 74HC14 che erano liberi — nessun componente aggiuntivo.

Chip comparatore: **TLV3501AIDR in SOIC-8** montato su breakout ProtoAdvantage PA0002 (adattatore passivo SOIC-8 → DIP-8). Il breakout si comporta come un normale DIP-8 sulla breadboard: 4 pin sul lato sinistro, 4 sul lato destro, a cavallo del canale centrale, righe 20-23.

---

## Sezione 1 — Orientamento del breakout TLV3501

Il breakout PA0002 ha stampati i numeri dei pin (1, 4, 5, 8) agli angoli. Sul chip stesso, pin 1 è segnato da un puntino o da uno smusso d'angolo.

**Devi montarlo con il pin 1 verso la riga 20.** Guardando il breadboard dall'alto:

```
                canale centrale
                      ↓
     riga 20  ┌─[1]──────────[8]─┐
     riga 21  │ [2]          [7] │
     riga 22  │ [3]          [6] │
     riga 23  └─[4]──────────[5]─┘
              sinistra       destra
```

**Verifica prima di cablare:** la serigrafia "1" del PA0002 deve essere sopra la riga 20 sinistra. Se il breakout è ruotato di 180°, tutto il cablaggio sotto è sbagliato.

---

## Sezione 2 — Pinout TLV3501 SOIC-8

| Pin | Riga + lato | Funzione | Cosa collegare |
|-----|-------------|----------|----------------|
| 1 | 20 sx | **NC** (no connection) | Lasciare flottante, niente filo |
| 2 | 21 sx | **IN−** (invertente) | Jumper dal SENSE NODE (riga 15 sx) |
| 3 | 22 sx | **IN+** (non-invertente) | Jumper dal VREF (riga 25 sx) |
| 4 | 23 sx | **V−** (GND) | Jumper al rail − |
| 5 | 23 dx | **NC** (no connection) | Lasciare flottante, niente filo |
| 6 | 22 dx | **OUT** (push-pull) | Filo esterno a **74HC14 pin 11** (ingresso gate 5 sulla breadboard principale) |
| 7 | 21 dx | **V+** (3.3 V) | Jumper al rail + |
| 8 | 20 dx | **SHDN** (shutdown active-HIGH) | **Jumper al rail −** (OBBLIGATORIO) |

⚠️ Il pin SHDN è **active-HIGH**: se lo lasci flottante, il chip può andare in shutdown a caso. Devi **per forza** metterci un ponticello corto da riga 20 destra al rail −.

ℹ️ Differenza v2 → v3: in v2 il filo esterno da riga 22 dx andava direttamente a GPIO5. In v3 va invece a **74HC14 pin 11** (input gate 5). GPIO5 prende il segnale dal pin 12 del 74HC14 (output gate 6). Vedere Sezione 8.

---

## Sezione 3 — Se stai migrando da v2 (TLV3501 già montato, uscita diretta a GPIO5)

Se la breadboard è già cablata in v2 — TLV3501 funzionante, OUT (riga 22 dx) collegata direttamente a GPIO5 — per passare a v3 serve solo spostare un filo e aggiungere un ponticello sulla breadboard principale. Nessuna modifica sulla breadboard RailCom.

### 3a. Filo da spostare

- [ ] Filo **GPIO5 ↔ riga 22 dx (OUT TLV3501)**: scollegare il capo sul lato breadboard principale (lato GPIO5). L'altro capo, su riga 22 dx RailCom, rimane.

### 3b. Fili/ponticelli da aggiungere sulla breadboard principale

- [ ] **TLV3501 OUT → ingresso buffer**: il filo che parte da riga 22 dx RailCom ora va a **74HC14 pin 11** (riga 13 sinistra della breadboard principale — vedere tabella Sezione 8).
- [ ] **Ponte gate 5 → gate 6**: ponticello corto tra **pin 10** (riga 14 sx main) e **pin 13** (riga 11 sx main) del 74HC14.
- [ ] **Uscita buffer → GPIO5**: filo da **74HC14 pin 12** (riga 12 sx main) a GPIO5 sull'ESP32.

Alla fine: il segnale RailCom entra nel 74HC14 gate 5, esce invertito, rientra in gate 6, esce re-invertito e arriva a GPIO5. Polarità finale = polarità TLV3501 = polarità v2 (nessun cambiamento firmware).

### 3c. Fili che NON si toccano

- [ ] Cablaggio interno breadboard RailCom (righe 20-23 del TLV3501, MOSFET, partitore VREF, resistori): **invariato**.
- [ ] Fili CUTOUT_INV, GND, 3.3V tra breadboard principale e RailCom: **invariati**.
- [ ] Gate 1-4 del 74HC14 (DCC, cortocircuito, cutout): **invariati**.

---

## Sezione 4 — Se stai migrando direttamente da LM393 (v1) a v3

Segui la Sezione 3 del documento v2 (montaggio TLV3501) e poi la Sezione 3 di questo documento (aggiunta buffer). Le due migrazioni sono ortogonali.

---

## Sezione 5 — Mappa righe finale (breadboard RailCom)

### Bancata sinistra

| Riga | Funzione | Cosa c'è |
|------|----------|----------|
| 2 | TRACK_A | Jumper da riga 6 dx (Drain Q1), filo a M+ BTS7960, filo al binario A |
| 3 | TRACK_B | Jumper da riga 12 dx (Drain Q2), filo a M− BTS7960, filo al binario B |
| 15 | SENSE NODE | Jumper da riga 7 dx (Source Q1), piedino 4.7R, jumper a riga 21 sx |
| 20 | TLV3501 pin 1 (NC) | *Niente* |
| 21 | TLV3501 pin 2 (IN−) | Jumper da riga 15 sx (SENSE NODE) |
| 22 | TLV3501 pin 3 (IN+) | Jumper da riga 25 sx (VREF) |
| 23 | TLV3501 pin 4 (GND) | Jumper al rail − |
| 25 | VREF (~33 mV) | Piedino 100K (da rail +), piedino 1K (a rail −), jumper a riga 22 sx |

### Bancata destra

| Riga | Funzione | Cosa c'è |
|------|----------|----------|
| 3 | CUTOUT_INV | Filo da 74HC14 pin 8, piedino 220R (a Q1 gate), piedino 220R (a Q2 gate) |
| 5 | Gate Q1 | Pin G dell'IRLZ44N Q1, piedino 220R, piedino 100K (a rail −) |
| 6 | Drain Q1 | Pin D dell'IRLZ44N Q1, jumper a riga 2 sx (TRACK_A) |
| 7 | Source Q1 | Pin S dell'IRLZ44N Q1, jumper a riga 15 sx (SENSE NODE) |
| 11 | Gate Q2 | Pin G dell'IRLZ44N Q2, piedino 220R, piedino 100K (a rail −) |
| 12 | Drain Q2 | Pin D dell'IRLZ44N Q2, jumper a riga 3 sx (TRACK_B) |
| 13 | Source Q2 | Pin S dell'IRLZ44N Q2, piedino 1R (a rail −) |
| 20 | TLV3501 pin 8 (SHDN) | **Jumper al rail −** (obbligatorio) |
| 21 | TLV3501 pin 7 (V+) | Jumper al rail +, piedino C_bypass |
| 22 | TLV3501 pin 6 (OUT) | **Filo esterno a 74HC14 pin 11** (in gate 5, riga 13 sx main) |
| 23 | TLV3501 pin 5 (NC) | *Niente* |

---

## Sezione 6 — MOSFET IRLZ44N (invariato)

Entrambi montati sulla bancata destra, scritta leggibile verso l'alto, placca metallica dietro.

```
      +----------+
      | IRLZ44N  |
      |          |
      +--+--+--++
         |  |  |
         G  D  S
```

- Q1: G=riga 5 dx, D=riga 6 dx, S=riga 7 dx
- Q2: G=riga 11 dx, D=riga 12 dx, S=riga 13 dx

Le piastre metalliche NON si devono toccare (4 righe di distanza).

---

## Sezione 7 — Lista Componenti

Nessuna modifica componenti tra v2 e v3. Il buffer riusa due gate già presenti sul 74HC14.

| Rif | Componente | Valore | Posizione (da → a) |
|-----|-----------|--------|---------------------|
| Q1 | IRLZ44N | — | G: riga 5 dx, D: riga 6 dx, S: riga 7 dx |
| Q2 | IRLZ44N | — | G: riga 11 dx, D: riga 12 dx, S: riga 13 dx |
| R_sense | Resistore | 4.7R | Riga 15 sx (SENSE) → rail − (GND) |
| R_limit | Resistore | 1R | Riga 13 dx (Source Q2) → rail − (GND) |
| R_gate1 | Resistore | 220R | Riga 3 dx (CUTOUT_INV) → riga 5 dx (Gate Q1) |
| R_gate2 | Resistore | 220R | Riga 3 dx (CUTOUT_INV) → riga 11 dx (Gate Q2) |
| R_gate_pd1 | Resistore | 100K | Riga 5 dx (Gate Q1) → rail − (GND) |
| R_gate_pd2 | Resistore | 100K | Riga 11 dx (Gate Q2) → rail − (GND) |
| R_ref_high | Resistore | 100K | Rail + (3.3 V) → riga 25 sx (VREF) |
| R_ref_low | Resistore | 1K | Riga 25 sx (VREF) → rail − (GND) |
| R_hyst | Resistore | 47K | Riga 22 dx (OUT) → riga 22 sx (IN+), **opzionale** |
| U2 | TLV3501AIDR su breakout PA0002 | SOIC-8 DIP-8 | Righe 20-23, pin 1 verso riga 20 |
| C_bypass | Condensatore | 100nF | Riga 20 dx (SHDN/GND) → riga 21 dx (V+) — sopra i pin di alimentazione |

---

## Sezione 8 — Stadio buffer 74HC14 (novità v3)

Il 74HC14 sulla breadboard principale occupa le righe 10-16 (pin 1 sul lato destro, pin 14 sul lato sinistro riga 10). Gate 5 e gate 6 — prima inutilizzati — ora fanno da buffer Schmitt sull'uscita del TLV3501.

### Percorso segnale

```
TLV3501 OUT (riga 22 dx RailCom)
          │
          ▼  filo esterno
74HC14 pin 11 (riga 13 sx main) — ingresso gate 5
          │  [Schmitt inverter, inversione #1]
74HC14 pin 10 (riga 14 sx main) — uscita gate 5
          │  ponte corto
          ▼
74HC14 pin 13 (riga 11 sx main) — ingresso gate 6
          │  [Schmitt inverter, inversione #2]
74HC14 pin 12 (riga 12 sx main) — uscita gate 6
          │  filo
          ▼
        GPIO5 ESP32
```

Due inversioni in cascata = buffer non-invertente. La polarità all'uscita (pin 12) è identica a quella all'ingresso (pin 11), quindi GPIO5 vede esattamente la stessa forma d'onda che vedrebbe dal TLV3501 diretto, ma con:

- fronti "ripuliti" dagli Schmitt trigger del 74HC14 (tipicamente < 20 ns);
- una dose di isteresi extra sul percorso (oltre a quella interna del TLV3501 e all'eventuale R_hyst da 47K);
- buffer di corrente da 74HC14 tra il comparatore e GPIO5.

### Mappa pin 74HC14 ↔ righe breadboard principale

| Pin 74HC14 | Riga + lato main | Funzione v3 |
|------------|------------------|-------------|
| 1 | 10 dx | Gate 1 in ← GPIO2 (DCC) |
| 2 | 11 dx | Gate 1 out → LPWM |
| 3 | 12 dx | Gate 2 in |
| 4 | 13 dx | Gate 2 out → RPWM |
| 5 | 14 dx | Gate 3 in ← trimmer IS |
| 6 | 15 dx | Gate 3 out → 1K → GPIO3 |
| 7 | 16 dx | GND |
| 8 | 16 sx | Gate 4 out → CUTOUT_INV (filo a breadboard RailCom riga 3 dx) |
| 9 | 15 sx | Gate 4 in ← GPIO18 |
| 10 | 14 sx | **Gate 5 out** → ponte a pin 13 |
| 11 | 13 sx | **Gate 5 in** ← filo da riga 22 dx RailCom (TLV3501 OUT) |
| 12 | 12 sx | **Gate 6 out** → filo a GPIO5 |
| 13 | 11 sx | **Gate 6 in** ← ponte da pin 10 |
| 14 | 10 sx | VCC 3.3 V |

---

## Sezione 9 — Fili tra breadboard principale e breadboard RailCom

| Filo | Da (breadboard principale) | A (breadboard RailCom) |
|------|---------------------------|------------------------|
| GND | Rail GND | Rail − |
| 3.3 V | Pin 3.3 V ESP32 | Rail + |
| CUTOUT_INV | 74HC14 pin 8 (riga 16 sx main) | Riga 3 destra |
| **TLV3501 OUT** | **74HC14 pin 11 (riga 13 sx main)** | **Riga 22 destra** |

Jumper sulla breadboard principale:

| Da | A | Scopo |
|----|---|-------|
| GPIO18 | 74HC14 pin 9 (riga 15 sx main) | Ingresso gate 4 (invariato v1/v2) |
| **74HC14 pin 10 (riga 14 sx main)** | **74HC14 pin 13 (riga 11 sx main)** | **Ponte gate 5 → gate 6 (nuovo v3)** |
| **74HC14 pin 12 (riga 12 sx main)** | **GPIO5 ESP32** | **Uscita buffer verso MCU (nuovo v3)** |

⚠️ Differenza v2 → v3: in v2 il filo GPIO5 usciva **direttamente** da riga 22 dx della breadboard RailCom. In v3 GPIO5 si collega al pin 12 del 74HC14 sulla breadboard principale; la riga 22 dx RailCom si collega invece al pin 11 del 74HC14.

---

## Sezione 10 — Jumper interni alla breadboard RailCom (invariato)

| Da | A | Scopo |
|----|---|-------|
| Rail + sinistra | Rail + destra | 3.3 V su entrambi i lati |
| Rail − sinistra | Rail − destra | GND su entrambi i lati |
| Riga 6 dx (Drain Q1) | Riga 2 sx (TRACK_A) | Drain Q1 al binario A |
| Riga 12 dx (Drain Q2) | Riga 3 sx (TRACK_B) | Drain Q2 al binario B |
| Riga 7 dx (Source Q1) | Riga 15 sx (SENSE NODE) | Source Q1 al nodo di sensing |
| Riga 15 sx (SENSE) | Riga 21 sx (TLV3501 pin 2 IN−) | Segnale al comparatore |
| Riga 25 sx (VREF) | Riga 22 sx (TLV3501 pin 3 IN+) | Riferimento al comparatore |
| Riga 21 dx (TLV3501 pin 7 V+) | Rail + | Alimentazione comparatore |
| Riga 23 sx (TLV3501 pin 4 GND) | Rail − | GND comparatore |
| Riga 20 dx (TLV3501 pin 8 SHDN) | Rail − | Abilitazione (active-HIGH shutdown → tie GND) |

---

## Sezione 11 — Verifica prima di alimentare

Checklist di continuità (multimetro in modalità buzzer).

### Breadboard RailCom (invariato da v2)

- [ ] Riga 21 dx (V+) ↔ rail + : continuo
- [ ] Riga 23 sx (GND) ↔ rail − : continuo
- [ ] Riga 20 dx (SHDN) ↔ rail − : continuo (se aperto, chip in shutdown!)
- [ ] Riga 21 sx (IN−) ↔ riga 15 sx (SENSE NODE) : continuo
- [ ] Riga 22 sx (IN+) ↔ riga 25 sx (VREF) : continuo
- [ ] Rail + ↔ rail − : **NON continuo** (se suona, c'è un corto)
- [ ] Riga 20 sx (NC) e riga 23 dx (NC) : non collegate a niente

### Stadio buffer 74HC14 (nuovo v3)

- [ ] Riga 22 dx RailCom ↔ 74HC14 pin 11 (riga 13 sx main) : **continuo** (filo esterno)
- [ ] 74HC14 pin 10 (riga 14 sx main) ↔ 74HC14 pin 13 (riga 11 sx main) : **continuo** (ponte)
- [ ] 74HC14 pin 12 (riga 12 sx main) ↔ GPIO5 ESP32 : **continuo**
- [ ] Riga 22 dx RailCom ↔ GPIO5 : **NON continuo diretto** (ora passa per il chip)
- [ ] 74HC14 pin 11 (riga 13 sx) ↔ 74HC14 pin 10 (riga 14 sx) : **NON continuo** (ingresso e uscita gate 5 separati)
- [ ] 74HC14 pin 13 (riga 11 sx) ↔ 74HC14 pin 12 (riga 12 sx) : **NON continuo** (ingresso e uscita gate 6 separati)

### Misure a 3.3 V alimentato

- [ ] Rail + (RailCom) = 3.3 V
- [ ] Rail − (RailCom) = 0 V
- [ ] Riga 25 sx (VREF) = ~33 mV
- [ ] Riga 20 dx (SHDN) = 0 V (deve essere solido a GND)
- [ ] Riga 22 dx (TLV3501 OUT) con SENSE = 0 V → **HIGH (~3.3 V, idle)**
- [ ] 74HC14 pin 10 (uscita gate 5) in idle → **LOW** (inversione di pin 11 HIGH)
- [ ] 74HC14 pin 12 (uscita gate 6) in idle → **HIGH** (inversione di pin 13 LOW)
- [ ] GPIO5 in idle → **HIGH** (3.3 V, UART idle)

Se le due uscite intermedie (pin 10 e pin 12) rispettano questa sequenza HIGH-LOW-HIGH-LOW-HIGH dalla catena, il buffer funziona.

---

## Sezione 12 — Tabella stati (comportamento atteso)

Polarità invariata rispetto a v2 perché la doppia inversione del buffer è trasparente.

| Stato | GPIO18 | Q1, Q2 | BTS7960 | SENSE NODE | TLV3501 OUT (riga 22 dx) | 74HC14 pin 10 | 74HC14 pin 12 | GPIO5 |
|-------|--------|--------|---------|------------|--------------------------|---------------|---------------|-------|
| DCC normale | HIGH | OFF | Attivo ±15 V | 0 V | HIGH | LOW | HIGH | HIGH (idle) |
| Cutout, decoder risponde (+) | LOW | ON | Disabilitato | +180 mV | LOW | HIGH | LOW | LOW (dati UART) |
| Cutout, decoder risponde (−) | LOW | ON | Disabilitato | −180 mV | HIGH | LOW | HIGH | HIGH (perso) |
| Cutout, nessuna risposta | LOW | ON | Disabilitato | 0 V | HIGH | LOW | HIGH | HIGH (idle) |
| E-stop / fault | LOW | ON | Disabilitato | 0 V | HIGH | LOW | HIGH | HIGH (idle) |
| Boot (GPIO18 non configurato) | LOW default | OFF (100K pull-down) | Disabilitato | 0 V | HIGH | LOW | HIGH | HIGH (idle) |

---

## Sezione 13 — Budget elettrico (invariato dalla v2)

| Parametro | Valore | Limite NMRA S-9.3.2 | Conforme |
|-----------|--------|---------------------|----------|
| Burden a 34 mA | (4.7 + 1 + 0.044) × 34 mA = **195 mV** | ≤ 200 mV | ✅ |
| Soglia rilevamento | 33 mV / 4.7R = **7 mA** | gap 6-10 mA | ✅ |
| Rise/fall TLV3501 | **4.5 ns** | ≤ 0.5 μs | ✅ (ampiamente) |
| Propagazione 74HC14 (per gate, 3.3 V) | ~15 ns tipici | — | trascurabile per UART 250 kbps |
| Ritardo totale buffer (gate 5 + gate 6) | ~30 ns | — | trascurabile |
| Durata cutout | 460 μs | 454-488 μs | ✅ |
| Return path | 2 × Rds_on = 44 mΩ | bassa impedenza | ✅ |

Il ritardo aggiunto dal doppio inverter (~30 ns) è del tutto irrilevante rispetto al bit time UART RailCom (250 kbps → 4 μs/bit). L'impatto è inferiore all'1% della finestra di campionamento.

---

## Sezione 14 — Limitazioni note

1. **Rilevamento single-ended (50%):** invariato rispetto a v2. Senza ponte a diodi, il comparatore rileva solo una direzione della corrente RailCom. ~50% delle finestre saranno vuote. Se **tutte** le finestre sono vuote, scambia i fili TRACK_A e TRACK_B.
2. **Isteresi consigliata:** ora meno critica perché il buffer 74HC14 aggiunge Schmitt hysteresis in cascata. Se vedi byte errati, aggiungi comunque R_hyst 47K da riga 22 dx a riga 22 sx.
3. **Overlap timing al cutout start:** invariato. I MOSFET si accendono ~0.4 μs prima che il BTS7960 si disabiliti (~3-10 μs). I resistori R_sense (4.7R) e R_limit (1R) limitano il surge a livelli sicuri.

---

## Sezione 15 — Risoluzione problemi

| Sintomo | Causa probabile | Soluzione |
|---------|----------------|-----------|
| GPIO5 resta LOW anche senza DCC | Chip TLV3501 in shutdown (SHDN flottante) o buffer invertito al contrario | Controlla ponticello riga 20 dx → rail −; verifica che il filo TLV3501 OUT vada a pin 11 (input) e non a pin 10 (output) |
| GPIO5 resta HIGH sempre, nessun dato | Polarità sbagliata (50%) | Scambia TRACK_A ↔ TRACK_B sui MOSFET |
| Segnale DCC distorto | MOSFET bloccato ON | Controllare pull-down 100K sul gate; 74HC14 pin 8 deve essere LOW durante DCC |
| rx_empty = rx_windows (tutte vuote) | Decoder non risponde o polarità sbagliata | Verifica burden con oscilloscopio su SENSE NODE; prova a scambiare TRACK_A/TRACK_B |
| GPIO5 resta LOW durante DCC | SENSE NODE non a 0 V, oppure ponte pin 10 ↔ pin 13 aperto (gate 6 input flottante → uscita indeterminata) | Verifica Q1 OFF (gate a 0 V); verifica ponte gate 5 → gate 6 con tester |
| Corto / breadboard calda | Piastre MOSFET si toccano | Distanzia i MOSFET; verifica continuità tra Drain Q1 e Drain Q2 |
| Byte errati / rx_err alto | Rumore | Aggiungi R_hyst 47K (riga 22 dx → riga 22 sx); accorcia i fili |
| Oscillazioni rapide su GPIO5 | Buffer 74HC14 con ingresso flottante | Verifica che il filo da riga 22 dx RailCom entri davvero in pin 11 e che il ponte pin 10 → pin 13 sia in contatto |
| Pin 10 (out gate 5) segue pin 11 (in gate 5) senza invertire | Gate 74HC14 danneggiato o alimentazione mancante | Verifica pin 14 = 3.3 V, pin 7 = 0 V; sostituisci 74HC14 |
| Output TLV3501 sempre ~1.6 V (metà tensione) | SHDN non a GND, chip in shutdown | Controlla ponticello riga 20 dx → rail − |

---

## Appendice A — Riferimento chip

- **Datasheet TLV3501:** Texas Instruments TLV3501/TLV3502, 4.5-ns Rail-to-Rail High-Speed Comparator (https://www.ti.com/product/TLV3501)
- **Part number montato:** TLV3501**AIDR** (A = grade, I = industrial temp, D = SOIC-8, R = tape-and-reel)
- **Breakout:** ProtoAdvantage PA0002 (SOIC-8 300 mil → DIP-8 passivo, 1:1)
- **Adattatore DIP:** ASM-HELP-300
- **Datasheet 74HC14:** Nexperia / TI 74HC14, Hex Schmitt-Trigger Inverter

**Waveforms catturate:** `C:\Users\crist\OneDrive\Desktop\Forme onda dcc-railcom\`
