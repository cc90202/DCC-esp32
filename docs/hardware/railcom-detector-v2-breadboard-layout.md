# Detector RailCom v2 — Layout Breadboard Bianca (30 righe, rail +/−)

Ultimo aggiornamento: 2026-04-21

Breadboard dedicata al detector RailCom v2. Collegata alla breadboard principale (ESP32) tramite 4 fili di segnale + 2 di alimentazione.

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
| 6 | 22 dx | **OUT** (push-pull) | Filo esterno a GPIO5 |
| 7 | 21 dx | **V+** (3.3 V) | Jumper al rail + |
| 8 | 20 dx | **SHDN** (shutdown active-HIGH) | **Jumper al rail −** (OBBLIGATORIO) |

⚠️ Il pin SHDN è **active-HIGH**: se lo lasci flottante, il chip può andare in shutdown a caso. Devi **per forza** metterci un ponticello corto da riga 20 destra al rail −.

---

## Sezione 3 — Se stai migrando da LM393: cosa fare passo-passo

Se sulla breadboard hai ancora l'LM393 (DIP-8) sulle stesse righe 20-23, segui questa lista. Fai le operazioni **in ordine**, spuntandole man mano.

### 3a. Smontaggio vecchio hardware

- [ ] Staccare l'LM393 dallo zoccolo / righe 20-23 (estrarre delicatamente con leva)
- [ ] Rimuovere il resistore **pull-up 2.2K** (era tra rail + e riga 20 sinistra, pin 1 OUT1 dell'LM393). Il TLV3501 è push-pull, non serve più.
- [ ] Rimuovere eventuali jumper **a GND** sui pin inutilizzati dell'LM393 (erano su righe 22 dx, 23 dx, e 21 dx per pin 5, 6, 7)
- [ ] Rimuovere il jumper **V+ dell'LM393**: era da riga 20 destra (pin 8 LM393) al rail +. Va rimosso perché su TLV3501 il V+ è su un'altra riga.

Alla fine di questo passo la zona righe 20-23 deve essere vuota, tranne (forse) i jumper di IN− e IN+ sul lato sinistro — quelli li lasciamo e ci riattacchiamo.

### 3b. Montaggio TLV3501

- [ ] Inserire il breakout PA0002 (con TLV3501 saldato) nell'adattatore DIP, se non è già infilato
- [ ] Piantare il breakout sulla breadboard a cavallo del canale centrale, righe **20-23**, **pin 1 verso riga 20** (verifica serigrafia PA0002)
- [ ] Controllare che tutte le 8 zampette siano entrate bene nei fori (nessuna piegata sotto)

### 3c. Fili da spostare

- [ ] Filo **GPIO5 → OUT**: era su riga 20 sinistra (OUT1 LM393). Sposta su **riga 22 destra** (pin 6 TLV3501).
- [ ] Jumper **V+ → rail +**: nuovo percorso. Metti un ponticello corto da **riga 21 destra** (pin 7) al rail + (3.3 V).
- [ ] Jumper **GND → rail −**: nuovo percorso. Metti un ponticello corto da **riga 23 sinistra** (pin 4) al rail − (GND).

### 3d. Fili da AGGIUNGERE (nuovi)

- [ ] **SHDN a GND**: ponticello corto da **riga 20 destra** (pin 8) al rail −. Questo fa stare il chip sempre acceso. Senza di questo il chip va in shutdown casuale e non funziona.
- [ ] **Condensatore bypass 100nF**: posizione ottima è **tra riga 20 destra (SHDN, tirata a GND) e riga 21 destra (V+)** — cavalca proprio sopra i due pin di alimentazione del chip, percorso cortissimo. (Se hai già il 100nF dell'LM393 lì, lascialo: è esattamente dove serve.) Alternativa equivalente: tra riga 21 destra (V+) e riga 23 sinistra (GND), oppure tra rail + e rail − vicino al breakout.

### 3e. Fili che NON si toccano (devono rimanere)

- [ ] Jumper **SENSE NODE → IN−**: riga 15 sinistra → riga 21 sinistra. Invariato (prima andava allo stesso posto sull'LM393).
- [ ] Jumper **VREF → IN+**: riga 25 sinistra → riga 22 sinistra. Invariato.
- [ ] Tutto il resto della breadboard (MOSFET, resistori sense/limit, partitore VREF, cablaggio 74HC14) è **invariato**.

### 3f. Pin NC (Not Connected)

- [ ] Riga 20 sx (pin 1) e riga 23 dx (pin 5): **non collegarci niente**. Non a GND, non a V+, non flottanti con fili vaganti. Lasciare le righe completamente libere.

---

## Sezione 4 — Mappa righe finale

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
| 22 | TLV3501 pin 6 (OUT) | Filo esterno a GPIO5 |
| 23 | TLV3501 pin 5 (NC) | *Niente* |

---

## Sezione 5 — MOSFET IRLZ44N (invariato)

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

## Sezione 6 — Lista Componenti

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

**Componenti RIMOSSI rispetto alla versione LM393:**

| Rif | Componente | Motivo |
|-----|-----------|--------|
| R_pullup | 2.2K | TLV3501 è push-pull, non serve pull-up sull'uscita |
| ~~Jumper GND~~ | — | L'LM393 aveva 3 pin inutilizzati tirati a GND; TLV3501 ha 2 NC che lasci flottanti e il pin SHDN che va a GND (1 solo jumper, non 3) |

---

## Sezione 7 — Fili tra breadboard principale e breadboard RailCom

| Filo | Da (breadboard principale) | A (breadboard RailCom) |
|------|---------------------------|------------------------|
| GND | Rail GND | Rail − |
| 3.3 V | Pin 3.3V ESP32 | Rail + |
| CUTOUT_INV | 74HC14 pin 8 (uscita gate 4) | Riga 3 destra |
| GPIO5 | GPIO5 ESP32 | **Riga 22 destra** (OUT TLV3501) |

⚠️ Cambio rispetto al setup LM393: il filo GPIO5 va alla **riga 22 destra** (con LM393 era riga 20 sinistra).

Jumper sulla breadboard principale (invariato): GPIO18 → 74HC14 pin 9 (ingresso gate 4).

---

## Sezione 8 — Fili dalla breadboard RailCom al BTS7960

| Filo | Da (breadboard RailCom) | A |
|------|------------------------|---|
| TRACK_A | Riga 2 sinistra | Morsetto M+ BTS7960 |
| TRACK_B | Riga 3 sinistra | Morsetto M− BTS7960 |

I fili verso i binari partono dai morsetti M+ e M− del BTS7960, non dalla breadboard RailCom. La breadboard RailCom si inserisce in parallelo (righe 2 e 3 sx si collegano ai fili M+/M− che già vanno ai binari).

---

## Sezione 9 — Jumper interni alla breadboard RailCom

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
| **Riga 20 dx (TLV3501 pin 8 SHDN)** | **Rail −** | **Abilitazione (active-LOW per enable)** |

---

## Sezione 10 — Verifica prima di alimentare

Checklist di continuità (multimetro in modalità buzzer):

- [ ] Riga 21 dx (V+) ↔ rail + : **continuo**
- [ ] Riga 23 sx (GND) ↔ rail − : **continuo**
- [ ] Riga 20 dx (SHDN) ↔ rail − : **continuo** (se aperto, il chip va in shutdown!)
- [ ] Riga 22 dx (OUT) ↔ GPIO5 ESP32 : **continuo**
- [ ] Riga 21 sx (IN−) ↔ riga 15 sx (SENSE NODE) : **continuo**
- [ ] Riga 22 sx (IN+) ↔ riga 25 sx (VREF) : **continuo**
- [ ] Rail + ↔ rail − : **NON continuo** (se suona, c'è un corto; controlla il bypass cap e il partitore VREF)
- [ ] Riga 20 sx (NC) e riga 23 dx (NC): **non collegate a niente**

Poi alimenta a 3.3 V e misura:
- [ ] Rail + = 3.3 V
- [ ] Rail − = 0 V
- [ ] Riga 25 sx (VREF) = ~33 mV
- [ ] Riga 20 dx (SHDN) = 0 V (deve essere a GND solido)
- [ ] Riga 22 dx (OUT) con SENSE = 0 V → output HIGH (~3.3 V, stato idle)

Se tutti i punti sono OK, sei pronto per fare girare i test RailCom dal firmware.

---

## Sezione 11 — Tabella stati (comportamento atteso)

| Stato | GPIO18 | Q1, Q2 | BTS7960 | SENSE NODE | TLV3501 OUT | GPIO5 |
|-------|--------|--------|---------|------------|-------------|-------|
| DCC normale | HIGH | OFF | Attivo ±15 V | 0 V | HIGH (3.3 V) | HIGH (idle) |
| Cutout, decoder risponde (+) | LOW | ON | Disabilitato | +180 mV | LOW (0 V) | LOW (dati UART) |
| Cutout, decoder risponde (−) | LOW | ON | Disabilitato | −180 mV | HIGH (3.3 V) | HIGH (perso) |
| Cutout, nessuna risposta | LOW | ON | Disabilitato | 0 V | HIGH (3.3 V) | HIGH (idle) |
| E-stop / fault | LOW | ON | Disabilitato | 0 V | HIGH (3.3 V) | HIGH (idle) |
| Boot (GPIO18 non configurato) | LOW default | OFF (100K pull-down) | Disabilitato | 0 V | HIGH (3.3 V) | HIGH (idle) |

---

## Sezione 12 — Budget elettrico (invariato dalla v2 precedente)

| Parametro | Valore | Limite NMRA S-9.3.2 | Conforme |
|-----------|--------|---------------------|----------|
| Burden a 34 mA | (4.7 + 1 + 0.044) × 34 mA = **195 mV** | ≤ 200 mV | ✅ |
| Soglia rilevamento | 33 mV / 4.7R = **7 mA** | gap 6-10 mA | ✅ |
| Rise/fall TLV3501 | **4.5 ns** | ≤ 0.5 μs | ✅ (ampiamente) |
| Durata cutout | 460 μs | 454-488 μs | ✅ |
| Return path | 2 × Rds_on = 44 mΩ | bassa impedenza | ✅ |

---

## Sezione 13 — Limitazioni note

1. **Rilevamento single-ended (50%):** senza ponte a diodi, il comparatore rileva solo una direzione della corrente RailCom. ~50% delle finestre saranno vuote. Se **tutte** le finestre sono vuote, scambia i fili TRACK_A e TRACK_B.
2. **Isteresi consigliata:** il TLV3501 è molto veloce (4.5 ns). Senza R_hyst (47K da riga 22 dx a riga 22 sx) il rumore elettrico vicino alla soglia potrebbe causare commutazioni spurie. Se vedi byte errati, aggiungi R_hyst.
3. **Overlap timing al cutout start:** i MOSFET si accendono ~0.4 μs prima che il BTS7960 si disabiliti (~3-10 μs). I resistori R_sense (4.7R) e R_limit (1R) limitano il surge a livelli sicuri (3.2 A e 15 A rispettivamente).

---

## Sezione 14 — Risoluzione problemi

| Sintomo | Causa probabile | Soluzione |
|---------|----------------|-----------|
| GPIO5 resta LOW anche senza DCC | Chip in shutdown (SHDN flottante) | Controlla ponticello riga 20 dx → rail − |
| GPIO5 resta HIGH sempre, nessun dato | Polarità sbagliata (50%) | Scambia TRACK_A ↔ TRACK_B sui MOSFET |
| Segnale DCC distorto | MOSFET bloccato ON | Controllare pull-down 100K sul gate; 74HC14 pin 8 deve essere LOW durante DCC |
| rx_empty = rx_windows (tutte vuote) | Decoder non risponde o polarità sbagliata | Verifica burden con oscilloscopio su SENSE NODE; prova a scambiare TRACK_A/TRACK_B |
| GPIO5 resta LOW durante DCC | SENSE NODE non a 0 V | Verifica Q1 OFF (gate a 0 V); controlla 4.7R collegato a rail − |
| Corto / breadboard calda | Piastre MOSFET si toccano | Distanzia i MOSFET; verifica continuità tra Drain Q1 e Drain Q2 |
| Byte errati / rx_err alto | Rumore, manca isteresi | Aggiungi R_hyst 47K (riga 22 dx → riga 22 sx); accorcia i fili |
| Oscillazioni rapide su GPIO5 | TLV3501 oscilla vicino alla soglia | Aggiungi R_hyst 47K; verifica C_bypass 100nF vicino ai pin V+/GND del chip |
| Output TLV3501 sempre ~1.6 V (metà tensione) | SHDN non a GND, chip in shutdown | Controlla ponticello riga 20 dx → rail − |

---

## Appendice A — Riferimento chip

- **Datasheet:** Texas Instruments TLV3501/TLV3502, 4.5-ns Rail-to-Rail High-Speed Comparator (https://www.ti.com/product/TLV3501)
- **Part number montato:** TLV3501**AIDR** (A = grade, I = industrial temp, D = SOIC-8, R = tape-and-reel)
- **Breakout:** ProtoAdvantage PA0002 (SOIC-8 300 mil → DIP-8 passivo, 1:1)
- **Adattatore DIP:** ASM-HELP-300

**Waveforms catturate:** `C:\Users\crist\OneDrive\Desktop\Forme onda dcc-railcom\`
