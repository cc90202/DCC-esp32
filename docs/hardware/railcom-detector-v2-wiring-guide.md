# Detector RailCom v2 — Guida Cablaggio Breadboard

Ultimo aggiornamento: 2026-04-21

Stato: **POC conforme NMRA** — burden 195mV < 200mV, nessun diodo nel percorso corrente RailCom.

Sostituisce il design v1 (ponte a diodi + burden) che superava il budget NMRA di 200mV e impediva al decoder ESU LokSound 5 di trasmettere.

Per il layout fisico passo-passo sulla breadboard (righe, pin, lista operazioni di migrazione da LM393), vedere `railcom-detector-v2-breadboard-layout.md`.

## Panoramica

Questo circuito aggiunge cutout dedicato + detector RailCom alla command station DCC esistente. A differenza del v1, il return path per la corrente RailCom passa attraverso 2 MOSFET esterni (non un ponte a diodi), eliminando le cadute dei diodi dal percorso della corrente.

### Differenze rispetto al v1

| Aspetto | v1 (ponte a diodi) | v2 (MOSFET cutout) |
|---------|--------------------|--------------------|
| Return path | 4x 1N5819 bridge + MOSFET | 2x IRLZ44N diretti |
| Burden a 34mA | ~940mV (2 diodi + 10R) | **195mV** (4.7R + 1R + Rds_on) |
| Conforme NMRA | No | **Si** |
| UART durante DCC | Spazzatura (V+ alto) | **Idle** (sense = 0V) |
| Polarita' rilevamento | Entrambe (ponte rettifica) | Solo una (~50% finestre vuote) |
| Componenti attivi nel path | 2 diodi + 1 MOSFET | 2 MOSFET (no diodi) |

### Diagramma a blocchi

```
GPIO18 ──┬──> BTS7960 R_EN + L_EN (come oggi)
         │
         └──> 74HC14 gate 4 (pin 9 -> pin 8, invertito)
                    │
                    └──> 220R ──┬── gate Qc1
                                ├── gate Qc2
                                └── 100K ── GND

Binario A ── Qc1 drain ── Qc1 source ── 4.7R ── GND
                                  │
                             SENSE NODE ──> TLV3501 pin 2 (IN-)

Binario B ── Qc2 drain ── Qc2 source ── 1R ── GND

TLV3501 pin 3 (IN+) ── Vref (33mV)
TLV3501 pin 6 (OUT)  ──────────── GPIO5 (push-pull, no pull-up)
TLV3501 pin 8 (SHDN) ── GND (OBBLIGATORIO, altrimenti chip in shutdown)
```

### Come funziona

**DCC normale (GPIO18 HIGH):**
- BTS7960 attivo, pilota i binari a 0V / +15V alternati
- 74HC14 pin 8 = LOW -> MOSFET OFF
- Nessuna connessione tra binari e GND (i MOSFET sono spenti)
- SENSE NODE = 0V (tirato a GND da 4.7R, nessuna corrente)
- TLV3501 output = HIGH (idle) -> GPIO5 = 3.3V -> UART idle
- I body diode dei MOSFET NON conducono perche' i binari sono sempre tra 0V e +15V (il BTS7960 con B- = GND non produce mai tensioni negative sui binari)

**Cutout (GPIO18 LOW):**
- BTS7960 disabilitato (R_EN = L_EN = LOW -> M+, M- alta impedenza)
- 74HC14 pin 8 = HIGH -> MOSFET ON
- Qc1 cortocircuita Binario A a GND attraverso 4.7R
- Qc2 cortocircuita Binario B a GND attraverso 1R
- Se il decoder trasmette corrente da Binario A verso Binario B:
  - Percorso: Track A -> Qc1 -> 4.7R -> GND -> 1R -> Qc2 -> Track B
  - SENSE NODE = 4.7R x I (positivo, sopra GND)
  - A 30mA: SENSE = +141mV > Vref (33mV) -> TLV3501 output LOW -> GPIO5 LOW -> UART rileva
- Se il decoder trasmette nella direzione opposta:
  - SENSE NODE = -141mV (sotto GND)
  - TLV3501 output = HIGH -> GPIO5 = 3.3V -> UART idle (finestra persa)

**E-stop / fault (GPIO18 LOW permanente):**
- BTS7960 disabilitato, MOSFET ON
- Entrambi i binari a ~0V (scaricati a GND attraverso i MOSFET)
- Sicuro: nessuna tensione sul binario

## Overlap di timing al cutout start

Quando GPIO18 va LOW, i MOSFET si accendono (~0.4us) PRIMA che il BTS7960 si sia disabilitato (~3-10us). Durante questo overlap, un MOSFET potrebbe cortocircuitare a GND un binario ancora pilotato a +15V.

| Percorso | Surge massimo | Protezione |
|----------|---------------|------------|
| Qc1 (attraverso 4.7R) | 15V / 4.7R = **3.2A** | 4.7R limita la corrente |
| Qc2 (attraverso 1R) | 15V / 1R = **15A** | 1R limita la corrente |

Entrambi i surge sono brevi (~5us) e dentro i limiti degli IRLZ44N. Il BTS7960 ha protezione sovracorrente interna che interviene in ~1us ai livelli piu' alti.

Al cutout END non c'e' overlap: i MOSFET si spengono (~0.6us) prima che il BTS7960 si riabiliti (~3-5us).

## Body Diode dei MOSFET

I body diode degli IRLZ44N (da source a drain) NON conducono durante il DCC normale perche':
- Il BTS7960 con B- = GND produce tensioni tra 0V e +15V su ogni binario
- I binari non vanno MAI sotto 0V rispetto a GND
- I body diode conducono solo quando drain < source - 0.7V, cioe' quando il binario va sotto -0.7V
- Questo non succede con alimentazione unipolata (B- = GND)

Nota: con alimentazione simmetrica (+/-15V), i binari andrebbero negativi e servirebbero MOSFET back-to-back. Ma col BTS7960 non e' necessario.

## Lista Componenti

| Rif | Componente | Valore | Scopo |
|-----|-----------|--------|-------|
| Qc1 | IRLZ44N | 55V, logic-level | Switch cutout Binario A + sense |
| Qc2 | IRLZ44N | 55V, logic-level | Switch cutout Binario B |
| R_sense | Resistore | **4.7R** 1/2W | Sensing corrente RailCom (burden) |
| R_limit | Resistore | **1R** 1/2W | Limitatore surge su Qc2 |
| R_gate | Resistore | **220R** | Serie ai gate MOSFET (limita oscillazioni) |
| R_gate_pd | Resistore | **100K** | Pull-down gate (sicurezza al boot) |
| R_ref_high | Resistore | **100K** | Gamba superiore divisore Vref |
| R_ref_low | Resistore | **1K** | Gamba inferiore divisore Vref |
| R_hyst | Resistore | **47K** | Isteresi (feedback OUT -> IN+), opzionale |
| U2 | TLV3501AIDR | SOIC-8 su breakout PA0002 (DIP-8) | Comparatore veloce (4.5ns), push-pull output |
| C_bypass | Condensatore | **100nF** | Bypass alimentazione TLV3501 |

**Nota:** il TLV3501 ha uscita **push-pull** — NON serve resistore di pull-up (a differenza dell'LM393 che aveva uscita open-collector). L'uscita commuta tra 0V e 3.3V con fronti da 4.5ns.

## Identificazione Componenti

### MOSFET IRLZ44N (TO-220)

Guardandolo di fronte (scritta leggibile, piastra metallica dietro):

```
      +----------+
      | IRLZ44N  |
      |          |
      +--+--+--++
         |  |  |
         G  D  S

     G = Gate   (sinistra)  controlla ON/OFF
     D = Drain  (centro)    collegamento al binario
     S = Source (destra)     collegamento al sense/GND
```

### Comparatore TLV3501AIDR (SOIC-8 su breakout PA0002)

Il TLV3501AIDR e' un chip SMD in package SOIC-8 (8 zampette, 4 per lato) montato su un breakout ProtoAdvantage PA0002 che lo adatta a un footprint DIP-8 a passo 2.54mm per breadboard. Il breakout e' passivo 1:1 (pin N del SOIC = pin N del DIP).

**Pinout TLV3501 SOIC-8 (da datasheet TI):**

```
     +-------+
  1  | NC    |  8  SHDN  (active-HIGH shutdown -> tie a GND)
  2  | IN-   |  7  V+
  3  | IN+   |  6  OUT
  4  | V-    |  5  NC
     +-------+
```

**Breakout su breadboard (a cavallo del canale centrale, pin 1 verso riga 20):**

```
  Sinistra          Destra
  pin 1 (NC)        pin 8 (SHDN)   riga 20
  pin 2 (IN-)       pin 7 (V+)     riga 21
  pin 3 (IN+)       pin 6 (OUT)    riga 22
  pin 4 (V-/GND)    pin 5 (NC)     riga 23
```

IMPORTANTE: il pin **SHDN (pin 8) e' active-HIGH shutdown**. Va OBBLIGATORIAMENTE tirato a GND (con un ponticello da riga 20 dx al rail -), altrimenti il chip entra in shutdown a caso. I pin **NC (1 e 5) devono restare flottanti** (niente fili).

**Differenze chiave rispetto all'LM393:**

| | LM393 (vecchio) | TLV3501AIDR (nuovo) |
|---|---|---|
| Package | DIP-8 nativo | SOIC-8 su breakout PA0002 (DIP-8 passivo) |
| Pin | 8 (2 comparatori) | 8 (1 comparatore + SHDN + 2 NC) |
| Uscita | Open-collector (serve pull-up) | **Push-pull** (no pull-up) |
| Rise/fall | ~1300ns | **4.5ns** (~290x piu' veloce) |
| Righe breadboard | 4 (righe 20-23) | 4 (righe 20-23) |
| SHDN | assente | pin 8 -> tie a GND |

## Circuito Dettagliato

### Blocco 1: Gate Drive (74HC14)

La 74HC14 gate 4 (pin 9 ingresso, pin 8 uscita) inverte GPIO18. Il jumper GPIO18 -> pin 9 e' gia' presente dal cablaggio v1.

| Da | A | Componente |
|----|---|-----------|
| 74HC14 pin 8 | Gate Qc1 e Qc2 | Attraverso R_gate 220R (condiviso) |
| Gate Qc1/Qc2 | GND | Attraverso R_gate_pd 100K |

### Blocco 2: Cutout Switch + Sense (Qc1)

Qc1 cortocircuita Binario A a GND attraverso il resistore di sense 4.7R.

| Da | A | Componente/Filo |
|----|---|----------------|
| Binario A (filo da BTS7960 M+) | Qc1 Drain | Filo jumper |
| Qc1 Source | GND | Attraverso R_sense 4.7R |
| Qc1 Source (SENSE NODE) | TLV3501 pin 2 (IN-) | Filo jumper |

### Blocco 3: Cutout Switch + Limitatore (Qc2)

Qc2 cortocircuita Binario B a GND attraverso il resistore limitatore 1R.

| Da | A | Componente/Filo |
|----|---|----------------|
| Binario B (filo da BTS7960 M-) | Qc2 Drain | Filo jumper |
| Qc2 Source | GND | Attraverso R_limit 1R |

### Blocco 4: Divisore di Riferimento (Vref ~ 33mV)

```
3.3V --100K--+--1K-- GND
             |
             +----> TLV3501 pin 3 (IN+)
```

Vref = 3.3V x 1K / (100K + 1K) = 32.7mV

Soglia di rilevamento: 32.7mV / 4.7R = 6.96mA. Lo standard NMRA specifica:
- Sopra 10mA = logico "0" (il decoder sta trasmettendo) -> 10mA x 4.7R = 47mV > 33mV, rilevato
- Sotto 6mA = logico "1" (riposo) -> 6mA x 4.7R = 28mV < 33mV, non rilevato

La soglia di 7mA si trova tra questi due livelli.

### Blocco 5: Comparatore TLV3501 — Connessioni

| Pin | Collegamento |
|-----|-------------|
| 1 (NC) | Flottante — niente filo |
| 2 (IN-) | SENSE NODE (Qc1 source) |
| 3 (IN+) | Vref (~33mV dal divisore) |
| 4 (V-) | GND |
| 5 (NC) | Flottante — niente filo |
| 6 (OUT) | Filo diretto a GPIO5 |
| 7 (V+) | 3.3V |
| 8 (SHDN) | GND (OBBLIGATORIO — active-HIGH shutdown) |

Condensatore bypass 100nF tra pin 7 (V+) e pin 4 (V-/GND), il piu' vicino possibile al chip.

**Isteresi (opzionale ma consigliata):** resistore 47K tra pin 6 (OUT) e pin 3 (IN+). Aggiunge feedback positivo per evitare oscillazioni vicino alla soglia. Con il TLV3501 cosi' veloce (4.5ns), senza isteresi il rumore potrebbe causare commutazioni spurie.

Con R_hyst = 47K:
- Soglia alta (per commutare da HIGH a LOW): ~102mV / 4.7R = ~22mA
- Soglia bassa (per commutare da LOW a HIGH): ~33mV / 4.7R = ~7mA
- Il segnale RailCom a 30mA supera ampiamente la soglia alta

**Polarita':** la configurazione SENSE su IN- e VREF su IN+ produce la polarita' UART corretta:
- Corrente alta (space/0): SENSE > Vref -> IN- > IN+ -> output LOW -> GPIO5 LOW (UART "0")
- Corrente bassa (mark/1): SENSE < Vref -> IN+ > IN- -> output HIGH -> GPIO5 HIGH (UART "1")
- Idle: SENSE = 0V -> output HIGH -> GPIO5 HIGH (UART idle)

## Budget Elettrico

| Parametro | Valore | Limite NMRA S-9.3.2 | Conforme |
|-----------|--------|---------------------|----------|
| Burden a 34mA | 4.7R + 1R + 2xRds_on = **195mV** | <= 200mV | **Si** |
| Soglia rilevamento | 7mA (33mV / 4.7R) | gap 6-10mA | Si |
| Rise/fall TLV3501 | **4.5ns** | <= 0.5us (500ns) | **Si, ampiamente conforme** |
| Durata cutout | 460us | 454-488us | Si |
| Return path | 2xRds_on = 44mohm | bassa impedenza | Si |
| Vgs MOSFET | 3.3V | soglia 1-2V | Completamente acceso |

## Riepilogo GPIO Dopo la Modifica

| GPIO | Funzione | Direzione | Note |
|------|----------|-----------|------|
| 2 | Generazione DCC | Uscita | Invariato |
| 3 | Rilevamento cortocircuito | Ingresso | Invariato |
| **5** | **RailCom UART RX** | **Ingresso** | Dal pin 6 (OUT) del TLV3501 — push-pull, no pull-up |
| 18 | Abilitazione H-bridge + cutout | Uscita | Invariato (ora pilota anche Qc1/Qc2 via 74HC14 gate 4) |
| 19 | OLED SDA | I/O | Invariato |
| 20 | OLED SCL | Uscita | Invariato |
| 21 | Pulsante Resume | Ingresso | Invariato |
| 22 | Pulsante Stop | Ingresso | Invariato |

## Utilizzo Gate 74HC14 Dopo la Modifica

| Gate | Pin | Funzione |
|------|-----|----------|
| 1 | 1 (in) / 2 (out) | Segnale DCC a LPWM |
| 2 | 3 (in) / 4 (out) | DCC invertito a RPWM |
| 3 | 5 (in) / 6 (out) | Soglia IS cortocircuito a GPIO3 |
| **4** | **9 (in) / 8 (out)** | **GPIO18 invertito -> gate Qc1 + Qc2 (switch cutout)** |
| 5 | 11 (in) / 10 (out) | Libero |
| 6 | 13 (in) / 12 (out) | Libero |

## Collegamento tra Breadboard

### Fili dalla breadboard principale alla breadboard nuova

| Filo | Da (breadboard principale) | A (breadboard nuova) |
|------|---------------------------|---------------------|
| GND | Rail GND | GND breadboard nuova |
| 3.3V | Pin 3.3V ESP32 (o riga 74HC14 pin 14) | 3.3V breadboard nuova |
| Cutout | 74HC14 pin 8 (uscita gate 4) | Gate Qc1 + Qc2 (attraverso 220R) |
| UART RX | GPIO5 ESP32 | Uscita TLV3501 pin 6 (OUT) |

### Fili dalla breadboard nuova al BTS7960

| Filo | Da (breadboard nuova) | A (BTS7960) |
|------|----------------------|-------------|
| Binario A | Qc1 Drain | Filo che va da M+ al binario |
| Binario B | Qc2 Drain | Filo che va da M- al binario |

### Jumper sulla breadboard principale (invariato da v1)

| Da | A | Scopo |
|----|---|-------|
| Nodo GPIO18 | Pin 9 della 74HC14 | Ingresso Gate 4 (gia' presente) |

## Firmware

Nessuna modifica firmware necessaria. GPIO18 continua a fare lo stesso toggle LOW/HIGH per cutout/DCC. Il significato elettrico cambia (da "enable BTS7960" a "enable BTS7960 + clamp binari a GND via MOSFET"), ma la logica e la temporizzazione restano identiche.

Il software RailCom (UART reader, parser, pipeline, window bridge, POM actor) e' gia' completo e testato.

## Risoluzione Problemi

| Sintomo | Causa probabile | Soluzione |
|---------|----------------|-----------|
| Segnale DCC distorto dopo l'aggiunta | MOSFET bloccato ON (body diode o gate) | Controllare pull-down 100K sul gate; verificare 74HC14 pin 8 = LOW durante DCC |
| rx_windows cresce ma rx_empty = rx_windows | Decoder non trasmette o polarita' sbagliata | Verificare burden con oscilloscopio; provare a invertire il filo su Qc1 (scambiare Binario A e B) |
| rx_ok = 0 ma rx_err cresce | Segnale troppo debole o rumoroso | Verificare Vref (~33mV); accorciare fili; aggiungere isteresi (47K da OUT a IN+) |
| GPIO5 resta LOW durante il DCC | SENSE NODE non a 0V | Verificare che Qc1 sia OFF (gate a 0V); controllare 4.7R collegato a GND |
| GPIO5 resta HIGH durante cutout | Polarita' sbagliata (50%) | Scambiare i fili Binario A e Binario B su Qc1 e Qc2 |
| Surge/fumo al cutout | Overlap timing senza limitatore | Verificare 1R su Qc2; verificare 4.7R su Qc1 |
| Oscillazioni rapide su GPIO5 | TLV3501 troppo veloce, rumore sulla soglia | Aggiungere R_hyst 47K (OUT -> IN+); accorciare filo SENSE; verificare bypass cap |
