# Breadboard Unica — Layout Modulare DCC + RailCom v3 (60 righe)

Ultimo aggiornamento: 2026-04-25

Stato: **proposta**, in attesa di montaggio fisico.

Questo documento descrive una **singola breadboard da 60 righe** che sostituisce le due breadboard attuali (DCC principale + RailCom). Il circuito è organizzato in **tre moduli funzionali indipendenti** che si montano e si testano in sequenza.

---

## Filosofia modulare

| Modulo | Cosa fa | Hardware aggiunto | Test finale del modulo |
|--------|---------|-------------------|------------------------|
| **1 — DCC base** | Genera DCC, alimenta binari, rileva cortocircuiti | ESP32 + 74HC14 (gates 1-3) + BTS7960 + trimmer 10K | Loco corre, scope vede onda DCC |
| **2 — RailCom v3** | Genera cutout 460 µs, rileva risposta decoder, instrada UART su GPIO5 | TLV3501 + 2× IRLZ44N + 74HC14 (gates 4-6) + partitore VREF + R_sense | Decoder risponde a POM |
| **3 — UI** | Display stato + LED + pulsanti manuali | OLED SSD1306 + LED verde/rosso + 2 pulsanti + 2 cap debounce | OLED mostra IP, LED indicano stato, pulsanti pausano/riprendono |

I moduli **2** e **3** sono opzionali e indipendenti l'uno dall'altro: puoi avere `1`, `1+2`, `1+3`, oppure `1+2+3`. Aggiungere il **Modulo 2** richiede di **deviare due fili** del Modulo 1 (M+/M− del BTS7960 passano per i MOSFET) e di collegare **GPIO4** al gate 4 del 74HC14 per il comando cutout. GPIO18 resta dedicato solo all'abilitazione del BTS7960.

Le sezioni **A, B, C** sono comuni e vanno lette prima di iniziare. I moduli sono nelle sezioni **M1, M2, M3**. Le sezioni **R1-R3** sono di riferimento (BOM, wire list, troubleshooting).

---

# Sezioni comuni

## A — Pre-requisiti hardware

Prima di cablare verifica questi tre punti, altrimenti le righe non tornano:

- [x] **Breadboard 60 righe con rail continui**: verificato col multimetro in modalità buzzer (continuità riga 1 ↔ riga 60 sul rail +). Le righe vuote ogni 5 fori sono solo marker visivi: il rail è continuo. **Nessun ponticello necessario.**
- [x] **ESP32-C6 Mini (Waveshare) con 9 pin per lato** = 18 pin totali, 9 righe occupate a cavallo del canale centrale. Confermato.
- [x] **GPIO esposti sul modulo**: GPIO2, GPIO3, GPIO4, GPIO5, GPIO18, GPIO19, GPIO20, GPIO21, GPIO22 + 3V3 + GND + 5V. Confermato — tutti i pin usati sono presenti sulla serigrafia del modulo Waveshare in uso.

---

## B — Schema alimentazioni

Sulla breadboard nuova circolano **tre tensioni distinte** che devono restare separate:

| Tensione | Sorgente | Destinazione | Percorso |
|----------|----------|--------------|----------|
| **3.3 V** | ESP32 pin 3V3 | rail + breadboard → 74HC14 VCC, TLV3501 V+, partitore VREF, OLED VCC | jumper `ESP32 3V3 → rail +` (entrambi i lati del canale) |
| **5 V** | ESP32 pin 5V (USB VBUS) | BTS7960 pin Vcc (logica) | filo dedicato ESP32 → BTS7960, **fuori dalla breadboard** |
| **15 V** | alimentatore da banco | BTS7960 B+ (potenza binari) | morsetto + alimentatore → fusibile in serie → BTS7960 B+ |
| **GND** | comune | tutto | rail − breadboard, GND BTS7960, GND alimentatore — tutti collegati insieme |

Nota pratica per la massa: il TLV3501 legge segnali molto piccoli. Per questo il GND del RailCom deve arrivare al GND comune con un filo corto e pulito, non passando in mezzo ai ritorni di potenza dei binari. In pratica: fai incontrare GND breadboard, GND BTS7960 e negativo alimentatore in un punto solo vicino all'alimentatore/BTS7960.

### Differenza con la scheda attuale

Sulla breadboard precedente il 5 V dell'ESP32 era portato sul rail + e da lì raggiungeva il Vcc del BTS7960. Sulla breadboard nuova **questo schema non funziona più**, perché il rail + è ora occupato dai 3.3 V che alimentano 74HC14 e TLV3501.

Soluzione: il 5 V passa con un **filo dedicato** dal pin 5V dell'ESP32 direttamente al pin Vcc del BTS7960, senza mai toccare le strisce della breadboard.

### ⚠️ Errore da evitare

**Non mettere mai 5 V sul rail +.** Tutto quello che è collegato al rail + è progettato per 3.3 V. Se ci finisce 5 V, succedono tre cose brutte in cascata:

1. **GPIO5 ESP32 a 5 V** (Modulo 2): il 74HC14 alimentato a 5 V mette in uscita gate 6 un'oscillazione 0-5 V che entra in GPIO5. L'ESP32-C6 non è 5V-tolerant sui GPIO → danno permanente.
2. **VREF sbagliato** (Modulo 2): il partitore 100K/1K darebbe ~50 mV invece di 33 mV → soglia di rilevamento sale a ~10 mA, fuori dal gap NMRA 6-10 mA.
3. **DCC instabile** (Modulo 1): 74HC14 a Vcc=5 V richiede V_IH_min ≈ 3.5 V; GPIO2 a 3.3 V è sotto soglia → il primo gate (DCC) commuterebbe in modo erratico.

La verifica di continuità di ciascun modulo include un controllo esplicito che il rail + non sia in continuità col pin 5V dell'ESP32.

---

## C — Mappa zone breadboard

```
riga  1-9    ZONA A — ESP32-C6 Mini (USB verso riga 1)             [Modulo 1]
riga  9-11   ZONA PULSANTI — debounce Stop/Resume lato dx              [Modulo 3]
riga 10-12   trimmer 10K IS a cavallo del canale (outer sx, cursore dx)  [Modulo 1]
riga 12-13   corridoio jumper ESP32 ↔ 74HC14
riga 14-20   ZONA B — 74HC14 DIP-14                                [Modulo 1+2]
riga 21-24   ZONA LED — verde 21-22 dx, rosso 23-24 dx     [Modulo 3]
riga 25-28   ZONA C — TLV3501 PA0002 DIP-8                         [Modulo 2]
riga 29-35   ZONA D — VREF + SENSE NODE                            [Modulo 2]
riga 36-40   corridoio isolamento comparatore ↔ MOSFET
riga 41-43   ZONA E — MOSFET Q1 (IRLZ44N)                          [Modulo 2]
riga 44-48   corridoio placche MOSFET (5 righe di distanza)
riga 49-51   ZONA F — MOSFET Q2 (IRLZ44N)                          [Modulo 2]
riga 52-60   ZONA G — terminali TRACK_A / TRACK_B, spazio libero   [Modulo 2]
```

**Rail:** rail **+** (3.3 V) sul lato **superiore**, rail **−** (GND) sul lato **inferiore**, su entrambi i lati del canale centrale.

Costruendo solo il **Modulo 1**, le righe 21-24 lato dx possono ospitare i LED di stato DCC (vedi M3). Le righe 25-60 restano vuote e sono già allocate per l'eventuale aggiunta del Modulo 2 senza riprogettare la mappa. I LED stanno sul lato dx perché GPIO14 e GPIO15 sull'ESP32-C6 Mini sono sul lato dx (righe 3 e 4 dx), così il jumper di alimentazione del LED resta sullo stesso lato del canale.

### Motivazioni di posizionamento

- **ESP32 in alto** → USB verso il bordo, non ostacola il cablaggio.
- **74HC14 subito sotto l'ESP32** → GPIO2/3/4/5/18 fanno salti corti attraverso il corridoio righe 10-13.
- **TLV3501 sotto il 74HC14** → il filo OUT → pin 11 (gate 5) del 74HC14 resta corto e non attraversa zone ad alta corrente.
- **Partitore VREF e SENSE NODE al centro** → il percorso SENSE → IN− resta minimo (~4 righe), riducendo la captazione di disturbi DCC.
- **MOSFET in fondo** → la zona "alta corrente" (surge fino a 15 A al cutout start su Q2) è fisicamente lontana dal comparatore. I fili ai Drain sono corti perché la Zona G è adiacente.
- **Corridoi vuoti** → margine per correzioni senza riprogettare, e isolamento tra sezioni sensibili e rumorose.

---

# M1 — Modulo DCC base

**Obiettivo:** generare il segnale DCC su GPIO2, invertirlo correttamente, pilotare il BTS7960 verso i binari, rilevare cortocircuiti su GPIO3.

**Funziona da solo:** sì. Al termine del Modulo 1 hai una centralina DCC operativa.

## M1.1 — Componenti del modulo

**Sulla breadboard:**

| Rif | Componente | Posizione |
|-----|-----------|-----------|
| U1 | ESP32-C6 Mini Waveshare | Righe 1-9, USB verso riga 1 |
| U2 | 74HC14 DIP-14 | Righe 14-20, pin 1 verso riga 14 sx |
| Trimmer | 10 KΩ multigiri (BOJACK P103, 3 pin a triangolo) | A cavallo del canale, righe 10-12: outer pin a sx (10, 12), cursore a dx (11) |
| C_is | Condensatore 100 nF | Riga 18 sx ↔ rail − (filtro su pin 5 74HC14) |
| R_gpio3 | Resistore 1 KΩ | Riga 19 sx ↔ riga 11 sx |
| C2 | Condensatore 100 nF (consigliato) | Rail + ↔ rail − vicino al 74HC14 |

**Fuori dalla breadboard:**

| Rif | Componente | Note |
|-----|-----------|------|
| BTS7960 | H-bridge 43 A | logica 5 V, B+ 15 V, fusibile in serie su B+ |
| Alimentatore | 15 V banco | + fusibile in serie sulla linea B+ |

Del 74HC14 il Modulo 1 usa solo i **gate 1, 2, 3** (pin 1-7). I gate 4, 5, 6 (pin 8-13) restano scollegati e saranno usati dal Modulo 2.

## M1.2 — Cablaggio passo passo

### Step 1: pianta i due chip

- [ ] **ESP32-C6 Mini** a cavallo del canale centrale, **USB verso riga 1**, righe 1-9.
- [ ] **74HC14 DIP-14** a cavallo del canale centrale, **pin 1 sulla riga 14 sx** (verifica la tacca o il pallino sul package).

Annota su una tabella personale a quale riga corrisponde ciascun GPIO della tua ESP32: la posizione esatta dipende dal modulo specifico Waveshare in tuo possesso.

### Step 2: alimentazione (la più importante)

- [ ] **Jumper 3V3 ESP32 → rail +** (corto, di colore rosso)
- [ ] **Jumper GND ESP32 → rail −** (corto, di colore nero)
- [ ] **Filo diretto ESP32 5V → Vcc BTS7960**: passa **fuori dalla breadboard**, NON tocca nessun rail. Lunghezza tale da raggiungere comodamente la scheda BTS7960.
- [ ] **Jumper rail + → 74HC14 pin 14** (riga 14 dx → rail +)
- [ ] **Jumper 74HC14 pin 7 → rail −** (riga 20 sx → rail −)
- [x] **Bypass C2 100 nF**: tra rail + e rail − all'altezza della **riga 15** — il più vicino possibile al pin 14 (VCC) del 74HC14 che sta sulla riga 14 dx. Posizione confermata in montaggio.

### Step 3: cascata gate 1 → gate 2 (per ottenere RPWM)

Il segnale DCC esce da GPIO2 in un certo livello logico. Il gate 1 lo inverte (LPWM), il gate 2 lo re-inverte (RPWM): le due uscite pilotano i due semibridge del BTS7960 in modo complementare.

- [ ] **Ponte breve** tra **riga 15 sx (pin 2)** e **riga 16 sx (pin 3)** del 74HC14.

### Step 4: protezione GPIO3 (catena cortocircuito)

- [ ] **Resistore 1 KΩ** con una gamba su **riga 19 sx** (pin 6 del 74HC14) e l'altra su **riga 11 sx** (landing pad nel corridoio).
- [ ] **Jumper riga 11 sx → GPIO3** dell'ESP32.

Il resistore protegge GPIO3 se per errore venisse configurato come output mentre il 74HC14 pilota la stessa linea.

### Step 5: trimmer IS + filtro (catena cortocircuito, lato analogico)

Il trimmer 10 KΩ BOJACK P103 ha **3 pin a triangolo**: due outer pin sullo stesso lato del corpo (distanziati ~5 mm = 2 fori) e il cursore sul lato opposto, sfalsato di una riga. **Va piantato a cavallo del canale centrale**, corpo lungo le righe 10-12. Ruota il trimmer finché i pin entrano nei fori indicati qui sotto:

```
       lato sx           canale          lato dx
       a b c d e            |            f g h i j

riga 10  . . . . ●          |          . . . . .   ← outer pin
riga 11  . . . . .          |          ● . . . .   ← cursore
riga 12  . . . . ●          |          . . . . .   ← outer pin
```

I **2 pin allineati** del triangolo finiscono in **colonna E (ultima a sinistra), righe 10 e 12**. Il **cursore** finisce in **colonna F (prima a destra), riga 11**. Se non entrano in questa configurazione, ruota il trimmer di 90° o 180° finché ci entrano.

Disposizione finale:

| Pin trimmer | Riga / lato | Ruolo |
|-------------|-------------|-------|
| Outer A | 10 sx | **USATO** — segnale IS |
| Outer B | 12 sx | **FLOTTANTE** — nessun collegamento |
| Cursore (wiper) | 11 dx | a rail − dx |

Cablaggio:

- [ ] **Trimmer a cavallo del canale, righe 10-12**, due outer pin a sinistra (righe 10 sx e 12 sx), cursore a destra (riga 11 dx).
- [ ] **Jumper riga 11 dx (cursore) → rail − dx**.
- [ ] **Jumper riga 10 sx (Outer A) → riga 18 sx (74HC14 pin 5)** — porta la tensione di sense al gate 3 in.
- [ ] **C_is 100 nF** tra **riga 18 sx (74HC14 pin 5)** e **rail − sx** — filtra le commutazioni DCC sul nodo IS.

Il pin Outer B (riga 12 sx) resta scollegato (configurazione "reostato": solo cursore + un outer pin in serie). Il filo da BTS7960 R_IS+L_IS arriverà sulla riga 10 sx nello Step 6.

### Step 6: fili esterni del Modulo 1

| Da | A | Lato/posizione |
|----|---|----------------|
| ESP32 GPIO2 | 74HC14 pin 1 | riga 14 sx |
| 74HC14 pin 2 | BTS7960 LPWM | riga 15 sx → BTS7960 |
| 74HC14 pin 4 | BTS7960 RPWM | riga 17 sx → BTS7960 |
| BTS7960 R_IS | riga 10 sx | filo diretto |
| BTS7960 L_IS | riga 10 sx | filo diretto (arriva sulla stessa riga) |
| ESP32 GPIO3 | riga 11 sx | jumper diretto |
| ESP32 GPIO18 | BTS7960 R_EN | filo diretto |
| ESP32 GPIO18 | BTS7960 L_EN | filo diretto (secondo filo dallo stesso GPIO18) |
| ESP32 GND | BTS7960 GND (oltre al rail −) | filo diretto, costituisce il return path comune |
| Alimentatore 15 V + | fusibile → BTS7960 B+ | esterno, **scollegato finché test M1.4 non completati** |
| Alimentatore 15 V − | BTS7960 B− | esterno |
| BTS7960 M+ | binario A | filo verso WAGO/morsetti binari |
| BTS7960 M− | binario B | filo verso WAGO/morsetti binari |

⚠️ Le uscite logiche del BTS7960 (LPWM, RPWM, R_EN, L_EN) accettano 3.3 V in ingresso anche se Vcc=5 V — il livello soglia è ~2.5 V.

## M1.3 — Verifica continuità (alimentazione SCOLLEGATA)

Multimetro in modalità buzzer.

### Alimentazione

- [ ] 3V3 ESP32 ↔ rail + : **continuo**
- [ ] GND ESP32 ↔ rail − : **continuo**
- [ ] **5V ESP32 ↔ rail +** : **NON continuo**
- [ ] **5V ESP32 ↔ Vcc BTS7960** : **continuo** (filo diretto)
- [ ] **Vcc BTS7960 ↔ rail +** : **NON continuo** (5 V e 3.3 V devono restare separati)
- [ ] pin 14 74HC14 ↔ rail + : **continuo**
- [ ] pin 7 74HC14 ↔ rail − : **continuo**
- [ ] Rail + ↔ rail − : **NON continuo**

### Segnali

- [ ] pin 2 (riga 15 sx) ↔ pin 3 (riga 16 sx) : **continuo** (ponte cascata)
- [ ] Riga 19 sx ↔ riga 11 sx : **NON continuo** (R 1K in mezzo)
- [ ] Ohmmetro riga 19 sx ↔ riga 11 sx : **~1000 Ω**
- [ ] Riga 11 sx ↔ GPIO3 : **continuo**

### Trimmer IS

- [ ] Riga 11 dx (cursore trimmer) ↔ rail − dx : **continuo**
- [ ] Riga 10 sx (Outer A) ↔ riga 18 sx (74HC14 pin 5) : **continuo**
- [ ] Riga 12 sx (Outer B) ↔ qualunque altro nodo : **NON continuo** (è flottante)
- [ ] Riga 18 sx ↔ rail − sx : **NON continuo** in continuità diretta (C_is in mezzo, ohmmetro alto in carica)
- [ ] Ohmmetro riga 10 sx ↔ rail − dx : tra 0 Ω e 10 KΩ a seconda di dove sta il cursore (se misuri esattamente 0 o 10 KΩ controlla che il trimmer non sia a fine corsa)

## M1.4 — Test alimentato (USB only, B+ 15 V SPENTO)

Connetti l'ESP32 alla USB del PC. Il BTS7960 ha già il Vcc 5 V dal filo dedicato ma B+ resta scollegato.

- [ ] Rail + = **3.3 V** (se misuri 5 V: stacca subito USB e ricontrolla la Sezione B!)
- [ ] Rail − = 0 V
- [ ] ESP32 pin 5V ↔ Vcc BTS7960 = **5 V**
- [ ] pin 14 74HC14 = 3.3 V, pin 7 = 0 V

## M1.5 — Test funzionale

1. Carica firmware DCC: `cargo run`.
2. Accendi l'alimentatore 15 V.
3. **Scope su GPIO2**: deve vedere onda DCC con bit "1" = 58 µs e "0" = ≥ 100 µs.
4. **Scope su BTS7960 M+**: ±15 V (rispetto a M−).
5. Comanda una loco da firmware o app Z21 → si muove.
6. **Test cortocircuito**: tocca brevemente i due binari con un fil di rame; il firmware deve rilevare il fault su GPIO3 e disabilitare il BTS7960 entro pochi ms.

Se tutti i test passano, il **Modulo 1 è completo**. Puoi fermarti qui (centralina DCC operativa) oppure proseguire al Modulo 2 per aggiungere RailCom.

---

# M2 — Modulo RailCom v3

**Obiettivo:** generare la finestra di cutout (~460 µs) cortocircuitando i binari su un sense resistor, rilevare la corrente del decoder con il TLV3501, instradare il bit RailCom su GPIO5 attraverso il buffer Schmitt a due stadi (gate 5 + gate 6 del 74HC14).

**Pre-requisito:** Modulo 1 montato e testato.

**Modifica retroattiva al Modulo 1:**

1. **GPIO18 resta invariato**: alimenta solo BTS7960 R_EN+L_EN.
2. **GPIO4** viene aggiunto come comando RailCom cutout: GPIO4 → 74HC14 pin 9.
3. **BTS7960 M+ e M−** non vanno più direttamente ai binari. Passano per i Drain dei MOSFET Q1 e Q2 sulla breadboard, che a loro volta vanno ai binari. Sposta i due fili.

## M2.1 — Componenti del modulo

**Sulla breadboard:**

| Rif | Componente | Posizione |
|-----|-----------|-----------|
| U3 | TLV3501AIDR su breakout PA0002 | Righe 25-28, pin 1 riga 25 sx |
| Q1 | IRLZ44N | Righe 41-43, lato destro, piastra fuori dalla breadboard |
| Q2 | IRLZ44N | Righe 49-51, lato destro, piastra fuori dalla breadboard |
| R_sense | Resistore 4.7 Ω, non induttivo, almeno 3 W | Riga 30 sx → rail − |
| R_limit | Resistore 1 Ω 1/2 W | Riga 51 dx → rail − |
| R_ref_high | Resistore 100 KΩ | Rail + → riga 33 sx |
| R_ref_low | Resistore 1 KΩ | Riga 33 sx → rail − |
| R_gate1 | Resistore 220 Ω | Riga 45 dx → riga 41 dx |
| R_gate2 | Resistore 220 Ω | Riga 45 dx → riga 49 dx |
| R_gate_pd1 | Resistore 100 KΩ | Riga 41 dx → rail − |
| R_gate_pd2 | Resistore 100 KΩ | Riga 49 dx → rail − |
| R_cutout_pu | Resistore 100 KΩ | Riga 19 dx → rail + |
| C1 | Condensatore 100 nF | Riga 25 dx ↔ riga 26 dx (bypass TLV3501) |
| C_vref | Condensatore 100 nF ceramico | Riga 33 sx (VREF) → rail − |
| D_tvs | TVS bidirezionale, es. P6KE18CA | Tra TRACK_A e TRACK_B, vicino ai fili binari |
| R_hyst | Resistore 47 KΩ (opzionale) | Riga 27 dx ↔ riga 27 sx |

Del 74HC14 il Modulo 2 usa i **gate 4, 5, 6** (pin 8-13).

### Cosa aggiungono le protezioni

- **D_tvs**: va messo tra i due fili dei binari. Normalmente non fa nulla; interviene solo se dai binari arriva un picco di tensione.
- **C_vref**: va tra VREF e GND. Tiene ferma la soglia del comparatore, così il rumore della breadboard non viene letto come bit RailCom.
- **R_sense più robusta**: durante RailCom scalda pochissimo, ma in caso di errore software o cablaggio sbagliato può trovarsi molta più corrente addosso. Per questo non usare la 1/2 W come versione finale.

## M2.2 — Cablaggio passo passo

### Step 1: pianta i tre nuovi chip

- [ ] **TLV3501 su PA0002** a cavallo del canale, **pin 1 (serigrafia "1") sulla riga 25 sx**, righe 25-28.
- [ ] **MOSFET Q1 IRLZ44N** sulla bancata destra, righe 41-43, scritta leggibile verso l'alto, **piastra metallica fuori dalla breadboard** verso il bordo destro.
- [ ] **MOSFET Q2 IRLZ44N** sulla bancata destra, righe 49-51, idem.

⚠️ Le piastre metalliche dei MOSFET **NON devono toccarsi**. La distanza di 5 righe tra Source Q1 (riga 43 dx) e Gate Q2 (riga 49 dx) garantisce ~12 mm di separazione, sufficiente.

```
      +----------+
      | IRLZ44N  |     pinout (vista frontale, scritta in alto):
      |          |     G  D  S
      +--+--+--++
         |  |  |
         G  D  S
```

| Pin MOSFET | Q1 | Q2 |
|-----------|-----|-----|
| Gate | riga 41 dx | riga 49 dx |
| Drain | riga 42 dx | riga 50 dx |
| Source | riga 43 dx | riga 51 dx |

### Step 2: alimentazione TLV3501

| Pin TLV3501 | Riga | Cosa fare |
|-------------|------|-----------|
| 7 (V+) | 26 dx | jumper a rail + |
| 4 (V−) | 28 sx | jumper a rail − |
| 8 (SHDN, active-HIGH) | 25 dx | **jumper a rail −** (OBBLIGATORIO, altrimenti il chip va in shutdown casuale) |
| 1 (NC) | 25 sx | flottante |
| 5 (NC) | 28 dx | flottante |

- [ ] **Bypass C1 100 nF** tra **riga 25 dx (SHDN/GND)** e **riga 26 dx (V+)** — cavalca esattamente sopra i due pin di alimentazione, percorso minimo.

### Step 3: partitore VREF (riga 33 sx)

- [ ] **R_ref_high 100 KΩ** da rail + a riga 33 sx
- [ ] **R_ref_low 1 KΩ** da riga 33 sx a rail −
- [ ] **C_vref 100 nF** da riga 33 sx a rail −

Il punto riga 33 sx è la soglia del detector RailCom. Deve stare a circa **33 mV**. Il condensatore serve solo a tenerla stabile.

### Step 4: SENSE NODE (riga 30 sx)

- [ ] **R_sense 4.7 Ω non induttiva, almeno 3 W** da riga 30 sx a rail −

Usa una resistenza fisicamente più grande della classica 1/4 W o 1/2 W. Non serve perché RailCom consuma tanto; serve perché se il cutout resta acceso per errore, questa è una delle prime parti che si scalda.

### Step 5: ingressi del comparatore

- [ ] **Jumper riga 33 sx (VREF) → riga 27 sx (TLV3501 IN+, pin 3)**
- [ ] **Jumper riga 30 sx (SENSE NODE) → riga 26 sx (TLV3501 IN−, pin 2)**

### Step 6: catena buffer (gate 5 + gate 6 del 74HC14)

- [ ] **Filo riga 27 dx (TLV3501 OUT, pin 6) → riga 17 dx (74HC14 pin 11, gate 5 in)**
- [ ] **Ponte breve riga 18 dx (74HC14 pin 10, gate 5 out) → riga 15 dx (74HC14 pin 13, gate 6 in)** — questo è il "ponte buffer v3"
- [ ] **Filo riga 16 dx (74HC14 pin 12, gate 6 out) → ESP32 GPIO5**

Catena completa: `SENSE → IN− TLV3501 → OUT (push-pull) → pin 11 → pin 10 → ponte → pin 13 → pin 12 → GPIO5`. La doppia inversione mantiene la polarità del bit RailCom (0=dato presente, 1=idle/perso).

### Step 7: cutout chain (gate 4 del 74HC14 + MOSFET)

- [ ] **Filo ESP32 GPIO4 → 74HC14 pin 9 (riga 19 dx)**. Questo è il comando cutout separato; GPIO18 non va collegato a questo pin.
- [ ] **R_cutout_pu 100 KΩ** da **riga 19 dx (74HC14 pin 9)** a **rail +**. Tiene il cutout spento durante boot/reset, prima che GPIO4 sia configurato.
- [ ] **Filo riga 20 dx (74HC14 pin 8, gate 4 out) → riga 45 dx** (nodo CUTOUT_INV)
- [ ] **R_gate1 220 Ω** da riga 45 dx → riga 41 dx (Gate Q1)
- [ ] **R_gate2 220 Ω** da riga 45 dx → riga 49 dx (Gate Q2)
- [ ] **R_gate_pd1 100 KΩ** da riga 41 dx (Gate Q1) → rail −
- [ ] **R_gate_pd2 100 KΩ** da riga 49 dx (Gate Q2) → rail −

Logica: GPIO18=HIGH abilita le uscite del BTS7960; il modulo BTS resta comunque alimentato da 5 V logici e 15 V di potenza. GPIO4=HIGH → gate 4 out=LOW → MOSFET OFF. Durante un cutout il firmware porta prima GPIO18=LOW per mettere le uscite M+/M− del BTS in stato non pilotante, attende un dead-time iniziale configurato a 15 µs, poi porta GPIO4=LOW → gate 4 out=HIGH → MOSFET ON → cutout cortocircuitato sul sense resistor. I 15 µs sono un valore di partenza da validare allo scope sul modulo BTS7960 reale, non una garanzia: se M+/M− stanno ancora pilotando quando GPIO4 scende, aumenta il dead-time firmware prima di usare locomotive; se il margine e' eccessivo rispetto alla finestra RailCom misurata, riducilo solo dopo verifica allo scope. A fine finestra GPIO4 torna HIGH prima di riabilitare le uscite del BTS con GPIO18.

### Step 8: alta corrente (MOSFET → SENSE → TRACK)

- [ ] **Jumper riga 43 dx (Source Q1) → riga 30 sx (SENSE NODE)**
- [ ] **Jumper riga 42 dx (Drain Q1) → riga 53 sx (nodo TRACK_A)**
- [ ] **Jumper riga 50 dx (Drain Q2) → riga 54 sx (nodo TRACK_B)**
- [ ] **R_limit 1 Ω 1/2 W** da riga 51 dx (Source Q2) → rail −

### Step 9: re-routing dei binari (modifica al Modulo 1)

- [ ] **Filo BTS7960 M+ → riga 53 sx (TRACK_A)** invece che direttamente al binario A
- [ ] **Filo BTS7960 M− → riga 54 sx (TRACK_B)** invece che direttamente al binario B
- [ ] **Filo riga 53 sx → binario A** (nuova connessione esterna)
- [ ] **Filo riga 54 sx → binario B** (nuova connessione esterna)
- [ ] **D_tvs bidirezionale** tra riga 53 sx (TRACK_A) e riga 54 sx (TRACK_B), oppure direttamente sui morsetti binari se lì sta più saldo

In pratica i binari ora "vedono" il BTS7960 e, solo durante la finestra RailCom, il clamp MOSFET. Il TVS sta semplicemente in parallelo ai binari: non ha verso, perché è bidirezionale. Se lo monti sui morsetti invece che sulla breadboard, va comunque bene.

### Step 10: opzionali

- [ ] **R_hyst 47 KΩ** tra riga 27 dx (TLV3501 OUT) e riga 27 sx (IN+): aggiunge isteresi esterna oltre a quella interna del TLV3501. Utile **solo** se in test funzionale vedi oscillazioni o byte errati.

Nota sul dead-time hardware: non mettere per ora una rete RC sui gate dei MOSFET. Sembra una protezione semplice, ma può accendere i MOSFET lentamente e farli scaldare. Per questa versione resta il dead-time firmware; se in futuro serve una protezione hardware vera, meglio aggiungere un piccolo interlock logico che impedisca ai MOSFET di accendersi mentre il BTS7960 è ancora abilitato.

## M2.3 — Verifica continuità (alimentazione SCOLLEGATA)

### Alimentazione TLV3501

- [ ] pin 7 TLV3501 ↔ rail + : **continuo**
- [ ] pin 4 TLV3501 ↔ rail − : **continuo**
- [ ] pin 8 TLV3501 ↔ rail − : **continuo** (SHDN tirato a GND!)
- [ ] **Vcc BTS7960 ↔ rail +** : **NON continuo** (resta vero anche dopo il Modulo 2)

### Catena RailCom

- [ ] Riga 27 dx (TLV3501 OUT) ↔ riga 17 dx (74HC14 pin 11) : **continuo**
- [ ] Pin 10 (riga 18 dx) ↔ pin 13 (riga 15 dx) : **continuo** (ponte buffer v3)
- [ ] Pin 12 (riga 16 dx) ↔ GPIO5 : **continuo**
- [ ] Pin 11 ↔ pin 10 : **NON continuo** (sono ingresso e uscita di gate 5)
- [ ] Pin 13 ↔ pin 12 : **NON continuo** (sono ingresso e uscita di gate 6)
- [ ] Riga 27 dx (OUT TLV3501) ↔ GPIO5 : **NON continuo diretto** (passa per il buffer)

### Catena cutout (MOSFET)

- [ ] Riga 20 dx (74HC14 pin 8) ↔ riga 45 dx (CUTOUT_INV) : **continuo**
- [ ] Riga 19 dx (74HC14 pin 9) ↔ GPIO4 : **continuo**
- [ ] Riga 19 dx ↔ rail + : **NON continuo** (R_cutout_pu 100 KΩ)
- [ ] Ohmmetro riga 19 dx ↔ rail + : **~100 KΩ**
- [ ] Riga 19 dx ↔ GPIO18 : **NON continuo** (cutout e enable BTS sono separati)
- [ ] Riga 45 dx ↔ riga 41 dx (Gate Q1) : **NON continuo** (R_gate1 220 Ω)
- [ ] Riga 45 dx ↔ riga 49 dx (Gate Q2) : **NON continuo** (R_gate2 220 Ω)
- [ ] Ohmmetro riga 45 dx ↔ riga 41 dx : **~220 Ω**
- [ ] Ohmmetro riga 45 dx ↔ riga 49 dx : **~220 Ω**
- [ ] Riga 41 dx (Gate Q1) ↔ rail − : **NON continuo** (R_gate_pd1 100 KΩ)
- [ ] Riga 49 dx (Gate Q2) ↔ rail − : **NON continuo** (R_gate_pd2 100 KΩ)
- [ ] Riga 42 dx (Drain Q1) ↔ riga 53 sx (TRACK_A) : **continuo**
- [ ] Riga 50 dx (Drain Q2) ↔ riga 54 sx (TRACK_B) : **continuo**
- [ ] Riga 43 dx (Source Q1) ↔ riga 30 sx (SENSE NODE) : **continuo**
- [ ] Riga 30 sx ↔ rail − : **NON continuo** (R_sense 4.7 Ω)
- [ ] Riga 51 dx (Source Q2) ↔ rail − : **NON continuo** (R_limit 1 Ω)

### Comparatore

- [ ] Riga 30 sx (SENSE) ↔ riga 26 sx (IN−) : **continuo**
- [ ] Riga 33 sx (VREF) ↔ riga 27 sx (IN+) : **continuo**
- [ ] C_vref presente tra riga 33 sx (VREF) e rail −

### Protezione binari

- [ ] D_tvs presente tra TRACK_A e TRACK_B
- [ ] Il TVS è vicino ai fili/morsetti binari, non in mezzo alla zona TLV3501

### Pin NC

- [ ] Riga 25 sx (TLV3501 pin 1 NC) e riga 28 dx (pin 5 NC) : **NON collegate a niente**

## M2.4 — Test alimentato (USB only, B+ 15 V SPENTO)

- [ ] Riga 33 sx (VREF) = **~33 mV** (tolleranza ±2 mV)
- [ ] Riga 25 dx (SHDN) = 0 V
- [ ] GPIO4 / riga 19 dx idle = **HIGH (~3.3 V)**
- [ ] 74HC14 pin 8 (riga 20 dx) idle = **LOW** (MOSFET OFF)
- [ ] Riga 27 dx (TLV3501 OUT) con SENSE = 0 V → **HIGH (~3.3 V, idle)**
- [ ] 74HC14 pin 10 (riga 18 dx) idle → **LOW**
- [ ] 74HC14 pin 12 (riga 16 dx) idle → **HIGH**
- [ ] GPIO5 idle → **HIGH** (3.3 V, UART idle)

Catena attesa in idle: `OUT(HIGH) → pin10(LOW) → pin12(HIGH) → GPIO5(HIGH)`. Se rispettata, il buffer funziona.

## M2.5 — Test funzionale

1. Accendi B+ 15 V e fai partire il firmware come nel Modulo 1.
2. Comanda una scrittura POM (CV programming on Main) verso il decoder.
3. Verifica nella seriale che `rx_full` o equivalente metrica RailCom incrementi.
4. Se `rx_empty` = `rx_windows`, **scambia TRACK_A e TRACK_B** sui morsetti M+/M− del BTS7960 (il rilevamento single-ended cattura solo una direzione di corrente; senza ponte a diodi serve la polarità giusta).

Se il decoder risponde, il **Modulo 2 è completo**.

## M2.6 — Taratura dead-time BTS7960 → MOSFET RailCom

Per misurare il dead-time ed evitare il cortocircuito tra il BTS7960 e i MOSFET RailCom, usa un oscilloscopio a due canali. La misura corretta confronta il segnale che disabilita il ponte di potenza con il segnale che abilita il clamp RailCom.

### Collegamento sonde

- [ ] **Canale 1**: collega la sonda al comando enable del BTS7960: **ESP32 GPIO18**, oppure direttamente ai pin **R_EN/L_EN** del BTS7960. Questo segnale spegne la potenza quando scende LOW.
- [ ] **Canale 2**: collega la sonda al comando cutout: **ESP32 GPIO4** oppure **riga 19 dx** (74HC14 pin 9). Questo nodo e' active-low: quando scende LOW, il gate 4 del 74HC14 accende il clamp RailCom.
- [ ] **Masse sonde**: collega entrambe le clip di massa al **rail -** della breadboard, cioe' al GND comune.

Nota: nel layout attuale GPIO18 **non** passa dal 74HC14. Non usare `riga 18 dx` come riferimento per l'enable BTS: quella riga appartiene al gate 5 del buffer RailCom.

### Impostazione oscilloscopio

- [ ] **Timebase**: 2-5 µs/div per vedere bene il ritardo iniziale.
- [ ] **Trigger**: Canale 1, fronte di discesa, cioe' quando GPIO18 passa da HIGH a LOW.
- [ ] **Modalita'**: Single Shot / Single Sequence, cosi' catturi un singolo pacchetto DCC e il relativo cutout.

### Cosa osservare

La zona di sicurezza e' lo spazio temporale tra lo spegnimento del BTS7960 e l'attivazione dei MOSFET:

1. **Fase 1**: Canale 1 scende LOW: il firmware disabilita R_EN/L_EN del BTS7960.
2. **Fase 2**: Canale 2 scende LOW solo dopo il dead-time configurato: il firmware comanda il cutout RailCom su GPIO4.

Con la configurazione iniziale il firmware usa **15 µs**. Se i due fronti sono troppo vicini, o se il BTS7960 sta ancora pilotando quando GPIO4 scende, aumenta `CUTOUT_START_DEADTIME_US` in `src/track_output.rs`.

### Verifica reale sulle uscite binario

Per verificare che il BTS7960 sia davvero spento, sposta il Canale 1 su una uscita di potenza:

- [ ] Canale 1 su **TRACK_A** o **TRACK_B** rispetto a GND, oppure misura differenziale tra TRACK_A e TRACK_B se il tuo oscilloscopio/sonde lo permettono in sicurezza.
- [ ] Canale 2 resta su **GPIO4 / riga 19 dx**.

Vedrai l'onda DCC interrompersi. Il BTS7960 puo' avere una coda di discesa: la tensione sui binari non cade a zero istantaneamente. Il cutout e' sicuro solo se il comando MOSFET arriva quando la tensione sul binario e' gia' scesa in modo significativo e non c'e' piu' pilotaggio attivo contro massa.

### Attenzione alle inversioni

Il 74HC14 inverte i segnali:

- **GPIO4 / riga 19 dx**: active-low; LOW richiede il cutout.
- **74HC14 pin 8 / riga 20 dx / nodo CUTOUT_INV**: active-high; HIGH accende i MOSFET.
- **Gate MOSFET Q1/Q2**: active-high dopo le resistenze da 220 Ω.

Quando misuri un nodo diverso da GPIO4, controlla sempre se quel punto e' attivo alto o attivo basso prima di interpretare il ritardo.

---

# M3 — Modulo UI / stato DCC

**Obiettivo:** LED verde/rosso di stato sulla breadboard principale, display OLED per stato sistema, due pulsanti per Stop/Resume manuali.

**Pre-requisito:** Modulo 1 montato e testato. **Indipendente** dal Modulo 2.

## M3.1 — Componenti del modulo

I **LED di stato stanno sulla breadboard principale**, righe 21-24 lato dx. I pulsanti usano i nodi di debounce sulla breadboard principale, righe 9-11 lato dx. L'OLED resta fuori dalla breadboard principale oppure su una mini-breadboard di estensione.

| Rif | Componente | Posizione / Note |
|-----|-----------|-----------------|
| OLED | SSD1306 / SSD1315 0.96" 128×64 I²C | esterno, indirizzo 0x3C, bus 400 kHz, blu+giallo |
| LED_GREEN | LED verde | breadboard principale, anodo riga 21 dx, catodo riga 22 dx |
| LED_RED | LED rosso | breadboard principale, anodo riga 23 dx, catodo riga 24 dx |
| R_led_green | Resistore 1 KΩ | riga 22 dx → rail − dx (in serie dopo LED verde) |
| R_led_red | Resistore 1 KΩ | riga 24 dx → rail − dx (in serie dopo LED rosso) |
| SW1 | Pulsante rosso | Stop / E-Stop, nodo riga 9 dx |
| SW2 | Pulsante blu | Resume, nodo riga 10 dx |
| C_db1 | Condensatore 100 nF | riga 9 dx → riga 11 dx |
| C_db2 | Condensatore 100 nF | riga 10 dx → riga 11 dx |

## M3.2 — Cablaggio

### OLED

- [ ] OLED VCC → ESP32 3V3 (oppure rail + della breadboard)
- [ ] OLED GND → rail − (massa comune)
- [ ] OLED SDA → ESP32 **GPIO19**
- [ ] OLED SCL → ESP32 **GPIO20**

### LED stato

I LED sono **active-high**: il firmware porta il GPIO a 3.3 V per accendere il LED. La gamba lunga del LED è l'anodo, la gamba corta è il catodo. Il resistore di limitazione sta **dopo** il LED (tra catodo e GND) per accorciare i salti di breadboard: i fori GPIO14/15 sono in righe 3-4 dx, le righe del LED sono 21-24 dx → un salto lungo lo coprono i jumper M-M, mentre le gambe del resistore restano corte.

LED verde (righe 21-22 dx):

- [ ] **Jumper M-M: GPIO14 (riga 3 dx) → riga 21 dx**
- [ ] **Anodo LED verde → riga 21 dx**, catodo LED verde → **riga 22 dx**
- [ ] **R_led_green 1 KΩ**: una gamba in **riga 22 dx**, altra gamba direttamente nel **rail − dx**

LED rosso (righe 23-24 dx):

- [ ] **Jumper M-M: GPIO15 (riga 4 dx) → riga 23 dx**
- [ ] **Anodo LED rosso → riga 23 dx**, catodo LED rosso → **riga 24 dx**
- [ ] **R_led_red 1 KΩ**: una gamba in **riga 24 dx**, altra gamba nel **rail − dx**

Schema fisico:

```text
GPIO14 (riga 3 dx) ──[jumper M-M]── riga 21 dx ──>|── riga 22 dx ──[ 1K ]── rail − dx
                                            LED verde

GPIO15 (riga 4 dx) ──[jumper M-M]── riga 23 dx ──>|── riga 24 dx ──[ 1K ]── rail − dx
                                            LED rosso
```

Verifica continuità (alimentazione SCOLLEGATA):

- [ ] **GPIO14 ↔ riga 21 dx**: continuo (jumper)
- [ ] **GPIO15 ↔ riga 23 dx**: continuo (jumper)
- [ ] Ohmmetro **riga 22 dx ↔ rail − dx**: ~1 KΩ
- [ ] Ohmmetro **riga 24 dx ↔ rail − dx**: ~1 KΩ
- [ ] **riga 21 dx ↔ riga 23 dx**: NON continuo
- [ ] **riga 22 dx ↔ riga 24 dx**: NON continuo (i due ritorni sono indipendenti, condividono solo la rail)

Pattern firmware:

| Stato | LED |
|-------|-----|
| Boot | verde lampeggiante rapido |
| WiFi in connessione | rosso lampeggiante |
| Running | verde fisso |
| E-stop / fault | rosso fisso |

### Pulsanti

I due pulsanti sfruttano il pull-up interno dell'ESP32 (configurato in firmware): uno dei due capi al GPIO, l'altro a GND. Il condensatore di debounce sta in parallelo al pulsante.

I nodi sono sul lato destro della breadboard:

| Riga | Funzione |
|------|----------|
| riga 9 dx | STOP node, collegato a GPIO22 |
| riga 10 dx | RESUME node, collegato a GPIO21 |
| riga 11 dx | GND comune pulsanti, gia' collegato a rail - dx dal cursore del trimmer IS |

Cablaggio:

- [ ] **ESP32 GPIO22 → riga 9 dx**.
- [ ] **Pulsante rosso (Stop)** tra **riga 9 dx** e **riga 11 dx**.
- [ ] **C_db1 100 nF** tra **riga 9 dx** e **riga 11 dx**.
- [ ] **ESP32 GPIO21 → riga 10 dx**.
- [ ] **Pulsante blu (Resume)** tra **riga 10 dx** e **riga 11 dx**.
- [ ] **C_db2 100 nF** tra **riga 10 dx** e **riga 11 dx**.

Schema fisico:

```text
GPIO22 -> riga 9 dx  ─┬─ pulsante STOP ─┐
                      └─ C_db1 100 nF ──┤
                                        ├─ riga 11 dx -> rail - dx
GPIO21 -> riga 10 dx ─┬─ pulsante RESUME┤
                      └─ C_db2 100 nF ──┘
```

Verifica continuita' (alimentazione SCOLLEGATA):

- [ ] **riga 11 dx ↔ rail - dx**: continuo.
- [ ] **GPIO22 ↔ riga 9 dx**: continuo.
- [ ] **GPIO21 ↔ riga 10 dx**: continuo.
- [ ] **riga 9 dx ↔ riga 11 dx**: NON continuo a pulsante rilasciato; continuo a pulsante premuto.
- [ ] **riga 10 dx ↔ riga 11 dx**: NON continuo a pulsante rilasciato; continuo a pulsante premuto.

## M3.3 — Test funzionale

- [ ] Avvia il firmware: l'OLED mostra "DCC Command Station" e l'IP WiFi entro pochi secondi.
- [ ] LED verde lampeggia durante boot e poi resta acceso fisso in stato Running.
- [ ] Durante connessione WiFi il LED rosso lampeggia.
- [ ] Il display mostra il numero di loco attive nel pool.
- [ ] Premi **Stop**: la loco si ferma, display indica E-Stop o Fault, LED rosso fisso.
- [ ] Premi **Resume**: ripresa normale, LED verde fisso.

Se il display non si accende, verifica con un I²C scanner (sketch di test) che l'indirizzo 0x3C sia raggiungibile sul bus.

---

# Sezioni di riferimento

## R1 — BOM completo (tutti i moduli)

| Modulo | Rif | Componente | Valore |
|--------|-----|-----------|--------|
| 1 | U1 | ESP32-C6 Mini Waveshare | — |
| 1 | U2 | 74HC14 DIP-14 | hex Schmitt-trigger inverter |
| 1 | Trimmer | Multigiri BOJACK P103 | 10 KΩ (outer usato riga 10 sx, cursore riga 11 dx) |
| 1 | C_is | Condensatore | 100 nF (sulla breadboard, riga 18 sx ↔ rail −) |
| 1 | R_gpio3 | Resistore 1 KΩ | 1/4 W |
| 1 | C2 | Condensatore | 100 nF (consigliato) |
| 1 | BTS7960 | H-bridge | 43 A, Vcc 5 V, B+ 15 V |
| 1 | Fuse | Fusibile + portafusibile | dimensionato per il carico |
| 1 | PSU | Alimentatore da banco | 15 V regolabile |
| 2 | U3 | TLV3501AIDR su breakout PA0002 | comparatore 4.5 ns |
| 2 | Q1, Q2 | IRLZ44N | 55 V, 47 A, logic-level |
| 2 | R_sense | Resistore non induttivo | 4.7 Ω, almeno 3 W |
| 2 | R_limit | Resistore | 1 Ω 1/2 W |
| 2 | R_ref_high | Resistore | 100 KΩ |
| 2 | R_ref_low | Resistore | 1 KΩ |
| 2 | R_gate1, R_gate2 | Resistori | 220 Ω |
| 2 | R_gate_pd1, R_gate_pd2 | Resistori | 100 KΩ |
| 2 | R_cutout_pu | Resistore | 100 KΩ |
| 2 | C1 | Condensatore | 100 nF |
| 2 | C_vref | Condensatore ceramico | 100 nF |
| 2 | D_tvs | TVS bidirezionale | P6KE18CA o equivalente compatibile con tensione DCC reale |
| 2 | R_hyst | Resistore (opzionale) | 47 KΩ |
| 3 | OLED | SSD1306/SSD1315 | 0.96" 128×64 I²C |
| 3 | LED_GREEN | LED | verde |
| 3 | LED_RED | LED | rosso |
| 3 | R_led_green, R_led_red | Resistori | 1 KΩ |
| 3 | SW1 (rosso) | Pulsante | Stop, riga 9 dx ↔ riga 11 dx |
| 3 | SW2 (blu) | Pulsante | Resume, riga 10 dx ↔ riga 11 dx |
| 3 | C_db1, C_db2 | Condensatori | 100 nF, in parallelo ai pulsanti |

## R2 — Wire list complessiva (riferimento, sistema completo)

### Jumper interni alla breadboard

| # | Da | A | Modulo | Scopo |
|---|----|---|--------|-------|
| 1 | 3V3 ESP32 | rail + | 1 | Alimentazione 3.3 V |
| 2 | GND ESP32 | rail − | 1 | Riferimento |
| 3 | rail + | riga 14 dx (74HC14 pin 14) | 1 | VCC 74HC14 |
| 4 | riga 20 sx (74HC14 pin 7) | rail − | 1 | GND 74HC14 |
| 5 | riga 15 sx (pin 2) | riga 16 sx (pin 3) | 1 | Cascata gate 1→2 |
| 6 | riga 19 sx (pin 6) | riga 11 sx | 1 | R 1K verso GPIO3 |
| 6a | riga 11 dx (cursore trimmer IS) | rail − dx | 1 | Riferimento reostato IS |
| 6b | riga 10 sx (Outer A trimmer) | riga 18 sx (74HC14 pin 5) | 1 | Tensione IS al gate 3 in |
| 6c | riga 18 sx (pin 5) | rail − sx | 1 | C_is 100 nF, filtro DCC sul nodo IS |
| 6d | riga 22 dx (catodo LED verde) | rail − dx | 3 | R_led_green 1 KΩ in serie post-LED |
| 6e | riga 24 dx (catodo LED rosso) | rail − dx | 3 | R_led_red 1 KΩ in serie post-LED |
| 6f | riga 9 dx | riga 11 dx | 3 | C_db1 100 nF in parallelo al pulsante Stop |
| 6g | riga 10 dx | riga 11 dx | 3 | C_db2 100 nF in parallelo al pulsante Resume |
| 7 | riga 26 dx (TLV3501 V+) | rail + | 2 | Alimentazione comparatore |
| 8 | riga 28 sx (TLV3501 V−) | rail − | 2 | GND comparatore |
| 9 | riga 25 dx (TLV3501 SHDN) | rail − | 2 | Abilitazione (OBBLIGATORIO) |
| 10 | riga 30 sx (SENSE NODE) | riga 26 sx (IN−) | 2 | Segnale al comparatore |
| 11 | riga 33 sx (VREF) | riga 27 sx (IN+) | 2 | Riferimento al comparatore |
| 11a | riga 33 sx (VREF) | rail − | 2 | C_vref 100 nF, filtro riferimento comparatore |
| 12 | riga 27 dx (OUT TLV3501) | riga 17 dx (pin 11) | 2 | Ingresso buffer |
| 13 | riga 18 dx (pin 10) | riga 15 dx (pin 13) | 2 | Ponte gate 5→6 |
| 14 | riga 43 dx (Source Q1) | riga 30 sx (SENSE NODE) | 2 | Sense path Q1 |
| 15 | riga 42 dx (Drain Q1) | riga 53 sx (TRACK_A) | 2 | Q1 al binario A |
| 16 | riga 50 dx (Drain Q2) | riga 54 sx (TRACK_B) | 2 | Q2 al binario B |
| 17 | riga 20 dx (pin 8) | riga 45 dx (CUTOUT_INV) | 2 | Gate 4 out → MOSFET |
| 18 | riga 19 dx (pin 9) | rail + | 2 | R_cutout_pu 100 KΩ, default cutout OFF |

### Fili esterni alla breadboard

| Da | A | Modulo | Scopo |
|----|---|--------|-------|
| ESP32 5V | Vcc BTS7960 | 1 | **Logica BTS7960 — filo diretto, NON sul rail +** |
| ESP32 GND | BTS7960 GND | 1 | Riferimento comune addizionale |
| ESP32 GPIO2 | 74HC14 pin 1 (riga 14 sx) | 1 | DCC → gate 1 in |
| ESP32 GPIO3 | riga 11 sx (dopo R 1K) | 1 | Cortocircuito ← gate 3 out |
| ESP32 GPIO18 | BTS7960 R_EN + L_EN | 1 | Abilitazione H-bridge |
| ESP32 GPIO4 | 74HC14 pin 9 (riga 19 dx) | 2 | RailCom cutout command, attivo LOW |
| ESP32 GPIO5 | 74HC14 pin 12 (riga 16 dx) | 2 | RailCom UART RX ← gate 6 out |
| ESP32 GPIO19 | OLED SDA | 3 | I²C dati |
| ESP32 GPIO20 | OLED SCL | 3 | I²C clock |
| ESP32 GPIO14 (riga 3 dx) | jumper M-M → riga 21 dx (anodo LED verde) | 3 | LED stato verde, active-high; R 1 KΩ in serie post-LED |
| ESP32 GPIO15 (riga 4 dx) | jumper M-M → riga 23 dx (anodo LED rosso) | 3 | LED stato rosso, active-high; R 1 KΩ in serie post-LED |
| ESP32 GPIO21 | riga 10 dx (Resume node) | 3 | Input pull-up; pulsante/C_db2 verso riga 11 dx GND |
| ESP32 GPIO22 | riga 9 dx (Stop node) | 3 | Input pull-up; pulsante/C_db1 verso riga 11 dx GND |
| ESP32 3V3 | OLED VCC | 3 | Alimentazione OLED (anche da rail +) |
| 74HC14 pin 2 (riga 15 sx) | BTS7960 LPWM | 1 | DCC invertito |
| 74HC14 pin 4 (riga 17 sx) | BTS7960 RPWM | 1 | DCC non invertito |
| BTS7960 R_IS + L_IS | riga 10 sx (trimmer Outer A) | 1 | Sense cortocircuito (trimmer e C_is sono on-board, vedi jumper 6a-6c) |
| BTS7960 M+ | (M1) binario A diretto / (M2) riga 53 sx TRACK_A | 1→2 | M1: diretto al binario; M2: passa per Q1 |
| BTS7960 M− | (M1) binario B diretto / (M2) riga 54 sx TRACK_B | 1→2 | M1: diretto al binario; M2: passa per Q2 |
| Riga 53 sx TRACK_A | Binario A | 2 | Solo se Modulo 2 montato |
| Riga 54 sx TRACK_B | Binario B | 2 | Solo se Modulo 2 montato |
| D_tvs | TRACK_A ↔ TRACK_B | 2 | Protezione picchi induttivi sui binari |
| Alimentatore 15 V + | Fusibile → BTS7960 B+ | 1 | Potenza binari |
| Alimentatore 15 V − | BTS7960 B− | 1 | Ritorno potenza |

### Tabella stati (sistema completo)

| Stato | GPIO18 | GPIO4 | Q1, Q2 | BTS7960 | SENSE NODE | TLV3501 OUT | pin 10 | pin 12 | GPIO5 |
|-------|--------|-------|--------|---------|------------|-------------|--------|--------|-------|
| DCC normale | HIGH | HIGH | OFF | ±15 V | 0 V | HIGH | LOW | HIGH | HIGH (idle) |
| Dead-time pre-cutout | LOW | HIGH | OFF | disab. | 0 V | HIGH | LOW | HIGH | HIGH (idle) |
| Cutout, decoder (+) | LOW | LOW | ON | disab. | +180 mV | LOW | HIGH | LOW | LOW (dato) |
| Cutout, decoder (−) | LOW | LOW | ON | disab. | −180 mV | HIGH | LOW | HIGH | HIGH (perso) |
| Cutout, silenzio | LOW | LOW | ON | disab. | 0 V | HIGH | LOW | HIGH | HIGH (idle) |
| E-stop / fault | LOW | HIGH | OFF | disab. | 0 V | HIGH | LOW | HIGH | HIGH (idle) |

### Budget elettrico (Modulo 2)

| Parametro | Valore | Limite NMRA S-9.3.2 | Conforme |
|-----------|--------|---------------------|----------|
| Burden a 34 mA | (4.7 + 1 + 0.044) × 34 mA = **195 mV** | ≤ 200 mV | ✅ |
| Soglia rilevamento | 33 mV / 4.7 Ω = **7 mA** | gap 6-10 mA | ✅ |
| Rise/fall TLV3501 | **4.5 ns** | ≤ 500 ns | ✅ (ampiamente) |
| Propagazione 74HC14 (per gate) | ~15 ns | — | trascurabile |
| Ritardo totale buffer (gate 5+6) | ~30 ns | — | trascurabile |
| Dead-time BTS→cutout | 15 µs iniziali | valore firmware di partenza | da validare allo scope sul BTS7960 reale |
| Durata cutout fisico | 460 µs | 454-488 µs | ✅ |
| Return path | 2 × Rds_on = 44 mΩ | bassa impedenza | ✅ |

Nota semplice su R_sense: in funzionamento normale RailCom scalda quasi zero. La versione da almeno 3 W serve come margine se sbagli cablaggio, se il firmware si blocca nel momento peggiore, o se il BTS7960 non si disabilita davvero prima del cutout. Non è una garanzia di sopravvivenza a un corto lungo: per quello servono fusibile, current limit basso nei test e spegnimento firmware.

### Limitazioni note (Modulo 2)

1. **Rilevamento single-ended (50%):** senza ponte a diodi, il comparatore rileva solo una direzione della corrente RailCom. ~50% delle finestre saranno vuote. Se **tutte** le finestre sono vuote, scambia i fili TRACK_A e TRACK_B verso il BTS7960.
2. **R_hyst opzionale:** il buffer Schmitt del 74HC14 aggiunge isteresi in cascata. Se persistono oscillazioni, aggiungi R_hyst 47 KΩ tra riga 27 dx e riga 27 sx.
3. **Sequenza cutout separata:** il firmware deve tenere GPIO4 HIGH fuori finestra, portare GPIO18 LOW per rendere non pilotanti le uscite del BTS, attendere un dead-time iniziale da validare allo scope, poi portare GPIO4 LOW per 460 µs. In fault/E-stop GPIO18 resta LOW e GPIO4 resta HIGH, quindi i MOSFET restano OFF.
4. **TVS:** se usi 15 V sui binari, P6KE18CA è un candidato ragionevole. Se cambi tensione binari, ricontrolla il modello: il TVS non deve intervenire durante il DCC normale, deve intervenire solo sui picchi.
5. **Massa:** tieni il GND del TLV3501 corto e pulito. Se lo fai passare insieme ai ritorni di potenza, il comparatore può vedere bit falsi.

## R3 — Risoluzione problemi

### Modulo 1

| Sintomo | Causa probabile | Soluzione |
|---------|----------------|-----------|
| Nessun segnale DCC su GPIO2 al scope | Firmware non avviato o RMT non configurato | Verifica `cargo run` e log seriale |
| GPIO2 oscilla ma BTS7960 M+/M− fermo | LPWM/RPWM non arrivano al BTS7960 | Verifica fili pin 2 e pin 4 del 74HC14 |
| LPWM e RPWM stessa fase | Ponte cascata pin 2 → pin 3 mancante | Aggiungi ponte (Step 3 M1.2) |
| Cortocircuito non rilevato | Trimmer mal regolato o R 1K aperto | Calibra trimmer per ~3 A trip; verifica R 1K con ohmmetro |
| Rail + a 5 V invece di 3.3 V | Filo 5V finito sul rail | **Stacca subito USB**, rivedi Sezione B |
| BTS7960 non si accende | Vcc 5 V mancante | Verifica filo dedicato ESP32 5V → BTS7960 Vcc |

### Modulo 2

| Sintomo | Causa probabile | Soluzione |
|---------|----------------|-----------|
| GPIO5 resta LOW senza DCC | TLV3501 in shutdown, oppure buffer cablato al contrario | Verifica riga 25 dx → rail −; verifica filo OUT a pin 11 (riga 17 dx) e non pin 10 |
| GPIO5 resta HIGH, nessun dato | Polarità sbagliata (50% single-ended) | Scambia TRACK_A ↔ TRACK_B sui morsetti M+/M− |
| Segnale DCC distorto dopo Modulo 2 | MOSFET bloccato ON | Verifica pull-down 100K sui gate; pin 8 74HC14 deve essere LOW durante DCC normale |
| MOSFET ON a boot/reset | R_cutout_pu mancante o GPIO4/pin 9 flottante | Verifica R_cutout_pu 100 KΩ da riga 19 dx a rail +; pin 8 deve restare LOW |
| `rx_empty == rx_windows` | Decoder non risponde o polarità | Scope sul SENSE NODE durante cutout; prova a scambiare TRACK_A/TRACK_B |
| GPIO5 resta LOW durante DCC | SENSE NODE non a 0 V, oppure ponte gate 5→6 aperto | Verifica Q1 OFF (gate a 0 V); verifica ponte riga 18 dx ↔ riga 15 dx |
| Breadboard calda / corto | Piastre MOSFET si toccano | Distanzia i MOSFET; verifica continuità tra Drain Q1 e Drain Q2 |
| Byte errati, rx_err alto | Rumore sulla soglia | Aggiungi R_hyst 47 KΩ; accorcia i fili SENSE e OUT |
| Oscillazioni rapide su GPIO5 | Ingresso buffer flottante (ponte gate 5→6 aperto) | Verifica ponte riga 18 dx ↔ riga 15 dx; verifica C1 100 nF |
| Pin 10 segue pin 11 senza invertire | 74HC14 danneggiato o alim. mancante | Verifica pin 14 = 3.3 V, pin 7 = 0 V; sostituisci 74HC14 |
| TLV3501 OUT ~1.6 V costanti | SHDN non a GND, chip in shutdown | Verifica ponticello riga 25 dx → rail − |
| GPIO3 sempre HIGH anche a corto | R 1K aperto o jumper rotto | Ohmmetro tra riga 19 sx e riga 11 sx (~1 KΩ); verifica jumper riga 11 sx → GPIO3 |
| VREF ≠ 33 mV | Resistori sbagliati di valore o invertiti | Verifica R_ref_high = 100 KΩ verso rail +, R_ref_low = 1 KΩ verso rail − |

### Modulo 3

| Sintomo | Causa probabile | Soluzione |
|---------|----------------|-----------|
| OLED nero | Indirizzo I²C errato o alimentazione mancante | Lancia I²C scanner; verifica VCC = 3.3 V e GND |
| OLED scrive in caratteri stranii | Bus I²C disturbato (fili lunghi o GND debole) | Accorcia SDA/SCL; aggiungi pull-up esterni 4.7 KΩ se >30 cm |
| LED non si accende | Polarità LED invertita o resistenza aperta | Anodo verso GPIO (jumper diretto), catodo verso rail − tramite R 1 KΩ |
| LED sempre acceso | GPIO collegato al rail + invece che a GND, oppure R cortocircuitato | Verifica GPIO14/GPIO15 → anodo LED → catodo LED → R 1 KΩ → rail − |
| Pulsante "rimbalza" (più trigger per pressione) | Cap debounce mancante o pull-up interno disabilitato | Verifica firmware abiliti pull-up; aggiungi C_db 100 nF |

---

## Appendice — Riferimenti

- **Datasheet TLV3501:** Texas Instruments TLV3501/TLV3502, 4.5-ns Rail-to-Rail High-Speed Comparator (https://www.ti.com/product/TLV3501)
- **Part number:** TLV3501**AIDR** (A = grade, I = industrial, D = SOIC-8, R = tape-and-reel)
- **Breakout:** ProtoAdvantage PA0002 (SOIC-8 300 mil → DIP-8 passivo 1:1)
- **Adattatore DIP:** ASM-HELP-300
- **Datasheet 74HC14:** Nexperia / TI 74HC14, Hex Schmitt-Trigger Inverter (Vcc 2-6 V)
- **Datasheet IRLZ44N:** Vishay / IR IRLZ44N, Logic-Level N-Channel MOSFET

**Documenti precedenti sostituiti:**

- `railcom-detector-v2-breadboard-layout.md` (breadboard RailCom separata, v2 senza buffer)
- `railcom-detector-v3-breadboard-layout.md` (breadboard RailCom separata, v3 con buffer)
- `breadboard-wiring-guide.md` (breadboard DCC originale, 30 righe)

Restano validi come riferimento se preferisci mantenere le breadboard separate.
