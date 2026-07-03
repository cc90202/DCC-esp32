# Main DCC Board — Checklist di verifica

Ultimo aggiornamento: 2026-04-28

Breadboard 30 righe, configurazione DCC separata da RailCom. Caselle `[x]` = già verificate insieme; `[ ]` = ancora da fare.

Riferimenti: `breadboard-wiring-guide.md` (base, parzialmente stale) e `src/boot.rs` (autoritativo per i GPIO).

## ⚠️ Discrepanze doc → codice (già prese in carico)

| Funzione | Doc dice | Codice (`boot.rs`) | Checklist usa |
|----------|----------|---------------------|---------------|
| Cutout RailCom | GPIO18 | **GPIO4** (l. 714) | **GPIO4** |
| LED verde stato | non menzionato | **GPIO14** (l. 697) | GPIO14 |
| LED rosso stato | non menzionato | **GPIO15** (l. 698) | GPIO15 |

## Pin map 74HC14 sulla breadboard

Derivato da "GND (pin 7) on right bank row 16" del wiring guide. Il chip straddle del canale: pin 1-7 sul **lato dx**, pin 8-14 sul **lato sx**.

| Pin | Riga / lato | Funzione |
|-----|-------------|----------|
| 1 IN | 10 dx | DCC ← GPIO2 |
| 2 OUT | 11 dx | LPWM → BTS7960 |
| 3 IN | 12 dx | cascata ← pin 2 |
| 4 OUT | 13 dx | RPWM → BTS7960 |
| 5 IN | 14 dx | sense IS ← trimmer |
| 6 OUT | 15 dx | corto → R 1K → GPIO3 |
| 7 GND | 16 dx | rail − |
| 8 OUT | 16 sx | cutout out → RailCom |
| 9 IN | 15 sx | cutout in ← GPIO4 |
| 10 OUT | 14 sx | libero |
| 11 IN | 13 sx | libero |
| 12 OUT | 12 sx | libero |
| 13 IN | 11 sx | libero |
| 14 VCC | 10 sx | ESP32 3V3 |

⚠️ La tacca/punto del pin 1 deve trovarsi in **alto a destra** del chip (riga 10 dx). Se è speculare la mappa di sopra è invertita.

---

## A — Componenti fisici sulla breadboard

Spunta solo se è **fisicamente piantato** in posizione, indipendentemente dai fili.

- [ ] ESP32-C6 Mini Waveshare a cavallo del canale, USB raggiungibile
- [ ] 74HC14 DIP-14 a cavallo del canale, pin 1 verso riga 10 dx
- [x] Trimmer 10 KΩ BOJACK P103 — outer A su riga 2 dx, cursore a rail− sx, outer B flottante
- [x] R 1 KΩ — gambe su riga 15 dx (pin 6) e riga 17 dx (landing)
- [ ] C 100 nF filtro IS (pin 5 → GND)
- [ ] C 100 nF bypass 74HC14 (pin 14 → pin 7)
- [ ] C 100 nF debounce Stop
- [ ] C 100 nF debounce Resume
- [ ] LED verde stato
- [ ] LED rosso stato
- [x] R 330 Ω serie LED verde
- [x] R 330 Ω serie LED rosso
- [ ] Pulsante rosso (Stop) 4 zampe
- [ ] Pulsante blu (Resume) 4 zampe
- [ ] OLED SSD1306/SSD1315 4 pin

Niente pull-down su GPIO2 (lo aggiungiamo solo se al test funzionale il DCC parte sporco).

Esterno breadboard:

- [ ] BTS7960 sul tavolo, vite morsetti accessibili
- [ ] Fusibile + portafusibile in serie su B+
- [ ] Alimentatore 15 V banco (SPENTO)

---

## B — Alimentazioni e ground

- [ ] Jumper ESP32 5V → rail + sx
- [ ] Jumper ESP32 GND → rail − sx
- [ ] Filo ESP32 3V3 → riga 10 sx (pin 14) — dedicato, NON via rail
- [ ] Filo riga 16 dx (pin 7) → **rail − sx** (filo dedicato che attraversa la breadboard, da doc)
- [ ] Filo ESP32 GND → BTS7960 GND (return path comune)
- [ ] Massa unica: rail − sx ↔ BTS7960 B− ↔ − alimentatore 15 V

⚠️ Su questa breadboard il **rail + è a 5 V** (design 30 righe). Non confondere col layout 60 righe dove il rail + è a 3.3 V.

ℹ️ Il **rail − dx non si usa** in questa configurazione. Tutte le connessioni GND del lato destro (pin 7 del 74HC14, GND del C 100 nF filtro IS, eventuale GND del bypass) hanno un filo dedicato verso il rail − **sinistro**. Se in futuro vuoi semplificare puoi aggiungere un ponticello rail − sx ↔ rail − dx, ma per ora seguiamo il doc.

---

## C — Cablaggio 74HC14, catena DCC (gate 1 + gate 2)

- [ ] Filo ESP32 GPIO2 → riga 10 dx (pin 1)
- [ ] Ponte breve riga 11 dx → riga 12 dx (pin 2 → pin 3, cascata)
- [ ] Filo riga 11 dx (pin 2) → BTS7960 LPWM
- [ ] Filo riga 13 dx (pin 4) → BTS7960 RPWM
- [ ] C 100 nF tra **riga 10 sx (pin 14)** e **riga 9 sx** ⚠️ **POSIZIONE AGGIORNATA** (era riga 11 sx)
- [ ] Jumper **riga 9 sx → riga 16 dx (pin 7 / GND)** ⚠️ **POSIZIONE AGGIORNATA** (era riga 11 sx)

⚠️ **Modifica del 2026-05-01**: la gamba GND del cap di bypass è stata spostata da riga 11 sx a **riga 9 sx** per liberare riga 11 sx, che serve come pin 13 (input gate 6) del 74HC14 per il **buffer Schmitt v3 del RailCom**. Vedi `railcom-checklist.md` Sezione C per il contesto. Il cap continua a fare lo stesso lavoro elettrico (VCC ↔ GND in parallelo), solo via percorso diverso.

---

## D — Catena IS (corto, gate 3)

- [x] Trimmer cursore → rail − sx
- [x] Trimmer outer A → riga 2 dx (sense node)
- [x] Trimmer outer B → flottante (configurazione reostato)
- [x] Filo BTS7960 R_IS → riga 2 dx
- [x] Filo BTS7960 L_IS → riga 2 dx (stessa riga, sommati)
- [x] Jumper riga 2 dx ↔ riga 2 sx (ponte canale)
- [x] Jumper riga 2 sx → riga 14 dx (pin 5 in)
- [ ] C 100 nF tra riga 14 dx (pin 5) e rail − sx (filo dedicato dal capo GND del cap)
- [x] R 1 KΩ — gamba 1 su riga 15 dx (pin 6), gamba 2 su riga 17 dx
- [x] Filo ESP32 GPIO3 → riga 17 dx

---

## E — BTS7960 (esterno)

- [ ] B+ → fusibile → + alimentatore 15 V (alimentatore SPENTO)
- [ ] B− → − alimentatore 15 V (+ GND comune)
- [ ] Vcc (logica) → ESP32 5V (filo dedicato, NON dal rail)
- [ ] R_EN ← ESP32 GPIO18
- [ ] L_EN ← ESP32 GPIO18 (stesso GPIO, due fili)
- [ ] LPWM ← 74HC14 pin 2 (riga 11 dx)
- [ ] RPWM ← 74HC14 pin 4 (riga 13 dx)
- [x] R_IS → riga 2 dx
- [x] L_IS → riga 2 dx
- [ ] M+ → binario A (morsetto/WAGO)
- [ ] M− → binario B (morsetto/WAGO)

---

## F — LED stato (GPIO14 verde, GPIO15 rosso)

Anodo (gamba lunga) lato GPIO via R serie, catodo a GND.

- [ ] GPIO14 → R 330 Ω → anodo LED verde
- [ ] Catodo LED verde → rail −
- [ ] GPIO15 → R 330 Ω → anodo LED rosso
- [ ] Catodo LED rosso → rail −

---

## G — Pulsanti (Stop GPIO22, Resume GPIO21)

Configurati come input pull-up nel firmware → riposo HIGH, premuto LOW.

- [ ] Stop (rosso): zampa 1 → GPIO22, zampa opposta → rail −
- [ ] C 100 nF in parallelo Stop (tra GPIO22 e GND)
- [ ] Resume (blu): zampa 1 → GPIO21, zampa opposta → rail −
- [ ] C 100 nF in parallelo Resume (tra GPIO21 e GND)

---

## H — OLED SSD1306/SSD1315 (I2C)

- [ ] OLED VCC → **riga 10 sx** (= pin 14 / 3V3 — condivide la riga col cap di bypass; NON tappare al rail + che è 5 V)
- [ ] OLED GND → rail − sx
- [ ] OLED SDA → ESP32 GPIO19
- [ ] OLED SCL → ESP32 GPIO20

Indirizzo I2C atteso: **0x3C**.

---

## I — Interfaccia RailCom (opzionale, design v3 con buffer Schmitt)

Solo se la breadboard RailCom è collegata. Sono i fili di segnale che escono dalla main DCC verso quella RailCom.

### I.1 — Catena cutout (GPIO4 → MOSFET RailCom)

- [ ] Filo ESP32 GPIO4 → riga 15 sx (pin 9, input gate 4)
- [ ] Filo riga 16 sx (pin 8, output gate 4) → breadboard RailCom riga 3 dx (CUTOUT_INV)

### I.2 — Catena ricezione RailCom via buffer v3 (gate 5 + gate 6 del 74HC14) ➕

⚠️ **Pre-requisito**: il cap di bypass del 74HC14 deve essere già stato spostato da riga 11 sx a riga 9 sx (Sezione C). Se non l'hai ancora fatto, riga 11 sx è ancora a GND e il buffer v3 non funziona.

- [ ] Filo breadboard RailCom (uscita TLV3501 OUT) → **riga 13 sx (pin 11, input gate 5)**
- [ ] **Ponticello breve riga 14 sx (pin 10, output gate 5) → riga 11 sx (pin 13, input gate 6)**
- [ ] Filo **riga 12 sx (pin 12, output gate 6) → ESP32 GPIO5**

Doppia inversione = polarità invariata, ma fronti puliti dallo Schmitt trigger interno del 74HC14. Per dettagli e motivazione vedi `railcom-checklist.md` Sezione C.

⚠️ Se trovi un filo da **GPIO18 al pin 9** è la versione vecchia: scollegalo e cabla GPIO4 al suo posto. GPIO18 deve restare dedicato all'enable BTS7960.

⚠️ Se trovi un filo che va **direttamente da TLV3501 OUT a GPIO5** (saltando il 74HC14), è la versione v2 vecchia. Per il v3 il segnale deve passare per il buffer come descritto in I.2.

Se la RailCom è scollegata: salta la sezione, GPIO4 e GPIO5 restano flottanti, riga 11/12/13/14 sx restano libere.

---

## J — Verifica continuità (alimentazione SCOLLEGATA, multimetro buzzer)

### Power

- [ ] ESP32 5V ↔ rail + sx : continuo
- [ ] ESP32 GND ↔ rail − sx : continuo
- [ ] Rail + ↔ rail − : NON continuo
- [ ] ESP32 3V3 ↔ riga 10 sx (pin 14) : continuo
- [ ] Riga 16 dx (pin 7) ↔ rail − sx : continuo (via filo dedicato)
- [ ] Riga 10 sx (pin 14) ↔ rail + : NON continuo (3.3 V e 5 V devono restare separati)
- [ ] Riga 9 sx ↔ riga 16 dx (pin 7) : continuo (jumper dell'altro capo del bypass cap, posizione aggiornata)
- [ ] Riga 9 sx ↔ rail − sx : continuo (transitivo via pin 7 → rail −)
- [ ] Riga 11 sx ↔ qualsiasi nodo : NON continuo (riga liberata, sarà usata dal buffer v3 RailCom)

### Catena DCC

- [ ] GPIO2 ↔ riga 10 dx : continuo
- [ ] Riga 11 dx ↔ riga 12 dx : continuo (ponte cascata)
- [ ] Riga 11 dx ↔ BTS7960 LPWM : continuo
- [ ] Riga 13 dx ↔ BTS7960 RPWM : continuo

### Catena IS

- [ ] BTS7960 R_IS ↔ riga 2 dx : continuo
- [ ] Riga 2 dx ↔ riga 2 sx : continuo (ponte canale)
- [ ] Riga 2 sx ↔ riga 14 dx (pin 5) : continuo (jumper)
- [ ] Cursore trimmer ↔ rail − : continuo
- [ ] Outer B trimmer ↔ qualsiasi nodo : NON continuo
- [ ] Riga 14 dx ↔ rail − sx : NON continuo direttamente (C in mezzo, ohmmetro alto in carica)
- [ ] Ohmmetro GPIO3 (riga 17 dx) ↔ pin 6 (riga 15 dx) : **~1000 Ω**
- [ ] Riga 17 dx ↔ rail / GPIO2 / qualsiasi altro nodo : NON continuo (la riga è solo R 1K + filo GPIO3)

### LED e pulsanti

- [ ] GPIO14 ↔ catodo LED verde : NON continuo (R 330 Ω in mezzo)
- [ ] GPIO15 ↔ catodo LED rosso : NON continuo (R 330 Ω in mezzo)
- [ ] GPIO22 a Stop premuto ↔ rail − : continuo; rilasciato : NON continuo
- [ ] GPIO21 a Resume premuto ↔ rail − : continuo; rilasciato : NON continuo

### OLED

- [ ] OLED VCC ↔ riga 10 sx (pin 14) : continuo
- [ ] OLED VCC ↔ ESP32 3V3 : continuo (transitivo via pin 14)
- [ ] OLED GND ↔ rail − sx : continuo
- [ ] OLED SDA ↔ GPIO19 : continuo
- [ ] OLED SCL ↔ GPIO20 : continuo

---

## K — Test alimentato (USB only, B+ 15 V SPENTO)

Solo dopo che J è tutta verde. Alimentatore 15 V scollegato fisicamente o spento + fusibile estratto.

- [ ] USB ESP32 al PC
- [ ] Rail + sx = **5.0 V** (±5 %)
- [ ] Rail − = 0 V
- [ ] Riga 10 sx (pin 14) = **3.3 V**
- [ ] Riga 16 dx (pin 7) = 0 V
- [ ] BTS7960 Vcc = 5 V (filo dedicato)
- [ ] OLED si accende, mostra boot/IP
- [ ] LED verde reagisce (lampeggio o stato fisso)
- [ ] Stop premuto → log/LED reagisce
- [ ] Resume premuto → log/LED reagisce

Se uno fallisce: stacca subito USB e torna alla J.

---

## L — Test funzionale completo (15 V acceso)

- [ ] Inserisci fusibile, accendi alimentatore 15 V
- [ ] `cargo run`
- [ ] Scope su GPIO2: bit "1" = 58 µs ±3, "0" ≥ 100 µs
- [ ] Scope su BTS7960 M+ (vs M−): onda ±15 V, simmetrica
- [ ] App Z21 (o comando firmware) → loco si muove sui binari
- [ ] Test corto: rame sui due binari per ~100 ms → fault rilevato, BTS7960 disabilitato, LED rosso

Se tutto verde: **main DCC è in sesto**, si può ricollegare la breadboard RailCom (Sezione I).
