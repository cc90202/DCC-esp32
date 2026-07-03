# DRV8874 + RailCom detector bidirezionale con 2x TLV3501

Ultimo aggiornamento: 2026-05-25

> **Stop build - aggiornamento 2026-05-28:** non cablare questa checklist come
> schema finale. Il cutout via `GPIO4 -> EN/IN1` e brake-low del DRV8874 e'
> ancora la direzione corretta se si vuole evitare un terminatore di potenza
> esterno. Il detector descritto qui, pero', porta i nodi di sense direttamente
> agli ingressi TLV3501: fuori dal cutout quei nodi seguono il DCC normale e
> possono superare il range ammesso dal comparatore a 3.3 V. Serve prima un
> front-end di misura protetto o un terminatore esterno che tenga il sense node
> vicino a GND solo durante la finestra RailCom.
>
> **Decisione successiva:** il prossimo rebuild segue
> [`railcom-option-a-external-terminator.md`](railcom-option-a-external-terminator.md):
> DRV8874 in PWM mode, `GPIO4` come `DCC_RUN`, logica AND esterna per `IN1/IN2`
> e terminatore RailCom esterno attivo solo durante il cutout.

Questa checklist estende il circuito DCC gia' funzionante con Pololu DRV8874
aggiungendo il cutout RailCom e un detector veloce. E' pensata per il banco con:

- ESP32-C6 Mini
- Pololu DRV8874 #4035 gia' funzionante come driver DCC
- due comparatori veloci TLV3501 su breakout a 8 pin
- 74HC14 alimentato a 3.3 V
- opzionale: due MOSFET logic-level se vuoi tenere i resistori di sense fuori
  dal percorso DCC normale. Per il primo prototipo si puo' anche usare il
  brake-low interno del DRV8874.

## Pinout TLV3501 su breakout 8 pin

Ogni TLV3501AIDR in SOIC-8 montato su breakout si tratta come un DIP-8 a
cavallo del canale centrale della breadboard. Orienta il pin 1 usando il
pallino/tacca del chip o la serigrafia del breakout.

| Pin | Nome | Collegamento |
| --- | --- | --- |
| 1 | NC | flottante |
| 2 | IN- | SENSE_A per comparatore A, SENSE_B per comparatore B |
| 3 | IN+ | VREF_RAILCOM |
| 4 | V- | STAR_GND |
| 5 | NC | flottante |
| 6 | OUT | rete di combinazione active-low verso il gate RX del 74HC14 |
| 7 | V+ | 3.3 V |
| 8 | SHDN | STAR_GND, obbligatorio: se resta alto/flottante il chip va in shutdown |

## Scelta architetturale

Non usare `SLEEP` come cutout RailCom: e' troppo lento. La Pololu #4035 in
PH/EN mode pero' ha un altro ingresso utile: quando `SLEEP` resta HIGH e
`EN/IN1` va LOW, il DRV8874 mette entrambe le uscite a LOW (`brake low`) con i
MOSFET interni. Questo puo' creare il cutout senza MOSFET esterni.

Ci sono quindi due topologie possibili:

1. **Senza MOSFET esterni:** `GPIO4` abbassa `EN/IN1` durante il cutout; il
   DRV8874 porta OUT1 e OUT2 a massa internamente. I resistori di sense stanno
   in serie alle rotaie e sono letti dai TLV3501.
2. **Con MOSFET/switch esterni:** i resistori di sense vengono inseriti solo
   nella finestra RailCom. E' piu' pulito per plastici con piu' corrente, ma
   richiede piu' componenti.

La topologia senza MOSFET esterni e':

```text
Normal DCC:
  GPIO18 = HIGH -> DRV8874 SLEEP alto -> ponte attivo
  EN/IN1 = HIGH
  PH/IN2 = segnale DCC da GPIO2

RailCom cutout:
  GPIO18 = HIGH -> DRV8874 resta sveglio
  GPIO4  = LOW  -> logica cutout abbassa EN/IN1
  EN/IN1 = LOW  -> DRV8874 brake-low: OUT1 = LOW, OUT2 = LOW

OUT1 -- R_A 2.7 ohm -- TRACK_A
OUT2 -- R_B 2.7 ohm -- TRACK_B

TRACK_A side of R_A -> TLV3501_A IN-
TRACK_B side of R_B -> TLV3501_B IN-
VREF ~= 22 mV -> entrambi IN+
OUT_A/OUT_B -> Schottky OR active-low -> 74HC14 RX gate -> GPIO5
```

Con due comparatori il detector vede entrambe le direzioni della corrente
RailCom. Questo evita il limite della vecchia versione single-ended, dove circa
meta' finestre dipendevano dalla polarita' del decoder/binario.

## Firmware attuale

Il firmware e' vicino alla sequenza necessaria, ma per la Pololu #4035 va
adattato prima del bring-up RailCom:

- `GPIO18` e' l'enable fisico del track output. Sul DRV8874 e' collegato a
  `SLEEP`.
- `GPIO4` e' il comando cutout RailCom, active-low lato ESP32.
- `GPIO5` e' UART RX RailCom a 250000 baud.
- In `src/boot.rs` l'input UART RX e' invertito in firmware.
- Durante il cutout RailCom `GPIO18/SLEEP` deve restare HIGH. Il datasheet
  DRV8874 indica circa 1 ms per entrare/uscire da sleep, troppo per una finestra
  RailCom da circa 460 us.
- Per la variante senza MOSFET esterni, `EN/IN1` non puo' restare cablato fisso
  a 3.3 V: deve essere comandabile dalla logica di cutout, restando HIGH in DCC
  normale e andando LOW solo nella finestra RailCom.

Con questa configurazione, il cablaggio RX coerente e':

```text
RAILCOM_RAW -- 74HC14 gate singolo -- GPIO5
```

Non usare il doppio gate non-invertente dei documenti BTS v3 a meno di togliere
l'inversione firmware su GPIO5.

## Componenti

Minimo per il bring-up bidirezionale:

- 2 comparatori TLV3501 su breakout 8 pin
- 1 gate/logica veloce per comando cutout su `EN/IN1`
- 1 gate 74HC14 per buffer RX
- 2 resistori 2.7 ohm, almeno 0.5 W, meglio 1 W o piu' sul banco
- 150 kohm + 1 kohm per VREF ~= 21.9 mV
- 100 nF su VREF verso STAR_GND
- 100 nF vicino a ogni comparatore tra V+ e V-
- opzionale ma consigliato: 100 pF vicino a ogni comparatore in parallelo al 100 nF
- 2 diodi Schottky per combinare le uscite, per esempio BAT54 o 1N5819
- 4.7 kohm pull-up da RAILCOM_RAW a 3.3 V
- 100 kohm pull-up sul nodo che mantiene `EN/IN1` HIGH fuori cutout, se serve
  un default hardware sicuro

Il TLV3501 non ha pin LE/HYS. Parti senza feedback esterno. Se vedi
oscillazioni sullo scope, prima verifica massa, bypass e VREF; poi aggiungi
isteresi solo separando le due soglie:

```text
VREF_RAILCOM -> 10 kohm -> VREF_A / IN+ comparatore A
VREF_RAILCOM -> 10 kohm -> VREF_B / IN+ comparatore B
OUT_A -> 680 kohm - 1 Mohm -> VREF_A
OUT_B -> 680 kohm - 1 Mohm -> VREF_B
```

Non collegare le resistenze di isteresi direttamente al nodo VREF comune, perche'
un comparatore muoverebbe anche la soglia dell'altro. Non usare 47 kohm con
VREF basso: sposta troppo la soglia effettiva.

## Cablaggio cutout senza MOSFET esterni

Il cutout deve abbassare `EN/IN1`, non `SLEEP`.

```text
ESP32 GPIO4 HIGH fuori cutout
ESP32 GPIO4 LOW durante cutout

GPIO4 diretto, oppure logica non-invertente -> EN/IN1 HIGH fuori cutout
GPIO4 diretto, oppure logica non-invertente -> EN/IN1 LOW durante cutout

GPIO18/SLEEP resta HIGH durante tutta la finestra RailCom.
```

Con la polarita' firmware attuale non mettere un solo 74HC14 tra GPIO4 ed
`EN/IN1`, perche' invertirebbe il comando: servirebbe invece collegamento
diretto o doppia inversione.

Stati attesi:

| Stato | GPIO18/SLEEP | GPIO4 | EN/IN1 | OUT1/OUT2 |
| --- | --- | --- | --- | --- |
| DCC normale | HIGH | HIGH | HIGH | segue PH/IN2 |
| cutout RailCom | HIGH | LOW | LOW | LOW/LOW brake |
| estop/fault | LOW | HIGH | don't care | Hi-Z sleep dopo tSLEEP |

## Cablaggio detector bidirezionale

```text
DRV8874 OUT1 -> R_A 2.7 ohm -> TRACK_A/binario A

DRV8874 OUT2 -> R_B 2.7 ohm -> TRACK_B/binario B

TRACK_A side of R_A -> TLV3501_A IN-
TRACK_B side of R_B -> TLV3501_B IN-
VREF_RAILCOM -> TLV3501_A IN+
VREF_RAILCOM -> TLV3501_B IN+
TLV3501_A OUT -> diode-combine
TLV3501_B OUT -> diode-combine
diode-combine RAILCOM_RAW -> 74HC14 input RX
74HC14 output RX -> ESP32 GPIO5
```

VREF:

```text
3.3 V -> 150 kohm -> VREF_RAILCOM -> 1 kohm -> STAR_GND
VREF_RAILCOM -> 100 nF -> STAR_GND
```

Valori attesi:

```text
VREF = 3.3 V * 1 kohm / (150 kohm + 1 kohm) ~= 21.9 mV
I_threshold = 21.9 mV / 2.7 ohm ~= 8.1 mA
burden totale ~= 2.7 ohm + 2.7 ohm = 5.4 ohm
burden a 34 mA ~= 184 mV
```

## Combinazione uscite dei due TLV3501

I TLV3501 hanno uscita push-pull: non collegare OUT_A e OUT_B direttamente
insieme. Combinali con due Schottky e una pull-up:

```text
RAILCOM_RAW -- 4.7 kohm -- 3.3 V
RAILCOM_RAW --|<|-- OUT_A
RAILCOM_RAW --|<|-- OUT_B
RAILCOM_RAW -> 74HC14 RX gate -> GPIO5
```

Orientamento diodi:

- anodi insieme su `RAILCOM_RAW`
- catodo del primo diodo su `OUT_A`
- catodo del secondo diodo su `OUT_B`

Quando entrambi i comparatori sono idle, OUT_A e OUT_B sono HIGH e
`RAILCOM_RAW` resta HIGH tramite pull-up. Quando uno dei due rileva corrente
RailCom, la sua uscita va LOW e trascina `RAILCOM_RAW` LOW attraverso il diodo.

## Cablaggio RX coerente con il firmware attuale

Il firmware inverte l'input UART RX su GPIO5. Quindi usa un solo gate 74HC14:

```text
RAILCOM_RAW idle HIGH
  -> 74HC14 output LOW
  -> GPIO5 fisico LOW
  -> inverter firmware
  -> UART vede idle HIGH
```

Durante un impulso RailCom sopra soglia:

```text
SENSE_A > VREF oppure SENSE_B > VREF
  -> RAILCOM_RAW LOW
  -> 74HC14 output HIGH
  -> GPIO5 fisico HIGH
  -> inverter firmware
  -> UART vede bit LOW
```

## Verifiche prima di alimentare

Con USB e alimentatore 15 V scollegati:

- [ ] 3.3 V non e' in corto con GND.
- [ ] VIN DRV8874 non e' in corto con 3.3 V.
- [ ] STAR_GND e GND ESP32 sono continui.
- [ ] `EN/IN1` e' comandabile: HIGH fuori cutout, LOW durante cutout.
- [ ] `SLEEP` non viene usato per la finestra RailCom.
- [ ] VREF non e' in corto diretto a 3.3 V o GND.
- [ ] OUT_A e OUT_B non sono in corto diretto tra loro.
- [ ] `RAILCOM_RAW` legge verso 3.3 V tramite circa 4.7 kohm.
- [ ] `RAILCOM_RAW` non e' collegato direttamente a GPIO5: passa dal 74HC14.

## Test alimentato senza 15 V

Solo USB ESP32:

- [ ] 74HC14 VCC = 3.3 V, GND = 0 V.
- [ ] TLV3501_A pin 7 V+ = 3.3 V.
- [ ] TLV3501_A pin 4 V- = 0 V.
- [ ] TLV3501_A pin 8 SHDN = 0 V.
- [ ] TLV3501_B pin 7 V+ = 3.3 V.
- [ ] TLV3501_B pin 4 V- = 0 V.
- [ ] TLV3501_B pin 8 SHDN = 0 V.
- [ ] VREF_RAILCOM ~= 22 mV.
- [ ] GPIO4 idle HIGH.
- [ ] EN/IN1 idle HIGH.
- [ ] OUT_A idle HIGH se SENSE_A = 0 V.
- [ ] OUT_B idle HIGH se SENSE_B = 0 V.
- [ ] RAILCOM_RAW idle HIGH.
- [ ] GPIO5 fisico idle LOW se passa da un solo gate 74HC14.

## Test con 15 V e oscilloscopio

Prima senza locomotiva:

- [ ] DCC normale su OUT1/OUT2 resta pulito.
- [ ] Durante cutout, GPIO18 resta HIGH.
- [ ] Durante cutout, EN/IN1 va LOW.
- [ ] Durante cutout, OUT1 e OUT2 vanno entrambi LOW.
- [ ] Senza locomotiva, OUT_A/OUT_B e RAILCOM_RAW restano stabili, senza burst casuali.

Poi con locomotiva RailCom-capable:

- [ ] Su SENSE_A compaiono impulsi piccoli durante alcune finestre.
- [ ] Su SENSE_B compaiono impulsi piccoli nelle finestre complementari.
- [ ] OUT_A e OUT_B mostrano impulsi active-low puliti.
- [ ] RAILCOM_RAW mostra l'unione active-low dei due comparatori.
- [ ] GPIO5 fisico mostra impulsi active-high puliti, perche' passa da un
      singolo inverter.
- [ ] Nei log RailCom aumentano `rx_windows`.
- [ ] Se `rx_windows` aumenta ma tutto e' vuoto, verifica prima che il cutout
      stia fermando il pilotaggio DCC e che `RAILCOM_RAW` arrivi al 74HC14.

## Cose da non fare

- Non collegare VREF o SENSE al pin FAULT del DRV8874.
- Non usare il pin CS del DRV8874 come detector RailCom: e' il current sense
  del driver, non il ritorno RailCom durante cutout.
- Non mettere RC lenti su SENSE_A: RailCom e' UART 250 kbaud, quindi i fronti
  devono restare veloci.
- Non lasciare SHDN flottante.
- Non usare il doppio gate RX se il firmware continua a invertire GPIO5.
- Non provare RailCom con alimentatore senza limite di corrente basso.

## Note firmware

Per il bring-up non serve modificare il codice se:

- `GPIO18` resta collegato a `SLEEP` del DRV8874 e non viene toccato durante
  la finestra RailCom.
- `GPIO4` controlla la logica che abbassa `EN/IN1` durante il cutout RailCom.
- `GPIO5` riceve l'output di un solo gate 74HC14.

Se decidi di usare il doppio gate 74HC14 non-invertente per RX, cambia prima il
firmware rimuovendo `.with_input_inverter(true)` dalla configurazione di GPIO5.
