# DRV8874 + RailCom con 2x TLV3501

Ultimo aggiornamento: 2026-05-25

> **Stop build - aggiornamento 2026-05-28:** questo schema non va montato
> cosi' com'e'. La parte cutout `GPIO4 -> EN/IN1` in PH/EN mode resta valida:
> durante RailCom il DRV8874 deve restare sveglio e andare in brake-low
> (`OUT1=LOW`, `OUT2=LOW`). La parte detector invece collega i TLV3501
> direttamente ai nodi in serie alle rotaie; durante il DCC normale quei nodi
> vedono la tensione del binario e possono uscire dal range di ingresso del
> comparatore alimentato a 3.3 V. Prima del prossimo test serve ridisegnare il
> front-end di misura: shunt in serie con protezione/current-sense adatto al
> common-mode DCC, oppure terminatore esterno attivo solo durante cutout.
>
> **Decisione successiva:** per il prossimo rebuild usare l'opzione A in
> [`railcom-option-a-external-terminator.md`](railcom-option-a-external-terminator.md):
> DRV8874 in PWM mode, uscite Hi-Z durante cutout, e terminatore/detector
> RailCom esterno acceso solo nella finestra.

Schema del circuito RailCom usando la Pololu DRV8874 #4035 come ponte DCC e
come generatore del cutout tramite `EN/IN1` in modalita' PH/EN. Non usa MOSFET
esterni per il cutout.

## Idea base

In funzionamento DCC normale il DRV8874 pilota le due rotaie con il segnale DCC.
Durante il cutout RailCom, `SLEEP` resta alto e viene abbassato solo `EN/IN1`.
In PH/EN mode, `EN/IN1 = LOW` forza `OUT1 = LOW` e `OUT2 = LOW` (`brake low`),
portando entrambe le rotaie circa a massa per la finestra RailCom.

```text
DCC normale:
  GPIO18/SLEEP = HIGH
  GPIO4/EN     = HIGH
  GPIO2/PH     = DCC
  rotaie       = DCC ~15 V differenziale

RailCom cutout:
  GPIO18/SLEEP = HIGH
  GPIO4/EN     = LOW
  GPIO2/PH     = irrilevante
  OUT1/OUT2    = LOW/LOW
  rotaie       = circa 0 V differenziale
```

`SLEEP` non va usato per la finestra RailCom: e' troppo lento. Resta
l'interruttore generale per boot, fault ed emergenza.

## Collegamenti DRV8874

```text
                    ESP32-C6
                 +-----------+
 GPIO18 -------->| SLEEP     |  master enable / fault / estop
 GPIO4  -------->| EN/IN1    |  HIGH = DCC attivo, LOW = RailCom cutout
 GPIO2  -------->| PH/IN2    |  segnale DCC RMT
 GPIO5  <--------| RailCom RX|
                 +-----------+


              Pololu DRV8874 #4035
        +--------------------------------+
 15V -->| VIN                        OUT1|----[ R_A 2.2R >=1W ]----+---- ROTAIA A
 GND -->| GND                        OUT2|----[ R_B 2.2R >=1W ]----+---- ROTAIA B
 GPIO18>| SLEEP                          |
 GPIO4 >| EN/IN1                         |
 GPIO2 >| PH/IN2                         |
 GND -->| PMODE                          |  PH/EN mode
        +--------------------------------+
```

Note:

- `EN/IN1` non deve piu' stare fisso a 3.3 V: va pilotato da `GPIO4`.
- `PMODE` va tenuto a GND prima del risveglio del chip, cosi' il DRV8874 entra
  in PH/EN mode.
- `VIN` riceve i 15 V dell'alimentatore da banco.
- GND ESP32, GND DRV8874 e negativo alimentatore devono essere comuni.

## Resistenze di sense

I due resistori da 2.2 ohm sono in serie alle rotaie:

```text
DRV8874 OUT1 -> R_A 2.2R -> ROTAIA A
DRV8874 OUT2 -> R_B 2.2R -> ROTAIA B
```

I punti da leggere sono i lati rotaia:

```text
SENSE_A = nodo tra R_A e ROTAIA A
SENSE_B = nodo tra R_B e ROTAIA B
```

Durante il DCC normale le resistenze sono sempre in serie al carico. Usare
resistenze almeno da 1 W per il banco; se aumenti corrente o numero di
locomotive, ricalcolare dissipazione e caduta.

## Riferimento VREF RailCom

```text
                         3.3V
                          |
                       [150K]
                          |
                    VREF_RAILCOM  ~22 mV
                          |
                        [1K]
                          |
                         GND
                          |
                       [100nF]
                          |
                         GND
```

Valori:

```text
VREF ~= 3.3 V * 1K / (150K + 1K) ~= 21.9 mV
I_threshold ~= 21.9 mV / 2.2 ohm ~= 10 mA
```

## Comparatori TLV3501

Usare due TLV3501 su breakout 8 pin:

```text
TLV3501_A                          TLV3501_B
+---------+                         +---------+
| pin 7   |---- 3.3V                | pin 7   |---- 3.3V
| pin 4   |---- GND                 | pin 4   |---- GND
| pin 8   |---- GND                 | pin 8   |---- GND
| pin 2 - |<--- SENSE_A             | pin 2 - |<--- SENSE_B
| pin 3 + |<--- VREF_RAILCOM        | pin 3 + |<--- VREF_RAILCOM
| pin 6   |---- OUT_A               | pin 6   |---- OUT_B
+---------+                         +---------+
```

Pinout TLV3501:

| Pin | Nome | Collegamento |
| --- | --- | --- |
| 1 | NC | flottante |
| 2 | IN- | `SENSE_A` o `SENSE_B` |
| 3 | IN+ | `VREF_RAILCOM` |
| 4 | V- | GND |
| 5 | NC | flottante |
| 6 | OUT | `OUT_A` o `OUT_B` |
| 7 | V+ | 3.3 V |
| 8 | SHDN | GND obbligatorio |

Mettere un condensatore 100 nF vicino a ogni TLV3501 tra pin 7 e pin 4.

## Combinazione uscite

I TLV3501 hanno uscita push-pull, quindi `OUT_A` e `OUT_B` non vanno collegati
direttamente insieme. Usare due Schottky e una pull-up:

```text
                         3.3V
                          |
                        [4.7K]
                          |
                      RAILCOM_RAW
                       /        \
        anodo D_A ----+          +---- anodo D_B
        catodo D_A -------- OUT_A
        catodo D_B -------- OUT_B


RAILCOM_RAW ----> 74HC14 input
74HC14 output ---> GPIO5 ESP32
```

Logica:

- idle: `OUT_A = HIGH`, `OUT_B = HIGH`, `RAILCOM_RAW = HIGH`.
- impulso RailCom su una rotaia: il TLV3501 corrispondente porta la sua uscita
  LOW e tira `RAILCOM_RAW` LOW attraverso il diodo.
- il 74HC14 inverte `RAILCOM_RAW` e produce un segnale pulito per `GPIO5`.

Con il firmware attuale `GPIO5` e' invertito in input, quindi usare **un solo**
gate 74HC14 sul percorso RX.

## Verifiche rapide

Prima di alimentare:

- [ ] `VIN` non e' in corto con 3.3 V.
- [ ] GND ESP32, GND DRV8874 e negativo alimentatore sono comuni.
- [ ] `EN/IN1` e' collegato a `GPIO4`, non fisso a 3.3 V.
- [ ] `SLEEP` resta collegato a `GPIO18`.
- [ ] `PMODE` e' collegato a GND.
- [ ] `OUT_A` e `OUT_B` non sono cortocircuitati direttamente tra loro.
- [ ] `RAILCOM_RAW` ha pull-up 4.7K a 3.3 V.
- [ ] `SHDN` di entrambi i TLV3501 e' a GND.

Con USB soltanto:

- [ ] `GPIO4` idle HIGH.
- [ ] `EN/IN1` idle HIGH.
- [ ] `VREF_RAILCOM` circa 22 mV.
- [ ] `RAILCOM_RAW` idle HIGH.
- [ ] `GPIO5` fisico idle LOW dopo un solo gate 74HC14.

Con 15 V e oscilloscopio:

- [ ] DCC normale presente tra le rotaie.
- [ ] Durante cutout, `GPIO18/SLEEP` resta HIGH.
- [ ] Durante cutout, `GPIO4/EN` va LOW.
- [ ] Durante cutout, `OUT1` e `OUT2` vanno entrambi LOW.
- [ ] Con locomotiva RailCom-capable, `SENSE_A` e/o `SENSE_B` mostrano impulsi
      nella finestra RailCom.
- [ ] `RAILCOM_RAW` mostra impulsi active-low puliti.
