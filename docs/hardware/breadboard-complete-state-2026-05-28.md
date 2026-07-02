# Stato completo della breadboard al 28 maggio 2026

Snapshot di tutto il cablaggio attuale sulla breadboard, dal modulo
ESP32-C6 Mini fino ai connettori che vanno al binario, passando per il
percorso RailCom completo. Documento di riferimento per le prossime
sessioni di lavoro, scritto in prosa fluida senza acronimi non spiegati.

La sessione del 27 maggio ha rimesso in discussione la topologia del
rilevatore RailCom alla luce della norma RCN-217. La rettifica completa
con la discussione delle due strade alternative è in
[`docs/references/railcom-drv88xx/README.md`](../references/railcom-drv88xx/README.md).
Questo file invece descrive solo *cosa* è cablato adesso, non *perché*
funzioni o non funzioni.

## Alimentazioni e masse

Ci sono due alimentazioni distinte sulla breadboard, e una massa unica.

Il cavo USB-C dal PC entra nel modulo ESP32-C6 Mini, il quale ha al suo
interno un regolatore che produce i 3,3 volt che servono al
microcontrollore e alla logica. Il piedino "3V3" del modulo si attacca al
rail orizzontale superiore della breadboard, e il piedino "GND" del
modulo va al rail orizzontale inferiore.

L'alimentatore da banco fornisce 15 volt come tensione di trazione, e si
collega direttamente al piedino VIN del modulo Pololu DRV8874 con un
filo dedicato. Il polo negativo dell'alimentatore va in comune con la
massa dell'ESP32, cioè finisce sullo stesso rail GND della breadboard.

Tutte le masse sono unite in un solo nodo. Senza una massa comune fra
ESP32, Pololu e alimentatore, nessuna delle letture analogiche del
RailCom avrebbe senso.

## Il modulo Waveshare ESP32-C6 Mini

Il modulo è infilato a cavallo della breadboard. I piedini che usiamo
sono questi:

```
GPIO2  -> piedino PH/IN2 del DRV8874    (onda DCC dall'RMT)
GPIO3  -> uscita storica del rilevatore di corto sul 74HC14
GPIO4  -> piedino EN/IN1 del DRV8874    (idle ALTO, cutout BASSO)
GPIO5  <- uscita finale del rilevatore RailCom (dal 74HC14)
GPIO14 -> anodo del LED verde (tramite resistenza)
GPIO15 -> anodo del LED rosso (tramite resistenza)
GPIO18 -> piedino SLEEP del DRV8874     (alto = chip vivo)
GPIO19 -> linea dati I2C (SDA) verso l'OLED
GPIO20 -> linea clock I2C (SCL) verso l'OLED
GPIO21 -> pulsante "resume" (a massa quando premuto)
GPIO22 -> pulsante "stop"   (a massa quando premuto)
```

Le linee 3V3 e GND del modulo vanno ai due rail di alimentazione della
breadboard, come detto sopra.

## Il display OLED

L'OLED SSD1306 da 1,3 pollici è un modulino con quattro fili che esce a
destra della breadboard:

```
OLED VCC -- +3,3 V (rail della breadboard)
OLED GND -- GND   (rail della breadboard)
OLED SDA -- GPIO19 dell'ESP32
OLED SCL -- GPIO20 dell'ESP32
```

Le linee I2C SDA e SCL hanno bisogno di resistenze di tiraggio verso il
positivo per funzionare. Quelle sono già montate sul modulo OLED stesso
(resistenze SMD da 4,7 o 10 kiloohm sulla pcb del display), quindi non
sono state aggiunte separatamente sulla breadboard.

## I due LED di stato

Sul lato sinistro della breadboard ci sono due LED da tre millimetri con
la loro resistenza di limitazione in serie. Uno verde, uno rosso.

```
GPIO14 ---[ R_LEDv 330 Ω ]--- |>|-- GND
                              LED verde

GPIO15 ---[ R_LEDr 330 Ω ]--- |>|-- GND
                              LED rosso
```

Il verde lampeggia quando il sistema è in funzione normale, il rosso si
accende in caso di fault. Le resistenze sono nel valore tipico da 330
ohm, prese dal kit.

## I due pulsanti di controllo

Sono due tactile switch a quattro piedini, montati a cavallo della
breadboard. Sfruttano la resistenza di tiraggio verso l'alto interna
all'ESP32, quindi non hanno componenti esterni di contorno.

```
GPIO21 ---+
          |
          o   pulsante "resume" (normalmente aperto)
          |
         GND

GPIO22 ---+
          |
          o   pulsante "stop"   (normalmente aperto)
          |
         GND
```

Quando il pulsante è aperto il GPIO sta alto per via della resistenza
interna. Quando premuto va a massa, il firmware se ne accorge.

## Il modulo Pololu DRV8874

È un modulo carrier completo (Pololu #4035) con il chip Texas
Instruments DRV8874, i suoi condensatori di alimentazione, e otto
piedini sporgenti su due file. Si infila a cavallo del canale centrale
della breadboard.

```
Modulo Pololu DRV8874 (carrier #4035)

Lato logica (4 piedini):
   SLEEP    <-- GPIO18  (alto = chip vivo)
   EN/IN1   <-- GPIO4   (alto = DCC attivo, basso = cutout)
   PH/IN2   <-- GPIO2   (onda DCC)
   PMODE    --- GND     (collegato a massa = modalità "fase + abilitazione")

Lato potenza (4 piedini):
   VIN     <-- 15 V dall'alimentatore da banco
   GND     <-- massa comune
   OUT1    --> verso resistenza R_A    (vedi sotto)
   OUT2    --> verso resistenza R_B    (vedi sotto)
```

I piedini IPROPI, VREF e nFAULT del chip non sono cablati sulla
breadboard: restano scollegati. Il modulo Pololu ha già le sue
resistenze di default per VREF a bordo, e il nFAULT è lasciato flottare
per ora.

## Le resistenze di sense e l'uscita verso il binario

Fra le uscite del DRV e i connettori WAGO che portano al binario ci sono
due resistenze di potenza in serie:

```
DRV OUT1 ---[ R_A 2,2 Ω 1 W ]---+---> WAGO --> ROTAIA A
                                |
                              SENSE_A  (filo verso il comparatore A)

DRV OUT2 ---[ R_B 2,2 Ω 1 W ]---+---> WAGO --> ROTAIA B
                                |
                              SENSE_B  (filo verso il comparatore B)
```

I due nodi SENSE_A e SENSE_B sono fisicamente la gambetta della
resistenza dal lato del binario. Da lì parte un cavetto sottile che
attraversa la breadboard fino agli ingressi dei due comparatori.

I connettori WAGO 221-2411 sono i quick-connect a leva, dove si infila
il filo del binario senza saldare.

## Il riferimento di tensione del RailCom

In una zona libera della breadboard c'è un partitore di tensione che
produce un riferimento basso, attorno ai 22 millivolt, da usare come
soglia per i due comparatori:

```
+3,3 V ---[ R1 150 kΩ ]---+---[ R2 1 kΩ ]--- GND
                          |
                          +--- VREF_RAILCOM   (≈ 22 mV)
                          |
                          +---[ C1 100 nF ]--- GND
```

Il valore di 22 millivolt esce dal calcolo 3,3 volt moltiplicato 1
kiloohm diviso 151 kiloohm. Il condensatore da 100 nanofarad fra il
nodo VREF e massa tiene il riferimento stabile contro il rumore
raccolto dai cavetti.

## I due comparatori TLV3501

Due breakout PA0002 con su saldato un TLV3501AIDR ciascuno. Sono montati
su zoccolino sulla breadboard, vicino al nodo di sense che devono
leggere.

```
Comparatore A (legge SENSE_A):
   pin 1 (NC)      -- non collegato
   pin 2 (IN-)     <-- SENSE_A
   pin 3 (IN+)     <-- VREF_RAILCOM
   pin 4 (V-)      -- GND
   pin 5 (NC)      -- non collegato
   pin 6 (OUT)     --> nodo OUT_A
   pin 7 (V+)      -- +3,3 V
   pin 8 (SHDN)    -- GND   (obbligatorio basso per accendere il chip)

   Condensatore C_A 100 nF fra pin 7 e pin 4, vicino al chip

Comparatore B (legge SENSE_B):
   pin 1 (NC)      -- non collegato
   pin 2 (IN-)     <-- SENSE_B
   pin 3 (IN+)     <-- VREF_RAILCOM
   pin 4 (V-)      -- GND
   pin 5 (NC)      -- non collegato
   pin 6 (OUT)     --> nodo OUT_B
   pin 7 (V+)      -- +3,3 V
   pin 8 (SHDN)    -- GND

   Condensatore C_B 100 nF fra pin 7 e pin 4
```

Non c'è la resistenza da 1 megaohm di isteresi che era nel disegno
originale: è stata tolta nella sessione del 27 maggio e non è stata
rimessa. Non c'è neanche il filtro RC da 1 kiloohm e 100 nanofarad sul
ramo dei due ingressi negativi che era stato discusso in una versione
intermedia. Gli ingressi negativi sono collegati direttamente al nodo
di sense.

## La combinazione delle due uscite (or cablato con diodi Schottky)

I due nodi OUT_A e OUT_B non possono essere collegati direttamente
perché le uscite dei TLV3501 sono di tipo push-pull. Si uniscono
attraverso due diodi Schottky 1N5819 e una resistenza di tiraggio:

```
+3,3 V
  |
[ R_PU 4,7 kΩ ]   resistenza di pull-up verso il positivo
  |
  +---- RAILCOM_RAW   (nodo combinato, verso il 74HC14)
  |
  +-->|-- OUT_A    diodo D_A 1N5819 con anodo su RAILCOM_RAW
  |   D_A          e catodo su OUT_A
  |
  +-->|-- OUT_B    diodo D_B 1N5819 con anodo su RAILCOM_RAW
      D_B          e catodo su OUT_B
```

A riposo le due uscite del TLV3501 sono alte (vicine a 3,3 volt), i
diodi sono interdetti, e il nodo RAILCOM_RAW sta alto a 3,3 volt grazie
alla resistenza da 4,7 kiloohm. Quando uno qualunque dei due
comparatori porta la sua uscita bassa, il rispettivo diodo conduce e
tira giù il nodo RAILCOM_RAW. Cioè: "se uno dei due tira giù,
RAILCOM_RAW va basso".

## Il 74HC14 verso il GPIO5 del RailCom

Sulla breadboard c'è un 74HC14 a quattordici piedini, un esavinvertitore
Schmitt trigger. Si usa un solo gate dei sei disponibili (il gate 6)
per il percorso RailCom. C'è poi un altro gate del 74HC14 usato per il
rilevatore di corto storico sul GPIO3, anche se nella configurazione
con il modulo Pololu DRV8874 il rilevamento di corto vero e proprio è
fatto in altro modo (via IPROPI e nFAULT del chip, segnali che però
non abbiamo ancora cablato). I restanti gate sono inutilizzati.

```
74HC14

Alimentazione:
   pin 14 (Vcc) --- +3,3 V
   pin 7  (GND) --- GND

Gate 6 (usato per il RailCom):
   pin 13 (input)  <-- RAILCOM_RAW
   pin 12 (output) --> GPIO5 dell'ESP32

Gate storico dedicato al rilevatore di corto (rimasto cablato anche se
non attivo nel firmware corrente):
   pin X (input)  <-- segnale di sense del corto (storico, dai tempi BTS)
   pin Y (output) --> GPIO3 dell'ESP32

Gate non usati:
   ingressi (pin 1, 3, 5, 9, 11 a seconda di quali sono liberi) --- GND
   uscite corrispondenti --- lasciate in aria
```

Il 74HC14 fa due cose insieme sul segnale RailCom: inverte la polarità
(quando RAILCOM_RAW va basso, GPIO5 sale; e viceversa) e ripulisce con
la sua soglia di scatto a triggers di Schmitt il segnale eventualmente
rumoroso che arriva dalla combinazione dei due comparatori.

## Riassunto del percorso del segnale RailCom dall'inizio alla fine

Dalla locomotiva al firmware il segnale fa questo viaggio:

```
loco RailCom genera ±30 mA fra rotaia A e rotaia B
           |
           v
le 2,2 Ω in serie producono ±66 mV sui nodi SENSE_A / SENSE_B
           |
           v
TLV3501 confronta SENSE_x con i 22 mV di VREF
           |
           v
quando il segnale supera la soglia, OUT_x va basso
           |
           v
il diodo Schottky D_x tira giù il nodo RAILCOM_RAW
           |
           v
il 74HC14 inverte e ripulisce, e produce un fronte alto su GPIO5
           |
           v
il firmware sull'ESP32 campiona GPIO5 dentro la finestra di cutout
e tenta di decodificare il pacchetto UART RailCom
```

## Stato funzionale alla data di oggi

Il DCC base funziona: l'ESP32 genera l'onda DCC sul GPIO2, il DRV8874
la pilota sui binari a 15 volt, le locomotive rispondono ai comandi. Il
cutout viene generato correttamente sul GPIO4 e visualizzato allo scope
come finestra di 460 microsecondi durante la quale il segnale DCC
scompare.

Il RailCom invece non riceve dati. Le sessioni di banco del 27 maggio
hanno mostrato che, con la topologia cablata, il firmware conta migliaia
di finestre di cutout aperte senza ricevere nemmeno un byte dalla
LokSound 5. La diagnosi e le strade per risolvere sono in
[`docs/references/railcom-drv88xx/README.md`](../references/railcom-drv88xx/README.md).

## Componenti utilizzati

Modulo Waveshare ESP32-C6 Mini, modulo Pololu DRV8874 carrier #4035,
display OLED SSD1306 da 1,3 pollici, due TLV3501AIDR su breakout
PA0002, un 74HC14 in DIP-14, due diodi Schottky 1N5819, due LED da 3
millimetri (un verde e un rosso) con resistenze 330 ohm, due pulsanti
tactile, due resistenze di potenza 2,2 ohm da 1 watt, resistenze del
kit 150 kΩ e 1 kΩ per il riferimento e 4,7 kΩ per il pull-up, due
connettori WAGO 221-2411 per l'uscita al binario, condensatori 100 nF
ceramici per disaccoppiamento e filtro. Tutto presente in
[`docs/hardware/components-inventory.md`](components-inventory.md).
