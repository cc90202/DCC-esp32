# Flusso RMT, ISR, DCC e RailCom

Questo documento descrive come collaborano il task asincrono DCC, la periferica RMT e gli interrupt (ISR) nella generazione del segnale DCC e nella gestione delle finestre RailCom.

## I tre soggetti principali

Il flusso coinvolge tre soggetti distinti:

1. **Task asincrono DCC**: prepara i pacchetti e li converte in `PulseCode`.
2. **Periferica RMT**: legge le `PulseCode` dalla propria RAM e genera il segnale elettrico sul GPIO2.
3. **ISR** (*Interrupt Service Routine*): viene eseguita dalla CPU quando la RMT segnala la fine di un ciclo e aggiorna il buffer nel momento corretto.

Il task asincrono non genera direttamente gli impulsi sul GPIO2 e l'ISR non codifica da zero i pacchetti DCC.

## 1. Il task asincrono prepara il pacchetto

Il task `dcc_engine_task` si trova in [`src/dcc/engine.rs`](../../src/dcc/engine.rs).

Aspetta un nuovo comando DCC:

```rust
let frame = receiver.receive().await;
```

Quando riceve il comando:

1. codifica il pacchetto DCC;
2. lo converte in una sequenza di `PulseCode`;
3. calcola la durata del pacchetto;
4. decide se il pacchetto richiede un cutout RailCom;
5. pubblica il risultato con `rmt_driver::submit_packet(...)`.

La funzione `submit_packet` si trova in [`src/dcc/rmt_driver.rs`](../../src/dcc/rmt_driver.rs). Copia i dati in uno slot software e pubblica lo slot solo quando il pacchetto è completo.

Il lavoro più pesante—codifica, conversione e preparazione dei metadati—avviene fuori dall'interrupt.

## 2. Come è organizzata la RAM della RMT

La RMT possiede una RAM interna per la trasmissione. Il buffer contiene:

```text
[ preambolo fisso ][ dati variabili del pacchetto ][ end_marker ]
```

Il codice ISR non modifica la zona del preambolo; aggiorna soltanto la parte variabile.

Ogni `PulseCode` descrive due livelli consecutivi del segnale:

```text
livello 1 per durata 1
livello 2 per durata 2
```

La periferica RMT legge queste entry autonomamente. Per ogni `PulseCode` imposta il GPIO2 al primo livello, lo mantiene per la prima durata, imposta il secondo livello, lo mantiene per la seconda durata e passa alla `PulseCode` successiva.

La CPU, quindi, non deve eseguire un'istruzione per ogni transizione HIGH/LOW: è la RMT a produrre il segnale con il proprio timing hardware.

La trasmissione continua viene avviata in [`src/dcc/rmt_driver.rs`](../../src/dcc/rmt_driver.rs):

```rust
channel.transmit_continuously(
    idle_rmt,
    LoopMode::InfiniteWithInterrupt(1),
)
```

`InfiniteWithInterrupt(1)` significa che il buffer viene ripetuto continuamente e che viene generato un interrupt alla fine di ogni ciclo.

## 3. Come viene registrato l'handler dell'interrupt

L'ISR non viene chiamata dal codice con una normale istruzione Rust come
`rmt_interrupt()`. Viene chiamata dall'hardware quando la RMT segnala il proprio
interrupt.

Durante l'inizializzazione del driver, il codice associa la sorgente hardware
`Interrupt::RMT` alla funzione `rmt_interrupt`:

```rust
unsafe {
    interrupt::bind_interrupt(
        Interrupt::RMT,
        rmt_interrupt.handler(),
    )
};

interrupt::enable(Interrupt::RMT, Priority::Priority3)?;
```

`Interrupt::RMT` identifica la sorgente hardware. `rmt_interrupt.handler()`
fornisce all'HAL l'handler della funzione, cioè il riferimento necessario per
eseguirla come ISR. `bind_interrupt` registra l'associazione; `enable` abilita
la sorgente nel controller degli interrupt.

Il comportamento risultante è:

```text
RMT raggiunge la fine del buffer
        ↓
la RMT genera l'interrupt CH_TX_LOOP
        ↓
il controller identifica Interrupt::RMT
        ↓
il controller usa l'handler registrato
        ↓
la CPU esegue rmt_interrupt()
```

Il timer RailCom viene registrato attraverso l'astrazione `OneShotTimer`, in
`src/track_output.rs`:

```rust
let mut cutout_timer = OneShotTimer::new(timer0);

cutout_timer.set_interrupt_handler(
    InterruptHandler::new(
        cutout_timer_interrupt.handler().aligned_ptr(),
        Priority::Priority3,
    ),
);

cutout_timer.listen();
```

In questo caso `set_interrupt_handler` associa l'handler
`cutout_timer_interrupt` a quell'istanza del timer e `listen` abilita gli
eventi del timer.

Il timer non scatta subito: viene programmato dall'ISR RMT soltanto quando il
pacchetto corrente richiede un cutout RailCom. Alla scadenza del timer, la CPU
esegue `cutout_timer_interrupt()`.

Le due registrazioni hanno API diverse, ma il principio è identico: una
sorgente hardware viene associata all'indirizzo di una funzione. Quando la
sorgente genera l'interrupt, il controller trasferisce l'esecuzione a quella
funzione.

## 4. Cosa succede al confine tra due pacchetti

Supponiamo che la RMT stia trasmettendo il pacchetto A:

```text
[ preambolo ][ dati A ][ fine ]
```

Quando raggiunge `fine`:

1. la RMT genera l'interrupt `CH_TX_LOOP`;
2. il suo indice interno torna all'inizio del buffer;
3. la RMT ricomincia immediatamente a leggere il preambolo;
4. la CPU entra nella funzione `rmt_interrupt()`.

La funzione è in [`src/dcc/rmt_driver.rs`](../../src/dcc/rmt_driver.rs):

```rust
#[handler(priority = Priority::Priority3)]
#[ram]
fn rmt_interrupt() {
```

L'ISR e la RMT lavorano contemporaneamente:

```text
RMT hardware:  legge il preambolo del prossimo ciclo
CPU / ISR:     copia i dati del prossimo pacchetto nella RAM RMT
```

L'ISR esegue:

```rust
write_data_to_rmt_ram(data, cutout_allowed);
```

Questa funzione scrive soltanto la parte variabile del buffer, non il preambolo. Mentre la RMT sta leggendo le prime entry, l'ISR può aggiornare le entry successive:

```text
RMT legge:   [ preambolo ][              ]
ISR scrive:  [ preambolo ][ dati B       ]
```

Quando la RMT finisce il preambolo e arriva alla parte dati, trova già il pacchetto B. Non viene generato un secondo interrupt alla fine del preambolo: l'unico interrupt è quello al termine del ciclo precedente.

La correttezza dipende quindi da un vincolo temporale: l'ISR deve completare la scrittura prima che la RMT raggiunga la prima entry della parte variabile.

## 5. Perché esistono gli slot software

Il task e l'ISR non scrivono nello stesso slot contemporaneamente. Il driver usa due slot software (A e B):

```text
ISR sta consumando lo slot A
task prepara il prossimo pacchetto nello slot B
```

Quando il task ha finito, pubblica lo slot B con uno store atomico. L'ISR lo leggerà al successivo confine di ciclo. Dopo averlo copiato nella RAM RMT, l'ISR marca lo slot come consumato.

Il task può attendere questa conferma con `wait_for_packet_consumed()`. Questa è la sincronizzazione tra il produttore (task DCC) e il consumatore (ISR).

## 6. Il ruolo dell'ISR RMT in RailCom

Il task DCC decide in anticipo se un pacchetto deve avere un cutout RailCom e salva questa informazione nei metadati del pacchetto.

Quando l'ISR RMT raggiunge il confine del pacchetto, controlla:

```rust
let cutout_requested =
    !matches!(pkt.meta.cutout, CutoutMode::None);
```

Se il cutout è richiesto, chiama:

```rust
request_cutout_from_isr(...)
```

in [`src/track_output.rs`](../../src/track_output.rs).

L'ISR RMT non decodifica la risposta RailCom. Registra il timestamp del confine, salva i metadati e programma la sequenza temporale del cutout.

## 7. Il timer ISR gestisce la finestra RailCom

Il timer attiva il secondo interrupt:

```rust
fn cutout_timer_interrupt() {
```

Questo ISR:

1. attende la fine effettiva del pacchetto DCC;
2. attiva il cutout tramite GPIO4;
3. apre la finestra RailCom Channel 1;
4. chiude Channel 1 e apre Channel 2;
5. chiude Channel 2;
6. disattiva il cutout e riporta il sistema allo stato normale.

La sequenza fisica è:

```text
pacchetto DCC
    ↓
breve silenzio sui binari (cutout)
    ↓
il decoder trasmette RailCom
    ↓
UART riceve i livelli
    ↓
task RailCom interpreta i byte
```

RailCom non è generato dalla RMT. La RMT genera DCC; il timer ISR crea il silenzio temporizzato durante il quale il decoder può rispondere.

## 8. Perché il lavoro è diviso in task e ISR

Il task asincrono può svolgere operazioni relativamente lunghe e può usare `.await` senza bloccare gli altri task. L'ISR, invece, deve essere molto breve: non deve aspettare canali o timer asincroni, codificare pacchetti complessi, allocare memoria o eseguire logica non necessaria.

La divisione è quindi:

```text
task asincrono DCC → prepara i dati
ISR RMT             → trasferisce i dati al confine corretto
RMT                 → genera gli impulsi sul GPIO2
timer ISR           → apre e chiude il cutout RailCom
UART + task RailCom → ricevono e interpretano la risposta
```

L'attributo `#[ram]` sulle ISR colloca il loro codice in RAM, evitando che latenze della flash o della cache—possibili durante l'attività Wi-Fi—violino le finestre temporali DCC/RailCom.
