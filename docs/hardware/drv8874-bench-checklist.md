# DRV8874 (Pololu #4035) — Checklist di assemblaggio e verifica

Ultimo aggiornamento: 2026-05-25

Sostituisce il modulo BTS7960 nel ruolo di driver del binario. Pensato per una breadboard di prova con ESP32-C6 Mini collegato via USB, locomotiva H0 reale sui binari, alimentatore da banco esterno a 15 V. Verificato funzionante a banco in data 2026-05-25 con locomotiva LokSound a indirizzo corto 3.

Limite di corrente del modulo: circa 2.1 A continui, 3.5 A di picco con protezione interna. È adatto a un plastico di prova con 2-4 locomotive H0 contemporanee; per esigenze maggiori serve un driver diverso (vedi nota finale).

Riferimenti: pagina prodotto Pololu <https://www.pololu.com/product/4035>, datasheet TI DRV8874, `src/boot.rs` (autoritativo per i GPIO), `src/short_detector.rs` (logica del rilevatore di corti, con il `RECOVERY_SETTLE_MS` introdotto per questo circuito).

## Componenti necessari

Sulla breadboard:

- [ ] ESP32-C6 Mini Waveshare a cavallo del canale, USB raggiungibile
- [ ] Modulo Pololu DRV8874 Single Brushed DC Motor Driver Carrier (codice prodotto 4035) a cavallo del canale
- [ ] Resistenza 4.7 kΩ (codice colore: giallo, viola, nero, marrone, marrone su 5 anelli — tolleranza 1%) per il filtro contro il rumore sul piedino del segnale di guasto
- [ ] Condensatore ceramico 100 nF (stampigliatura "104") per il filtro contro il rumore sul piedino del segnale di guasto
- [ ] Per i LED di stato (opzionali ma consigliati): due LED da 5 mm (uno verde, uno rosso) + due resistenze da 330 Ω
- [ ] Per i pulsanti di stop / resume (opzionali ma consigliati): due pulsanti tattili a 4 zampe
- [ ] Per il display di stato (opzionale): un OLED SSD1306 a 4 pin con comunicazione I²C

Esterno alla breadboard:

- [ ] Alimentatore da banco regolato a 15 V (SPENTO durante l'assemblaggio)
- [ ] Multimetro digitale con misura di resistenza e di tensione in corrente continua
- [ ] Oscilloscopio per la verifica finale del segnale digitale sulle uscite
- [ ] Locomotiva H0 funzionante per il test end-to-end

## Step 0 — Verifiche a circuito spento prima di alimentare (OBBLIGATORIO)

Questo step non si salta mai. È stato introdotto dopo aver bruciato un ESP32 con un cablaggio precedente in cui il piedino dei 3.3 V dell'ESP32 era stato accidentalmente collegato all'ingresso di alimentazione potenza VIN del modulo Pololu, causando un cortocircuito diretto fra la rail logica e i 15 V dell'alimentatore.

Prima di dare tensione a qualsiasi cosa, con l'alimentatore da banco spento e il cavo USB scollegato dall'ESP32, fai queste misure con il multimetro in modalità resistenza:

- [ ] Resistenza fra il piedino VIN del modulo e GND: deve essere alta (almeno qualche centinaio di kΩ, tipicamente sui 290 kΩ). Se leggi 0 Ω c'è un corto.
- [ ] Resistenza fra il piedino VIN del modulo e il piedino 3V3 dell'ESP32: deve essere alta (decine o centinaia di kΩ). Se leggi 0 Ω hai un filo dei 3.3 V finito su VIN: STOP, da risolvere prima di alimentare.
- [ ] Resistenza fra il piedino 3V3 dell'ESP32 e GND: deve essere alta. Se leggi 0 Ω hai un corto sulla rail logica.
- [ ] Continuità (resistenza vicino a zero) fra il piedino GND del modulo e il piedino GND dell'ESP32: devono essere uniti.

Solo dopo aver superato queste quattro misure, si può procedere all'accensione.

## A — Alimentazioni e massa

Tutta la parte di potenza è semplice: una rail per i 15 V dell'alimentatore, una per la massa comune.

- [ ] Filo dal morsetto rosso dell'alimentatore (positivo, 15 V) al piedino VIN del modulo Pololu
- [ ] Filo dal morsetto nero dell'alimentatore (negativo) alla rail di massa della breadboard
- [ ] Filo dal piedino GND del modulo Pololu alla stessa rail di massa
- [ ] Filo da uno dei piedini GND dell'ESP32-C6 alla stessa rail di massa

Verifica finale: la rail di massa è elettricamente lo stesso punto del negativo dell'alimentatore, del GND del modulo, e del GND dell'ESP32. Tutte e tre devono "vedere" la stessa massa, altrimenti i segnali logici non hanno un riferimento comune.

Il piedino VM del modulo Pololu è un punto di accesso interno alla protezione contro l'inversione di polarità — non è un ingresso e non va collegato a nulla.

## B — Pin logici del modulo Pololu

Sulla scheda Pololu, oltre ai piedini di potenza VIN/VM/GND/OUT1/OUT2, ci sono diversi piedini logici. Alcuni vanno cablati, altri devono essere lasciati liberi perché la scheda ha già le resistenze di richiamo a bordo che li portano al valore corretto di default.

Da cablare:

- [ ] Filo dal piedino PH/IN2 del modulo al piedino GPIO 2 dell'ESP32. Questo porta il segnale digitale per i decoder dal generatore RMT interno dell'ESP32 fino al ponte H.
- [ ] Filo dal piedino EN/IN1 del modulo alla rail dei 3.3 V (alimentazione logica dell'ESP32). Tenuto sempre alto, abilita le uscite del ponte H.
- [ ] Filo dal piedino SLEEP del modulo al piedino GPIO 18 dell'ESP32. Funziona da interruttore generale: quando il firmware lo alza a 3.3 V il chip si sveglia, quando lo abbassa a 0 V le uscite verso il binario vanno in alta impedenza (binario "spento" senza dissipazione).
- [ ] Filo dal piedino PMODE del modulo alla rail di massa. Seleziona la modalità di controllo "fase ed enable" (la lettura di questo piedino avviene quando SLEEP transita da basso ad alto, quindi questo cablaggio deve essere già in posizione prima della prima accensione).
- [ ] Filo dal piedino FAULT del modulo al piedino GPIO 3 dell'ESP32, con il filtro descritto sotto.

Da lasciare liberi (le resistenze a bordo della scheda Pololu li portano già al valore corretto):

- VREF: pull-up interno verso SLEEP via 10 kΩ → soglia di limitazione di corrente al massimo
- IMODE: pull-down interno verso GND via 20 kΩ → modalità "current chopping con auto-retry" (NON è la modalità OCP latched, è già una modalità con regolazione attiva)
- CS: pull-down interno verso GND via 2.49 kΩ → uscita analogica della corrente, non utilizzata in questa configurazione

## C — Filtro contro il rumore sul piedino del segnale di guasto

Il piedino FAULT del modulo è un'uscita open-drain (collettore aperto) attiva bassa. La resistenza di richiamo interna all'ESP32 (circa 45 kΩ) è troppo debole per resistere al rumore capacitivo accoppiato dai fili delle uscite del binario, che oscillano fra 0 e 15 V decine di migliaia di volte al secondo. Senza un filtro esterno, il firmware vede falsi corti casuali ogni volta che il binario è attivo.

- [ ] Resistenza 4.7 kΩ collegata fra il piedino GPIO 3 dell'ESP32 e la rail dei 3.3 V (pull-up esterno, molto più "forte" di quello interno)
- [ ] Condensatore ceramico 100 nF collegato fra il piedino GPIO 3 dell'ESP32 e la rail di massa (filtra gli impulsi capacitivi veloci)

Entrambi i componenti convergono sullo stesso nodo elettrico del piedino GPIO 3 (stessa fila orizzontale della breadboard del cavetto FAULT). La resistenza tira verso l'alto, il condensatore tira verso massa.

## D — Periferiche di stato (opzionali)

Se vuoi avere il display, i LED di stato e i pulsanti di stop / resume come sul circuito BTS7960, il cablaggio è identico al BTS7960:

- [ ] LED verde: gambina lunga (anodo) al piedino GPIO 14 dell'ESP32; gambina corta (catodo) ad un piede di una resistenza da 330 Ω; altro piede della resistenza alla rail di massa
- [ ] LED rosso: gambina lunga al piedino GPIO 15 dell'ESP32; gambina corta ad un piede di una resistenza da 330 Ω; altro piede della resistenza alla rail di massa
- [ ] Pulsante rosso (stop di emergenza): un terminale al piedino GPIO 22 dell'ESP32; terminale opposto alla rail di massa. Il firmware abilita il pull-up interno.
- [ ] Pulsante verde (resume / sblocco fault): un terminale al piedino GPIO 21 dell'ESP32; terminale opposto alla rail di massa. Idem.
- [ ] Display OLED I²C: piedino VCC alla rail dei 3.3 V; piedino GND alla rail di massa; piedino SDA al piedino GPIO 19 dell'ESP32; piedino SCL al piedino GPIO 20 dell'ESP32.

## E — Accensione e verifica passo a passo

L'ordine di accensione è importante: prima l'ESP32 via USB, e solo dopo, con i fili logici già verificati, l'alimentatore da banco a 15 V.

- [ ] Verificate tutte le caselle dello Step 0 (multimetro a circuito spento)
- [ ] Collegare il cavo USB dell'ESP32 a un computer o caricatore. Verificare nel monitor seriale che il firmware si avvii senza errori.
- [ ] Misurare con il multimetro che la tensione fra il piedino VIN del modulo e la massa sia vicina a 0 V (l'alimentatore è ancora spento). Si possono leggere fino a 2-3 V di "back-feeding" attraverso i diodi di protezione interni del chip: è normale e innocuo, basta che non sia 3.3 V (che indicherebbe un filo logico erroneamente collegato a VIN).
- [ ] Accendere l'alimentatore da banco a 15 V. Sul display dovrebbe leggersi una corrente intorno a 0.00 A o pochi milliampere (il chip in modalità dormiente consuma pochissimo, sotto la risoluzione del display).
- [ ] Toccare delicatamente con un dito il chip nero del modulo (la parte sopra è marcata "DRV8874"): deve essere a temperatura ambiente, non tiepido né caldo. Se scotta, spegnere subito.
- [ ] Attendere il completamento del boot dell'ESP32 (il firmware riporta in log "Short detector active after 5000ms blanking" dopo i cinque secondi di silenzio iniziale del rilevatore di corti)
- [ ] Verificare con l'oscilloscopio sul piedino OUT1 del modulo che ci sia il segnale digitale per i decoder: base dei tempi 20 µs / divisione, scala verticale 5 V / divisione, trigger su fronte di salita con livello intorno a 7 V. Si deve vedere un'onda quadra che oscilla fra 0 e 15 V, con i bit "1" larghi circa 58 µs e i bit "0" larghi 100 µs o più.
- [ ] Collegare OUT1 e OUT2 alle due rotaie (non importa l'ordine). Posizionare la locomotiva sui binari.
- [ ] La locomotiva deve rispondere ai comandi (se il firmware sta mandando comandi automatici di velocità deve muoversi da sola, altrimenti si comanda via app Z21 sul telefono).
- [ ] Nel log NON deve apparire il messaggio `Short circuit detected on GPIO3`. Se appare, il filtro non è sufficiente o c'è un altro problema di rumore.
- [ ] Premere il pulsante rosso (stop di emergenza) e verificare che nel log appaia `event=StopPressed` seguito da `state Normal -> EstopLatched`, e che la locomotiva si fermi.
- [ ] Tenere premuto a lungo il pulsante verde (resume) e verificare che nel log appaia `event=ResumeLongPressed` seguito da `state EstopLatched -> Normal, track_enabled=true`, e che la locomotiva riprenda. **NON** deve apparire entro pochi millisecondi un falso `Short circuit detected`: se appare, vuol dire che il `RECOVERY_SETTLE_MS` nel rilevatore di corti è troppo basso oppure il filtro contro il rumore non è sufficiente.

## Note sul firmware

Per far funzionare correttamente questo circuito è stato introdotto in `src/short_detector.rs` un ritardo di assestamento di 100 millisecondi (costante `RECOVERY_SETTLE_MS`) fra il cambio di stato del fault manager a "Normal" e il primo controllo del livello del piedino GPIO 3. Senza questo ritardo, le transitorie del piedino FAULT durante il risveglio del chip Pololu e/o l'impulso di corrente di spunto del condensatore di filtro del decoder vengono interpretate come corto persistente, causando un loop di fault-recover infinito.

La logica di debounce sul fronte di discesa rilevato dall'interrupt è rimasta invariata: i corti veri (piedino FAULT che resta basso oltre la soglia di debounce) continuano a essere catturati correttamente.

## Limiti noti e direzioni future

Il modulo Pololu DRV8874 ha un limite di corrente continua di circa 2.1 A. Questo significa che il circuito è adatto per un plastico di prova con 2-4 locomotive H0 contemporanee (ognuna assorbe tipicamente 0.3-0.5 A in movimento con suoni e luci accesi). Per un plastico con più di una manciata di locomotive il chip diventa il collo di bottiglia: in quel caso si ritorna sul BTS7960 (capace di decine di ampere) oppure si valutano altri driver come il DRV8256 o l'IBT-2.

Non è ancora cablato il segnale di cutout per RailCom (sul circuito BTS7960 è gestito dal piedino GPIO 4 dell'ESP32 attraverso un MOSFET esterno). Con il DRV8874 **non** bisogna usare `SLEEP` come interruttore veloce di cutout: il datasheet indica tempi di sleep/wake dell'ordine di 1 ms, troppo lenti per la finestra RailCom di circa 460 µs. `SLEEP` resta l'interruttore generale per boot, fault ed emergenza; il cutout RailCom deve essere fatto con logica veloce e/o MOSFET esterni pilotati da GPIO4. Vedi `docs/hardware/drv8874-railcom-tlv3501-checklist.md` per il cablaggio proposto.
