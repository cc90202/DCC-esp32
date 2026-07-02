# Note sul circuito RailCom con i chip DRV8874 e DRV8873

Ultimo aggiornamento: 2026-05-28.

## Rettifica importante del 2026-05-28

La conclusione del 2026-05-27, secondo cui la strada giusta fosse lasciare il
DRV8874 in alta impedenza e leggere direttamente una piccola tensione fra le
rotaie, era incompleta e porta fuori strada per il nostro caso.

Il RailCom non richiede solo che il booster smetta di pilotare il DCC. Durante
il cutout le due rotaie devono essere scollegate dalla sorgente di alimentazione
e cortocircuitate fra loro a bassa impedenza, con il detector in serie alla
sorgente di corrente del decoder. RCN-217 descrive il cutout device proprio
cosi': le linee di track vengono separate dall'alimentazione e cortocircuitate;
la trasmissione avviene come anello di corrente fornito dal buffer interno del
decoder. Il drop del cutout device deve restare sotto 10 mV a 34 mA, mentre il
drop del detector deve restare sotto 200 mV a 34 mA.

Conseguenza pratica:

- **Hi-Z da solo non basta.** Se il DRV8874 viene messo in coast/alta
  impedenza, serve comunque un terminatore esterno comandato nello stesso
  cutout, altrimenti non esiste un percorso fisico affidabile per la corrente
  RailCom.
- **Brake-low del DRV8874 e' una strada valida.** In PH/EN mode, con
  `nSLEEP=1` ed `EN=0`, il DRV8874 porta `OUT1=L` e `OUT2=L`. Questo crea il
  ritorno a bassa impedenza senza MOSFET di potenza esterni.
- **Il detector deve stare nel percorso della corrente.** Se usiamo il
  brake-low interno, lo shunt va fisicamente in serie a uno o entrambi i binari,
  subito fuori da `OUT1`/`OUT2`, prima che la corrente venga assorbita dai
  low-side FET del DRV.
- **Il TLV3501 non va collegato nudo al lato rotaia.** Uno shunt in serie al
  binario vede il normale DCC fuori cutout; quindi il front-end del comparatore
  deve essere protetto, isolato durante il DCC, o sostituito con un circuito di
  current-sense adatto al common-mode del binario. Il semplice collegamento
  `SENSE_A -> TLV3501 IN-` dei vecchi appunti e' utile solo nei prototipi con
  MOSFET esterni che isolano il sense node durante il DCC normale.

Decisione operativa per il banco DRV8874:

1. Tenere `PMODE=GND` e usare PH/EN mode.
2. Tenere `GPIO18/nSLEEP` alto durante il cutout; non usarlo per RailCom.
3. Usare `GPIO4 -> EN/IN1`, idle HIGH e cutout LOW, per ottenere brake-low.
4. Non usare `IPROPI` come detector RailCom: in brake misura solo parte del
   comportamento dei low-side FET e non sostituisce il detector globale.
5. Scegliere una delle due implementazioni detector:
   - shunt in serie al binario con front-end bidirezionale protetto per il DCC;
   - terminatore esterno attivo solo durante cutout, con il sense node tenuto
     vicino a GND come nei prototipi MOSFET precedenti.

Finche' il punto 5 non e' risolto sullo schema, non ha senso cercare altri fix
firmware: il firmware attuale gia' comanda `GPIO4` come cutout veloce e lascia
`GPIO18/nSLEEP` invariato durante la finestra.

### Decisione successiva: opzione A

Scelta operativa successiva: usare un terminatore/detector esterno attivo solo
durante il cutout, documentato in
[`../../hardware/railcom-option-a-external-terminator.md`](../../hardware/railcom-option-a-external-terminator.md).

Questa scelta cambia la parte hardware rispetto alla decisione provvisoria
sopra:

- `PMODE` passa a 3.3 V, quindi il DRV8874 lavora in PWM mode.
- `GPIO2` resta il segnale DCC RMT.
- `GPIO4` resta il segnale temporizzato dal firmware, ma diventa `DCC_RUN`:
  HIGH fuori cutout, LOW dentro cutout.
- logica esterna genera `IN1 = DCC_RUN & DCC` e `IN2 = DCC_RUN & !DCC`.
- durante il cutout `IN1=IN2=0`, quindi il DRV8874 va in alta impedenza e il
  terminatore esterno puo' diventare l'unico percorso RailCom misurato.

Il firmware non deve cambiare per il primo rebuild: la polarita' e il timing di
`GPIO4` sono gia' quelli richiesti. Cambiano cablaggio, PMODE e logica esterna.

Le sezioni storiche sotto restano come diario di lavoro, ma vanno lette alla
luce di questa rettifica.

Questo è un quaderno di lavoro. Raccoglie quello che abbiamo capito sul
funzionamento del rilevatore RailCom quando il ponte di potenza che pilota le
rotaie è uno dei chip della famiglia DRV8874 e DRV8873. È stato scritto dopo
una sessione di banco molto lunga in cui il nostro rilevatore non riusciva a
leggere nemmeno un byte dalla locomotiva, e dopo aver studiato sul serio i
datasheet ufficiali e gli schemi di altri progetti pubblici che fanno la
stessa cosa.

Le parole tecniche che girano normalmente in inglese le usiamo solo dopo
averle spiegate a parole nostre. Quindi prima la sostanza, poi al limite il
nome che gli danno i datasheet.

## Le due modalità del DRV che ci interessano

Il DRV è un chip che pilota le rotaie. Per pilotarle ha quattro transistor
interni: due sopra che collegano la rotaia al positivo dell'alimentazione, e
due sotto che collegano la rotaia a massa. Combinandoli, può fare diverse
cose.

Le due cose che a noi interessano per il RailCom sono queste.

La prima è quella che abbiamo usato finora ed è chiamata nei datasheet
"brake-low". Tradotto: entrambe le rotaie vengono **tirate giù a massa**.
Cioè vengono inchiodate a zero volt attraverso i due transistor interni
sotto. È un cortocircuito controllato verso massa, fatto apposta, con
un'impedenza piccolissima. Il DRV diventa, per le rotaie, un grosso cavo
verso terra.

La seconda è quella che ci serve per leggere il RailCom ed è chiamata "alta
impedenza" oppure "coast". Tradotto: il chip **stacca le rotaie**. Tutti e
quattro i transistor interni vengono aperti, e le rotaie restano collegate
al chip solo attraverso fili che non portano corrente. È come se il DRV per
quel breve momento sparisse e lasciasse le rotaie libere di flottare in
aria.

Queste due modalità non sono entrambe disponibili in qualsiasi configurazione
del chip. Dipende da come è impostato un piedino del DRV che si chiama PMODE,
che decide come il chip interpreta i suoi ingressi di controllo.

Con PMODE collegato a massa il DRV lavora in modalità "fase più
abilitazione", e in questa modalità la combinazione che noi usiamo per il
cutout produce "rotaie inchiodate a zero". Non c'è modo, in questa
configurazione, di ottenere "rotaie staccate" abbastanza velocemente.
L'unico modo sarebbe spegnere il chip con il piedino di sleep, ma il
datasheet dice chiaramente che il chip impiega un millisecondo per spegnersi
e un millisecondo per riaccendersi. Il cutout del RailCom dura 460
microsecondi, quindi questa strada è morta in partenza.

Con PMODE collegato al positivo di alimentazione (3,3 volt), il DRV lavora
invece in modalità diversa: i suoi due ingressi diventano indipendenti e
controllano separatamente le due metà del ponte. In questa modalità la
combinazione "entrambi gli ingressi a zero" produce esattamente quello che
serve: rotaie staccate, in alta impedenza vera. Il datasheet del DRV8874 a
pagina 11 ha la tabella che lo dice chiaramente, e a pagina 6 dice anche che
la transizione fra una situazione e l'altra avviene in circa 550
nanosecondi, cioè in pratica istantanea rispetto ai 460 microsecondi del
cutout.

Il DRV8873, che è il fratello più potente del nostro DRV8874, ha tutto questo
e in più ha un piedino dedicato chiamato DISABLE: quando viene alzato, il
chip stacca le rotaie immediatamente, indipendentemente da come stanno gli
altri ingressi. Il nostro DRV8874 questo piedino non ce l'ha, quindi per noi
l'unica strada per ottenere le rotaie staccate dall'interno del chip è
cambiare la modalità di funzionamento e pilotare diversamente i suoi due
ingressi.

## Perché con le rotaie inchiodate a zero il RailCom non si legge

Questa è la parte che mi ero spiegato male nei primi tentativi e che è
importante capire bene.

La locomotiva, durante la sua finestra di trasmissione, spinge dentro a una
rotaia circa 30 milliampere di corrente e tira la stessa quantità fuori
dall'altra. Quando il DRV è nella modalità di freno, le sue uscite sono
collegate a massa con bassa impedenza. La corrente del decoder, in queste
condizioni, ha un giro chiuso ben preciso: entra nella rotaia A, attraversa
la resistenza di sense da 2,2 ohm sul lato A, raggiunge la massa attraverso
il transistor interno del DRV, risale dall'altra parte attraverso il
transistor interno opposto, attraversa la resistenza di sense del lato B, ed
esce dalla rotaia B per chiudere il giro dentro al decoder.

Su ciascuna delle due resistenze di sense, quei 30 milliampere producono una
caduta di tensione di circa 66 millivolt. Sulla carta sono 66 millivolt
perfettamente visibili dal comparatore TLV3501 che li deve leggere, con la
sua soglia di riferimento a 22 millivolt.

Quindi il problema **non è** che la corrente del decoder si perda per
strada. Nella modalità di freno la corrente arriva alle resistenze di sense
e i 66 millivolt ci sono.

Il problema vero è un altro, e riguarda il comparatore.

Il TLV3501 è un chip piccolo alimentato a 3,3 volt. I suoi due piedini di
ingresso accettano per specifica solo tensioni comprese fra zero e 3,3 volt.
Se gli arriva qualcosa di molto più alto, tipo 15 volt, oppure qualcosa di
negativo, tipo meno 15 volt, lui non si rompe subito ma entra in tilt:
dentro al chip ci sono dei diodi di protezione che si attivano per
scaricare la sovratensione, e nel frattempo il comparatore smette
temporaneamente di confrontare correttamente le due tensioni in ingresso.

Adesso pensa a cosa vede il comparatore quando le rotaie funzionano da DCC
normale. La rotaia A oscilla fra più 15 volt e meno 15 volt, e il nodo che
chiamiamo SENSE_A oscilla con lei. Il comparatore in quei momenti è in
tilt cronico: i suoi diodi di protezione sono in conduzione, l'uscita è
bloccata, e di lavoro utile non ne sta facendo. Poi arriva il cutout di 460
microsecondi: la centrale spegne il DCC, il DRV in modalità di freno
inchioda le rotaie a zero, SENSE_A scende vicino a zero, e adesso il
comparatore deve, nel giro di pochissimi microsecondi, smettere di essere
stordito e tornare lucido in tempo per leggere i 66 millivolt minuscoli del
segnale del decoder. Questo passaggio da "stordito" a "lucido" non è
istantaneo, dipende da quanto a lungo è stato spinto fuori specifica, e
non è garantito che riesca a recuperare in tempo prima che la finestra del
decoder finisca.

C'è anche un secondo problema. I 66 millivolt del segnale RailCom oscillano
fra più 66 millivolt e meno 66 millivolt rispetto a zero, cioè la metà
inferiore del segnale è leggermente negativa rispetto a massa. Il
comparatore alimentato fra zero e 3,3 volt lavora bene solo quando i suoi
ingressi stanno chiaramente dentro a quel rettangolo. La parte negativa del
segnale RailCom rischia di andare persa anche per questo.

In sintesi: il "rotaie inchiodate a zero" rompe la lettura non perché la
corrente del decoder non scorra dove dovrebbe (passa eccome, e produce i 66
millivolt previsti sulle resistenze) ma perché stordisce il comparatore con
i quindici volt del DCC normale e gli lascia troppo poco tempo per tornare
lucido durante la pausa del cutout.

## Perché cambiare solo modalità del DRV non basta

C'è una cosa che bisogna capire bene prima di pensare che sia sufficiente
mettere il DRV in modalità di alta impedenza per risolvere tutto. Le nostre
resistenze di sense da 2,2 ohm funzionano come misuratori di corrente solo
se l'estremità della resistenza collegata al chip è a sua volta collegata a
qualcosa che permetta alla corrente di chiudere il giro. Tipicamente quel
qualcosa è la massa, fornita dai transistor interni del DRV nella modalità
di freno.

Quando il DRV passa in alta impedenza, succede esattamente il contrario:
l'estremità della resistenza dal lato del chip diventa scollegata da tutto.
Le uscite del DRV in alta impedenza sono in aria. La resistenza adesso ha
un solo capo attaccato a qualcosa di reale, la rotaia. L'altro capo
fluttua.

La corrente del decoder, in queste condizioni, non passa più dalle nostre
resistenze. Trova un giro più corto e più comodo, che è quello interno al
decoder stesso: dentro alla locomotiva c'è l'elettronica del trasmettitore
che fa loop fra le due rotaie passando per i suoi propri componenti.
Quindi i 30 milliampere circolano dentro al decoder, attraversano l'aria
fra una rotaia e l'altra solo virtualmente, e mai vengono a fare visita
alle nostre resistenze di sense. Sui nodi SENSE_A e SENSE_B non si vede più
niente, nemmeno con il comparatore lucido e ben configurato.

Questa è la ragione per cui i progetti seri che leggono il RailCom (tipo
quello di atanisoft) non usano resistenze in serie alle rotaie come noi.
Loro misurano direttamente la **tensione** che si genera fra le due rotaie
durante il cutout, con un comparatore a soglia molto bassa (atanisoft usa
circa 19 millivolt). Le rotaie, durante il cutout, sono fluttuanti e si
piazzano a una tensione di riposo determinata da loro stesse e dal decoder;
sopra a questa tensione di riposo, il decoder genera la sua piccola
modulazione, e quella viene letta direttamente dalla rotaia. Non serve far
passare la corrente attraverso resistenze esterne, anzi è proprio il
contrario di quello che si vuole.

Conseguenza pratica per noi: il circuito che abbiamo montato stasera sulla
breadboard, con le resistenze da 2,2 ohm in serie alle rotaie e i
comparatori TLV3501 che le leggono, **funziona per costruzione solo con il
chip in modalità di freno**. Quella modalità è quella che impedisce al
comparatore di leggere il segnale del decoder per via dei quindici volt del
DCC. Quindi siamo in una situazione in cui il sense ha senso solo nella
modalità del chip che impedisce la lettura, e il sense diventa inutile nella
modalità del chip che permetterebbe la lettura.

In altre parole: il pezzo che abbiamo costruito stasera, per come è
disegnato, non legge il RailCom in nessuna configurazione possibile del
DRV8874. Per leggerlo davvero servono cambiamenti più seri.

## Le strade percorribili per noi

A questo punto le strade praticabili sono due, e nessuna è banale come "due
fili spostati".

La prima strada è cambiare contemporaneamente la modalità del DRV e la
topologia del rilevatore. Il DRV viene messo in modalità di alta impedenza
(spostando il piedino PMODE dal collegamento a massa al collegamento ai
3,3 volt, e pilotando i suoi due ingressi in modo complementare). La rete
di lettura viene rifatta sul modello di atanisoft: si toglie l'idea di
misurare corrente attraverso resistenze in serie alle rotaie, e si mettono
invece i comparatori a leggere direttamente la tensione fra le rotaie, con
una soglia molto bassa, attorno ai 19-22 millivolt. Le resistenze da 2,2
ohm vengono rimosse e sostituite da un partitore di tensione che porta la
soglia al comparatore. Il circuito attuale con i TLV3501 si può forse
recuperare per quanto riguarda i comparatori in sé, ma tutto il contorno
(le resistenze in serie, la rete di sense) va ridisegnato.

La seconda strada è quella di mettere transistor esterni in serie alle
rotaie come fa atanisoft, e usare la stessa topologia di rilevatore della
prima strada. Cioè: aggiungere quattro transistor di potenza fra le uscite
del DRV e le rotaie, e rifare comunque la rete dei comparatori sul modello
di atanisoft. Questa strada ha tutti i costi della prima più quelli dei
transistor di potenza, e in cambio offre due vantaggi: i transistor esterni
sono un interruttore di sicurezza hardware indipendente dallo stato del
chip, e tagliano completamente il legame fra rotaie e diodi parassiti
interni del DRV.

Per il nostro banco con una sola locomotiva, la prima strada è
ragionevolmente la prima da provare, perché evita la complessità dei
transistor di potenza. Ma resta una decisione di progetto da fare a mente
fresca, perché in entrambi i casi va buttata via una buona parte del
cablaggio fatto stasera e va ridisegnata la rete di lettura.

## Le fonti che ho letto per arrivare a queste conclusioni

Tutte le fonti sono salvate in questa cartella in modo che la prossima volta
non si debba ripartire da capo a cercarle in giro.

Il **datasheet ufficiale del DRV8874** (`ti-drv8874-datasheet.pdf`),
pubblicato da Texas Instruments nel dicembre 2019. Le pagine che servono
sono la 5 (i tempi di accensione e spegnimento del chip), la 6 (i tempi di
commutazione delle uscite), la 10 (il funzionamento del piedino PMODE), e
soprattutto la 11, dove c'è la tabella che descrive cosa fanno le uscite
del chip in funzione degli ingressi quando PMODE è collegato ai 3,3 volt.
È in quella tabella che si vede che la combinazione "entrambi gli ingressi
a zero" produce le rotaie staccate.

Il **datasheet ufficiale del DRV8873** (`ti-drv8873-datasheet.pdf`),
pubblicato da Texas Instruments nell'agosto 2018. È utile perché atanisoft
usa proprio questo chip, e quindi confrontare i due datasheet aiuta a
capire perché atanisoft fa certe scelte che a prima vista sembrerebbero
inutili. Le pagine importanti sono la 3 (la descrizione del piedino
DISABLE, che il nostro DRV8874 non ha), la 15 (la stessa tabella del
DRV8874, che mostra che anche qui le rotaie staccate sono disponibili in
modalità PWM), la 22 (i tempi di commutazione, leggermente più lenti del
DRV8874 ma comunque dentro il budget) e la 30 (la descrizione di cosa
succede quando DISABLE è alto).

Lo **schema della centrale DCC di atanisoft basata su ESP32**
(`atanisoft-esp32cs-schematic.pdf`), scaricato da
https://atanisoft.github.io/ESP32CommandStation/esp32-cs-pcb.html. La
pagina 2 contiene la sezione relativa al ponte di potenza per le rotaie
principali. Si vede chiaramente che fra le uscite del DRV8873 e le rotaie
ci sono due transistor in package doppio chiamato IRF7351, pilotati da un
solo segnale logico. La nota nello schematico dice "transistor di
disconnessione per la rotaia principale RailCom: tieni la rotaia
disconnessa tranne quando la centrale la attiva esplicitamente". I
comparatori che leggono il RailCom sono dei LMV339, una versione a basso
voltaggio dei classici LM339, con soglia di rilevamento calcolata
esplicitamente sullo schematico in circa 19 millivolt, e leggono
**direttamente la tensione fra le rotaie**, non la corrente attraverso
resistenze in serie. È questa la topologia che dobbiamo studiare meglio se
vogliamo replicare un funzionamento solido.

Lo **schema del motor shield DCC-EX EX-MotorShield8874**
(`dcc-ex-motorshield8874-schematic.pdf` e i file di sorgente KiCad
associati). Scaricati da https://github.com/DCC-EX/EX-MotorShield8874 e
https://dcc-ex.com/reference/hardware/motorboards/ex-motor-shield-8874.html.
Questo progetto è interessante perché usa il nostro stesso chip DRV8874 e
lo configura di default in modalità PWM (cioè PMODE ai 3,3 volt) come
suggeriamo di fare noi. La differenza è che la scheda DCC-EX è solo un
generatore di potenza: il rilevatore RailCom è un'altra scheda separata
che si collega in cascata, e che in genere ha i suoi transistor di
disconnessione e la sua rete di lettura sulla rotaia. Quindi non risolve
il nostro problema da sola, ma conferma che mettere il DRV8874 in modalità
PWM è una scelta consolidata nei progetti seri.

Lo **schema del rilevatore RailCom artigianale di dikkedimi**
(`dikkedimi-handmade-railcom-schematic-v0.2.pdf`). Scaricato da
https://github.com/dikkedimi/RailcomDetector, ed è la versione formalizzata
in KiCad di un progetto nato sul forum britannico RMweb
(https://www.rmweb.co.uk/topic/123719-handmade-railcom/). È un rilevatore
RailCom completamente passivo, costruito attorno a dei comparatori LM339
con soglia di rilevamento di ±18 millivolt, che ascolta la tensione fra le
due rotaie senza generare lui stesso il cutout. Per funzionare, presuppone
che il booster a monte metta le rotaie in alta impedenza durante la
finestra di cutout. Quindi non risolve il nostro problema, ma è una
conferma indiretta della regola di fondo: il rilevatore funziona solo se a
monte qualcuno stacca le rotaie, e la lettura va fatta sulla tensione fra
le rotaie, non sulla corrente attraverso resistenze in serie.

Il **manuale del booster CDE di Francesco Cañada** con cutout RailCom
integrato (`fmco-booster-cde-manual.pdf`), e il relativo manuale del
display RailCom (`fmco-railcom-display-manual.pdf`). Scaricati da
https://usuaris.tinet.cat/fmco/railcom_en.html. Cañada usa un chip di
potenza diverso dal nostro, un L6203 della STMicroelectronics, che ha un
piedino dedicato per staccare immediatamente le rotaie dall'esterno. Il suo
firmware abbassa questo piedino per i 460 microsecondi del cutout e poi lo
rialza. Niente transistor esterni, niente trucchi: il chip da solo ha la
modalità di alta impedenza vera, accessibile direttamente dagli ingressi di
controllo. Per noi è interessante come prova che, con il chip giusto, il
problema sarebbe banale. Il nostro DRV8874 non ha quel piedino così
comodo, ma offre la stessa funzione attraverso la modalità PWM descritta
nei datasheet.

Lo **standard ufficiale NMRA S-9.3.2** per il RailCom
(https://www.nmra.org/sites/default/files/s-9.3.2_2012_12_10.pdf), che
specifica la durata del cutout (460 microsecondi), il modo in cui il
decoder deve trasmettere il proprio dato, e la codifica usata. È il
documento normativo di riferimento per capire cosa deve fare la centrale e
cosa deve fare il decoder dentro la finestra di cutout.

Per riferimento storico, nei lavori precedenti abbiamo usato come base un
altro progetto chiamato **MTB-RC** (era stato clonato in una sessione di
maggio 2026 nella cartella temporanea `/tmp/mtb-rc/pcb/`), basato su un
chip di potenza diverso (BTS7960) e su comparatori LM339 con transistor
esterni. È da lì che era arrivato il dimensionamento empirico del nostro
filtro RC e dell'isteresi che abbiamo poi documentato nelle note di
diagnosi precedenti. Quel progetto, come atanisoft, leggeva la tensione
direttamente sulle rotaie, non la corrente attraverso resistenze in serie.

## Dove siamo adesso

Il circuito che abbiamo montato stasera sulla breadboard è strutturalmente
incompatibile con il RailCom. Nella configurazione attuale, con le
resistenze in serie alle rotaie e il DRV in modalità di freno, il
comparatore non riesce a leggere il segnale del decoder perché viene
stordito ogni otto millisecondi dai quindici volt del DCC e non ha tempo
di riprendersi prima della fine della finestra di cutout. E anche se si
mettesse il DRV in modalità di alta impedenza, le resistenze in serie
diventerebbero inutili perché la corrente del decoder non avrebbe più un
giro chiuso da fare attraverso di loro.

Per leggere davvero il RailCom servirà ridisegnare la rete di lettura sul
modello di atanisoft, con comparatori che leggono la tensione direttamente
fra le rotaie e con un riferimento basso, qualche decina di millivolt. Se
vogliamo anche aggiungere i transistor esterni di sicurezza alla atanisoft,
o se ci basta affidarci all'alta impedenza interna del chip in modalità
PWM, è una decisione da prendere quando si rifa il disegno della scheda di
lettura.

In ogni caso, la decisione va presa a mente fresca, in una sessione
dedicata, non in coda a una sessione di banco lunga come quella di stasera.
