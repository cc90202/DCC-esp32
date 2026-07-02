---
name: railcom-detector-breadboard-status
description: "Stato fisico attuale del rilevatore RailCom su breadboard (delta da doc v3 + modifiche utente non documentate). Topologia, valori calcolati, diagnosi, piano interventi."
metadata:
  source: /home/crist/.claude/projects/-home-crist-DCC-esp32/memory/project_railcom_status.md
  copied_on: 2026-05-21
---

# RailCom Detector Breadboard Status

Aggiornato 2026-05-20. Sostituisce lo stato precedente (montaggio TLV3501 da fare -> ora montato e modificato).

## Topologia attuale attorno al TLV3501

Breakout ProtoAdvantage PA0002 (SOIC-8 -> DIP-8) montato sulla breadboard RailCom. Le modifiche utente sono **delta rispetto al documento `docs/hardware/railcom-detector-v3-breadboard-layout.md`** e non sono ancora documentate nel repo.

```text
                                        rail+ 3.3V
                                            |
                                       [R 100 kOhm]
                                            |
SOURCE M1 (Q1 IRLZ44N) --[R 220 Ohm]--+-- pin 2 (IN-)   pin 3 (IN+) --+-- [R 220 Ohm]-- GND
                                      |                                |
                                   [C 4.7 nF]                       [C 100 nF]
                                      |                                |
                                     GND                              GND

V- (pin 4 = GND) --[R 300 kOhm]-- pin 6 (OUT) -> 74HC14 -> GPIO5
```

Modifiche fatte dall'utente rispetto al v3:

- **Aggiunto** filtro RC 220 Ohm + 4.7 nF in serie su IN-
- **Aggiunto** R 220 Ohm verso GND su IN+ (era già nel partitore, ma ora abbinato a 100 nF)
- **Aggiunto** C 100 nF in parallelo su IN+ verso GND
- **Aggiunto** R 300 kOhm tra V- (pin 4 = GND) e OUT (pin 6): pull-down inutile su uscita push-pull
- **Rimosso** R 47 kOhm di isteresi che era tra OUT (pin 6) e IN+ (pin 3): **niente più isteresi**

Sense resistor invariato dal doc v3: **4.7 Ohm** (codice giallo-viola-nero-argento-marrone) tra riga 15 sx (SENSE NODE) e GND.

## Valori calcolati

- **VREF su IN+** = 3.3 V x 220 / (100 000 + 220) ~= **7.24 mV** (v3 target 33 mV, v4 target 22 mV)
- **Soglia trigger corrente** = 7.24 mV / 4.7 Ohm ~= **1.54 mA** (target 6-10 mA)
- **Filtro RC su IN-**: tau ~= 1 us, fc ~= 154 kHz (attenua -6 dB a 250 kHz, frequenza RailCom)
- **Isteresi**: assente

## Diagnosi corrente

Tasso di letture CV29 valide molto basso (~1 lettura ogni ~250 s), regressione rispetto al baseline noto (`project_railcom_loksound_diagnosis` = 1/200 s). Cause sommabili in ordine di impatto:

1. Soglia di trigger ~5x troppo bassa -> falsi trigger su rumore residuo
2. Isteresi rimossa -> output oscilla sui fronti del segnale
3. Filtro RC attenua i fronti UART RailCom a 250 kBaud
4. R 300 kOhm tra GND e OUT: inutile (push-pull rail-to-rail), può restare ma è da pulire

**Why:** il TLV3501 senza isteresi più una soglia bassa genera multi-trigger sui fronti che il 74HC14 buffer fa solo da spettatore: il segnale UART arriva al GPIO5 già sporco, rx_err elevato, ch2_ok ~1%.

**How to apply:** prima di proporre modifiche software o nuovi build, verifica che il circuito sia in uno stato noto. Confronta con questa memoria prima di chiedere all'utente di rispiegarlo.

## Piano interventi proposto (un passo alla volta)

1. **Step 1** (proposto, non ancora montato): aggiungere **R 680 kOhm** (blu-grigio-nero-arancio-marrone) tra pin 6 (OUT, riga 22 dx) e pin 3 (IN+, riga 22 sx) -> ripristina isteresi. Valore raccomandato dal doc v4 per sense piccoli (47 k è troppo aggressivo).
2. **Step 2** (dopo verifica log): sostituire la R 220 Ohm verso GND su IN+ con **R 1 kOhm** (marrone-nero-nero-marrone-marrone) -> VREF da 7 mV a 32 mV -> soglia da 1.5 mA a 6.8 mA.
3. **Step 3** (cosmetico): rimuovere R 300 kOhm tra V- e OUT (non fa nulla).
4. **Step 4** (da valutare allo scope): considerare la rimozione del filtro RC su IN- se ancora rx_err alto dopo Step 1+2.

## Riferimenti

- `docs/hardware/railcom-detector-v3-breadboard-layout.md` (baseline circuito + tabella collegamenti righe breadboard)
- `docs/hardware/railcom-detector-v4-bidirectional-fix.md` (raccomandazione isteresi 680 k-1 M, indicazione "no 47 k con sense piccoli")
- `project_railcom_loksound_diagnosis` (baseline funzionamento pre-modifiche)
