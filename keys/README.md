# Chiavi OTA

Questi file contengono **solo chiavi pubbliche Ed25519**, 32 byte raw:

| File | SHA-256 completo (fingerprint) |
|---|---|
| `ota_release.pub` | `6ad67764ff8c8b1c85da77d91c6eab1fadb8f8ee6fa8c80d0ee97d37009ffac3` |
| `ota_dev.pub` | `1a7d732207b5949686904a87f6b85908652be20dcaa4184dd65b180afb361014` |

`key_id` nel pacchetto è costituito dai primi 8 byte del fingerprint. Non è una
firma. La CI fissa i fingerprint: ogni modifica va revisionata insieme alla
procedura di rotazione e alla nuova chiave incorporata nel firmware.

I seed privati corrispondenti sono stati generati in questo orb, fuori repo:
`~/.config/dcc-esp32/ota_release.key` e `~/.config/dcc-esp32/ota_dev.key`.
Sono 32 byte raw con permessi `0600`. **Non visualizzarli, committarli, inviarli
nei log/chat o caricarli in CI.** Prima di perdere l'orb, il proprietario deve
copiarli e farne un backup cifrato mediante un metodo sicuro approvato,
conservando accesso limitato. Non è stato effettuato un backup esterno qui.
La perdita del seed release richiede una strategia di rotazione già autorizzata
o recupero USB: non si può ricostruire dalla chiave pubblica.

Per generare una **nuova coppia**, senza sovrascrivere la coppia corrente:

```bash
install -d -m 700 ~/.config/dcc-esp32
cargo ota-pack keygen --private ~/.config/dcc-esp32/ota_next.key \
  --public ~/.config/dcc-esp32/ota_next.pub
```

La chiave release firma solo build di produzione senza feature dev/fault.
`ota-dev-key` incorpora la chiave dev; `ota-fault-inject` la abilita implicitamente.
`--development` è un consenso esplicito per firmare build bench dev/fault, non
un modo per pubblicarle come release. Una rotazione usa il vecchio seed per
firmare una build che incorpora la nuova chiave, con `--rotate-to NUOVA.pub`;
il tool verifica il match. Dopo conferma, la nuova build accetta la nuova chiave.
Documentare il percorso nelle release notes e mantenere recuperabili i seed
necessari al fallback. Chiave sconosciuta: non inventare versioni ponte.

Vedi [creazione e verifica pacchetti](../docs/ota.md). La CI usa solo seed
temporanei usa-e-getta per un roundtrip non pubblicato: nessun secret release.
