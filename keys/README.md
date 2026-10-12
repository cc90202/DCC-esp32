# Chiavi OTA

Questi file contengono **solo chiavi pubbliche Ed25519**, 32 byte raw:

| File | SHA-256 completo (fingerprint) |
|---|---|
| `ota_release.pub` | `333d21df8f456306b81716b63ce1be59d6d974d41b3d28dc7dce21d78394d052` |

`key_id` nel pacchetto è costituito dai primi 8 byte del fingerprint. Non è una
firma. La CI fissa i fingerprint: ogni modifica va revisionata insieme alla
procedura di rotazione e alla nuova chiave incorporata nel firmware.

Il seed privato corrispondente sta fuori dal repository, sul PC di chi
firma: `~/.config/dcc-esp32/ota_release_main.key`, 32 byte raw con permessi `0600`. **Non
visualizzarlo, committarlo, inviarlo in log o chat, né caricarlo in CI.** Va
tenuta una copia cifrata del seed release in un posto sicuro: perso il seed,
le schede non accettano più aggiornamenti firmati e serve la USB, perché non
si ricostruisce dalla chiave pubblica.

Per generare una **nuova coppia**, senza sovrascrivere la coppia corrente:

```bash
install -d -m 700 ~/.config/dcc-esp32
cargo ota-pack keygen --private ~/.config/dcc-esp32/ota_next.key \
  --public ~/.config/dcc-esp32/ota_next.pub
```

La chiave release firma solo build di produzione senza la feature fault.
`--development` è un consenso esplicito per firmare build bench fault, non
un modo per pubblicarle come release. Una rotazione usa il vecchio seed per
firmare una build che incorpora la nuova chiave, con `--rotate-to NUOVA.pub`;
il tool verifica il match. Dopo conferma, la nuova build accetta la nuova chiave.
Documentare il percorso nelle release notes e mantenere recuperabili i seed
necessari al fallback. Chiave sconosciuta: non inventare versioni ponte.

Vedi [creazione e verifica pacchetti](../docs/ota.md). La CI usa solo seed
temporanei usa-e-getta per un roundtrip non pubblicato: nessun secret release.
