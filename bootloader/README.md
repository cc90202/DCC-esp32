# Bootloader ESP32-C6 fissato

`esp32c6-bootloader.bin` è una copia byte-per-byte del bootloader stock di
**espflash v4.3.0**, non una build locale né un bootloader con rollback IDF.

- Provenienza verificata: [`espflash/resources/bootloaders/esp32c6-bootloader.bin`](https://github.com/esp-rs/espflash/blob/v4.3.0/espflash/resources/bootloaders/esp32c6-bootloader.bin)
- [Download originale](https://raw.githubusercontent.com/esp-rs/espflash/v4.3.0/espflash/resources/bootloaders/esp32c6-bootloader.bin)
- SHA-256: `402c2c64761034e10b36ca8699d641e2bc0c86fd245a859d78e1aae4e2d201cd`
- espflash è distribuito MIT OR Apache-2.0; si conserva sotto la licenza
  [MIT upstream](https://github.com/esp-rs/espflash/blob/v4.3.0/LICENSE-MIT),
  riprodotta sotto. Il bootloader deriva da ESP-IDF (Apache-2.0); copia locale
  [LICENSE-APACHE](LICENSE-APACHE), da
  [licenza ESP-IDF](https://github.com/espressif/esp-idf/blob/v5.5.1/LICENSE).

Verifica locale: `sha256sum bootloader/esp32c6-bootloader.bin`.
La CI verifica lo stesso hash. Non sostituire il file durante una normale OTA:
bootloader e partition table vengono scritti solamente nella migrazione USB.
Il rollback è applicativo tramite journal, non una garanzia del bootloader:
guasti prima dell'inizializzazione OTA possono richiedere USB. Vedi
[procedura operativa](../docs/ota.md) e [design](../docs/specs/ota-update.md).

## Avviso MIT upstream

Copyright (c) 2022-2025 The Espflash Project Developers

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
