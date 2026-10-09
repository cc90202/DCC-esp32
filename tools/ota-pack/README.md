# ota-pack

Host-only signing and inspection of DCC-OTA1 packages. Keep private seeds
outside the repository. Key files are **raw 32 bytes**, not hex/PEM.
Key generation uses the OS RNG, refuses overwrites, and creates private files
with Unix mode 0600. All output files refuse overwrites.

Run from the repository root with an explicit host target (Cargo does not
load this tool's nested config when invoked from the root):

```sh
cargo run --manifest-path tools/ota-pack/Cargo.toml --target x86_64-unknown-linux-gnu -- keygen --private /secure/ota.seed --public /secure/ota.pub
cargo run --manifest-path tools/ota-pack/Cargo.toml --target x86_64-unknown-linux-gnu -- sign --key /secure/ota.seed --image app.bin --out release.dccfw
cargo run --manifest-path tools/ota-pack/Cargo.toml --target x86_64-unknown-linux-gnu -- inspect release.dccfw
cargo run --manifest-path tools/ota-pack/Cargo.toml --target x86_64-unknown-linux-gnu -- verify release.dccfw --public /secure/ota.pub
cargo run --manifest-path tools/ota-pack/Cargo.toml --target x86_64-unknown-linux-gnu -- extract release.dccfw --out exact-app.bin
cargo test --manifest-path tools/ota-pack/Cargo.toml --target x86_64-unknown-linux-gnu
```

Use the non-merged application output of `espflash save-image` for ESP32-C6,
8 MB flash, and the approved partition table; never a merged flash image.
Signing validates the full ESP image and unique build metadata, reads the
canonical version from app_desc, and signs the manifest plus image digest.
Dev/fault images require `--development` **for bench devices only**.

Ordinarily the embedded key must equal the signing key. For intentional
rotation, add `--rotate-to /secure/new.pub`; it must match the new embedded
key while the package is signed by the old trusted private seed. Document
the rotation path in release notes.

`inspect` validates package structure but explicitly leaves the signature
unverified. `extract` checks structure, SHA-256, prefix, and full ESP image;
it does **not** authenticate the package. Authenticate using `verify --public`
before flashing the extracted exact application bytes. No bootloader or
partition table is generated or merged.

Offline `verify` authenticates the signature and validates the image, not
eligibility on a particular device (running/rejected versions). It accepts
any authenticated canonical version, including 0.0.0. The firmware separately
enforces its strict upgrade policy and persistent failed-version floor.
