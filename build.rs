//! Makes `ld/isr_ram.x` visible to the firmware link.
//!
//! The script itself is passed by `.cargo/config.toml`, ahead of `linkall.x`;
//! this only adds its directory to the linker search path.

use std::env;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-changed=ld/isr_ram.x");
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("riscv32") {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("ld");
        println!("cargo:rustc-link-search={}", dir.display());
    }
}
