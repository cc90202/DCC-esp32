use dcc_esp32::ota::image::{self, BuildKind, BuildMetadata};
use ed25519_dalek::SigningKey;
use std::{fs, path::Path, process::Command};

fn cli(dir: &Path, args: &[&str]) -> bool {
    Command::new(env!("CARGO_BIN_EXE_ota-pack"))
        .current_dir(dir)
        .args(args)
        .status()
        .unwrap()
        .success()
}

fn fixture(key: [u8; 32], kind: BuildKind, version: &str) -> Vec<u8> {
    let mut bytes = vec![0; 4096];
    bytes[0] = 0xe9;
    bytes[1] = 1;
    bytes[3] = 0x30;
    bytes[12] = 13;
    bytes[28..32].copy_from_slice(&4060u32.to_le_bytes());
    bytes[32..36].copy_from_slice(&0xabcd5432u32.to_le_bytes());
    bytes[48..48 + version.len()].copy_from_slice(version.as_bytes());
    bytes[80..89].copy_from_slice(b"dcc-esp32");
    bytes[288..332].copy_from_slice(
        &BuildMetadata {
            kind,
            public_key: key,
        }
        .encode(),
    );
    bytes[4095] = bytes[32..4092].iter().fold(0xef, |a, b| a ^ b);
    assert!(image::inspect(&bytes).is_ok());
    bytes
}

#[test]
fn commands_roundtrip_and_safety() {
    let dir = std::env::temp_dir().join(format!("ota-pack-test-{}", std::process::id()));
    fs::create_dir(&dir).unwrap();
    let result = std::panic::catch_unwind(|| {
        assert!(cli(
            &dir,
            &["keygen", "--private", "seed", "--public", "public"]
        ));
        let seed = fs::read(dir.join("seed")).unwrap();
        let public: [u8; 32] = fs::read(dir.join("public")).unwrap().try_into().unwrap();
        assert_eq!(seed.len(), 32);
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(
                fs::metadata(dir.join("seed")).unwrap().permissions().mode() & 0o777,
                0o600
            );
        }
        assert!(!cli(
            &dir,
            &["keygen", "--private", "seed", "--public", "other"]
        ));
        assert_eq!(fs::read(dir.join("seed")).unwrap(), seed);
        assert!(!dir.join("other").exists());
        let partial = Command::new(env!("CARGO_BIN_EXE_ota-pack"))
            .current_dir(&dir)
            .args([
                "keygen",
                "--private",
                "partial-seed",
                "--public",
                "missing/public",
            ])
            .output()
            .unwrap();
        assert!(!partial.status.success());
        let diagnostic = String::from_utf8(partial.stderr).unwrap();
        assert!(diagnostic.contains("private file partial-seed was already created and retained"));
        assert!(diagnostic.contains("recover the public key locally"));
        assert!(diagnostic.contains("use new paths for a new pair"));
        assert_eq!(fs::metadata(dir.join("partial-seed")).unwrap().len(), 32);
        assert!(!dir.join("missing/public").exists());
        let sign = [
            "sign", "--key", "seed", "--image", "image", "--out", "package",
        ];
        for version in ["0.0.0", "0.1.0", "1.2.3"] {
            let image = fixture(public, BuildKind::Release, version);
            fs::write(dir.join("image"), &image).unwrap();
            assert!(cli(&dir, &sign));
            assert!(cli(&dir, &["inspect", "package"]));
            assert!(cli(&dir, &["verify", "package", "--public", "public"]));
            assert!(cli(&dir, &["extract", "package", "--out", "extracted"]));
            assert_eq!(fs::read(dir.join("extracted")).unwrap(), image);
            fs::remove_file(dir.join("extracted")).unwrap();
            let wrong = SigningKey::from_bytes(&[99; 32]).verifying_key().to_bytes();
            fs::write(dir.join("wrong"), wrong).unwrap();
            assert!(!cli(&dir, &["verify", "package", "--public", "wrong"]));
            let mut package = fs::read(dir.join("package")).unwrap();
            let mut bad_signature = package.clone();
            bad_signature[192] ^= 1;
            fs::write(dir.join("bad-signature"), bad_signature).unwrap();
            assert!(!cli(
                &dir,
                &["verify", "bad-signature", "--public", "public"]
            ));
            package[256 + 400] ^= 1;
            fs::write(dir.join("package"), package).unwrap();
            assert!(!cli(&dir, &["verify", "package", "--public", "public"]));
            assert!(!cli(&dir, &["extract", "package", "--out", "extracted"]));
            fs::remove_file(dir.join("package")).unwrap();
        }
        for kind in [BuildKind::Dev, BuildKind::Fault] {
            fs::write(dir.join("image"), fixture(public, kind, "1.2.3")).unwrap();
            assert!(!cli(&dir, &sign));
            let mut dev = sign.to_vec();
            dev.push("--development");
            assert!(cli(&dir, &dev));
            fs::remove_file(dir.join("package")).unwrap();
        }
        let new = SigningKey::from_bytes(&[98; 32]).verifying_key().to_bytes();
        fs::write(dir.join("new"), new).unwrap();
        fs::write(dir.join("image"), fixture(new, BuildKind::Release, "1.2.3")).unwrap();
        assert!(!cli(&dir, &sign));
        let mut rotation = sign.to_vec();
        rotation.extend(["--rotate-to", "wrong"]);
        assert!(!cli(&dir, &rotation));
        rotation.pop();
        rotation.push("new");
        assert!(cli(&dir, &rotation));
        assert!(cli(&dir, &["verify", "package", "--public", "public"]));
        assert!(!cli(&dir, &["inspect", "package", "--unknown"]));
        assert!(!cli(&dir, &["sign", "--development", "--development"]));
        fs::write(dir.join("oversize-key"), [0; 33]).unwrap();
        assert!(!cli(
            &dir,
            &["verify", "package", "--public", "oversize-key"]
        ));
        fs::remove_file(dir.join("package")).unwrap();
        let oversized = fs::File::create(dir.join("oversize")).unwrap();
        oversized
            .set_len(dcc_esp32::ota::SLOT_SIZE as u64 + 1)
            .unwrap();
        assert!(!cli(
            &dir,
            &[
                "sign", "--key", "seed", "--image", "oversize", "--out", "package"
            ]
        ));
        assert!(!dir.join("package").exists());
        oversized
            .set_len(dcc_esp32::ota::SLOT_SIZE as u64 + 257)
            .unwrap();
        for command in ["inspect", "verify", "extract"] {
            let args = match command {
                "verify" => vec![command, "oversize", "--public", "public"],
                "extract" => vec![command, "oversize", "--out", "extracted"],
                _ => vec![command, "oversize"],
            };
            assert!(!cli(&dir, &args));
        }
        assert!(!dir.join("extracted").exists());
    });
    fs::remove_dir_all(dir).unwrap();
    result.unwrap();
}
