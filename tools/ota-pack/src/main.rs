use dcc_esp32::ota::{
    SLOT_SIZE,
    image::{self, BuildKind},
    package::{HEADER_LEN, Manifest},
};
use ed25519_dalek::{Signer, SigningKey};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{Read, Write},
    path::Path,
};

type Result<T> = std::result::Result<T, String>;

fn read_bounded(path: &Path, max: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(max as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > max {
        return Err("input too large".into());
    }
    Ok(bytes)
}

fn read32(path: &Path) -> Result<[u8; 32]> {
    read_bounded(path, 32)?
        .try_into()
        .map_err(|_| "key file must contain exactly 32 raw bytes".into())
}

fn create(path: &Path, bytes: &[u8], private: bool) -> Result<()> {
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        if private {
            options.mode(0o600);
        }
    }
    #[cfg(not(unix))]
    if private {
        return Err("private key creation requires Unix mode 0600".into());
    }
    let mut file = options.open(path).map_err(|e| e.to_string())?;
    file.write_all(bytes)
        .and_then(|_| file.sync_all())
        .map_err(|e| e.to_string())
}

fn keygen(private: &Path, public: &Path) -> Result<()> {
    if private.exists() || public.exists() || private == public {
        return Err("refusing to overwrite key files".into());
    }
    let mut seed = [0; 32];
    getrandom::getrandom(&mut seed).map_err(|e| e.to_string())?;
    let key = SigningKey::from_bytes(&seed);
    create(private, &seed, true)?;
    create(public, &key.verifying_key().to_bytes(), false).map_err(|e| {
        format!(
            "public key creation failed: {e}; private file {} was already created and retained. Keep it secure; recover the public key locally from that seed with trusted Ed25519 tooling, or use new paths for a new pair. Do not rerun keygen on these paths or disclose the seed.",
            private.display()
        )
    })
}

fn sign(
    seed: [u8; 32],
    bytes: &[u8],
    rotate: Option<[u8; 32]>,
    development: bool,
) -> Result<Vec<u8>> {
    let (version, length) = image::inspect(bytes).map_err(|e| e.code().to_string())?;
    let (kind, embedded) = image::build_metadata(bytes).map_err(|e| e.code().to_string())?;
    if kind != BuildKind::Release && !development {
        return Err("dev/fault image requires explicit --development (bench only)".into());
    }
    let key = SigningKey::from_bytes(&seed);
    let public = key.verifying_key().to_bytes();
    if embedded != rotate.unwrap_or(public) {
        return Err(
            "embedded key differs: rotation requires explicit --rotate-to matching the new key"
                .into(),
        );
    }
    let mut header = Manifest::encode_unsigned(
        version,
        length as u32,
        Sha256::digest(bytes).into(),
        &public,
    )
    .map_err(|e| e.code().to_string())?;
    let signature = key.sign(&header[..192]).to_bytes();
    header[192..].copy_from_slice(&signature);
    let mut package = Vec::with_capacity(HEADER_LEN + bytes.len());
    package.extend_from_slice(&header);
    package.extend_from_slice(bytes);
    Ok(package)
}

// Structural decoding only: the identifier cannot establish authenticity.
fn shape(bytes: &[u8]) -> Result<(Manifest, &[u8])> {
    let h = bytes.get(..HEADER_LEN).ok_or("truncated package")?;
    let manifest = Manifest::decode(h).map_err(|e| e.code().to_string())?;
    if bytes.len() != HEADER_LEN + manifest.image_len as usize {
        return Err("bad_package".into());
    }
    Ok((manifest, &bytes[HEADER_LEN..]))
}

fn integrity(manifest: &Manifest, bytes: &[u8]) -> Result<()> {
    if Sha256::digest(bytes)[..] != manifest.digest {
        return Err("digest_mismatch".into());
    }
    image::check_prefix(bytes, manifest).map_err(|e| e.code().to_string())?;
    image::inspect(bytes).map_err(|e| e.code().to_string())?;
    Ok(())
}

fn verify(bytes: &[u8], public: &[u8; 32]) -> Result<()> {
    let (_, image) = shape(bytes)?;
    let manifest =
        Manifest::authenticate(&bytes[..HEADER_LEN], public).map_err(|e| e.code().to_string())?;
    integrity(&manifest, image)
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write;
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        write!(&mut output, "{byte:02x}").unwrap();
    }
    output
}

enum Command<'a> {
    Keygen { private: &'a Path, public: &'a Path },
    Sign(Cli<'a>),
    Inspect { package: &'a Path },
    Verify { package: &'a Path, public: &'a Path },
    Extract { package: &'a Path, out: &'a Path },
}

struct Cli<'a> {
    options: BTreeMap<&'a str, &'a str>,
    positional: Vec<&'a str>,
}

impl<'a> Cli<'a> {
    fn path(&self, name: &str) -> Result<&'a Path> {
        self.options
            .get(name)
            .copied()
            .map(Path::new)
            .ok_or_else(|| format!("missing {name}"))
    }

    fn command(self, command: &str) -> Result<Command<'a>> {
        match command {
            "keygen" => Ok(Command::Keygen {
                private: self.path("--private")?,
                public: self.path("--public")?,
            }),
            "sign" => Ok(Command::Sign(self)),
            "inspect" => Ok(Command::Inspect {
                package: Path::new(self.positional[0]),
            }),
            "verify" => Ok(Command::Verify {
                package: Path::new(self.positional[0]),
                public: self.path("--public")?,
            }),
            "extract" => Ok(Command::Extract {
                package: Path::new(self.positional[0]),
                out: self.path("--out")?,
            }),
            _ => Err("unknown command".into()),
        }
    }
}

fn parse(args: &[String]) -> Result<Command<'_>> {
    let command = args
        .first()
        .ok_or("expected keygen, sign, inspect, verify, or extract")?;
    let allowed: &[&str] = match command.as_str() {
        "keygen" => &["--private", "--public"],
        "sign" => &["--key", "--image", "--out", "--rotate-to", "--development"],
        "inspect" => &[],
        "verify" => &["--public"],
        "extract" => &["--out"],
        _ => return Err("unknown command".into()),
    };
    let mut options = BTreeMap::new();
    let mut positional = Vec::new();
    let mut args = args.iter().skip(1).map(String::as_str);
    while let Some(arg) = args.next() {
        if arg.starts_with('-') {
            if !allowed.contains(&arg) || options.contains_key(arg) {
                return Err(format!("unknown or duplicate option: {arg}"));
            }
            let value = if arg == "--development" {
                ""
            } else {
                let v = args.next().ok_or("missing option value")?;
                if v.starts_with("--") {
                    return Err("missing option value".into());
                }
                v
            };
            options.insert(arg, value);
        } else {
            positional.push(arg);
        }
    }
    if command == "keygen" || command == "sign" {
        if !positional.is_empty() {
            return Err("unexpected positional argument".into());
        }
    } else if positional.len() != 1 {
        return Err("expected exactly one PACKAGE path".into());
    }
    Cli {
        options,
        positional,
    }
    .command(command)
}

fn sign_command(cli: Cli<'_>) -> Result<()> {
    let bytes = read_bounded(cli.path("--image")?, SLOT_SIZE as usize)?;
    let rotate = cli
        .options
        .get("--rotate-to")
        .map(|p| read32(Path::new(p)))
        .transpose()?;
    let package = sign(
        read32(cli.path("--key")?)?,
        &bytes,
        rotate,
        cli.options.contains_key("--development"),
    )?;
    create(cli.path("--out")?, &package, false)
}

fn inspect_command(package: &Path) -> Result<()> {
    let bytes = read_bounded(package, HEADER_LEN + SLOT_SIZE as usize)?;
    let (manifest, _) = shape(&bytes)?;
    println!(
        "DCC-OTA1; header=256; chip=ESP32-C6; project=dcc-esp32\nversion={}\nimage_len={}\nsha256={}\nkey_id={}\nsignature: UNVERIFIED (use verify --public)",
        manifest.version,
        manifest.image_len,
        hex(&manifest.digest),
        hex(&manifest.key_id)
    );
    Ok(())
}

fn verify_command(package: &Path, public: &Path) -> Result<()> {
    let bytes = read_bounded(package, HEADER_LEN + SLOT_SIZE as usize)?;
    verify(&bytes, &read32(public)?)?;
    println!(
        "Signature, SHA-256 and application image verified; device version policy not checked."
    );
    Ok(())
}

fn extract_command(package: &Path, out: &Path) -> Result<()> {
    let bytes = read_bounded(package, HEADER_LEN + SLOT_SIZE as usize)?;
    let (manifest, image) = shape(&bytes)?;
    integrity(&manifest, image)?;
    create(out, image, false)?;
    println!(
        "Extracted non-merged application image; authenticity requires verify PACKAGE --public PUBPATH."
    );
    Ok(())
}

fn execute(command: Command<'_>) -> Result<()> {
    match command {
        Command::Keygen { private, public } => keygen(private, public),
        Command::Sign(cli) => sign_command(cli),
        Command::Inspect { package } => inspect_command(package),
        Command::Verify { package, public } => verify_command(package, public),
        Command::Extract { package, out } => extract_command(package, out),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if let Err(e) = parse(&args).and_then(execute) {
        eprintln!("ota-pack: {e}");
        std::process::exit(1);
    }
}
