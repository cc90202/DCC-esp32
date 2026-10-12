//! `ota-pack`: create keys, sign, inspect, verify and extract DCC-OTA1
//! packages on the host. The package format and every check come from
//! [`dcc_ota_format`], the same code the firmware runs.

use std::{
    fs,
    io::{Read, Write},
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, ensure};
use clap::{Parser, Subcommand};
use dcc_ota_format::{
    Digest, ImageLen,
    image::{self, BuildKind, EmbeddedKey, ImageError, Inspected},
    package::{
        HEADER_LEN, KeyId, Manifest, PackageError, SIGNED_LEN, UnverifiedManifest, unsigned_header,
    },
    target::{CHIP_NAME, PROJECT_NAME},
};
use ed25519_dalek::{Signer, SigningKey, VerifyingKey};
use zeroize::Zeroizing;

/// Longest package: header plus one full slot.
const MAX_PACKAGE: usize = HEADER_LEN + ImageLen::MAX.as_usize();

#[derive(Parser)]
#[command(about = "Sign and check DCC-OTA1 firmware packages")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Create a new key pair; never overwrites.
    Keygen {
        /// Private seed output, created with mode 0600.
        #[arg(long)]
        private: PathBuf,
        /// Public key output.
        #[arg(long)]
        public: PathBuf,
    },
    /// Sign a non-merged application image.
    Sign {
        /// Private seed of the key the devices trust now.
        #[arg(long)]
        key: PathBuf,
        /// Application image from `espflash save-image`.
        #[arg(long)]
        image: PathBuf,
        /// Package output.
        #[arg(long)]
        out: PathBuf,
        /// New public key the image embeds, for a key rotation.
        #[arg(long)]
        rotate_to: Option<PathBuf>,
        /// Allow a bench (dev or fault) build.
        #[arg(long)]
        development: bool,
    },
    /// Show a package header; the signature stays unverified.
    Inspect {
        /// Package to read.
        package: PathBuf,
    },
    /// Check the signature and the image.
    Verify {
        /// Package to read.
        package: PathBuf,
        /// Trusted public key.
        #[arg(long)]
        public: PathBuf,
    },
    /// Write out the image after checking its integrity, not its signature.
    Extract {
        /// Package to read.
        package: PathBuf,
        /// Image output.
        #[arg(long)]
        out: PathBuf,
    },
}

fn read_bounded(path: &Path, max: usize) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    fs::File::open(path)
        .with_context(|| format!("open {}", path.display()))?
        .take(max as u64 + 1)
        .read_to_end(&mut bytes)
        .with_context(|| format!("read {}", path.display()))?;
    ensure!(bytes.len() <= max, "{} is too large", path.display());
    Ok(bytes)
}

/// Reads a 32-byte key file into memory that is wiped on every path,
/// errors included.
fn read_key_bytes(path: &Path) -> Result<Zeroizing<[u8; 32]>> {
    let mut key = Zeroizing::new([0; 32]);
    let mut extra = Zeroizing::new([0; 1]);
    let mut file = fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    file.read_exact(key.as_mut())
        .context("key file must contain exactly 32 raw bytes")?;
    ensure!(
        file.read(extra.as_mut())? == 0,
        "key file must contain exactly 32 raw bytes"
    );
    Ok(key)
}

fn read_public(path: &Path) -> Result<VerifyingKey> {
    VerifyingKey::from_bytes(&*read_key_bytes(path)?).context("not an Ed25519 public key")
}

/// Writes a new file and publishes it only once complete: the bytes go to
/// a temporary sibling first, which is then linked to `path` without ever
/// replacing an existing file. A failure leaves nothing at `path`.
fn create(path: &Path, bytes: &[u8], private: bool) -> Result<()> {
    let mut temporary = path.as_os_str().to_owned();
    temporary.push(".partial");
    let temporary = PathBuf::from(temporary);
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    if private {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    #[cfg(not(unix))]
    if private {
        anyhow::bail!("private key creation requires Unix mode 0600");
    }
    let mut file = options.open(&temporary).with_context(|| {
        format!(
            "create {}; if an interrupted run left it behind, delete it",
            temporary.display()
        )
    })?;
    let published = file
        .write_all(bytes)
        .and_then(|()| file.sync_all())
        .and_then(|()| fs::hard_link(&temporary, path))
        .with_context(|| format!("create {}", path.display()));
    // Only the temporary file this call created is removed.
    let _ = fs::remove_file(&temporary);
    published
}

fn keygen(private: &Path, public: &Path) -> Result<()> {
    ensure!(
        !private.exists() && !public.exists() && private != public,
        "refusing to overwrite key files"
    );
    let mut seed = Zeroizing::new([0; 32]);
    getrandom::fill(seed.as_mut()).context("random source failed")?;
    let key = SigningKey::from_bytes(&seed);
    create(private, seed.as_ref(), true)?;
    create(public, key.verifying_key().as_bytes(), false).with_context(|| {
        format!(
            "public key creation failed; private file {} was already created and retained: \
             recover the public key locally from it, or use new paths for a new pair",
            private.display()
        )
    })
}

/// Which builds a signature may cover.
#[derive(Clone, Copy, PartialEq, Eq)]
enum BuildPolicy {
    /// Release builds only.
    ReleaseOnly,
    /// Bench builds too (`--development`).
    AllowBench,
}

fn sign(
    key: &SigningKey,
    image: &[u8],
    rotate_to: Option<VerifyingKey>,
    policy: BuildPolicy,
) -> Result<Vec<u8>> {
    let Inspected { version, len } = image::inspect(image)?;
    let build = EmbeddedKey::find(image)?;
    ensure!(
        build.kind() == BuildKind::Release || policy == BuildPolicy::AllowBench,
        "dev/fault image requires explicit --development (bench only)"
    );
    let signer = key.verifying_key();
    let embedded = rotate_to.unwrap_or(signer);
    ensure!(
        build.key() == embedded,
        "embedded key differs: rotation requires explicit --rotate-to matching the new key"
    );
    let mut header = unsigned_header(version, len, Digest::of(image), KeyId::of(&signer));
    let signature = key.sign(&header[..SIGNED_LEN]).to_bytes();
    header[SIGNED_LEN..].copy_from_slice(&signature);
    Ok([&header[..], image].concat())
}

/// Splits a package into its header, the decoded header fields and the
/// image; decoding alone authenticates nothing.
fn split(package: &[u8]) -> Result<(&[u8; HEADER_LEN], UnverifiedManifest, &[u8])> {
    let (header, image) = package
        .split_first_chunk::<HEADER_LEN>()
        .context("truncated package")?;
    let manifest = UnverifiedManifest::decode(header)?;
    ensure!(
        image.len() == manifest.image_len().as_usize(),
        PackageError::Malformed
    );
    Ok((header, manifest, image))
}

/// The checks the device makes on the image: digest, structure, version
/// and a valid build record. Which build kinds a device accepts is its own
/// policy and is not checked here.
fn check_integrity(manifest: UnverifiedManifest, image: &[u8]) -> Result<()> {
    ensure!(
        Digest::of(image) == manifest.digest(),
        ImageError::DigestMismatch
    );
    ensure!(
        image::inspect(image)?.version == manifest.version(),
        ImageError::Malformed
    );
    EmbeddedKey::find(image)?;
    Ok(())
}

fn sign_command(
    key: &Path,
    image: &Path,
    out: &Path,
    rotate_to: Option<&Path>,
    development: bool,
) -> Result<()> {
    let image = read_bounded(image, ImageLen::MAX.as_usize())?;
    let rotate_to = rotate_to.map(read_public).transpose()?;
    let key = SigningKey::from_bytes(&*read_key_bytes(key)?);
    let policy = if development {
        BuildPolicy::AllowBench
    } else {
        BuildPolicy::ReleaseOnly
    };
    create(out, &sign(&key, &image, rotate_to, policy)?, false)
}

fn inspect_command(package: &Path) -> Result<()> {
    let bytes = read_bounded(package, MAX_PACKAGE)?;
    let (_, manifest, _) = split(&bytes)?;
    println!(
        "DCC-OTA1; header={HEADER_LEN}; chip={CHIP_NAME}; project={PROJECT_NAME}\n\
         version={}\nimage_len={}\nsha256={}\nkey_id={}\n\
         signature: UNVERIFIED (use verify --public)",
        manifest.version(),
        manifest.image_len(),
        manifest.digest(),
        manifest.key_id()
    );
    Ok(())
}

fn verify_command(package: &Path, public: &Path) -> Result<()> {
    let bytes = read_bounded(package, MAX_PACKAGE)?;
    let (header, manifest, image) = split(&bytes)?;
    Manifest::authenticate(header, &read_public(public)?)?;
    check_integrity(manifest, image)?;
    println!(
        "Signature, SHA-256 and application image verified; device version policy not checked."
    );
    Ok(())
}

fn extract_command(package: &Path, out: &Path) -> Result<()> {
    let bytes = read_bounded(package, MAX_PACKAGE)?;
    let (_, manifest, image) = split(&bytes)?;
    check_integrity(manifest, image)?;
    create(out, image, false)?;
    println!(
        "Extracted non-merged application image; authenticity requires verify PACKAGE --public PUBPATH."
    );
    Ok(())
}

fn run(command: Command) -> Result<()> {
    match command {
        Command::Keygen { private, public } => keygen(&private, &public),
        Command::Sign {
            key,
            image,
            out,
            rotate_to,
            development,
        } => sign_command(&key, &image, &out, rotate_to.as_deref(), development),
        Command::Inspect { package } => inspect_command(&package),
        Command::Verify { package, public } => verify_command(&package, &public),
        Command::Extract { package, out } => extract_command(&package, &out),
    }
}

fn main() {
    if let Err(error) = run(Cli::parse().command) {
        eprintln!("ota-pack: {error:#}");
        std::process::exit(1);
    }
}
