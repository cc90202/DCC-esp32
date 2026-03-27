use std::fs;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-changed=.env");

    let (ssid, pass) = if Path::new(".env").exists() {
        parse_env_file(".env")
    } else {
        println!(
            "cargo:warning=No .env file found — WiFi credentials will be empty. \
             Copy .env.example to .env and fill in your credentials."
        );
        (String::new(), String::new())
    };

    println!("cargo:rustc-env=WIFI_SSID={ssid}");
    println!("cargo:rustc-env=WIFI_PASS={pass}");
}

/// Parse KEY=VALUE pairs from a .env file. Skips comments and blank lines.
fn parse_env_file(path: &str) -> (String, String) {
    let content = fs::read_to_string(path).expect("failed to read .env");
    let mut ssid = String::new();
    let mut pass = String::new();

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((key, value)) = line.split_once('=') {
            match key.trim() {
                "WIFI_SSID" => ssid = value.trim().to_string(),
                "WIFI_PASS" => pass = value.trim().to_string(),
                _ => {}
            }
        }
    }

    (ssid, pass)
}
