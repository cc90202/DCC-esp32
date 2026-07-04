//! Embedded provisioning HTML pages.

pub const SETUP_PAGE: &str = r#"<!doctype html>
<html lang="it">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Configura WiFi DCC</title>
<style>
:root{color-scheme:light;--bg:#eef3ed;--card:#fffdf7;--ink:#172019;--muted:#5a665d;--line:#d8ded6;--accent:#0f6b4f;--accent2:#f2c14e;--bad:#a62626}
*{box-sizing:border-box}body{margin:0;min-height:100vh;font-family:Georgia,"Times New Roman",serif;background:radial-gradient(circle at 15% 10%,#f7e7a5 0 14rem,transparent 14.5rem),linear-gradient(135deg,#e7efe7,#dbe7df 45%,#f7f1df);color:var(--ink);display:grid;place-items:center;padding:24px}
main{width:min(100%,430px);background:var(--card);border:1px solid var(--line);border-radius:28px;box-shadow:0 24px 70px #24402f2e;padding:28px}
.badge{display:inline-block;background:#173f32;color:#fff;border-radius:999px;padding:7px 12px;font:700 12px ui-monospace,monospace;letter-spacing:.08em;text-transform:uppercase}
h1{font-size:clamp(32px,9vw,46px);line-height:.95;margin:22px 0 10px;letter-spacing:-.04em}
p{color:var(--muted);line-height:1.45;margin:0 0 22px}
label{display:block;font-weight:700;margin:16px 0 7px}
input{width:100%;font:inherit;border:1px solid var(--line);border-radius:16px;padding:15px 14px;background:#fff;color:var(--ink);outline:none}
input:focus{border-color:var(--accent);box-shadow:0 0 0 4px #0f6b4f22}
.password{display:grid;grid-template-columns:1fr auto;gap:8px}.password button{border:1px solid var(--line);border-radius:16px;background:#f7f5ed;padding:0 12px;color:var(--ink)}
.submit{width:100%;margin-top:22px;border:0;border-radius:18px;background:linear-gradient(135deg,var(--accent),#114536);color:#fff;font:700 17px Georgia,"Times New Roman",serif;padding:16px;box-shadow:0 10px 20px #0f6b4f33}
.note{font-size:13px;margin-top:16px;color:var(--muted)}.error{border-left:4px solid var(--bad);background:#fff1ee;color:#6e1616;padding:12px;border-radius:12px;margin-bottom:16px}
</style>
</head>
<body>
<main>
<span class="badge">DCC Setup</span>
<h1>Configura WiFi DCC</h1>
<p>Inserisci le credenziali della rete WiFi. La centrale salva i dati in flash e li usera' al prossimo avvio.</p>
<form method="post" action="/save">
<label for="ssid">Nome rete WiFi</label>
<input id="ssid" name="ssid" autocomplete="off" maxlength="32" required>
<label for="password">Password</label>
<div class="password"><input id="password" name="password" type="password" minlength="8" maxlength="63" required><button type="button" onclick="const p=document.getElementById('password');p.type=p.type==='password'?'text':'password'">Mostra</button></div>
<button class="submit" type="submit">Salva credenziali</button>
</form>
<p class="note">Durante questa modalita' i treni restano fermi: DCC, RailCom e Z21 non sono avviati.</p>
</main>
</body>
</html>"#;

pub const INVALID_FORM_PAGE: &str = r#"<!doctype html><html lang="it"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Configura WiFi DCC</title><style>body{font-family:Georgia,"Times New Roman",serif;background:#eef3ed;color:#172019;padding:24px}main{max-width:430px;margin:auto;background:#fffdf7;border-radius:24px;padding:24px}.error{border-left:4px solid #a62626;background:#fff1ee;color:#6e1616;padding:12px;border-radius:12px}a{color:#0f6b4f;font-weight:700}</style></head><body><main><p class="error">Credenziali non valide. SSID obbligatorio, password WPA2 tra 8 e 63 caratteri.</p><p><a href="/">Torna al modulo</a></p></main></body></html>"#;

pub const SAVE_OK_PAGE: &str = r#"<!doctype html><html lang="it"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>WiFi salvato</title><style>body{font-family:Georgia,"Times New Roman",serif;background:#eef3ed;color:#172019;padding:24px}main{max-width:430px;margin:auto;background:#fffdf7;border-radius:24px;padding:24px}h1{font-size:36px;line-height:1}</style></head><body><main><h1>Credenziali salvate</h1><p>La rete WiFi e' stata salvata in flash. Il riavvio automatico sara' completato nel prossimo task; per ora riavvia manualmente la centrale.</p></main></body></html>"#;

pub const SAVE_ERROR_PAGE: &str = r#"<!doctype html><html lang="it"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Errore salvataggio</title><style>body{font-family:Georgia,"Times New Roman",serif;background:#eef3ed;color:#172019;padding:24px}main{max-width:430px;margin:auto;background:#fffdf7;border-radius:24px;padding:24px}.error{border-left:4px solid #a62626;background:#fff1ee;color:#6e1616;padding:12px;border-radius:12px}</style></head><body><main><p class="error">Salvataggio non riuscito. Le credenziali non sono state modificate.</p><p><a href="/">Riprova</a></p></main></body></html>"#;
