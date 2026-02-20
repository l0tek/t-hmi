use std::collections::BTreeMap;

const MAX_DEFAULT_WIFI: usize = 8;

#[derive(Default, Clone)]
struct WifiCred {
    ssid: String,
    password: String,
}

fn parse_line(line: &str) -> Option<(&str, &str)> {
    let (k, v) = line.split_once('=')?;
    Some((k.trim(), v.trim()))
}

fn parse_suffix_index(key: &str, prefix: &str) -> Option<usize> {
    key.strip_prefix(prefix)
        .and_then(|x| x.parse::<usize>().ok())
}

fn parse_wrapped_index(key: &str, head: &str, tail: &str) -> Option<usize> {
    let rest = key.strip_prefix(head)?;
    let (num, end) = rest.split_once('_')?;
    if end.eq_ignore_ascii_case(tail) {
        num.parse::<usize>().ok()
    } else {
        None
    }
}

fn set_ssid(by_idx: &mut BTreeMap<usize, WifiCred>, idx: usize, value: &str) {
    by_idx.entry(idx).or_default().ssid = value.to_string();
}

fn set_password(by_idx: &mut BTreeMap<usize, WifiCred>, idx: usize, value: &str) {
    by_idx.entry(idx).or_default().password = value.to_string();
}

fn load_wifi_creds() -> Vec<WifiCred> {
    let mut by_idx: BTreeMap<usize, WifiCred> = BTreeMap::new();

    if let Ok(content) = std::fs::read_to_string("wifi_secrets.local") {
        for raw_line in content.lines() {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let Some((key_raw, value)) = parse_line(line) else {
                continue;
            };
            let key = key_raw.trim().to_ascii_uppercase();

            if key == "SSID" || key == "WIFI_DEFAULT_SSID" {
                set_ssid(&mut by_idx, 1, value);
                continue;
            }
            if key == "PASSWORD" || key == "WIFI_DEFAULT_PASSWORD" {
                set_password(&mut by_idx, 1, value);
                continue;
            }

            if let Some(idx) = parse_suffix_index(&key, "SSID_")
                .or_else(|| parse_suffix_index(&key, "WIFI_DEFAULT_SSID_"))
                .or_else(|| parse_wrapped_index(&key, "WIFI_", "SSID"))
            {
                set_ssid(&mut by_idx, idx, value);
                continue;
            }

            if let Some(idx) = parse_suffix_index(&key, "PASSWORD_")
                .or_else(|| parse_suffix_index(&key, "WIFI_DEFAULT_PASSWORD_"))
                .or_else(|| parse_wrapped_index(&key, "WIFI_", "PASSWORD"))
            {
                set_password(&mut by_idx, idx, value);
            }
        }
    }

    let mut out = Vec::new();
    for (_idx, item) in by_idx {
        if item.ssid.trim().is_empty() {
            continue;
        }
        out.push(item);
        if out.len() >= MAX_DEFAULT_WIFI {
            break;
        }
    }
    out
}

fn main() {
    embuild::espidf::sysenv::output();
    println!("cargo:rerun-if-changed=wifi_secrets.local");

    let creds = load_wifi_creds();

    let first = creds.first().cloned().unwrap_or_default();
    println!("cargo:rustc-env=WIFI_DEFAULT_SSID={}", first.ssid);
    println!("cargo:rustc-env=WIFI_DEFAULT_PASSWORD={}", first.password);

    for i in 1..=MAX_DEFAULT_WIFI {
        let idx = i - 1;
        let (ssid, password) = if let Some(c) = creds.get(idx) {
            (c.ssid.as_str(), c.password.as_str())
        } else {
            ("", "")
        };
        println!("cargo:rustc-env=WIFI_DEFAULT_SSID_{}={}", i, ssid);
        println!("cargo:rustc-env=WIFI_DEFAULT_PASSWORD_{}={}", i, password);
    }
}
