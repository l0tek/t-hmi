fn main() {
    embuild::espidf::sysenv::output();
    println!("cargo:rerun-if-changed=wifi_secrets.local");

    let mut ssid = String::new();
    let mut password = String::new();

    if let Ok(content) = std::fs::read_to_string("wifi_secrets.local") {
        for raw_line in content.lines() {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some((k, v)) = line.split_once('=') {
                let key = k.trim();
                let value = v.trim();
                if key.eq_ignore_ascii_case("SSID") {
                    ssid = value.to_string();
                } else if key.eq_ignore_ascii_case("PASSWORD") {
                    password = value.to_string();
                }
            }
        }
    }

    println!("cargo:rustc-env=WIFI_DEFAULT_SSID={}", ssid);
    println!("cargo:rustc-env=WIFI_DEFAULT_PASSWORD={}", password);
}
