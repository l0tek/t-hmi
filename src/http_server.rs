use anyhow::Result;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread;
use std::time::Duration;

pub struct MiniHttpServer {
    stop: Arc<AtomicBool>,
    join: Option<thread::JoinHandle<()>>,
    port: u16,
}

impl MiniHttpServer {
    pub fn start(port: u16) -> Result<Self> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;
        if let Ok(addr) = listener.local_addr() {
            println!("[HTTP] listening on {}", addr);
        }
        listener.set_nonblocking(true)?;

        let stop = Arc::new(AtomicBool::new(false));
        let stop_thread = Arc::clone(&stop);

        let join = thread::Builder::new()
            .name("mini-http".to_string())
            // ESP pthread default stack is often too small for std::net + formatting.
            .stack_size(16 * 1024)
            .spawn(move || {
                while !stop_thread.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((stream, _addr)) => {
                            println!("[HTTP] client connected");
                            let _ = handle_client(stream);
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            thread::sleep(Duration::from_millis(30));
                        }
                        Err(_) => {
                            println!("[HTTP] accept error");
                            thread::sleep(Duration::from_millis(100));
                        }
                    }
                }
            })?;

        Ok(Self {
            stop,
            join: Some(join),
            port,
        })
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for MiniHttpServer {
    fn drop(&mut self) {
        self.stop();
    }
}

fn handle_client(mut stream: TcpStream) -> Result<()> {
    let mut req = [0u8; 1024];
    let n = stream.read(&mut req)?;
    if n == 0 {
        return Ok(());
    }

    let request = String::from_utf8_lossy(&req[..n]);
    let target = parse_path(&request);
    println!("[HTTP] request path={}", target);
    let (path, query) = split_path_query(target);

    match path {
        "/" => {
            let body = render_dashboard_page();
            respond_html(&mut stream, 200, &body)?;
        }
        "/logs" => {
            let body = render_logs_page();
            respond_html(&mut stream, 200, &body)?;
        }
        "/api/cmd" => {
            let body = match enqueue_remote_command(query) {
                Ok(msg) => format!(
                    "<!doctype html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0; url=/\"></head><body>{}</body></html>",
                    escape_html(&msg)
                ),
                Err(err) => format!(
                    "<!doctype html><html><head><meta charset=\"utf-8\"></head><body><h1>Command Fehler</h1><pre>{}</pre><p><a href=\"/\">Zurueck</a></p></body></html>",
                    escape_html(&format!("{}", err))
                ),
            };
            respond_html(&mut stream, 200, &body)?;
        }
        "/sd" => {
            let body = match crate::sdcard::list_sd_root() {
                Ok(entries) => {
                    let mut html = String::from("<html><head><meta charset=\"utf-8\"></head><body><h1>SD Karte</h1><ul>");
                    for e in entries {
                        if e.is_dir {
                            html.push_str(&format!("<li>[DIR] {}</li>", escape_html(&e.name)));
                        } else {
                            let encoded_name = url_encode_component(&e.name);
                            html.push_str(&format!(
                                "<li><a href=\"/sd/download?file={}\">{}</a> ({} Bytes)</li>",
                                encoded_name,
                                escape_html(&e.name),
                                e.size
                            ));
                        }
                    }
                    html.push_str("</ul><p><a href=\"/\">Zurueck</a></p></body></html>");
                    html
                }
                Err(err) => format!(
                    "<html><head><meta charset=\"utf-8\"></head><body><h1>SD Fehler</h1><pre>{}</pre><p><a href=\"/\">Zurueck</a></p></body></html>",
                    escape_html(&format!("{}", err))
                ),
            };
            respond_html(&mut stream, 200, &body)?;
        }
        "/sd/download" => {
            let Some(raw_file) = query_param(query, "file") else {
                respond_html(
                    &mut stream,
                    404,
                    "<html><body><h1>404</h1><p>Fehlender Parameter: file</p></body></html>",
                )?;
                return Ok(());
            };

            let file_name = url_decode_component(raw_file);
            match crate::sdcard::read_sd_file(&file_name) {
                Ok(data) => {
                    respond_download(&mut stream, &file_name, &data)?;
                }
                Err(err) => {
                    let body = format!(
                        "<html><head><meta charset=\"utf-8\"></head><body><h1>Download Fehler</h1><pre>{}</pre><p><a href=\"/sd\">Zurueck</a></p></body></html>",
                        escape_html(&format!("{}", err))
                    );
                    respond_html(&mut stream, 404, &body)?;
                }
            }
        }
        "/sd/view" => {
            let Some(raw_file) = query_param(query, "file") else {
                respond_html(
                    &mut stream,
                    404,
                    "<html><body><h1>404</h1><p>Fehlender Parameter: file</p></body></html>",
                )?;
                return Ok(());
            };

            let file_name = url_decode_component(raw_file);
            match crate::sdcard::read_sd_file(&file_name) {
                Ok(data) => {
                    let max = 64 * 1024usize;
                    let (slice, truncated) = if data.len() > max {
                        (&data[..max], true)
                    } else {
                        (data.as_slice(), false)
                    };
                    let text = String::from_utf8_lossy(slice);
                    let mut body = String::new();
                    body.push_str("<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                    body.push_str("<title>Datei Ansicht</title><link rel=\"stylesheet\" href=\"https://cdn.jsdelivr.net/npm/picnic@7.1.0/picnic.min.css\">");
                    body.push_str("<style>body{max-width:980px;margin:0 auto;padding:1rem} pre{white-space:pre-wrap;word-break:break-word;background:#f7f7f7;padding:12px;border-radius:8px;border:1px solid #ddd;}</style></head><body>");
                    body.push_str("<nav><a href=\"/\" class=\"brand\">t-hmi</a><div class=\"menu\"><a href=\"/\">Dashboard</a><a href=\"/logs\">Logfiles</a><a href=\"/sd\">SD Karte</a></div></nav>");
                    body.push_str(&format!(
                        "<h2>{}</h2><p><a class=\"button small\" href=\"/sd/download?file={}\">Download</a></p>",
                        escape_html(&file_name),
                        url_encode_component(&file_name)
                    ));
                    if truncated {
                        body.push_str("<article class=\"warning\">Datei ist gross, Anzeige auf 64 KiB begrenzt.</article>");
                    }
                    body.push_str("<pre>");
                    body.push_str(&escape_html(&text));
                    body.push_str("</pre></body></html>");
                    respond_html(&mut stream, 200, &body)?;
                }
                Err(err) => {
                    let body = format!(
                        "<html><head><meta charset=\"utf-8\"></head><body><h1>Ansicht Fehler</h1><pre>{}</pre><p><a href=\"/logs\">Zurueck</a></p></body></html>",
                        escape_html(&format!("{}", err))
                    );
                    respond_html(&mut stream, 404, &body)?;
                }
            }
        }
        _ => {
            respond_html(
                &mut stream,
                404,
                "<html><body><h1>404</h1><p>Not Found</p></body></html>",
            )?;
        }
    }

    Ok(())
}

fn parse_path(request: &str) -> &str {
    if let Some(line) = request.lines().next() {
        let mut parts = line.split_whitespace();
        let method = parts.next().unwrap_or("");
        let path = parts.next().unwrap_or("/");
        if method == "GET" {
            return path;
        }
    }
    "/"
}

fn split_path_query(target: &str) -> (&str, &str) {
    if let Some((path, query)) = target.split_once('?') {
        (path, query)
    } else {
        (target, "")
    }
}

fn query_param<'a>(query: &'a str, key: &str) -> Option<&'a str> {
    for pair in query.split('&') {
        if let Some((k, v)) = pair.split_once('=') {
            if k == key {
                return Some(v);
            }
        }
    }
    None
}

fn respond_html(stream: &mut TcpStream, status: u16, body: &str) -> Result<()> {
    let status_text = match status {
        200 => "OK",
        404 => "Not Found",
        _ => "OK",
    };
    let response = format!(
        "HTTP/1.1 {} {}\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
        status,
        status_text,
        body.len(),
        body
    );
    stream.write_all(response.as_bytes())?;
    Ok(())
}

fn render_dashboard_page() -> String {
    // Keep homepage lightweight; detailed listing is on /logs and /sd.
    let ip = crate::wifi::wifi_sta_ipv4().unwrap_or_else(|_| "-".to_string());
    let (sd_state, logs_count) = match crate::sdcard::list_sd_root() {
        Ok(entries) => {
            let logs = entries
                .iter()
                .filter(|e| !e.is_dir && is_log_file(&e.name))
                .count();
            ("OK".to_string(), logs.to_string())
        }
        Err(err) => (format!("Fehler: {}", err), "-".to_string()),
    };

    let remote_status = crate::remote::get_status();
    format!(
        "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
         <title>t-hmi Dashboard</title>\
         <link rel=\"stylesheet\" href=\"https://cdn.jsdelivr.net/npm/picnic@7.1.0/picnic.min.css\">\
         <style>body{{max-width:1100px;margin:0 auto;padding:1rem}} .hero{{padding:.8rem 0}} .grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}} .card{{padding:14px;border:1px solid #ddd;border-radius:8px;background:#fff}} .menu a{{margin-right:.5rem;margin-bottom:.5rem}} .cmd a,.cmd button{{margin:.2rem .2rem .2rem 0}} .inline{{display:inline-block}}</style>\
         </head><body>\
         <nav><a href=\"/\" class=\"brand\">t-hmi</a><div class=\"menu\"><a href=\"/\">Dashboard</a><a href=\"/logs\">Logfiles</a><a href=\"/sd\">SD Karte</a></div></nav>\
         <section class=\"hero\"><h2>System Dashboard</h2><p>Status und Schnellzugriffe</p></section>\
         <section class=\"grid\">\
           <article class=\"card\"><h3>Status</h3><p><b>HTTP:</b> Aktiv</p><p><b>IP:</b> {}</p><p><b>SD:</b> {}</p><p><b>Logfiles:</b> {}</p><p><b>Remote:</b> {}</p></article>\
           <article class=\"card cmd\"><h3>Navigation</h3>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_menu\">Main</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_wifi_menu\">WiFi</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_device_menu\">Device</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_gps\">GPS</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_battery\">Battery</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_loopback\">Loopback</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_http_menu\">HTTP Menu</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=goto_wifi_login\">WiFi Login</a>\
           </article>\
           <article class=\"card cmd\"><h3>Actions</h3>\
             <a class=\"button\" href=\"/api/cmd?cmd=wifi_scan\">WiFi Scan</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=wifi_monitor\">WiFi Monitor</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=wifi_channel_monitor\">Channel Monitor</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=wifi_monitor_stop\">Monitor Stop</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=wifi_connect_default\">WiFi Default</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=toggle_http\">HTTP Start/Stop</a>\
             <a class=\"button\" href=\"/api/cmd?cmd=sd_format\">SD Format</a>\
             <p><a class=\"button\" href=\"/logs\">Logfiles</a> <a class=\"button\" href=\"/sd\">SD Explorer</a></p>\
           </article>\
           <article class=\"card cmd\"><h3>WiFi Connect</h3>\
             <form action=\"/api/cmd\" method=\"get\">\
               <input type=\"hidden\" name=\"cmd\" value=\"wifi_connect\">\
               <label>SSID<input name=\"ssid\" placeholder=\"SSID\"></label>\
               <label>Password<input name=\"password\" placeholder=\"Passwort\"></label>\
               <button type=\"submit\">Connect</button>\
             </form>\
           </article>\
         </section>\
         </body></html>",
        escape_html(&ip),
        escape_html(&sd_state),
        escape_html(&logs_count),
        escape_html(&remote_status)
    )
}

fn enqueue_remote_command(query: &str) -> Result<String> {
    let Some(cmd) = query_param(query, "cmd").map(url_decode_component) else {
        return Err(anyhow::anyhow!("fehlender cmd parameter"));
    };

    let queued = match cmd.as_str() {
        "goto_menu" => crate::remote::RemoteCommand::GotoMenu,
        "goto_wifi_menu" => crate::remote::RemoteCommand::GotoWifiMenu,
        "goto_device_menu" => crate::remote::RemoteCommand::GotoDeviceMenu,
        "goto_gps" => crate::remote::RemoteCommand::GotoGps,
        "goto_battery" => crate::remote::RemoteCommand::GotoBattery,
        "goto_loopback" => crate::remote::RemoteCommand::GotoLoopback,
        "goto_http_menu" => crate::remote::RemoteCommand::GotoHttpMenu,
        "goto_wifi_login" => crate::remote::RemoteCommand::GotoWifiLogin,
        "sd_format" => crate::remote::RemoteCommand::RunSdFormat,
        "toggle_http" => crate::remote::RemoteCommand::ToggleHttp,
        "wifi_scan" => crate::remote::RemoteCommand::WifiScan,
        "wifi_monitor" => crate::remote::RemoteCommand::WifiMonitor,
        "wifi_channel_monitor" => crate::remote::RemoteCommand::WifiChannelMonitor,
        "wifi_monitor_stop" => crate::remote::RemoteCommand::WifiMonitorStop,
        "wifi_connect_default" => crate::remote::RemoteCommand::WifiConnectDefault,
        "wifi_connect" => {
            let ssid = query_param(query, "ssid")
                .map(url_decode_component)
                .unwrap_or_default();
            let password = query_param(query, "password")
                .map(url_decode_component)
                .unwrap_or_default();
            if ssid.trim().is_empty() {
                return Err(anyhow::anyhow!("ssid fehlt"));
            }
            crate::remote::RemoteCommand::WifiConnect { ssid, password }
        }
        _ => return Err(anyhow::anyhow!("unbekannter command: {}", cmd)),
    };

    crate::remote::enqueue(queued)?;
    Ok(format!("queued: {}", cmd))
}

fn render_logs_page() -> String {
    match crate::sdcard::list_sd_root() {
        Ok(entries) => {
            let mut files: Vec<_> = entries
                .into_iter()
                .filter(|e| !e.is_dir && is_log_file(&e.name))
                .collect();
            files.sort_by(|a, b| a.name.cmp(&b.name));

            let mut rows = String::new();
            if files.is_empty() {
                rows.push_str("<tr><td colspan=\"3\">Keine Logfiles gefunden</td></tr>");
            } else {
                for f in files {
                    let encoded_name = url_encode_component(&f.name);
                    rows.push_str(&format!(
                        "<tr><td>{}</td><td>{} Bytes</td><td><a class=\"button small\" href=\"/sd/view?file={}\">Ansehen</a> <a class=\"button small\" href=\"/sd/download?file={}\">Download</a></td></tr>",
                        escape_html(&f.name),
                        f.size,
                        encoded_name,
                        encoded_name
                    ));
                }
            }

            format!(
                "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
                 <title>t-hmi Logfiles</title>\
                 <link rel=\"stylesheet\" href=\"https://cdn.jsdelivr.net/npm/picnic@7.1.0/picnic.min.css\">\
                 <style>body{{max-width:980px;margin:0 auto;padding:1rem}}</style></head><body>\
                 <nav><a href=\"/\" class=\"brand\">t-hmi</a><div class=\"menu\"><a href=\"/\">Dashboard</a><a href=\"/logs\">Logfiles</a><a href=\"/sd\">SD Karte</a></div></nav>\
                 <h2>Logfiles</h2>\
                 <table class=\"primary\"><thead><tr><th>Datei</th><th>Groesse</th><th>Aktion</th></tr></thead><tbody>{}</tbody></table>\
                 </body></html>",
                rows
            )
        }
        Err(err) => format!(
            "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
             <title>t-hmi Logfiles</title>\
             <link rel=\"stylesheet\" href=\"https://cdn.jsdelivr.net/npm/picnic@7.1.0/picnic.min.css\"></head><body>\
             <nav><a href=\"/\" class=\"brand\">t-hmi</a></nav>\
             <h2>SD Fehler</h2><pre>{}</pre><p><a class=\"button\" href=\"/\">Zurueck</a></p></body></html>",
            escape_html(&format!("{}", err))
        ),
    }
}

fn is_log_file(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower.ends_with(".txt") || lower.ends_with(".log") || lower.ends_with(".csv")
}

fn respond_download(stream: &mut TcpStream, file_name: &str, data: &[u8]) -> Result<()> {
    let escaped_file_name = http_header_escape(file_name);
    let header = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Disposition: attachment; filename=\"{}\"\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        escaped_file_name,
        data.len()
    );
    stream.write_all(header.as_bytes())?;
    stream.write_all(data)?;
    Ok(())
}

fn escape_html(input: &str) -> String {
    input
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

fn url_encode_component(input: &str) -> String {
    let mut out = String::new();
    for b in input.bytes() {
        let is_unreserved = b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~');
        if is_unreserved {
            out.push(b as char);
        } else {
            out.push('%');
            out.push_str(&format!("{:02X}", b));
        }
    }
    out
}

fn url_decode_component(input: &str) -> String {
    let bytes = input.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            b'%' if i + 2 < bytes.len() => {
                let hi = bytes[i + 1] as char;
                let lo = bytes[i + 2] as char;
                if let (Some(hi), Some(lo)) = (hi.to_digit(16), lo.to_digit(16)) {
                    out.push(((hi << 4) as u8) | (lo as u8));
                    i += 3;
                } else {
                    out.push(bytes[i]);
                    i += 1;
                }
            }
            b => {
                out.push(b);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn http_header_escape(input: &str) -> String {
    input.replace('\\', "_").replace('"', "_")
}
