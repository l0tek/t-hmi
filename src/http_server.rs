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
        listener.set_nonblocking(true)?;

        let stop = Arc::new(AtomicBool::new(false));
        let stop_thread = Arc::clone(&stop);

        let join = thread::spawn(move || {
            while !stop_thread.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _addr)) => {
                        let _ = handle_client(stream);
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        thread::sleep(Duration::from_millis(30));
                    }
                    Err(_) => {
                        thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        });

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
    let path = parse_path(&request);

    match path {
        "/" => {
            let body = "<html><head><meta charset=\"utf-8\"></head><body><h1>t-hmi Mini HTTP</h1><ul><li><a href=\"/sd\">SD Karte anschauen</a></li></ul></body></html>";
            respond_html(&mut stream, 200, body)?;
        }
        "/sd" => {
            let body = match crate::sdcard::list_sd_root() {
                Ok(entries) => {
                    let mut html = String::from("<html><head><meta charset=\"utf-8\"></head><body><h1>SD Karte</h1><ul>");
                    for e in entries {
                        if e.is_dir {
                            html.push_str(&format!("<li>[DIR] {}</li>", escape_html(&e.name)));
                        } else {
                            html.push_str(&format!(
                                "<li>{} ({} Bytes)</li>",
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

fn escape_html(input: &str) -> String {
    input
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
