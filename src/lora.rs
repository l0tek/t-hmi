use anyhow::Result;
use esp_idf_hal::{
    delay::NON_BLOCK,
    gpio::{AnyIOPin, InputPin, OutputPin},
    peripheral::Peripheral,
    uart::{config::Config as UartConfig, Uart, UartDriver},
    units::Hertz,
};
use esp_idf_sys as sys;

use crate::{
    ui::{clear_screen, draw_header, draw_text_box_small},
    LCD_H_RES,
};

const LORA_DEFAULT_BAUDRATE: u32 = 9_600;
const LORA_BAUD_CANDIDATES: [u32; 1] = [9_600];
const LORA_TX_INTERVAL_MS: u32 = 1_000;
const LORA_BUF_MAX: usize = 160;
const LORA_RECENT_MAX: usize = 32;
const LORA_LINE_W: i32 = LCD_H_RES;
const LORA_LINE_H: i32 = 12;
const LORA_COLOR: embedded_graphics::pixelcolor::Rgb565 =
    embedded_graphics::pixelcolor::Rgb565::new(0, 63, 0);
const LORA_BG: embedded_graphics::pixelcolor::Rgb565 =
    embedded_graphics::pixelcolor::Rgb565::new(0, 0, 0);

pub struct LoRaTester<'d> {
    uart: UartDriver<'d>,
    tx_packets: u32,
    tx_bytes: u32,
    rx_bytes: u32,
    rx_lines: u32,
    rx_non_garbage_bytes: u32,
    last_tx_ms: u32,
    line_buf: Vec<u8>,
    last_line: String,
    recent: Vec<u8>,
    baud_index: usize,
    cmd_last_hex: String,
    ping_seen: u32,
    last_ping_seq: Option<u16>,
}

impl<'d> LoRaTester<'d> {
    pub fn new<UART: Uart>(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self> {
        let cfg = UartConfig::default().baudrate(Hertz(LORA_DEFAULT_BAUDRATE));
        let driver = UartDriver::new(
            uart,
            tx,
            rx,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &cfg,
        )?;

        Ok(Self {
            uart: driver,
            tx_packets: 0,
            tx_bytes: 0,
            rx_bytes: 0,
            rx_lines: 0,
            rx_non_garbage_bytes: 0,
            last_tx_ms: 0,
            line_buf: Vec::with_capacity(LORA_BUF_MAX),
            last_line: String::from("-"),
            recent: Vec::with_capacity(LORA_RECENT_MAX),
            baud_index: 0,
            cmd_last_hex: String::from("-"),
            ping_seen: 0,
            last_ping_seq: None,
        })
    }

    pub fn reset(&mut self) {
        self.tx_packets = 0;
        self.tx_bytes = 0;
        self.rx_bytes = 0;
        self.rx_lines = 0;
        self.rx_non_garbage_bytes = 0;
        self.last_tx_ms = 0;
        self.line_buf.clear();
        self.last_line = String::from("-");
        self.recent.clear();
        self.cmd_last_hex = String::from("-");
        self.ping_seen = 0;
        self.last_ping_seq = None;
        self.baud_index = 0;
        let _ = self
            .uart
            .change_baudrate(Hertz(LORA_BAUD_CANDIDATES[self.baud_index]));
    }

    pub fn poll(&mut self, now_ms: u32) -> Result<bool> {
        let mut changed = false;

        if self.last_tx_ms == 0 || now_ms.wrapping_sub(self.last_tx_ms) >= LORA_TX_INTERVAL_MS {
            let next = self.tx_packets.saturating_add(1);
            let seq = next % 1000;
            let payload = format!("ping #{seq:03}\r\n");
            let written = self.uart.write(payload.as_bytes())?;
            if written > 0 {
                self.tx_packets = next;
                self.tx_bytes = self.tx_bytes.saturating_add(written as u32);
                self.last_tx_ms = now_ms;
                println!("E220 UART accepted {} bytes: ping #{seq:03}", written);
                changed = true;
            }
        }

        let mut buf = [0u8; 64];
        let n = self.uart.read(&mut buf, NON_BLOCK)?;
        if n > 0 {
            self.rx_bytes = self.rx_bytes.saturating_add(n as u32);
            self.recent.extend_from_slice(&buf[..n]);
            if self.recent.len() > LORA_RECENT_MAX {
                let extra = self.recent.len() - LORA_RECENT_MAX;
                self.recent.drain(0..extra);
            }

            for &b in &buf[..n] {
                if b != 0xFF && b != 0xFE {
                    self.rx_non_garbage_bytes = self.rx_non_garbage_bytes.saturating_add(1);
                }
                if b == b'\r' {
                    continue;
                }
                if b == b'\n' {
                    if !self.line_buf.is_empty() {
                        self.rx_lines = self.rx_lines.saturating_add(1);
                        self.last_line = String::from_utf8_lossy(&self.line_buf).to_string();
                        if let Some(seq) = parse_ping_seq(&self.last_line) {
                            self.ping_seen = self.ping_seen.saturating_add(1);
                            self.last_ping_seq = Some(seq);
                        }
                        self.line_buf.clear();
                    }
                    continue;
                }
                if self.line_buf.len() >= LORA_BUF_MAX {
                    self.line_buf.clear();
                }
                self.line_buf.push(b);
            }
            changed = true;
        }

        Ok(changed)
    }

    pub fn tx_packets(&self) -> u32 {
        self.tx_packets
    }

    pub fn tx_bytes(&self) -> u32 {
        self.tx_bytes
    }

    pub fn rx_bytes(&self) -> u32 {
        self.rx_bytes
    }

    pub fn rx_lines(&self) -> u32 {
        self.rx_lines
    }

    pub fn last_line(&self) -> &str {
        &self.last_line
    }

    pub fn recent_hex(&self) -> String {
        if self.recent.is_empty() {
            return "-".to_string();
        }
        format_hex_bytes(&self.recent)
    }

    pub fn current_baud(&self) -> u32 {
        LORA_BAUD_CANDIDATES[self.baud_index]
    }

    pub fn ff_only_alert(&self) -> bool {
        self.rx_bytes >= 6 && self.rx_non_garbage_bytes == 0
    }

    pub fn cmd_probe_ok(&self) -> bool {
        false
    }

    pub fn cmd_probe_hex(&self) -> &str {
        &self.cmd_last_hex
    }

    pub fn cmd_probe_status(&self) -> &'static str {
        "n/a"
    }

    pub fn ping_seen(&self) -> u32 {
        self.ping_seen
    }

    pub fn last_ping_seq(&self) -> Option<u16> {
        self.last_ping_seq
    }
}

fn parse_ping_seq(line: &str) -> Option<u16> {
    let trimmed = line.trim();
    let digits = trimmed.strip_prefix("ping #")?;
    if digits.len() != 3 || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    digits.parse::<u16>().ok()
}

fn format_hex_bytes(data: &[u8]) -> String {
    let mut out = String::new();
    for (i, b) in data.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        out.push_str(&format!("{:02X}", b));
    }
    out
}

fn draw_line(panel: sys::esp_lcd_panel_handle_t, y: i32, text: &str) -> Result<()> {
    draw_text_box_small(
        panel,
        0,
        y,
        LORA_LINE_W,
        LORA_LINE_H,
        text,
        LORA_COLOR,
        LORA_BG,
    )
}

fn truncate_chars(s: &str, max_chars: usize) -> String {
    let count = s.chars().count();
    if count <= max_chars {
        return s.to_string();
    }
    let mut out = String::new();
    for ch in s.chars().take(max_chars.saturating_sub(1)) {
        out.push(ch);
    }
    out.push('~');
    out
}

pub fn draw_lora_test_frame(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, LORA_BG)?;
    draw_header(panel, "LoRa Test", true)?;
    Ok(())
}

pub fn draw_lora_test_values(
    panel: sys::esp_lcd_panel_handle_t,
    lora: &LoRaTester<'_>,
    log_info: &str,
) -> Result<()> {
    draw_line(
        panel,
        36,
        &format!("UART2 RX18 TX17 Baud:{}", lora.current_baud()),
    )?;
    draw_line(panel, 48, "E220 Normal: M1=0 M0=0")?;
    draw_line(panel, 60, "Default: 873.125MHz Air:2.4k")?;
    draw_line(panel, 72, &format!("TX packets: {}", lora.tx_packets()))?;
    draw_line(panel, 84, &format!("TX bytes:   {}", lora.tx_bytes()))?;
    draw_line(panel, 96, &format!("RX bytes:   {}", lora.rx_bytes()))?;
    draw_line(panel, 108, &format!("RX lines:   {}", lora.rx_lines()))?;
    draw_line(
        panel,
        120,
        &format!(
            "RX quality: {}",
            if lora.ff_only_alert() {
                "ERROR only FE/FF"
            } else {
                "OK"
            }
        ),
    )?;
    draw_line(
        panel,
        132,
        &format!("Last line: {}", truncate_chars(lora.last_line(), 24)),
    )?;
    draw_line(
        panel,
        144,
        &format!("RX hex: {}", truncate_chars(&lora.recent_hex(), 22)),
    )?;
    draw_line(
        panel,
        156,
        &format!(
            "CMD {}: {}",
            lora.cmd_probe_status(),
            truncate_chars(lora.cmd_probe_hex(), 20)
        ),
    )?;
    draw_line(panel, 168, "CFG read: M1=1 M0=1, C1 00 08")?;
    draw_line(
        panel,
        180,
        &format!(
            "PING rx:{} last:#{}",
            lora.ping_seen(),
            lora.last_ping_seq()
                .map(|v| format!("{v:03}"))
                .unwrap_or_else(|| "---".to_string())
        ),
    )?;
    draw_line(panel, 192, &format!("SD: {}", truncate_chars(log_info, 30)))?;
    draw_line(panel, 204, "Sendet alle 1s: ping #NNN")?;
    Ok(())
}
