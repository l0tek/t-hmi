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

const GPS_BAUDRATE: u32 = 9_600;
const GPS_BAUD_CANDIDATES: [u32; 3] = [9_600, 38_400, 115_200];
const GPS_BAUD_SCAN_INTERVAL_MS: u32 = 1_800;
const NMEA_BUF_MAX: usize = 160;
const RAW_RECENT_MAX: usize = 24;
const GPS_LINE_W: i32 = LCD_H_RES;
const GPS_LINE_H: i32 = 12;
const GPS_COLOR: embedded_graphics::pixelcolor::Rgb565 =
    embedded_graphics::pixelcolor::Rgb565::new(0, 63, 0);
const GPS_BG: embedded_graphics::pixelcolor::Rgb565 =
    embedded_graphics::pixelcolor::Rgb565::new(0, 0, 0);

pub struct GpsReader<'d> {
    uart: UartDriver<'d>,
    line_buf: Vec<u8>,
    last_sentence: String,
    fix: bool,
    sats: Option<u8>,
    lat: Option<f32>,
    lon: Option<f32>,
    baud_index: usize,
    raw_bytes_acc: u32,
    raw_bytes_per_sec: u32,
    raw_window_start_ms: u32,
    last_baud_switch_ms: u32,
    has_valid_nmea: bool,
    lb_tx_ok: u32,
    lb_rx_ok: u32,
    lb_rx_err: u32,
    lb_expected: u8,
    lb_next_tx: u8,
    lb_last_send_ms: u32,
    lb_last_rx: Option<u8>,
    raw_recent: Vec<u8>,
}

impl<'d> GpsReader<'d> {
    pub fn new<UART: Uart>(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self> {
        let cfg = UartConfig::default().baudrate(Hertz(GPS_BAUDRATE));
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
            line_buf: Vec::with_capacity(NMEA_BUF_MAX),
            last_sentence: String::from("waiting for NMEA..."),
            fix: false,
            sats: None,
            lat: None,
            lon: None,
            baud_index: 0,
            raw_bytes_acc: 0,
            raw_bytes_per_sec: 0,
            raw_window_start_ms: 0,
            last_baud_switch_ms: 0,
            has_valid_nmea: false,
            lb_tx_ok: 0,
            lb_rx_ok: 0,
            lb_rx_err: 0,
            lb_expected: 0x41,
            lb_next_tx: 0x41,
            lb_last_send_ms: 0,
            lb_last_rx: None,
            raw_recent: Vec::with_capacity(RAW_RECENT_MAX),
        })
    }

    pub fn poll(&mut self, now_ms: u32) -> Result<bool> {
        if self.raw_window_start_ms == 0 {
            self.raw_window_start_ms = now_ms;
        }
        if self.last_baud_switch_ms == 0 {
            self.last_baud_switch_ms = now_ms;
        }

        let mut tmp = [0u8; 64];
        let read = self.uart.read(&mut tmp, NON_BLOCK)?;
        if read > 0 {
            self.raw_bytes_acc = self.raw_bytes_acc.saturating_add(read as u32);
            self.raw_recent.extend_from_slice(&tmp[..read]);
            if self.raw_recent.len() > RAW_RECENT_MAX {
                let extra = self.raw_recent.len() - RAW_RECENT_MAX;
                self.raw_recent.drain(0..extra);
            }
        }

        let mut changed = false;
        if read > 0 {
            for &b in &tmp[..read] {
                if b == b'\r' {
                    continue;
                }
                if b == b'\n' {
                    if !self.line_buf.is_empty() {
                        let line = String::from_utf8_lossy(&self.line_buf).to_string();
                        let was_valid = self.has_valid_nmea;
                        self.handle_sentence(&line);
                        self.line_buf.clear();
                        if self.has_valid_nmea && !was_valid {
                            self.last_sentence =
                                format!("NMEA lock @ {} baud", self.current_baud());
                        }
                        changed = true;
                    }
                    continue;
                }
                if self.line_buf.len() >= NMEA_BUF_MAX {
                    self.line_buf.clear();
                }
                self.line_buf.push(b);
            }
        }

        let raw_elapsed = now_ms.wrapping_sub(self.raw_window_start_ms);
        if raw_elapsed >= 1_000 {
            self.raw_bytes_per_sec = if raw_elapsed > 0 {
                (self.raw_bytes_acc.saturating_mul(1_000)) / raw_elapsed
            } else {
                0
            };
            self.raw_bytes_acc = 0;
            self.raw_window_start_ms = now_ms;
            changed = true;
        }

        if !self.has_valid_nmea
            && now_ms.wrapping_sub(self.last_baud_switch_ms) >= GPS_BAUD_SCAN_INTERVAL_MS
        {
            self.switch_to_next_baud()?;
            self.last_baud_switch_ms = now_ms;
            changed = true;
        }

        Ok(changed)
    }

    fn switch_to_next_baud(&mut self) -> Result<()> {
        self.baud_index = (self.baud_index + 1) % GPS_BAUD_CANDIDATES.len();
        let baud = self.current_baud();
        self.uart.change_baudrate(Hertz(baud))?;
        self.line_buf.clear();
        self.last_sentence = format!("Auto-scan: trying {} baud", baud);
        Ok(())
    }

    fn handle_sentence(&mut self, sentence: &str) {
        if !sentence.starts_with('$') {
            return;
        }

        self.has_valid_nmea = true;
        self.last_sentence.clear();
        self.last_sentence.push_str(sentence);

        let sentence_wo_checksum = sentence.split('*').next().unwrap_or(sentence);
        let parts: Vec<&str> = sentence_wo_checksum.split(',').collect();
        if parts.is_empty() {
            return;
        }

        match parts[0] {
            "$GPRMC" | "$GNRMC" => {
                if let Some(status) = parts.get(2) {
                    self.fix = *status == "A";
                }
                self.lat = parse_lat_lon(parts.get(3), parts.get(4), 2);
                self.lon = parse_lat_lon(parts.get(5), parts.get(6), 3);
            }
            "$GPGGA" | "$GNGGA" => {
                if let Some(fix_quality) = parts.get(6) {
                    self.fix = fix_quality.parse::<u8>().ok().unwrap_or(0) > 0;
                }
                self.sats = parts.get(7).and_then(|v| v.parse::<u8>().ok());
                self.lat = parse_lat_lon(parts.get(2), parts.get(3), 2);
                self.lon = parse_lat_lon(parts.get(4), parts.get(5), 3);
            }
            _ => {}
        }
    }

    pub fn fix(&self) -> bool {
        self.fix
    }

    pub fn sats(&self) -> Option<u8> {
        self.sats
    }

    pub fn lat(&self) -> Option<f32> {
        self.lat
    }

    pub fn lon(&self) -> Option<f32> {
        self.lon
    }

    pub fn last_sentence(&self) -> &str {
        &self.last_sentence
    }

    pub fn current_baud(&self) -> u32 {
        GPS_BAUD_CANDIDATES[self.baud_index]
    }

    pub fn auto_scan_active(&self) -> bool {
        !self.has_valid_nmea
    }

    pub fn raw_bytes_per_sec(&self) -> u32 {
        self.raw_bytes_per_sec
    }

    pub fn raw_recent_hex(&self) -> String {
        if self.raw_recent.is_empty() {
            return "-".to_string();
        }
        let mut out = String::new();
        for (i, b) in self.raw_recent.iter().enumerate() {
            if i > 0 {
                out.push(' ');
            }
            out.push_str(&format!("{:02X}", b));
        }
        out
    }

    pub fn raw_recent_ascii(&self) -> String {
        if self.raw_recent.is_empty() {
            return "-".to_string();
        }
        self.raw_recent
            .iter()
            .map(|b| match *b {
                32..=126 => *b as char,
                b'\r' => 'r',
                b'\n' => 'n',
                _ => '.',
            })
            .collect()
    }

    pub fn loopback_reset(&mut self) {
        self.lb_tx_ok = 0;
        self.lb_rx_ok = 0;
        self.lb_rx_err = 0;
        self.lb_expected = 0x41;
        self.lb_next_tx = 0x41;
        self.lb_last_send_ms = 0;
        self.lb_last_rx = None;
        self.line_buf.clear();
        self.last_sentence = String::from("Loopback ready");
    }

    pub fn loopback_poll(&mut self, now_ms: u32) -> Result<bool> {
        let mut changed = false;
        if self.lb_last_send_ms == 0 || now_ms.wrapping_sub(self.lb_last_send_ms) >= 100 {
            let b = [self.lb_next_tx];
            let written = self.uart.write(&b)?;
            if written > 0 {
                self.lb_tx_ok = self.lb_tx_ok.saturating_add(written as u32);
                self.lb_last_send_ms = now_ms;
                self.lb_next_tx = next_pattern_byte(self.lb_next_tx);
                changed = true;
            }
        }

        let mut buf = [0u8; 64];
        let n = self.uart.read(&mut buf, NON_BLOCK)?;
        if n > 0 {
            for &b in &buf[..n] {
                self.lb_last_rx = Some(b);
                if b == self.lb_expected {
                    self.lb_rx_ok = self.lb_rx_ok.saturating_add(1);
                    self.lb_expected = next_pattern_byte(self.lb_expected);
                } else {
                    self.lb_rx_err = self.lb_rx_err.saturating_add(1);
                    self.lb_expected = next_pattern_byte(b);
                }
            }
            changed = true;
        }

        Ok(changed)
    }

    pub fn loopback_tx_ok(&self) -> u32 {
        self.lb_tx_ok
    }

    pub fn loopback_rx_ok(&self) -> u32 {
        self.lb_rx_ok
    }

    pub fn loopback_rx_err(&self) -> u32 {
        self.lb_rx_err
    }

    pub fn loopback_last_rx(&self) -> Option<u8> {
        self.lb_last_rx
    }
}

fn next_pattern_byte(v: u8) -> u8 {
    if v >= 0x5A {
        0x41
    } else {
        v + 1
    }
}

fn parse_lat_lon(raw: Option<&&str>, hemi: Option<&&str>, degree_digits: usize) -> Option<f32> {
    let raw = *raw?;
    let hemi = *hemi?;
    if raw.len() < degree_digits + 3 {
        return None;
    }
    let (deg_s, min_s) = raw.split_at(degree_digits);
    let deg = deg_s.parse::<f32>().ok()?;
    let min = min_s.parse::<f32>().ok()?;
    let mut out = deg + (min / 60.0);
    if hemi == "S" || hemi == "W" {
        out = -out;
    }
    Some(out)
}

fn draw_line(panel: sys::esp_lcd_panel_handle_t, y: i32, text: &str) -> Result<()> {
    draw_text_box_small(panel, 0, y, GPS_LINE_W, GPS_LINE_H, text, GPS_COLOR, GPS_BG)
}

pub fn draw_gps_frame(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, GPS_BG)?;
    draw_header(panel, "GPS", true)?;
    Ok(())
}

pub fn draw_gps_values(panel: sys::esp_lcd_panel_handle_t, gps: &GpsReader<'_>) -> Result<()> {
    draw_line(
        panel,
        36,
        &format!(
            "UART1 RX15 TX16 Baud:{} {}",
            gps.current_baud(),
            if gps.auto_scan_active() {
                "scan"
            } else {
                "lock"
            }
        ),
    )?;
    draw_line(
        panel,
        48,
        &format!("RAW bytes/s: {}", gps.raw_bytes_per_sec()),
    )?;
    draw_line(panel, 60, if gps.fix() { "Fix: YES" } else { "Fix: NO" })?;
    draw_line(
        panel,
        72,
        &format!(
            "Satellites: {}",
            gps.sats()
                .map(|v| v.to_string())
                .unwrap_or_else(|| "-".to_string())
        ),
    )?;
    draw_line(
        panel,
        84,
        &format!(
            "Lat: {}",
            gps.lat()
                .map(|v| format!("{:.6}", v))
                .unwrap_or_else(|| "-".to_string())
        ),
    )?;
    draw_line(
        panel,
        96,
        &format!(
            "Lon: {}",
            gps.lon()
                .map(|v| format!("{:.6}", v))
                .unwrap_or_else(|| "-".to_string())
        ),
    )?;
    draw_line(panel, 112, "Last NMEA:")?;

    let nmea = gps.last_sentence();
    let nmea_a = nmea.get(0..nmea.len().min(38)).unwrap_or("");
    let nmea_b = if nmea.len() > 38 {
        nmea.get(38..nmea.len().min(76)).unwrap_or("")
    } else {
        ""
    };
    let nmea_c = if nmea.len() > 76 {
        nmea.get(76..nmea.len().min(114)).unwrap_or("")
    } else {
        ""
    };

    draw_line(panel, 124, nmea_a)?;
    draw_line(panel, 136, nmea_b)?;
    draw_line(panel, 148, nmea_c)?;
    draw_line(panel, 168, "RAW HEX:")?;
    let raw_hex = gps.raw_recent_hex();
    let raw_hex_a = raw_hex.get(0..raw_hex.len().min(38)).unwrap_or("");
    let raw_hex_b = if raw_hex.len() > 38 {
        raw_hex.get(38..raw_hex.len().min(76)).unwrap_or("")
    } else {
        ""
    };
    draw_line(panel, 180, raw_hex_a)?;
    draw_line(panel, 192, raw_hex_b)?;
    draw_line(panel, 208, "RAW ASCII:")?;
    let raw_ascii = gps.raw_recent_ascii();
    let raw_ascii_a = raw_ascii.get(0..raw_ascii.len().min(38)).unwrap_or("");
    let raw_ascii_b = if raw_ascii.len() > 38 {
        raw_ascii.get(38..raw_ascii.len().min(76)).unwrap_or("")
    } else {
        ""
    };
    draw_line(panel, 220, raw_ascii_a)?;
    draw_line(panel, 232, raw_ascii_b)?;

    Ok(())
}

pub fn draw_gps_screen(panel: sys::esp_lcd_panel_handle_t, gps: &GpsReader<'_>) -> Result<()> {
    draw_gps_frame(panel)?;
    draw_gps_values(panel, gps)
}

pub fn draw_uart_loopback_frame(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, GPS_BG)?;
    draw_header(panel, "UART Loopback", true)?;
    Ok(())
}

pub fn draw_uart_loopback_values(
    panel: sys::esp_lcd_panel_handle_t,
    gps: &GpsReader<'_>,
) -> Result<()> {
    draw_line(panel, 36, "Wire bridge required: IO16 -> IO15")?;
    draw_line(panel, 48, "UART1 TX=IO16 RX=IO15")?;
    draw_line(panel, 64, &format!("TX sent: {}", gps.loopback_tx_ok()))?;
    draw_line(panel, 76, &format!("RX ok:   {}", gps.loopback_rx_ok()))?;
    draw_line(panel, 88, &format!("RX err:  {}", gps.loopback_rx_err()))?;
    draw_line(
        panel,
        104,
        &format!(
            "Last RX: {}",
            gps.loopback_last_rx()
                .map(|v| format!("0x{:02X}", v))
                .unwrap_or_else(|| "-".to_string())
        ),
    )?;
    draw_line(
        panel,
        124,
        if gps.loopback_rx_ok() > 0 && gps.loopback_rx_err() == 0 {
            "Status: OK (echo data received)"
        } else if gps.loopback_rx_ok() > 0 {
            "Status: DATA with mismatches"
        } else {
            "Status: NO ECHO"
        },
    )?;
    Ok(())
}

pub fn draw_uart_loopback_screen(
    panel: sys::esp_lcd_panel_handle_t,
    gps: &GpsReader<'_>,
) -> Result<()> {
    draw_uart_loopback_frame(panel)?;
    draw_uart_loopback_values(panel, gps)
}
