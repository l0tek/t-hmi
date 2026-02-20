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
const HOST_LOG_INTERVAL_MS: u32 = 300;
const HOST_RAW_LOW_BPS: u32 = 40;
const HOST_NO_NMEA_WARN_MS: u32 = 5_000;
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
    last_host_log_ms: u32,
    last_nmea_ms: u32,
    last_raw_rx_ms: u32,
    host_log_enabled: bool,
    has_valid_nmea: bool,
    total_nmea: u32,
    total_non_nmea_lines: u32,
    lb_tx_ok: u32,
    lb_rx_ok: u32,
    lb_rx_err: u32,
    lb_expected: u8,
    lb_next_tx: u8,
    lb_last_send_ms: u32,
    lb_last_rx: Option<u8>,
    raw_recent: Vec<u8>,
    last_utc_unix_secs: Option<u64>,
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
            last_host_log_ms: 0,
            last_nmea_ms: 0,
            last_raw_rx_ms: 0,
            host_log_enabled: false,
            has_valid_nmea: false,
            total_nmea: 0,
            total_non_nmea_lines: 0,
            lb_tx_ok: 0,
            lb_rx_ok: 0,
            lb_rx_err: 0,
            lb_expected: 0x41,
            lb_next_tx: 0x41,
            lb_last_send_ms: 0,
            lb_last_rx: None,
            raw_recent: Vec::with_capacity(RAW_RECENT_MAX),
            last_utc_unix_secs: None,
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
            self.last_raw_rx_ms = now_ms;
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
                        let is_nmea = self.handle_sentence(&line);
                        if is_nmea {
                            self.total_nmea = self.total_nmea.saturating_add(1);
                            self.last_nmea_ms = now_ms;
                        } else {
                            self.total_non_nmea_lines = self.total_non_nmea_lines.saturating_add(1);
                        }
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

        if self.host_log_enabled
            && (self.last_host_log_ms == 0
                || now_ms.wrapping_sub(self.last_host_log_ms) >= HOST_LOG_INTERVAL_MS)
        {
            self.print_host_diagnostics(now_ms);
            self.last_host_log_ms = now_ms;
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

    fn handle_sentence(&mut self, sentence: &str) -> bool {
        if !sentence.starts_with('$') {
            return false;
        }

        let Some((sentence_wo_checksum, checksum_ok)) = split_nmea_checksum(sentence) else {
            return false;
        };
        if !checksum_ok {
            return false;
        }

        let parts: Vec<&str> = sentence_wo_checksum.split(',').collect();
        if parts.is_empty() {
            return false;
        }

        self.has_valid_nmea = true;
        self.last_sentence.clear();
        self.last_sentence.push_str(sentence);

        match parts[0] {
            "$GPRMC" | "$GNRMC" => {
                if let Some(status) = parts.get(2) {
                    if *status == "A" {
                        self.fix = true;
                    } else if *status == "V" {
                        self.fix = false;
                    }
                }
                if let Some(lat) = parse_lat_lon(parts.get(3), parts.get(4), 2) {
                    self.lat = Some(lat);
                }
                if let Some(lon) = parse_lat_lon(parts.get(5), parts.get(6), 3) {
                    self.lon = Some(lon);
                }
                if let Some(unix) = parse_rmc_utc_unix(&parts) {
                    self.last_utc_unix_secs = Some(unix);
                }
            }
            "$GPGGA" | "$GNGGA" => {
                if let Some(fix_quality) = parts.get(6) {
                    if let Ok(q) = fix_quality.parse::<u8>() {
                        self.fix = q > 0;
                    }
                }
                if let Some(sats) = parts.get(7).and_then(|v| v.parse::<u8>().ok()) {
                    self.sats = Some(sats);
                }
                if let Some(lat) = parse_lat_lon(parts.get(2), parts.get(3), 2) {
                    self.lat = Some(lat);
                }
                if let Some(lon) = parse_lat_lon(parts.get(4), parts.get(5), 3) {
                    self.lon = Some(lon);
                }
            }
            "$GPGLL" | "$GNGLL" => {
                if let Some(lat) = parse_lat_lon(parts.get(1), parts.get(2), 2) {
                    self.lat = Some(lat);
                }
                if let Some(lon) = parse_lat_lon(parts.get(3), parts.get(4), 3) {
                    self.lon = Some(lon);
                }
                if let Some(status) = parts.get(6) {
                    if *status == "A" {
                        self.fix = true;
                    } else if *status == "V" {
                        self.fix = false;
                    }
                }
            }
            "$GPZDA" | "$GNZDA" => {
                if let Some(unix) = parse_zda_utc_unix(&parts) {
                    self.last_utc_unix_secs = Some(unix);
                }
            }
            _ => {}
        }
        true
    }

    fn print_host_diagnostics(&self, now_ms: u32) {
        let mode = if self.auto_scan_active() {
            "scan"
        } else {
            "lock"
        };
        let sats = self
            .sats()
            .map(|v| v.to_string())
            .unwrap_or_else(|| "-".to_string());
        let lat = self
            .lat()
            .map(|v| format!("{:.6}", v))
            .unwrap_or_else(|| "-".to_string());
        let lon = self
            .lon()
            .map(|v| format!("{:.6}", v))
            .unwrap_or_else(|| "-".to_string());

        let mut analysis = Vec::<&str>::new();
        if self.auto_scan_active()
            && now_ms.wrapping_sub(self.last_nmea_ms.max(self.last_baud_switch_ms))
                >= HOST_NO_NMEA_WARN_MS
        {
            analysis.push("NO_NMEA_LOCK");
        }
        if now_ms.wrapping_sub(self.last_raw_rx_ms) >= 2_000 {
            analysis.push("NO_UART_RX");
        } else if self.raw_bytes_per_sec < HOST_RAW_LOW_BPS {
            analysis.push("LOW_RAW_BPS");
        }
        if self.has_valid_nmea && !self.fix {
            analysis.push("NMEA_NO_FIX");
        }
        if self.fix && self.sats.unwrap_or(0) < 4 {
            analysis.push("LOW_SATS");
        }
        if self.fix && (self.lat.is_none() || self.lon.is_none()) {
            analysis.push("NO_COORDS");
        }
        let analysis = if analysis.is_empty() {
            "OK".to_string()
        } else {
            analysis.join("|")
        };

        let last = self.last_sentence.chars().take(28).collect::<String>();
        println!(
            "[GPS] baud={} mode={} raw_bps={} fix={} sats={} lat={} lon={} nmea={} non_nmea={} last=\"{}\" analysis={}",
            self.current_baud(),
            mode,
            self.raw_bytes_per_sec(),
            if self.fix() { "Y" } else { "N" },
            sats,
            lat,
            lon,
            self.total_nmea,
            self.total_non_nmea_lines,
            last,
            analysis
        );
    }

    pub fn fix(&self) -> bool {
        self.fix
    }

    pub fn set_host_log_enabled(&mut self, enabled: bool) {
        self.host_log_enabled = enabled;
        if enabled {
            self.last_host_log_ms = 0;
        }
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

    pub fn current_utc_unix_secs(&self) -> Option<u64> {
        self.last_utc_unix_secs
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
    if hemi != "N" && hemi != "S" && hemi != "E" && hemi != "W" {
        return None;
    }
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

fn split_nmea_checksum(sentence: &str) -> Option<(&str, bool)> {
    let star = sentence.rfind('*')?;
    if star + 3 != sentence.len() {
        return None;
    }
    let payload = &sentence[..star];
    let expected = u8::from_str_radix(&sentence[star + 1..], 16).ok()?;
    let calc = payload
        .as_bytes()
        .iter()
        .skip(1) // skip '$'
        .fold(0u8, |acc, b| acc ^ b);
    Some((payload, calc == expected))
}

fn parse_rmc_utc_unix(parts: &[&str]) -> Option<u64> {
    let time = *parts.get(1)?;
    let date = *parts.get(9)?;
    if date.len() < 6 {
        return None;
    }
    let day = date.get(0..2)?.parse::<u32>().ok()?;
    let month = date.get(2..4)?.parse::<u32>().ok()?;
    let yy = date.get(4..6)?.parse::<u32>().ok()?;
    let year = if yy >= 80 {
        1900 + yy as i32
    } else {
        2000 + yy as i32
    };
    let (hour, minute, second) = parse_nmea_hms(time)?;
    unix_from_utc_ymdhms(year, month, day, hour, minute, second)
}

fn parse_zda_utc_unix(parts: &[&str]) -> Option<u64> {
    let time = *parts.get(1)?;
    let day = parts.get(2)?.parse::<u32>().ok()?;
    let month = parts.get(3)?.parse::<u32>().ok()?;
    let year = parts.get(4)?.parse::<i32>().ok()?;
    let (hour, minute, second) = parse_nmea_hms(time)?;
    unix_from_utc_ymdhms(year, month, day, hour, minute, second)
}

fn parse_nmea_hms(raw: &str) -> Option<(u32, u32, u32)> {
    let dot = raw.find('.').unwrap_or(raw.len());
    if dot < 6 {
        return None;
    }
    let hh = raw.get(0..2)?.parse::<u32>().ok()?;
    let mm = raw.get(2..4)?.parse::<u32>().ok()?;
    let ss = raw.get(4..6)?.parse::<u32>().ok()?;
    if hh > 23 || mm > 59 || ss > 59 {
        return None;
    }
    Some((hh, mm, ss))
}

fn unix_from_utc_ymdhms(
    year: i32,
    month: u32,
    day: u32,
    hour: u32,
    minute: u32,
    second: u32,
) -> Option<u64> {
    if !(1..=12).contains(&month) || !(1..=31).contains(&day) || year < 1970 {
        return None;
    }
    let days = days_from_civil(year, month, day)?;
    Some(
        (days as u64)
            .saturating_mul(86_400)
            .saturating_add((hour as u64) * 3_600)
            .saturating_add((minute as u64) * 60)
            .saturating_add(second as u64),
    )
}

fn days_from_civil(year: i32, month: u32, day: u32) -> Option<i64> {
    let y = year as i64 - if month <= 2 { 1 } else { 0 };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let m = month as i64;
    let d = day as i64;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + d - 1;
    if !(0..=365).contains(&doy) {
        return None;
    }
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era * 146_097 + doe - 719_468)
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
