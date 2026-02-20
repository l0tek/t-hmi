use anyhow::Result;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::Text,
};
use esp_idf_sys as sys;

use crate::{FrameBuffer, LCD_H_RES, LCD_V_RES};

pub const MENU_BTN_W: i32 = 180;
pub const MENU_BTN_H: i32 = 28;
pub const MENU_BTN_X: i32 = 20;
pub const MENU_BTN1_Y: i32 = 40;
pub const MENU_BTN2_Y: i32 = 80;
pub const BACK_BTN_W: i32 = 70;
pub const BACK_BTN_H: i32 = 22;
pub const BACK_BTN_X: i32 = 10;
pub const BACK_BTN_Y: i32 = 4;
pub const HEADER_H: i32 = 30;
pub const WIFI_MENU_BTN1_Y: i32 = 60;
pub const WIFI_MENU_BTN2_Y: i32 = 100;
pub const WIFI_MENU_BTN3_Y: i32 = 140;
pub const DEVICE_MENU_BTN1_Y: i32 = 60;
pub const DEVICE_MENU_BTN2_Y: i32 = 100;
pub const DEVICE_MENU_BTN3_Y: i32 = 140;
pub const DEVICE_MENU_BTN4_Y: i32 = 180;
pub const DEVICE_MENU_BTN5_Y: i32 = 220;
pub const DEVICE_MENU_BTN6_Y: i32 = 260;
pub const HTTP_MENU_BTN1_Y: i32 = 80;
pub const WIFI_CH_BTN_W: i32 = 28;
pub const WIFI_CH_BTN_H: i32 = 22;
pub const WIFI_CH_BTN_Y: i32 = LCD_V_RES - 28;
pub const WIFI_CH_BTN_LEFT_X: i32 = 10;
pub const WIFI_CH_BTN_RIGHT_X: i32 = LCD_H_RES - 10 - WIFI_CH_BTN_W;

const MONITOR_GRAPH_X: i32 = 10;
const MONITOR_GRAPH_Y: i32 = 40;
const MONITOR_GRAPH_W: i32 = 220;
const MONITOR_GRAPH_H: i32 = 180;
const CHANNEL_BAR_X: i32 = 10;
const CHANNEL_BAR_Y: i32 = 40;
const CHANNEL_BAR_W: i32 = 220;
const CHANNEL_BAR_H: i32 = 170;

const ICON_W: i32 = 16;
const ICON_H: i32 = 16;

const ICON_GPS: [u16; 16] = [
    0b0000000000000000,
    0b0000001110000000,
    0b0000011111000000,
    0b0000111111100000,
    0b0001110001110000,
    0b0001100000110000,
    0b0001100010110000,
    0b0001100000110000,
    0b0001110001110000,
    0b0000111111100000,
    0b0000011111000000,
    0b0000001110000000,
    0b0000000100000000,
    0b0000000100000000,
    0b0000001000000000,
    0b0000010000000000,
];

const ICON_DEVICE: [u16; 16] = [
    0b0000000000000000,
    0b0000011111100000,
    0b0001111111111000,
    0b0010000000000100,
    0b0010111111100100,
    0b0010100000100100,
    0b0010101110100100,
    0b0010101110100100,
    0b0010100000100100,
    0b0010111111100100,
    0b0010000000000100,
    0b0001111111111000,
    0b0000011111100000,
    0b0000001000010000,
    0b0000000000000000,
    0b0000000000000000,
];

const ICON_REBOOT: [u16; 16] = [
    0b0000000111010000,
    0b0000001111110000,
    0b0000011101111000,
    0b0000111000111000,
    0b0001110000011100,
    0b0011100000001110,
    0b0011100000001110,
    0b0011100000001110,
    0b0001110000011100,
    0b0000111000111000,
    0b0000011101110000,
    0b0000001111100000,
    0b0000000111000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
];

const ICON_BATTERY: [u16; 16] = [
    0b0000000000000000,
    0b0000001111110000,
    0b0000010000010000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000110000011000,
    0b0000010000010000,
    0b0000001111110000,
    0b0000000000000000,
    0b0000000000000000,
];

pub struct BatteryLabels {
    pub y_vbat: i32,
    pub y_adc: i32,
    pub y_raw: i32,
    pub y_soc: i32,
    pub y_cal: i32,
}

pub fn draw_bitmap(
    panel: sys::esp_lcd_panel_handle_t,
    x: i32,
    y: i32,
    w: i32,
    h: i32,
    buffer: &[u16],
) -> Result<()> {
    let x_end = x + w;
    let y_end = y + h;
    crate::esp_ok(unsafe {
        sys::esp_lcd_panel_draw_bitmap(panel, x, y, x_end, y_end, buffer.as_ptr() as *const _)
    })
}

pub fn clear_screen(panel: sys::esp_lcd_panel_handle_t, color: Rgb565) -> Result<()> {
    let buffer = vec![color.into_storage(); (LCD_H_RES * LCD_V_RES) as usize];
    draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, &buffer)
}

pub fn draw_text_box(
    panel: sys::esp_lcd_panel_handle_t,
    x: i32,
    y: i32,
    w: i32,
    h: i32,
    text: &str,
    fg: Rgb565,
    bg: Rgb565,
) -> Result<()> {
    let mut buffer = vec![bg.into_storage(); (w * h) as usize];
    let mut fb = FrameBuffer {
        width: w,
        height: h,
        data: &mut buffer,
    };
    fb.clear(bg)?;

    let style = MonoTextStyle::new(&FONT_10X20, fg);
    Text::new(text, Point::new(10, 20), style).draw(&mut fb)?;

    draw_bitmap(panel, x, y, w, h, &buffer)
}

pub fn draw_text_box_small(
    panel: sys::esp_lcd_panel_handle_t,
    x: i32,
    y: i32,
    w: i32,
    h: i32,
    text: &str,
    fg: Rgb565,
    bg: Rgb565,
) -> Result<()> {
    let mut buffer = vec![bg.into_storage(); (w * h) as usize];
    let mut fb = FrameBuffer {
        width: w,
        height: h,
        data: &mut buffer,
    };
    fb.clear(bg)?;

    let style = MonoTextStyle::new(&FONT_6X10, fg);
    Text::new(text, Point::new(6, 11), style).draw(&mut fb)?;

    draw_bitmap(panel, x, y, w, h, &buffer)
}

pub fn draw_rect_box(
    panel: sys::esp_lcd_panel_handle_t,
    x: i32,
    y: i32,
    w: i32,
    h: i32,
    stroke: u32,
    fg: Rgb565,
    bg: Rgb565,
    text: Option<&str>,
) -> Result<()> {
    let mut buffer = vec![bg.into_storage(); (w * h) as usize];
    let mut fb = FrameBuffer {
        width: w,
        height: h,
        data: &mut buffer,
    };
    fb.clear(bg)?;

    let style = PrimitiveStyle::with_stroke(fg, stroke);
    Rectangle::new(Point::new(0, 0), Size::new(w as u32, h as u32))
        .into_styled(style)
        .draw(&mut fb)?;

    if let Some(label) = text {
        let text_w = (label.len() as i32) * 10;
        let tx = (w - text_w).max(0) / 2;
        let ty = (h / 2) + 6;
        let style = MonoTextStyle::new(&FONT_10X20, fg);
        Text::new(label, Point::new(tx, ty), style).draw(&mut fb)?;
    }

    draw_bitmap(panel, x, y, w, h, &buffer)
}

pub fn draw_header(panel: sys::esp_lcd_panel_handle_t, title: &str, show_back: bool) -> Result<()> {
    let header_bg = Rgb565::new(24, 48, 24);
    let header_fg = Rgb565::BLACK;
    let header_border = Rgb565::BLACK;
    draw_rect_box(
        panel,
        0,
        0,
        LCD_H_RES,
        HEADER_H,
        2,
        header_border,
        header_bg,
        None,
    )?;
    let title_x = if show_back {
        draw_rect_box(
            panel,
            BACK_BTN_X,
            BACK_BTN_Y,
            BACK_BTN_W,
            BACK_BTN_H,
            2,
            header_border,
            header_bg,
            Some("< Back"),
        )?;
        BACK_BTN_X + BACK_BTN_W + 6
    } else {
        6
    };
    let title_w = (LCD_H_RES - title_x - 1).max(0);
    let title_y = 3;
    let title_h = 24;
    draw_text_box(
        panel, title_x, title_y, title_w, title_h, title, header_fg, header_bg,
    )?;
    Ok(())
}

pub fn draw_header_with_icon(
    panel: sys::esp_lcd_panel_handle_t,
    title: &str,
    show_back: bool,
    icon: &[u16; 16],
) -> Result<()> {
    draw_header(panel, title, show_back)?;
    let icon_x = LCD_H_RES - ICON_W - 6;
    let icon_y = 7;
    let mut buffer = vec![Rgb565::new(24, 48, 24).into_storage(); (ICON_W * ICON_H) as usize];
    let mut fb = FrameBuffer {
        width: ICON_W,
        height: ICON_H,
        data: &mut buffer,
    };
    draw_icon_fb(&mut fb, 0, 0, Rgb565::BLACK, icon)?;
    draw_bitmap(panel, icon_x, icon_y, ICON_W, ICON_H, &buffer)
}

fn draw_icon_fb(fb: &mut FrameBuffer, x: i32, y: i32, fg: Rgb565, rows: &[u16]) -> Result<()> {
    for (yy, row) in rows.iter().enumerate() {
        for xx in 0..ICON_W {
            let bit = 1u16 << (ICON_W - 1 - xx);
            if (row & bit) != 0 {
                let px = x + xx;
                let py = y + yy as i32;
                if px < 0 || py < 0 || px >= fb.width || py >= fb.height {
                    continue;
                }
                let idx = (py as usize * fb.width as usize) + px as usize;
                fb.data[idx] = fg.into_storage();
            }
        }
    }
    Ok(())
}

fn draw_wifi_icon(fb: &mut FrameBuffer, x: i32, y: i32, fg: Rgb565, bg: Rgb565) -> Result<()> {
    let stroke = PrimitiveStyle::with_stroke(fg, 1);
    let fill_fg = PrimitiveStyle::with_fill(fg);
    let fill_bg = PrimitiveStyle::with_fill(bg);

    let cx = x + (ICON_W / 2);
    let cy = y + 11;

    Circle::new(Point::new(cx - 6, cy - 6), 13)
        .into_styled(stroke)
        .draw(fb)?;
    Circle::new(Point::new(cx - 4, cy - 4), 9)
        .into_styled(stroke)
        .draw(fb)?;
    Circle::new(Point::new(cx - 2, cy - 2), 5)
        .into_styled(stroke)
        .draw(fb)?;

    Rectangle::new(
        Point::new(x, cy),
        Size::new(ICON_W as u32, (ICON_H - (cy - y)) as u32),
    )
    .into_styled(fill_bg)
    .draw(fb)?;

    Circle::new(Point::new(cx - 1, y + 12), 3)
        .into_styled(fill_fg)
        .draw(fb)?;
    Ok(())
}

fn draw_bt_icon(fb: &mut FrameBuffer, x: i32, y: i32, fg: Rgb565) -> Result<()> {
    let stroke = PrimitiveStyle::with_stroke(fg, 1);
    let stem_x = x + 7;

    Line::new(Point::new(stem_x, y + 1), Point::new(stem_x, y + 14))
        .into_styled(stroke)
        .draw(fb)?;
    Line::new(
        Point::new(stem_x + 1, y + 1),
        Point::new(stem_x + 1, y + 14),
    )
    .into_styled(stroke)
    .draw(fb)?;

    Line::new(Point::new(stem_x + 1, y + 2), Point::new(x + 12, y + 5))
        .into_styled(stroke)
        .draw(fb)?;
    Line::new(Point::new(x + 12, y + 5), Point::new(stem_x + 1, y + 8))
        .into_styled(stroke)
        .draw(fb)?;
    Line::new(Point::new(stem_x + 1, y + 8), Point::new(x + 12, y + 11))
        .into_styled(stroke)
        .draw(fb)?;
    Line::new(Point::new(x + 12, y + 11), Point::new(stem_x + 1, y + 14))
        .into_styled(stroke)
        .draw(fb)?;
    Ok(())
}

fn draw_radar_icon(fb: &mut FrameBuffer, x: i32, y: i32, fg: Rgb565) -> Result<()> {
    let stroke = PrimitiveStyle::with_stroke(fg, 1);
    let cx = x + (ICON_W / 2);
    let cy = y + (ICON_H / 2);

    Circle::new(Point::new(cx - 7, cy - 7), 15)
        .into_styled(stroke)
        .draw(fb)?;
    Circle::new(Point::new(cx - 5, cy - 5), 11)
        .into_styled(stroke)
        .draw(fb)?;
    Circle::new(Point::new(cx - 3, cy - 3), 7)
        .into_styled(stroke)
        .draw(fb)?;

    Line::new(Point::new(cx, cy), Point::new(x + 13, y + 3))
        .into_styled(stroke)
        .draw(fb)?;
    Ok(())
}

fn draw_menu_line(
    panel: sys::esp_lcd_panel_handle_t,
    y: i32,
    line: &str,
    fg: Rgb565,
    bg: Rgb565,
) -> Result<()> {
    let mut buffer = vec![bg.into_storage(); (LCD_H_RES * MENU_BTN_H) as usize];
    let mut fb = FrameBuffer {
        width: LCD_H_RES,
        height: MENU_BTN_H,
        data: &mut buffer,
    };
    let mut x = 8;
    let baseline_y = 20;
    let icon_y = (MENU_BTN_H - ICON_H) / 2;

    for ch in line.chars() {
        match ch {
            'ᯤ' => {
                draw_wifi_icon(&mut fb, x, icon_y, fg, bg)?;
                x += ICON_W + 4;
            }
            'ᛒ' => {
                draw_bt_icon(&mut fb, x, icon_y, fg)?;
                x += ICON_W + 4;
            }
            'ᚱ' => {
                draw_radar_icon(&mut fb, x, icon_y, fg)?;
                x += ICON_W + 4;
            }
            'ᚷ' => {
                draw_icon_fb(&mut fb, x, icon_y, fg, &ICON_GPS)?;
                x += ICON_W + 4;
            }
            'ᛞ' => {
                draw_icon_fb(&mut fb, x, icon_y, fg, &ICON_DEVICE)?;
                x += ICON_W + 4;
            }
            'ᚲ' => {
                draw_icon_fb(&mut fb, x, icon_y, fg, &ICON_REBOOT)?;
                x += ICON_W + 4;
            }
            ' ' => {
                x += FONT_10X20.character_size.width as i32;
            }
            _ => {
                let mut buf = [0u8; 4];
                let s = ch.encode_utf8(&mut buf);
                let style = MonoTextStyle::new(&FONT_10X20, fg);
                Text::new(s, Point::new(x, baseline_y), style).draw(&mut fb)?;
                x += FONT_10X20.character_size.width as i32;
            }
        }
    }

    draw_bitmap(panel, 0, y, LCD_H_RES, MENU_BTN_H, &buffer)
}

pub fn draw_menu(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "Main Menu", false)?;
    let fg = Rgb565::new(0, 63, 0);
    draw_menu_line(panel, MENU_BTN1_Y, "ᯤ WiFi", fg, Rgb565::BLACK)?;
    draw_menu_line(panel, MENU_BTN2_Y, "ᛒ Bluetooth", fg, Rgb565::BLACK)?;
    draw_menu_line(panel, MENU_BTN2_Y + MENU_BTN_H, "ᚷ GPS", fg, Rgb565::BLACK)?;
    draw_menu_line(
        panel,
        MENU_BTN2_Y + (MENU_BTN_H * 2),
        "ᛞ Device",
        fg,
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        MENU_BTN2_Y + (MENU_BTN_H * 3),
        "ᚲ Reboot",
        fg,
        Rgb565::BLACK,
    )?;
    Ok(())
}

pub fn draw_device_menu(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "Device", true)?;
    draw_menu_line(
        panel,
        DEVICE_MENU_BTN1_Y,
        "  Batteriestatus",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        DEVICE_MENU_BTN2_Y,
        "ᚷ GPS",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        DEVICE_MENU_BTN3_Y,
        "  UART Loopback",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        DEVICE_MENU_BTN4_Y,
        "  SD Format",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        DEVICE_MENU_BTN5_Y,
        "  Mini HTTP",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        DEVICE_MENU_BTN6_Y,
        "  WiFi Login",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}

pub fn draw_http_menu(
    panel: sys::esp_lcd_panel_handle_t,
    running: bool,
    status: &str,
    ip: Option<&str>,
) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "Mini HTTP", true)?;
    let button_text = if running {
        "  Stop HTTP SD"
    } else {
        "  Start HTTP SD"
    };
    let button_fg = if running {
        Rgb565::new(63, 16, 0)
    } else {
        Rgb565::new(0, 63, 0)
    };
    draw_menu_line(
        panel,
        HTTP_MENU_BTN1_Y,
        button_text,
        button_fg,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        0,
        130,
        LCD_H_RES,
        24,
        "Tippen: Start/Stop",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    let status_fg = if status.starts_with("Fehler")
        || status.starts_with("IP Fehler")
        || status.starts_with("Kein IPv4")
    {
        Rgb565::new(63, 0, 0)
    } else if running {
        Rgb565::new(0, 63, 0)
    } else {
        Rgb565::WHITE
    };
    draw_text_box(
        panel,
        0,
        156,
        LCD_H_RES,
        24,
        &format!("Status: {}", status),
        status_fg,
        Rgb565::BLACK,
    )?;
    let url_line = if let Some(ip) = ip {
        format!("URL: http://{}:8080/sd", ip)
    } else {
        "URL: WLAN/IPv4 fehlt".to_string()
    };
    draw_text_box(
        panel,
        0,
        186,
        LCD_H_RES,
        24,
        &url_line,
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    Ok(())
}

pub fn draw_battery_status_frame(panel: sys::esp_lcd_panel_handle_t) -> Result<BatteryLabels> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header_with_icon(panel, "Batteriestatus", true, &ICON_BATTERY)?;

    let fg = Rgb565::new(0, 63, 0);
    let mut y = 50;
    let line_h = 20;
    draw_text_box(panel, 0, y, LCD_H_RES, line_h, "VBAT:", fg, Rgb565::BLACK)?;
    let y_vbat = y;
    y += line_h;
    draw_text_box(panel, 0, y, LCD_H_RES, line_h, "ADC:", fg, Rgb565::BLACK)?;
    let y_adc = y;
    y += line_h;
    draw_text_box(panel, 0, y, LCD_H_RES, line_h, "RAW:", fg, Rgb565::BLACK)?;
    let y_raw = y;
    y += line_h;
    draw_text_box(panel, 0, y, LCD_H_RES, line_h, "SoC:", fg, Rgb565::BLACK)?;
    let y_soc = y;
    y += line_h;
    draw_text_box_small(panel, 0, y, LCD_H_RES, 12, "Cal:", fg, Rgb565::BLACK)?;
    let y_cal = y;

    Ok(BatteryLabels {
        y_vbat,
        y_adc,
        y_raw,
        y_soc,
        y_cal,
    })
}

pub fn draw_battery_status_values(
    panel: sys::esp_lcd_panel_handle_t,
    labels: &BatteryLabels,
    status: &crate::device::BatteryStatus,
) -> Result<()> {
    let fg = Rgb565::new(0, 63, 0);
    draw_text_box(
        panel,
        70,
        labels.y_vbat,
        LCD_H_RES - 70,
        20,
        &format!("{} mV", status.v_bat_mv),
        fg,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        70,
        labels.y_adc,
        LCD_H_RES - 70,
        20,
        &format!("{} mV", status.v_adc_mv),
        fg,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        70,
        labels.y_raw,
        LCD_H_RES - 70,
        20,
        &format!("{}", status.raw),
        fg,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        70,
        labels.y_soc,
        LCD_H_RES - 70,
        20,
        &format!("{} %", status.percent),
        fg,
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        70,
        labels.y_cal,
        LCD_H_RES - 70,
        12,
        if status.calibrated { "yes" } else { "no" },
        fg,
        Rgb565::BLACK,
    )?;
    Ok(())
}

pub fn draw_battery_unavailable(panel: sys::esp_lcd_panel_handle_t, reason: &str) -> Result<()> {
    let _ = draw_battery_status_frame(panel)?;
    draw_text_box(
        panel,
        0,
        80,
        LCD_H_RES,
        24,
        "Battery ADC",
        Rgb565::new(63, 0, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        108,
        LCD_H_RES,
        12,
        reason,
        Rgb565::new(63, 0, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}

pub fn draw_wifi_menu(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "WiFi", true)?;
    draw_menu_line(
        panel,
        WIFI_MENU_BTN1_Y,
        "ᚱ Scan Wifi Network",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        WIFI_MENU_BTN2_Y,
        "  Monitor RSSI",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_menu_line(
        panel,
        WIFI_MENU_BTN3_Y,
        "  Channel Monitor",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}

pub fn draw_wifi_screen(panel: sys::esp_lcd_panel_handle_t, items: &[(String, i8)]) -> Result<()> {
    draw_wifi_frame(panel)?;

    if items.is_empty() {
        draw_text_box(
            panel,
            0,
            60,
            LCD_H_RES,
            24,
            "No networks",
            Rgb565::new(0, 63, 0),
            Rgb565::BLACK,
        )?;
        return Ok(());
    }

    let line_h = 16;
    let mut y = 56;
    for (i, (ssid, rssi)) in items.iter().enumerate() {
        if y + line_h > LCD_V_RES {
            break;
        }
        let line = format!("{}. {} ({})", i + 1, ssid, rssi);
        draw_text_box_small(
            panel,
            0,
            y,
            LCD_H_RES,
            line_h,
            &line,
            Rgb565::new(0, 63, 0),
            Rgb565::BLACK,
        )?;
        y += line_h;
    }
    Ok(())
}

pub fn draw_wifi_frame(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "WiFi Networks", true)?;
    Ok(())
}

pub fn draw_wifi_message(
    panel: sys::esp_lcd_panel_handle_t,
    line1: &str,
    line2: Option<&str>,
) -> Result<()> {
    draw_text_box(
        panel,
        0,
        60,
        LCD_H_RES,
        24,
        line1,
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    if let Some(line2) = line2 {
        draw_text_box(
            panel,
            0,
            84,
            LCD_H_RES,
            24,
            line2,
            Rgb565::new(0, 63, 0),
            Rgb565::BLACK,
        )?;
    }
    Ok(())
}

pub struct RssiSeries<'a> {
    pub label: &'a str,
    pub samples: &'a [Option<i8>],
    pub color: Rgb565,
}

pub struct PacketSample {
    pub mgmt: u32,
    pub data: u32,
    pub ctrl: u32,
}

pub fn draw_wifi_monitor(
    panel: sys::esp_lcd_panel_handle_t,
    series: &[RssiSeries],
    rssi_min: i8,
    rssi_max: i8,
) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "WiFi Monitor", true)?;
    draw_rect_box(
        panel,
        MONITOR_GRAPH_X - 1,
        MONITOR_GRAPH_Y - 1,
        MONITOR_GRAPH_W + 2,
        MONITOR_GRAPH_H + 2,
        1,
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
        None,
    )?;

    let mut buffer =
        vec![Rgb565::BLACK.into_storage(); (MONITOR_GRAPH_W * MONITOR_GRAPH_H) as usize];
    let mut fb = FrameBuffer {
        width: MONITOR_GRAPH_W,
        height: MONITOR_GRAPH_H,
        data: &mut buffer,
    };
    fb.clear(Rgb565::BLACK)?;

    let grid = PrimitiveStyle::with_stroke(Rgb565::new(0, 20, 0), 1);
    for i in 1..5 {
        let y = (MONITOR_GRAPH_H * i / 5) - 1;
        Line::new(Point::new(0, y), Point::new(MONITOR_GRAPH_W - 1, y))
            .into_styled(grid)
            .draw(&mut fb)?;
    }
    for i in 1..5 {
        let x = (MONITOR_GRAPH_W * i / 5) - 1;
        Line::new(Point::new(x, 0), Point::new(x, MONITOR_GRAPH_H - 1))
            .into_styled(grid)
            .draw(&mut fb)?;
    }

    let span = (rssi_max as i32 - rssi_min as i32).max(1);
    for s in series {
        if s.samples.len() < 2 {
            continue;
        }
        let mut last: Option<Point> = None;
        for (idx, sample) in s.samples.iter().enumerate() {
            let sample = match sample {
                Some(v) => *v as i32,
                None => {
                    last = None;
                    continue;
                }
            };
            let x = (idx as i32 * (MONITOR_GRAPH_W - 1)) / (s.samples.len() as i32 - 1);
            let clamped = sample.clamp(rssi_min as i32, rssi_max as i32);
            let y =
                MONITOR_GRAPH_H - 1 - ((clamped - rssi_min as i32) * (MONITOR_GRAPH_H - 1) / span);
            let p = Point::new(x, y);
            if let Some(prev) = last {
                Line::new(prev, p)
                    .into_styled(PrimitiveStyle::with_stroke(s.color, 1))
                    .draw(&mut fb)?;
            }
            last = Some(p);
        }
    }

    draw_bitmap(
        panel,
        MONITOR_GRAPH_X,
        MONITOR_GRAPH_Y,
        MONITOR_GRAPH_W,
        MONITOR_GRAPH_H,
        &buffer,
    )?;

    let legend_y = MONITOR_GRAPH_Y + MONITOR_GRAPH_H + 6;
    let line_h = 12;
    for (idx, s) in series.iter().enumerate() {
        if legend_y + (idx as i32 * line_h) + line_h > LCD_V_RES {
            break;
        }
        let last = s.samples.iter().rev().find_map(|v| *v);
        let value = last
            .map(|v| format!("{}", v))
            .unwrap_or_else(|| "-".to_string());
        let label = format!("{}: {}", s.label, value);
        draw_text_box_small(
            panel,
            0,
            legend_y + (idx as i32 * line_h),
            LCD_H_RES,
            line_h,
            &label,
            s.color,
            Rgb565::BLACK,
        )?;
    }
    Ok(())
}

pub fn draw_wifi_channel_monitor(
    panel: sys::esp_lcd_panel_handle_t,
    channel: u8,
    samples: &[PacketSample],
) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "WiFi Channel", true)?;
    draw_rect_box(
        panel,
        WIFI_CH_BTN_LEFT_X,
        WIFI_CH_BTN_Y,
        WIFI_CH_BTN_W,
        WIFI_CH_BTN_H,
        2,
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
        Some("<"),
    )?;
    draw_rect_box(
        panel,
        WIFI_CH_BTN_RIGHT_X,
        WIFI_CH_BTN_Y,
        WIFI_CH_BTN_W,
        WIFI_CH_BTN_H,
        2,
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
        Some(">"),
    )?;
    draw_text_box(
        panel,
        60,
        WIFI_CH_BTN_Y,
        120,
        WIFI_CH_BTN_H,
        &format!("CH {}", channel),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;

    draw_rect_box(
        panel,
        CHANNEL_BAR_X - 1,
        CHANNEL_BAR_Y - 1,
        CHANNEL_BAR_W + 2,
        CHANNEL_BAR_H + 2,
        1,
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
        None,
    )?;

    let mut buffer = vec![Rgb565::BLACK.into_storage(); (CHANNEL_BAR_W * CHANNEL_BAR_H) as usize];
    let mut fb = FrameBuffer {
        width: CHANNEL_BAR_W,
        height: CHANNEL_BAR_H,
        data: &mut buffer,
    };
    fb.clear(Rgb565::BLACK)?;

    let grid = PrimitiveStyle::with_stroke(Rgb565::new(0, 20, 0), 1);
    for i in 1..5 {
        let y = (CHANNEL_BAR_H * i / 5) - 1;
        Line::new(Point::new(0, y), Point::new(CHANNEL_BAR_W - 1, y))
            .into_styled(grid)
            .draw(&mut fb)?;
    }

    if !samples.is_empty() {
        let max_val = samples
            .iter()
            .map(|s| s.mgmt + s.data + s.ctrl)
            .max()
            .unwrap_or(1)
            .max(1);
        let n = samples.len() as i32;
        let mgmt_color = Rgb565::new(0, 63, 0);
        let data_color = Rgb565::new(0, 40, 63);
        let ctrl_color = Rgb565::new(63, 20, 0);
        for (idx, val) in samples.iter().enumerate() {
            let x = (idx as i32 * (CHANNEL_BAR_W - 1)) / n;
            let h_mgmt = ((val.mgmt as i32) * (CHANNEL_BAR_H - 2)) / (max_val as i32);
            let h_data = ((val.data as i32) * (CHANNEL_BAR_H - 2)) / (max_val as i32);
            let h_ctrl = ((val.ctrl as i32) * (CHANNEL_BAR_H - 2)) / (max_val as i32);
            let mut y = CHANNEL_BAR_H - 1;
            if h_mgmt > 0 {
                let y0 = y - h_mgmt;
                Line::new(Point::new(x, y), Point::new(x, y0))
                    .into_styled(PrimitiveStyle::with_stroke(mgmt_color, 1))
                    .draw(&mut fb)?;
                y = y0;
            }
            if h_data > 0 {
                let y0 = y - h_data;
                Line::new(Point::new(x, y), Point::new(x, y0))
                    .into_styled(PrimitiveStyle::with_stroke(data_color, 1))
                    .draw(&mut fb)?;
                y = y0;
            }
            if h_ctrl > 0 {
                let y0 = y - h_ctrl;
                Line::new(Point::new(x, y), Point::new(x, y0))
                    .into_styled(PrimitiveStyle::with_stroke(ctrl_color, 1))
                    .draw(&mut fb)?;
            }
        }
    }

    draw_bitmap(
        panel,
        CHANNEL_BAR_X,
        CHANNEL_BAR_Y,
        CHANNEL_BAR_W,
        CHANNEL_BAR_H,
        &buffer,
    )?;
    let legend_y = CHANNEL_BAR_Y + CHANNEL_BAR_H + 2;
    draw_text_box_small(
        panel,
        0,
        legend_y,
        70,
        12,
        "Mgmt",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        70,
        legend_y,
        70,
        12,
        "Data",
        Rgb565::new(0, 40, 63),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        140,
        legend_y,
        70,
        12,
        "Ctrl",
        Rgb565::new(63, 20, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}
