use anyhow::{anyhow, Result};
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
};
use esp_idf_hal::{
    delay::{Ets, FreeRtos},
    gpio::{AnyIOPin, PinDriver, Pull},
    peripherals::Peripherals,
};
use esp_idf_sys as sys;
use std::{convert::Infallible, ffi::CString, ptr};

pub mod device;
pub mod gps;
pub mod http_server;
pub mod sdcard;
pub mod ui;
pub mod wifi;

use device::read_battery_once;
use ui::{
    clear_screen, draw_battery_status_frame, draw_battery_status_values, draw_battery_unavailable,
    draw_bitmap, draw_device_menu, draw_header, draw_http_menu, draw_menu, draw_text_box,
    draw_text_box_small, draw_wifi_channel_monitor, draw_wifi_frame, draw_wifi_menu,
    draw_wifi_message, draw_wifi_monitor, draw_wifi_screen, BatteryLabels, PacketSample,
    RssiSeries, BACK_BTN_H, BACK_BTN_W, BACK_BTN_X, BACK_BTN_Y, DEVICE_MENU_BTN1_Y,
    DEVICE_MENU_BTN2_Y, DEVICE_MENU_BTN3_Y, DEVICE_MENU_BTN4_Y, DEVICE_MENU_BTN5_Y,
    DEVICE_MENU_BTN6_Y, HTTP_MENU_BTN1_Y, MENU_BTN1_Y, MENU_BTN2_Y, MENU_BTN_H, MENU_BTN_W,
    MENU_BTN_X, WIFI_CH_BTN_H, WIFI_CH_BTN_LEFT_X, WIFI_CH_BTN_RIGHT_X, WIFI_CH_BTN_W,
    WIFI_CH_BTN_Y, WIFI_MENU_BTN1_Y, WIFI_MENU_BTN2_Y, WIFI_MENU_BTN3_Y,
};

pub const LCD_H_RES: i32 = 240;
pub const LCD_V_RES: i32 = 320;

const LCD_DATA_PINS: [i32; 8] = [48, 47, 39, 40, 41, 42, 45, 46];
const LCD_SCLK: i32 = 8;
const LCD_CS: i32 = 6;
const LCD_DC: i32 = 7;

const TOUCH_SCLK: i32 = 1;
const TOUCH_MISO: i32 = 4;
const TOUCH_MOSI: i32 = 3;
const TOUCH_CS: i32 = 2;
const TOUCH_IRQ: i32 = 9;
// BAT_ADC shares IO03 with LCD_MOSI/Touch MOSI on this board.
// We only read at boot and when entering the BatteryStatus screen.
const BAT_ADC_GPIO: i32 = 3;
const PWR_EN_GPIO: i32 = 10;
const PWR_ON_GPIO: i32 = 14;
const BL_GPIO: i32 = 38;
// External/reed switch input; Arduino example treats it as active-high.
const POWER_BTN_GPIO: i32 = 21;
const POWER_BTN_DEBOUNCE_MS: u32 = 40;
fn wifi_default_ssid() -> &'static str {
    option_env!("WIFI_DEFAULT_SSID").unwrap_or("")
}

fn wifi_default_password() -> &'static str {
    option_env!("WIFI_DEFAULT_PASSWORD").unwrap_or("")
}

fn enter_deep_sleep() -> ! {
    unsafe {
        sys::gpio_hold_en(PWR_ON_GPIO);
        sys::gpio_hold_en(PWR_EN_GPIO);
        sys::gpio_hold_en(BL_GPIO);
        sys::gpio_deep_sleep_hold_en();
        sys::esp_sleep_enable_ext0_wakeup(POWER_BTN_GPIO, 1);
    }
    unsafe { sys::esp_deep_sleep_start() };
    loop {
        FreeRtos::delay_ms(1000);
    }
}

pub fn esp_ok(code: sys::esp_err_t) -> Result<()> {
    if code == sys::ESP_OK {
        Ok(())
    } else {
        if let Some(err) = sys::EspError::from(code) {
            Err(err.into())
        } else {
            Err(anyhow!("ESP error code: {}", code))
        }
    }
}

pub struct FrameBuffer<'a> {
    pub width: i32,
    pub height: i32,
    pub data: &'a mut [u16],
}

impl OriginDimensions for FrameBuffer<'_> {
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}

impl DrawTarget for FrameBuffer<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> core::result::Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x < 0 || coord.y < 0 || coord.x >= self.width || coord.y >= self.height {
                continue;
            }
            let idx = (coord.y as usize * self.width as usize) + coord.x as usize;
            self.data[idx] = color.into_storage();
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> core::result::Result<(), Self::Error> {
        self.data.fill(color.into_storage());
        Ok(())
    }
}

fn init_lcd() -> Result<sys::esp_lcd_panel_handle_t> {
    let mut io_handle: sys::esp_lcd_panel_io_handle_t = ptr::null_mut();
    let mut panel_handle: sys::esp_lcd_panel_handle_t = ptr::null_mut();

    let mut buscfg = sys::spi_bus_config_t::default();
    buscfg.__bindgen_anon_1 = sys::spi_bus_config_t__bindgen_ty_1 {
        data0_io_num: LCD_DATA_PINS[0],
    };
    buscfg.__bindgen_anon_2 = sys::spi_bus_config_t__bindgen_ty_2 {
        data1_io_num: LCD_DATA_PINS[1],
    };
    buscfg.sclk_io_num = LCD_SCLK;
    buscfg.__bindgen_anon_3 = sys::spi_bus_config_t__bindgen_ty_3 {
        data2_io_num: LCD_DATA_PINS[2],
    };
    buscfg.__bindgen_anon_4 = sys::spi_bus_config_t__bindgen_ty_4 {
        data3_io_num: LCD_DATA_PINS[3],
    };
    buscfg.data4_io_num = LCD_DATA_PINS[4];
    buscfg.data5_io_num = LCD_DATA_PINS[5];
    buscfg.data6_io_num = LCD_DATA_PINS[6];
    buscfg.data7_io_num = LCD_DATA_PINS[7];
    buscfg.max_transfer_sz = (LCD_H_RES * 40 * 2) as i32;
    buscfg.flags = sys::SPICOMMON_BUSFLAG_OCTAL;

    esp_ok(unsafe {
        sys::spi_bus_initialize(
            sys::spi_host_device_t_SPI2_HOST,
            &buscfg,
            sys::spi_common_dma_t_SPI_DMA_CH_AUTO,
        )
    })?;

    let mut io_config = sys::esp_lcd_panel_io_spi_config_t::default();
    io_config.cs_gpio_num = LCD_CS;
    io_config.dc_gpio_num = LCD_DC;
    io_config.pclk_hz = 20_000_000;
    io_config.trans_queue_depth = 10;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;

    let mut flags = sys::esp_lcd_panel_io_spi_config_t__bindgen_ty_1::default();
    flags.set_octal_mode(1);
    io_config.flags = flags;
    io_config.spi_mode = 3;

    esp_ok(unsafe {
        sys::esp_lcd_new_panel_io_spi(
            sys::spi_host_device_t_SPI2_HOST as sys::esp_lcd_spi_bus_handle_t,
            &io_config,
            &mut io_handle,
        )
    })?;

    let mut panel_config: sys::esp_lcd_panel_dev_config_t = unsafe { core::mem::zeroed() };
    panel_config.reset_gpio_num = -1;
    panel_config.__bindgen_anon_1 = sys::esp_lcd_panel_dev_config_t__bindgen_ty_1 {
        rgb_ele_order: sys::lcd_rgb_element_order_t_LCD_RGB_ELEMENT_ORDER_RGB,
    };
    panel_config.data_endian = sys::lcd_rgb_data_endian_t_LCD_RGB_DATA_ENDIAN_LITTLE;
    panel_config.bits_per_pixel = 16;
    panel_config.flags = sys::esp_lcd_panel_dev_config_t__bindgen_ty_2::default();
    panel_config.vendor_config = ptr::null_mut();

    esp_ok(unsafe { sys::esp_lcd_new_panel_st7789(io_handle, &panel_config, &mut panel_handle) })?;
    esp_ok(unsafe { sys::esp_lcd_panel_reset(panel_handle) })?;
    esp_ok(unsafe { sys::esp_lcd_panel_init(panel_handle) })?;
    esp_ok(unsafe { sys::esp_lcd_panel_disp_on_off(panel_handle, true) })?;

    Ok(panel_handle)
}

fn draw_circle(
    panel: sys::esp_lcd_panel_handle_t,
    cx: i32,
    cy: i32,
    r: i32,
    fg: Rgb565,
    bg: Rgb565,
) -> Result<()> {
    let size = r * 2 + 1;
    let mut buffer = vec![bg.into_storage(); (size * size) as usize];
    let mut fb = FrameBuffer {
        width: size,
        height: size,
        data: &mut buffer,
    };
    fb.clear(bg)?;
    let style = PrimitiveStyle::with_stroke(fg, 2);
    Circle::new(Point::new(0, 0), size as u32)
        .into_styled(style)
        .draw(&mut fb)?;
    draw_bitmap(panel, cx - r, cy - r, size, size, &buffer)
}

#[derive(Copy, Clone, Default)]
struct TouchPoint {
    raw_x: u16,
    raw_y: u16,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct CalibrationBlob {
    magic: u32,
    points: [TouchPoint; 4],
}

const CALIB_MAGIC: u32 = 0x5443_4831;

struct Calibration {
    x_min: i32,
    x_max: i32,
    y_min: i32,
    y_max: i32,
}

impl Calibration {
    fn from_points(points: &[TouchPoint; 4]) -> Self {
        let mut x_min = i32::MAX;
        let mut x_max = i32::MIN;
        let mut y_min = i32::MAX;
        let mut y_max = i32::MIN;

        for p in points {
            let x = p.raw_x as i32;
            let y = p.raw_y as i32;
            x_min = x_min.min(x);
            x_max = x_max.max(x);
            y_min = y_min.min(y);
            y_max = y_max.max(y);
        }

        Self {
            x_min,
            x_max,
            y_min,
            y_max,
        }
    }

    fn map(&self, raw_x: u16, raw_y: u16) -> (i32, i32) {
        let rx = raw_x as i32;
        let ry = raw_y as i32;
        let x = if self.x_max != self.x_min {
            (rx - self.x_min) * (LCD_H_RES - 1) / (self.x_max - self.x_min)
        } else {
            0
        };
        let y = if self.y_max != self.y_min {
            (ry - self.y_min) * (LCD_V_RES - 1) / (self.y_max - self.y_min)
        } else {
            0
        };
        (x.clamp(0, LCD_H_RES - 1), y.clamp(0, LCD_V_RES - 1))
    }
}

fn nvs_init() -> Result<()> {
    let mut err = unsafe { sys::nvs_flash_init() };
    if err == sys::ESP_ERR_NVS_NO_FREE_PAGES || err == sys::ESP_ERR_NVS_NEW_VERSION_FOUND {
        esp_ok(unsafe { sys::nvs_flash_erase() })?;
        err = unsafe { sys::nvs_flash_init() };
    }
    esp_ok(err)
}

fn calibration_load() -> Result<Option<[TouchPoint; 4]>> {
    let namespace = CString::new("touch")?;
    let key = CString::new("data")?;
    let mut handle: sys::nvs_handle_t = 0;

    let err = unsafe {
        sys::nvs_open(
            namespace.as_ptr(),
            sys::nvs_open_mode_t_NVS_READWRITE,
            &mut handle,
        )
    };
    if err != sys::ESP_OK {
        return Ok(None);
    }

    let mut size: usize = 0;
    let err = unsafe { sys::nvs_get_blob(handle, key.as_ptr(), ptr::null_mut(), &mut size) };
    if err == sys::ESP_ERR_NVS_NOT_FOUND || size == 0 {
        unsafe { sys::nvs_close(handle) };
        return Ok(None);
    }
    let expected = core::mem::size_of::<CalibrationBlob>();
    if size != expected {
        unsafe { sys::nvs_close(handle) };
        return Ok(None);
    }

    let mut blob = CalibrationBlob {
        magic: 0,
        points: [TouchPoint::default(); 4],
    };
    let err = unsafe {
        sys::nvs_get_blob(
            handle,
            key.as_ptr(),
            (&mut blob as *mut CalibrationBlob).cast(),
            &mut size,
        )
    };
    unsafe { sys::nvs_close(handle) };
    if err != sys::ESP_OK || blob.magic != CALIB_MAGIC {
        return Ok(None);
    }
    Ok(Some(blob.points))
}

fn calibration_save(points: &[TouchPoint; 4]) -> Result<()> {
    let namespace = CString::new("touch")?;
    let key = CString::new("data")?;
    let mut handle: sys::nvs_handle_t = 0;

    esp_ok(unsafe {
        sys::nvs_open(
            namespace.as_ptr(),
            sys::nvs_open_mode_t_NVS_READWRITE,
            &mut handle,
        )
    })?;

    let blob = CalibrationBlob {
        magic: CALIB_MAGIC,
        points: *points,
    };
    let size = core::mem::size_of::<CalibrationBlob>();
    esp_ok(unsafe {
        sys::nvs_set_blob(
            handle,
            key.as_ptr(),
            (&blob as *const CalibrationBlob).cast(),
            size,
        )
    })?;
    esp_ok(unsafe { sys::nvs_commit(handle) })?;
    unsafe { sys::nvs_close(handle) };
    Ok(())
}

struct Xpt2046 {
    sclk: PinDriver<'static, AnyIOPin, esp_idf_hal::gpio::Output>,
    mosi: PinDriver<'static, AnyIOPin, esp_idf_hal::gpio::Output>,
    miso: PinDriver<'static, AnyIOPin, esp_idf_hal::gpio::Input>,
    cs: PinDriver<'static, AnyIOPin, esp_idf_hal::gpio::Output>,
    irq: PinDriver<'static, AnyIOPin, esp_idf_hal::gpio::Input>,
}

impl Xpt2046 {
    fn new() -> Result<Self> {
        let mut sclk = PinDriver::output(unsafe { AnyIOPin::new(TOUCH_SCLK) })?;
        let mut mosi = PinDriver::output(unsafe { AnyIOPin::new(TOUCH_MOSI) })?;
        let mut cs = PinDriver::output(unsafe { AnyIOPin::new(TOUCH_CS) })?;
        let mut miso = PinDriver::input(unsafe { AnyIOPin::new(TOUCH_MISO) })?;
        let mut irq = PinDriver::input(unsafe { AnyIOPin::new(TOUCH_IRQ) })?;

        irq.set_pull(Pull::Up)?;
        sclk.set_low()?;
        mosi.set_low()?;
        cs.set_high()?;
        miso.set_pull(Pull::Up)?;

        Ok(Self {
            sclk,
            mosi,
            miso,
            cs,
            irq,
        })
    }

    fn pressed(&self) -> bool {
        self.irq.is_low()
    }

    fn read_raw(&mut self) -> Result<Option<TouchPoint>> {
        if !self.pressed() {
            return Ok(None);
        }
        let raw_x = self.read_axis(0xD0)?;
        let raw_y = self.read_axis(0x90)?;
        Ok(Some(TouchPoint { raw_x, raw_y }))
    }

    fn read_axis(&mut self, cmd: u8) -> Result<u16> {
        self.cs.set_low()?;
        Ets::delay_us(1);

        for bit in (0..8).rev() {
            if (cmd >> bit) & 1 == 1 {
                self.mosi.set_high()?;
            } else {
                self.mosi.set_low()?;
            }
            self.sclk.set_low()?;
            Ets::delay_us(1);
            self.sclk.set_high()?;
            Ets::delay_us(1);
        }

        let mut value: u16 = 0;
        for _ in 0..16 {
            self.sclk.set_low()?;
            Ets::delay_us(1);
            self.sclk.set_high()?;
            Ets::delay_us(1);
            value = (value << 1) | if self.miso.is_high() { 1 } else { 0 };
        }

        self.cs.set_high()?;
        Ok((value >> 4) & 0x0FFF)
    }
}

fn run_calibration(
    panel: sys::esp_lcd_panel_handle_t,
    touch: &mut Xpt2046,
) -> Result<[TouchPoint; 4]> {
    let targets = [
        (10, 10),
        (LCD_H_RES - 11, 10),
        (LCD_H_RES - 11, LCD_V_RES - 11),
        (10, LCD_V_RES - 11),
    ];
    let mut points = [TouchPoint::default(); 4];

    for (idx, (x, y)) in targets.iter().enumerate() {
        clear_screen(panel, Rgb565::BLACK)?;
        draw_circle(panel, *x, *y, 10, Rgb565::WHITE, Rgb565::BLACK)?;
        draw_text_box(
            panel,
            40,
            150,
            160,
            40,
            "Please click",
            Rgb565::WHITE,
            Rgb565::BLACK,
        )?;
        draw_text_box(
            panel,
            40,
            190,
            160,
            40,
            "the circle",
            Rgb565::WHITE,
            Rgb565::BLACK,
        )?;

        let mut samples: Vec<TouchPoint> = Vec::new();
        let mut idle_ms = 0;
        loop {
            if let Some(p) = touch.read_raw()? {
                samples.push(p);
                idle_ms = 0;
            } else if !samples.is_empty() {
                idle_ms += 10;
                if idle_ms >= 200 {
                    break;
                }
            }
            FreeRtos::delay_ms(10);
        }

        if samples.is_empty() {
            continue;
        }
        let mut sum_x: u32 = 0;
        let mut sum_y: u32 = 0;
        for s in &samples {
            sum_x += s.raw_x as u32;
            sum_y += s.raw_y as u32;
        }
        points[idx] = TouchPoint {
            raw_x: (sum_x / samples.len() as u32) as u16,
            raw_y: (sum_y / samples.len() as u32) as u16,
        };
    }

    clear_screen(panel, Rgb565::BLACK)?;
    draw_text_box(
        panel,
        30,
        140,
        180,
        40,
        "Calibration",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        30,
        180,
        180,
        40,
        "complete",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    FreeRtos::delay_ms(800);

    Ok(points)
}

enum Screen {
    Menu,
    WifiMenu,
    WifiList,
    WifiMonitor,
    WifiChannelMonitor,
    Gps,
    UartLoopback,
    DeviceMenu,
    BatteryStatus,
    SdCardFormat,
    HttpMenu,
    WifiLogin,
    WifiConnected,
}

struct MonitorSeries {
    ssid: String,
    samples: Vec<Option<i8>>,
    color: Rgb565,
}

const MONITOR_MAX_SERIES: usize = 3;
const MONITOR_MAX_SAMPLES: usize = 60;
const MONITOR_SCAN_INTERVAL_MS: u32 = 2000;
const RSSI_MIN: i8 = -90;
const RSSI_MAX: i8 = -30;
const CHANNEL_MIN: u8 = 1;
const CHANNEL_MAX: u8 = 13;
const CHANNEL_SAMPLE_MAX: usize = 60;
const CHANNEL_SAMPLE_INTERVAL_MS: u32 = 500;
const WIFI_LOGIN_LINE_Y: i32 = 52;
const WIFI_LOGIN_LINE_H: i32 = 18;
const WIFI_LOGIN_MAX_ITEMS: usize = 4;
const WIFI_LOGIN_SCROLL_Y: i32 = 126;
const WIFI_LOGIN_SCROLL_BTN_W: i32 = 52;
const WIFI_LOGIN_SCROLL_BTN_H: i32 = 22;
const WIFI_LOGIN_SCROLL_UP_X: i32 = 8;
const WIFI_LOGIN_SCROLL_DOWN_X: i32 = 66;
const WIFI_LOGIN_SCAN_BTN_X: i32 = 124;
const WIFI_LOGIN_SCAN_BTN_Y: i32 = 126;
const WIFI_LOGIN_SCAN_BTN_W: i32 = 108;
const WIFI_LOGIN_DEFAULT_BTN_X: i32 = 8;
const WIFI_LOGIN_DEFAULT_BTN_Y: i32 = 152;
const WIFI_LOGIN_DEFAULT_BTN_W: i32 = 108;
const WIFI_LOGIN_CONNECT_BTN_X: i32 = 124;
const WIFI_LOGIN_CONNECT_BTN_Y: i32 = 152;
const WIFI_LOGIN_CONNECT_BTN_W: i32 = 108;
const WIFI_LOGIN_CHAR_BTN_Y: i32 = 212;
const WIFI_LOGIN_CHAR_BTN_W: i32 = 52;
const WIFI_LOGIN_CHAR_BTN_H: i32 = 22;
const WIFI_LOGIN_CHAR_PREV_X: i32 = 8;
const WIFI_LOGIN_CHAR_NEXT_X: i32 = 66;
const WIFI_LOGIN_CHAR_ADD_X: i32 = 124;
const WIFI_LOGIN_CHAR_DEL_X: i32 = 182;
const WIFI_LOGIN_CHARSET: &[u8] =
    b"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()-_=+.";

fn wifi_login_current_char(idx: usize) -> char {
    WIFI_LOGIN_CHARSET.get(idx).copied().unwrap_or(b'a') as char
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

fn update_monitor_series(series: &mut Vec<MonitorSeries>, scan: &[(String, i8)]) {
    let mut top: Vec<(String, i8)> = scan.to_vec();
    top.sort_by(|a, b| b.1.cmp(&a.1));
    if top.len() > MONITOR_MAX_SERIES {
        top.truncate(MONITOR_MAX_SERIES);
    }

    let mut updated = vec![false; series.len()];
    for (idx, s) in series.iter_mut().enumerate() {
        if let Some((_, rssi)) = top.iter().find(|(ssid, _)| ssid == &s.ssid) {
            s.samples.push(Some(*rssi));
            updated[idx] = true;
        }
    }

    for (ssid, rssi) in &top {
        if series.iter().any(|s| s.ssid == *ssid) {
            continue;
        }
        if series.len() < MONITOR_MAX_SERIES {
            let color = match series.len() {
                0 => Rgb565::new(0, 63, 0),
                1 => Rgb565::new(0, 40, 63),
                _ => Rgb565::new(63, 20, 0),
            };
            series.push(MonitorSeries {
                ssid: ssid.clone(),
                samples: vec![Some(*rssi)],
                color,
            });
            updated.push(true);
        } else if let Some(idx) = series
            .iter()
            .position(|s| !top.iter().any(|(t, _)| t == &s.ssid))
        {
            let color = series[idx].color;
            series[idx] = MonitorSeries {
                ssid: ssid.clone(),
                samples: vec![Some(*rssi)],
                color,
            };
            if idx < updated.len() {
                updated[idx] = true;
            }
        }
    }

    for (idx, s) in series.iter_mut().enumerate() {
        if !updated.get(idx).copied().unwrap_or(false) {
            s.samples.push(None);
        }
        if s.samples.len() > MONITOR_MAX_SAMPLES {
            let extra = s.samples.len() - MONITOR_MAX_SAMPLES;
            s.samples.drain(0..extra);
        }
    }
}

fn strongest_channel() -> Result<u8> {
    let records = wifi::wifi_scan_records()?;
    if records.is_empty() {
        return Ok(CHANNEL_MIN);
    }
    let mut best = &records[0];
    for r in &records[1..] {
        if r.rssi > best.rssi {
            best = r;
        }
    }
    Ok(best.channel.clamp(CHANNEL_MIN, CHANNEL_MAX))
}

fn draw_wifi_login_screen(
    panel: sys::esp_lcd_panel_handle_t,
    items: &[wifi::WifiAp],
    selected: Option<usize>,
    offset: usize,
    status: &str,
    password: &str,
    char_idx: usize,
) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "WiFi Login", true)?;

    let end = (offset + WIFI_LOGIN_MAX_ITEMS).min(items.len());
    for (row, ap) in items[offset..end].iter().enumerate() {
        let idx = offset + row;
        let y = WIFI_LOGIN_LINE_Y + (row as i32 * WIFI_LOGIN_LINE_H);
        let fg = if Some(idx) == selected {
            Rgb565::new(63, 63, 0)
        } else {
            Rgb565::new(0, 63, 0)
        };
        let lock = if ap.authmode == sys::wifi_auth_mode_t_WIFI_AUTH_OPEN {
            "O"
        } else {
            "L"
        };
        let ssid = truncate_chars(&ap.ssid, 16);
        draw_text_box_small(
            panel,
            0,
            y,
            LCD_H_RES,
            WIFI_LOGIN_LINE_H,
            &format!("{} {} {}dBm {}", idx + 1, ssid, ap.rssi, lock),
            fg,
            Rgb565::BLACK,
        )?;
    }

    draw_text_box(
        panel,
        WIFI_LOGIN_SCROLL_UP_X,
        WIFI_LOGIN_SCROLL_Y,
        WIFI_LOGIN_SCROLL_BTN_W,
        WIFI_LOGIN_SCROLL_BTN_H,
        "Up",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_SCROLL_DOWN_X,
        WIFI_LOGIN_SCROLL_Y,
        WIFI_LOGIN_SCROLL_BTN_W,
        WIFI_LOGIN_SCROLL_BTN_H,
        "Down",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_SCAN_BTN_X,
        WIFI_LOGIN_SCAN_BTN_Y,
        WIFI_LOGIN_SCAN_BTN_W,
        WIFI_LOGIN_SCROLL_BTN_H,
        "Scan",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_DEFAULT_BTN_X,
        WIFI_LOGIN_DEFAULT_BTN_Y,
        WIFI_LOGIN_DEFAULT_BTN_W,
        WIFI_LOGIN_SCROLL_BTN_H,
        "Default",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_CONNECT_BTN_X,
        WIFI_LOGIN_CONNECT_BTN_Y,
        WIFI_LOGIN_CONNECT_BTN_W,
        WIFI_LOGIN_SCROLL_BTN_H,
        "Connect",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;

    let selected_ssid = selected
        .and_then(|i| items.get(i))
        .map(|ap| truncate_chars(&ap.ssid, 18))
        .unwrap_or_else(|| "-".to_string());
    draw_text_box_small(
        panel,
        0,
        180,
        LCD_H_RES,
        12,
        &format!("SSID: {}", selected_ssid),
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;

    let masked = "*".repeat(password.chars().count());
    let selected_channel = selected
        .and_then(|i| items.get(i))
        .map(|ap| ap.channel)
        .unwrap_or(0);
    draw_text_box_small(
        panel,
        0,
        194,
        LCD_H_RES,
        12,
        &format!(
            "PWD: {}  CH:{}  CHR:{}",
            masked,
            selected_channel,
            wifi_login_current_char(char_idx)
        ),
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_CHAR_PREV_X,
        WIFI_LOGIN_CHAR_BTN_Y,
        WIFI_LOGIN_CHAR_BTN_W,
        WIFI_LOGIN_CHAR_BTN_H,
        "<",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_CHAR_NEXT_X,
        WIFI_LOGIN_CHAR_BTN_Y,
        WIFI_LOGIN_CHAR_BTN_W,
        WIFI_LOGIN_CHAR_BTN_H,
        ">",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_CHAR_ADD_X,
        WIFI_LOGIN_CHAR_BTN_Y,
        WIFI_LOGIN_CHAR_BTN_W,
        WIFI_LOGIN_CHAR_BTN_H,
        "Add",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box(
        panel,
        WIFI_LOGIN_CHAR_DEL_X,
        WIFI_LOGIN_CHAR_BTN_Y,
        WIFI_LOGIN_CHAR_BTN_W,
        WIFI_LOGIN_CHAR_BTN_H,
        "Del",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        284,
        LCD_H_RES,
        12,
        &truncate_chars(status, 30),
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    Ok(())
}

fn draw_wifi_connected_screen(
    panel: sys::esp_lcd_panel_handle_t,
    ssid: &str,
    rssi: i8,
    channel: u8,
    ip: &str,
) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "WiFi Verbunden", true)?;
    draw_text_box_small(
        panel,
        0,
        64,
        LCD_H_RES,
        14,
        &format!("SSID: {}", truncate_chars(ssid, 22)),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        84,
        LCD_H_RES,
        14,
        &format!("Kanal: {}", channel),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        104,
        LCD_H_RES,
        14,
        &format!("RSSI: {} dBm", rssi),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        124,
        LCD_H_RES,
        14,
        &format!("IP: {}", ip),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}

fn main() -> Result<()> {
    sys::link_patches();

    let p = Peripherals::take().unwrap();

    // Power and backlight control.
    let mut pwr_en = PinDriver::output(p.pins.gpio10)?;
    let mut pwr_on = PinDriver::output(p.pins.gpio14)?;
    let mut bl = PinDriver::output(p.pins.gpio38)?;
    let mut power_btn = PinDriver::input(unsafe { AnyIOPin::new(POWER_BTN_GPIO) })?;
    pwr_en.set_high()?;
    pwr_on.set_high()?;
    bl.set_high()?;
    power_btn.set_pull(Pull::Down)?;

    // Read battery before LCD init since BAT_ADC shares IO03 with LCD_MOSI.
    let mut battery_cache = if BAT_ADC_GPIO >= 0 {
        read_battery_once(BAT_ADC_GPIO).ok()
    } else {
        None
    };

    let panel = init_lcd()?;

    draw_menu(panel)?;

    nvs_init()?;
    let mut touch = Xpt2046::new()?;
    let points = match calibration_load()? {
        Some(points) => points,
        None => {
            draw_text_box(
                panel,
                10,
                80,
                220,
                40,
                "No calibration",
                Rgb565::WHITE,
                Rgb565::BLACK,
            )?;
            draw_text_box(
                panel,
                10,
                120,
                220,
                40,
                "Starting...",
                Rgb565::WHITE,
                Rgb565::BLACK,
            )?;
            let points = run_calibration(panel, &mut touch)?;
            calibration_save(&points)?;
            points
        }
    };
    let calibration = Calibration::from_points(&points);
    let mut touch = Some(touch);
    let mut gps_reader = gps::GpsReader::new(p.uart1, p.pins.gpio16, p.pins.gpio15)?;

    let mut was_pressed = false;
    let mut screen = Screen::Menu;
    let mut tick_ms: u32 = 0;
    let mut monitor_series: Vec<MonitorSeries> = Vec::new();
    let mut monitor_last_scan: u32 = 0;
    let mut channel_monitor_channel: u8 = CHANNEL_MIN;
    let mut channel_monitor_samples: Vec<PacketSample> = Vec::new();
    let mut channel_monitor_last_sample: u32 = 0;
    let mut power_btn_last_raw = power_btn.is_high();
    let mut battery_last_read: u32 = 0;
    let mut battery_ignore_back_until: u32 = 0;
    let mut battery_ignore_back = false;
    let mut battery_labels: Option<BatteryLabels> = None;
    let mut gps_last_redraw: u32 = 0;
    let mut gps_back_to_device = false;
    let mut loopback_last_redraw: u32 = 0;
    let mut sd_result_ok = false;
    let mut sd_result_msg = String::new();
    let mut http_server: Option<http_server::MiniHttpServer> = None;
    let mut http_status = String::from("Aus");
    let mut wifi_login_items: Vec<wifi::WifiAp> = Vec::new();
    let mut wifi_login_selected: Option<usize> = None;
    let mut wifi_login_offset: usize = 0;
    let mut wifi_login_status = String::from("Scan und Netz waehlen");
    let mut wifi_login_password = wifi_default_password().to_string();
    let mut wifi_login_char_idx: usize = 0;
    let mut wifi_connected_ssid = String::new();
    let mut wifi_connected_rssi: i8 = 0;
    let mut wifi_connected_channel: u8 = 0;
    let mut wifi_connected_ip = String::new();
    loop {
        let power_btn_raw = power_btn.is_high();
        if power_btn_raw && !power_btn_last_raw {
            bl.set_low()?;
            pwr_on.set_high()?;
            pwr_en.set_high()?;
            enter_deep_sleep();
        }
        power_btn_last_raw = power_btn_raw;

        let raw = if let Some(t) = touch.as_mut() {
            t.read_raw()?
        } else {
            None
        };
        let touch_down = raw.is_some();
        let (x, y) = if let Some(raw) = raw {
            calibration.map(raw.raw_x, raw.raw_y)
        } else {
            (-1, -1)
        };

        match screen {
            Screen::Menu => {
                let hit_menu = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= MENU_BTN1_Y
                    && y < MENU_BTN1_Y + MENU_BTN_H;
                let hit_device = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= MENU_BTN2_Y + (MENU_BTN_H * 2)
                    && y < MENU_BTN2_Y + (MENU_BTN_H * 2) + MENU_BTN_H;
                let hit_gps = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= MENU_BTN2_Y + MENU_BTN_H
                    && y < MENU_BTN2_Y + (MENU_BTN_H * 2);
                if hit_menu && !was_pressed {
                    screen = Screen::WifiMenu;
                    draw_wifi_menu(panel)?;
                } else if hit_gps && !was_pressed {
                    screen = Screen::Gps;
                    gps_back_to_device = false;
                    gps_last_redraw = tick_ms;
                    gps_reader.set_host_log_enabled(false);
                    gps::draw_gps_frame(panel)?;
                    gps::draw_gps_values(panel, &gps_reader)?;
                } else if hit_device && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                }
                was_pressed = hit_menu || hit_gps || hit_device;
            }
            Screen::WifiMenu => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_scan = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= WIFI_MENU_BTN1_Y
                    && y < WIFI_MENU_BTN1_Y + MENU_BTN_H;
                let hit_monitor = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= WIFI_MENU_BTN2_Y
                    && y < WIFI_MENU_BTN2_Y + MENU_BTN_H;
                let hit_channel = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= WIFI_MENU_BTN3_Y
                    && y < WIFI_MENU_BTN3_Y + MENU_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::Menu;
                    draw_menu(panel)?;
                } else if hit_scan && !was_pressed {
                    screen = Screen::WifiList;
                    draw_wifi_frame(panel)?;
                    draw_wifi_message(panel, "Scanning...", None)?;
                    match wifi::wifi_scan() {
                        Ok(items) => {
                            draw_wifi_screen(panel, &items)?;
                        }
                        Err(err) => {
                            draw_wifi_frame(panel)?;
                            draw_wifi_message(panel, "Scan error:", Some(&format!("{}", err)))?;
                        }
                    }
                } else if hit_monitor && !was_pressed {
                    screen = Screen::WifiMonitor;
                    monitor_series.clear();
                    monitor_last_scan = 0;
                    draw_wifi_monitor(panel, &[], RSSI_MIN, RSSI_MAX)?;
                } else if hit_channel && !was_pressed {
                    screen = Screen::WifiChannelMonitor;
                    channel_monitor_channel = strongest_channel().unwrap_or(CHANNEL_MIN);
                    channel_monitor_samples.clear();
                    channel_monitor_last_sample = 0;
                    let _ = wifi::wifi_monitor_start(channel_monitor_channel);
                    let _ = wifi::wifi_monitor_take_counts();
                    draw_wifi_channel_monitor(
                        panel,
                        channel_monitor_channel,
                        &channel_monitor_samples,
                    )?;
                }
                was_pressed = hit_back || hit_scan || hit_monitor || hit_channel;
            }
            Screen::WifiList => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::WifiMenu;
                    draw_wifi_menu(panel)?;
                }
                was_pressed = hit_back;
            }
            Screen::WifiMonitor => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::WifiMenu;
                    draw_wifi_menu(panel)?;
                } else if tick_ms.wrapping_sub(monitor_last_scan) >= MONITOR_SCAN_INTERVAL_MS {
                    monitor_last_scan = tick_ms;
                    match wifi::wifi_scan() {
                        Ok(items) => {
                            update_monitor_series(&mut monitor_series, &items);
                            let series: Vec<RssiSeries> = monitor_series
                                .iter()
                                .map(|s| RssiSeries {
                                    label: s.ssid.as_str(),
                                    samples: &s.samples,
                                    color: s.color,
                                })
                                .collect();
                            draw_wifi_monitor(panel, &series, RSSI_MIN, RSSI_MAX)?;
                        }
                        Err(err) => {
                            draw_wifi_monitor(panel, &[], RSSI_MIN, RSSI_MAX)?;
                            draw_text_box(
                                panel,
                                0,
                                230,
                                LCD_H_RES,
                                24,
                                &format!("Scan error: {}", err),
                                Rgb565::new(63, 0, 0),
                                Rgb565::BLACK,
                            )?;
                        }
                    }
                }
                was_pressed = hit_back;
            }
            Screen::WifiChannelMonitor => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_prev = touch_down
                    && x >= WIFI_CH_BTN_LEFT_X
                    && x < WIFI_CH_BTN_LEFT_X + WIFI_CH_BTN_W
                    && y >= WIFI_CH_BTN_Y
                    && y < WIFI_CH_BTN_Y + WIFI_CH_BTN_H;
                let hit_next = touch_down
                    && x >= WIFI_CH_BTN_RIGHT_X
                    && x < WIFI_CH_BTN_RIGHT_X + WIFI_CH_BTN_W
                    && y >= WIFI_CH_BTN_Y
                    && y < WIFI_CH_BTN_Y + WIFI_CH_BTN_H;
                if hit_back && !was_pressed {
                    let _ = wifi::wifi_monitor_stop();
                    screen = Screen::WifiMenu;
                    draw_wifi_menu(panel)?;
                } else if hit_prev && !was_pressed {
                    if channel_monitor_channel > CHANNEL_MIN {
                        channel_monitor_channel -= 1;
                    }
                    channel_monitor_samples.clear();
                    channel_monitor_last_sample = 0;
                    let _ = wifi::wifi_monitor_start(channel_monitor_channel);
                    let _ = wifi::wifi_monitor_take_counts();
                    draw_wifi_channel_monitor(
                        panel,
                        channel_monitor_channel,
                        &channel_monitor_samples,
                    )?;
                } else if hit_next && !was_pressed {
                    if channel_monitor_channel < CHANNEL_MAX {
                        channel_monitor_channel += 1;
                    }
                    channel_monitor_samples.clear();
                    channel_monitor_last_sample = 0;
                    let _ = wifi::wifi_monitor_start(channel_monitor_channel);
                    let _ = wifi::wifi_monitor_take_counts();
                    draw_wifi_channel_monitor(
                        panel,
                        channel_monitor_channel,
                        &channel_monitor_samples,
                    )?;
                } else if tick_ms.wrapping_sub(channel_monitor_last_sample)
                    >= CHANNEL_SAMPLE_INTERVAL_MS
                {
                    channel_monitor_last_sample = tick_ms;
                    let count = wifi::wifi_monitor_take_counts();
                    channel_monitor_samples.push(PacketSample {
                        mgmt: count.mgmt,
                        data: count.data,
                        ctrl: count.ctrl,
                    });
                    if channel_monitor_samples.len() > CHANNEL_SAMPLE_MAX {
                        let extra = channel_monitor_samples.len() - CHANNEL_SAMPLE_MAX;
                        channel_monitor_samples.drain(0..extra);
                    }
                    draw_wifi_channel_monitor(
                        panel,
                        channel_monitor_channel,
                        &channel_monitor_samples,
                    )?;
                }
                was_pressed = hit_back || hit_prev || hit_next;
            }
            Screen::DeviceMenu => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_batt = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN1_Y
                    && y < DEVICE_MENU_BTN1_Y + MENU_BTN_H;
                let hit_gps = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN2_Y
                    && y < DEVICE_MENU_BTN2_Y + MENU_BTN_H;
                let hit_loopback = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN3_Y
                    && y < DEVICE_MENU_BTN3_Y + MENU_BTN_H;
                let hit_sd_format = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN4_Y
                    && y < DEVICE_MENU_BTN4_Y + MENU_BTN_H;
                let hit_http_menu = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN5_Y
                    && y < DEVICE_MENU_BTN5_Y + MENU_BTN_H;
                let hit_wifi_login = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN6_Y
                    && y < DEVICE_MENU_BTN6_Y + MENU_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::Menu;
                    draw_menu(panel)?;
                } else if hit_batt && !was_pressed {
                    screen = Screen::BatteryStatus;
                    battery_last_read = 0;
                    battery_ignore_back_until = tick_ms.wrapping_add(1200);
                    battery_ignore_back = true;
                    battery_labels = Some(draw_battery_status_frame(panel)?);
                } else if hit_gps && !was_pressed {
                    screen = Screen::Gps;
                    gps_back_to_device = true;
                    gps_last_redraw = tick_ms;
                    gps_reader.set_host_log_enabled(true);
                    gps::draw_gps_frame(panel)?;
                    gps::draw_gps_values(panel, &gps_reader)?;
                } else if hit_loopback && !was_pressed {
                    screen = Screen::UartLoopback;
                    gps_reader.set_host_log_enabled(false);
                    gps_reader.loopback_reset();
                    loopback_last_redraw = tick_ms;
                    gps::draw_uart_loopback_frame(panel)?;
                    gps::draw_uart_loopback_values(panel, &gps_reader)?;
                } else if hit_sd_format && !was_pressed {
                    screen = Screen::SdCardFormat;
                    clear_screen(panel, Rgb565::BLACK)?;
                    draw_header(panel, "SD Format", true)?;
                    draw_text_box(
                        panel,
                        0,
                        100,
                        LCD_H_RES,
                        30,
                        "Formatiere SD...",
                        Rgb565::WHITE,
                        Rgb565::BLACK,
                    )?;
                    draw_text_box(
                        panel,
                        0,
                        140,
                        LCD_H_RES,
                        40,
                        "0%",
                        Rgb565::WHITE,
                        Rgb565::BLACK,
                    )?;
                    match sdcard::run_sdcard_format_test(|percent, msg| {
                        draw_text_box(
                            panel,
                            0,
                            100,
                            LCD_H_RES,
                            30,
                            msg,
                            Rgb565::WHITE,
                            Rgb565::BLACK,
                        )?;
                        let pct = format!("{percent}%");
                        draw_text_box(
                            panel,
                            0,
                            140,
                            LCD_H_RES,
                            40,
                            &pct,
                            Rgb565::WHITE,
                            Rgb565::BLACK,
                        )?;
                        Ok(())
                    }) {
                        Ok(()) => {
                            sd_result_ok = true;
                            sd_result_msg = String::from("OK");
                        }
                        Err(err) => {
                            sd_result_ok = false;
                            sd_result_msg = format!("{}", err);
                        }
                    }
                    clear_screen(panel, Rgb565::BLACK)?;
                    draw_header(panel, "SD Format", true)?;
                    if sd_result_ok {
                        draw_text_box(
                            panel,
                            0,
                            100,
                            LCD_H_RES,
                            30,
                            "test.txt gelesen:",
                            Rgb565::new(0, 63, 0),
                            Rgb565::BLACK,
                        )?;
                        draw_text_box(
                            panel,
                            0,
                            140,
                            LCD_H_RES,
                            40,
                            "OK",
                            Rgb565::new(0, 63, 0),
                            Rgb565::BLACK,
                        )?;
                    } else {
                        draw_text_box(
                            panel,
                            0,
                            100,
                            LCD_H_RES,
                            30,
                            "SD Fehler",
                            Rgb565::new(63, 0, 0),
                            Rgb565::BLACK,
                        )?;
                        draw_text_box(
                            panel,
                            0,
                            140,
                            LCD_H_RES,
                            40,
                            &sd_result_msg,
                            Rgb565::new(63, 0, 0),
                            Rgb565::BLACK,
                        )?;
                    }
                } else if hit_http_menu && !was_pressed {
                    screen = Screen::HttpMenu;
                    draw_http_menu(panel)?;
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
                    draw_text_box(
                        panel,
                        0,
                        156,
                        LCD_H_RES,
                        24,
                        &format!("Status: {}", http_status),
                        Rgb565::WHITE,
                        Rgb565::BLACK,
                    )?;
                    draw_text_box(
                        panel,
                        0,
                        186,
                        LCD_H_RES,
                        24,
                        "Pfad: /sd",
                        Rgb565::WHITE,
                        Rgb565::BLACK,
                    )?;
                } else if hit_wifi_login && !was_pressed {
                    screen = Screen::WifiLogin;
                    wifi_login_status = String::from("Scanne...");
                    match wifi::wifi_scan_records() {
                        Ok(items) => {
                            wifi_login_items = items;
                            wifi_login_selected = wifi_login_items
                                .iter()
                                .position(|ap| {
                                    !wifi_default_ssid().is_empty()
                                        && ap.ssid == wifi_default_ssid()
                                })
                                .or_else(|| {
                                    if wifi_login_items.is_empty() {
                                        None
                                    } else {
                                        Some(0)
                                    }
                                });
                            wifi_login_offset = wifi_login_selected
                                .map(|idx| (idx / WIFI_LOGIN_MAX_ITEMS) * WIFI_LOGIN_MAX_ITEMS)
                                .unwrap_or(0);
                            if wifi_login_items.is_empty() {
                                wifi_login_status = String::from("Keine Netze gefunden");
                            } else if let Some(idx) = wifi_login_selected {
                                wifi_login_status =
                                    format!("Ausgewaehlt: {}", wifi_login_items[idx].ssid);
                            } else {
                                wifi_login_status = String::from("Netz waehlen, dann Connect");
                            }
                        }
                        Err(err) => {
                            wifi_login_items.clear();
                            wifi_login_selected = None;
                            wifi_login_offset = 0;
                            wifi_login_status = format!("Scan Fehler: {}", err);
                        }
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                }
                was_pressed = hit_back
                    || hit_batt
                    || hit_gps
                    || hit_loopback
                    || hit_sd_format
                    || hit_http_menu
                    || hit_wifi_login;
            }
            Screen::Gps => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    gps_reader.set_host_log_enabled(false);
                    if gps_back_to_device {
                        screen = Screen::DeviceMenu;
                        draw_device_menu(panel)?;
                    } else {
                        screen = Screen::Menu;
                        draw_menu(panel)?;
                    }
                } else {
                    let gps_changed = gps_reader.poll(tick_ms)?;
                    if gps_changed || tick_ms.wrapping_sub(gps_last_redraw) >= 300 {
                        gps::draw_gps_values(panel, &gps_reader)?;
                        gps_last_redraw = tick_ms;
                    }
                }
                was_pressed = hit_back;
            }
            Screen::UartLoopback => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                } else {
                    let changed = gps_reader.loopback_poll(tick_ms)?;
                    if changed || tick_ms.wrapping_sub(loopback_last_redraw) >= 300 {
                        gps::draw_uart_loopback_values(panel, &gps_reader)?;
                        loopback_last_redraw = tick_ms;
                    }
                }
                was_pressed = hit_back;
            }
            Screen::BatteryStatus => {
                if battery_ignore_back && !touch_down {
                    battery_ignore_back = false;
                }
                let ignore_back = battery_ignore_back
                    || battery_ignore_back_until.wrapping_sub(tick_ms) < 0x8000_0000;
                let hit_back = !ignore_back
                    && touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                    battery_labels = None;
                } else if battery_cache.is_none() {
                    if battery_last_read == 0 {
                        draw_battery_unavailable(panel, "ADC disabled/NA")?;
                        battery_last_read = tick_ms;
                    }
                } else if battery_last_read == 0 {
                    if let (Some(labels), Some(status)) = (&battery_labels, &battery_cache) {
                        draw_battery_status_values(panel, labels, status)?;
                    }
                    battery_last_read = tick_ms;
                }
                was_pressed = hit_back;
            }
            Screen::SdCardFormat => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                }
                was_pressed = hit_back;
            }
            Screen::HttpMenu => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_sd_view = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= HTTP_MENU_BTN1_Y
                    && y < HTTP_MENU_BTN1_Y + MENU_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                } else if hit_sd_view && !was_pressed {
                    if http_server.is_none() {
                        match http_server::MiniHttpServer::start(8080) {
                            Ok(server) => {
                                let port = server.port();
                                http_server = Some(server);
                                http_status = format!("AN (Port {})", port);
                            }
                            Err(err) => {
                                http_status = format!("Fehler: {}", err);
                            }
                        }
                    } else if let Some(mut server) = http_server.take() {
                        server.stop();
                        http_status = String::from("Aus");
                    }

                    draw_http_menu(panel)?;
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
                    draw_text_box(
                        panel,
                        0,
                        156,
                        LCD_H_RES,
                        24,
                        &format!("Status: {}", http_status),
                        Rgb565::WHITE,
                        Rgb565::BLACK,
                    )?;
                    draw_text_box(
                        panel,
                        0,
                        186,
                        LCD_H_RES,
                        24,
                        "Browser: http://<IP>:8080/sd",
                        Rgb565::WHITE,
                        Rgb565::BLACK,
                    )?;
                }
                was_pressed = hit_back || hit_sd_view;
            }
            Screen::WifiLogin => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_scan = touch_down
                    && x >= WIFI_LOGIN_SCAN_BTN_X
                    && x < WIFI_LOGIN_SCAN_BTN_X + WIFI_LOGIN_SCAN_BTN_W
                    && y >= WIFI_LOGIN_SCAN_BTN_Y
                    && y < WIFI_LOGIN_SCAN_BTN_Y + WIFI_LOGIN_SCROLL_BTN_H;
                let hit_connect = touch_down
                    && x >= WIFI_LOGIN_CONNECT_BTN_X
                    && x < WIFI_LOGIN_CONNECT_BTN_X + WIFI_LOGIN_CONNECT_BTN_W
                    && y >= WIFI_LOGIN_CONNECT_BTN_Y
                    && y < WIFI_LOGIN_CONNECT_BTN_Y + WIFI_LOGIN_SCROLL_BTN_H;
                let hit_default = touch_down
                    && x >= WIFI_LOGIN_DEFAULT_BTN_X
                    && x < WIFI_LOGIN_DEFAULT_BTN_X + WIFI_LOGIN_DEFAULT_BTN_W
                    && y >= WIFI_LOGIN_DEFAULT_BTN_Y
                    && y < WIFI_LOGIN_DEFAULT_BTN_Y + WIFI_LOGIN_SCROLL_BTN_H;
                let hit_scroll_up = touch_down
                    && x >= WIFI_LOGIN_SCROLL_UP_X
                    && x < WIFI_LOGIN_SCROLL_UP_X + WIFI_LOGIN_SCROLL_BTN_W
                    && y >= WIFI_LOGIN_SCROLL_Y
                    && y < WIFI_LOGIN_SCROLL_Y + WIFI_LOGIN_SCROLL_BTN_H;
                let hit_scroll_down = touch_down
                    && x >= WIFI_LOGIN_SCROLL_DOWN_X
                    && x < WIFI_LOGIN_SCROLL_DOWN_X + WIFI_LOGIN_SCROLL_BTN_W
                    && y >= WIFI_LOGIN_SCROLL_Y
                    && y < WIFI_LOGIN_SCROLL_Y + WIFI_LOGIN_SCROLL_BTN_H;
                let hit_char_prev = touch_down
                    && x >= WIFI_LOGIN_CHAR_PREV_X
                    && x < WIFI_LOGIN_CHAR_PREV_X + WIFI_LOGIN_CHAR_BTN_W
                    && y >= WIFI_LOGIN_CHAR_BTN_Y
                    && y < WIFI_LOGIN_CHAR_BTN_Y + WIFI_LOGIN_CHAR_BTN_H;
                let hit_char_next = touch_down
                    && x >= WIFI_LOGIN_CHAR_NEXT_X
                    && x < WIFI_LOGIN_CHAR_NEXT_X + WIFI_LOGIN_CHAR_BTN_W
                    && y >= WIFI_LOGIN_CHAR_BTN_Y
                    && y < WIFI_LOGIN_CHAR_BTN_Y + WIFI_LOGIN_CHAR_BTN_H;
                let hit_char_add = touch_down
                    && x >= WIFI_LOGIN_CHAR_ADD_X
                    && x < WIFI_LOGIN_CHAR_ADD_X + WIFI_LOGIN_CHAR_BTN_W
                    && y >= WIFI_LOGIN_CHAR_BTN_Y
                    && y < WIFI_LOGIN_CHAR_BTN_Y + WIFI_LOGIN_CHAR_BTN_H;
                let hit_char_del = touch_down
                    && x >= WIFI_LOGIN_CHAR_DEL_X
                    && x < WIFI_LOGIN_CHAR_DEL_X + WIFI_LOGIN_CHAR_BTN_W
                    && y >= WIFI_LOGIN_CHAR_BTN_Y
                    && y < WIFI_LOGIN_CHAR_BTN_Y + WIFI_LOGIN_CHAR_BTN_H;

                let mut hit_list = None;
                if touch_down {
                    let visible = WIFI_LOGIN_MAX_ITEMS
                        .min(wifi_login_items.len().saturating_sub(wifi_login_offset));
                    for row in 0..visible {
                        let ly = WIFI_LOGIN_LINE_Y + (row as i32 * WIFI_LOGIN_LINE_H);
                        if x >= 0 && x < LCD_H_RES && y >= ly && y < ly + WIFI_LOGIN_LINE_H {
                            hit_list = Some(wifi_login_offset + row);
                            break;
                        }
                    }
                }

                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                } else if hit_scan && !was_pressed {
                    wifi_login_status = String::from("Scanne...");
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                    match wifi::wifi_scan_records() {
                        Ok(items) => {
                            wifi_login_items = items;
                            wifi_login_selected = wifi_login_items
                                .iter()
                                .position(|ap| {
                                    !wifi_default_ssid().is_empty()
                                        && ap.ssid == wifi_default_ssid()
                                })
                                .or_else(|| {
                                    if wifi_login_items.is_empty() {
                                        None
                                    } else {
                                        Some(0)
                                    }
                                });
                            wifi_login_offset = wifi_login_selected
                                .map(|idx| (idx / WIFI_LOGIN_MAX_ITEMS) * WIFI_LOGIN_MAX_ITEMS)
                                .unwrap_or(0);
                            wifi_login_status = if wifi_login_items.is_empty() {
                                String::from("Keine Netze gefunden")
                            } else if let Some(idx) = wifi_login_selected {
                                format!("Ausgewaehlt: {}", wifi_login_items[idx].ssid)
                            } else {
                                String::from("Netz waehlen, dann Connect")
                            };
                        }
                        Err(err) => {
                            wifi_login_status = format!("Scan Fehler: {}", err);
                        }
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_scroll_up && !was_pressed {
                    if wifi_login_offset >= WIFI_LOGIN_MAX_ITEMS {
                        wifi_login_offset -= WIFI_LOGIN_MAX_ITEMS;
                    } else {
                        wifi_login_offset = 0;
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_scroll_down && !was_pressed {
                    if wifi_login_offset + WIFI_LOGIN_MAX_ITEMS < wifi_login_items.len() {
                        wifi_login_offset += WIFI_LOGIN_MAX_ITEMS;
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if let Some(idx) = hit_list {
                    if !was_pressed {
                        wifi_login_selected = Some(idx);
                        wifi_login_status = format!("Ausgewaehlt: {}", wifi_login_items[idx].ssid);
                        draw_wifi_login_screen(
                            panel,
                            &wifi_login_items,
                            wifi_login_selected,
                            wifi_login_offset,
                            &wifi_login_status,
                            &wifi_login_password,
                            wifi_login_char_idx,
                        )?;
                    }
                } else if hit_char_prev && !was_pressed {
                    if wifi_login_char_idx == 0 {
                        wifi_login_char_idx = WIFI_LOGIN_CHARSET.len().saturating_sub(1);
                    } else {
                        wifi_login_char_idx -= 1;
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_char_next && !was_pressed {
                    if !WIFI_LOGIN_CHARSET.is_empty() {
                        wifi_login_char_idx = (wifi_login_char_idx + 1) % WIFI_LOGIN_CHARSET.len();
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_char_add && !was_pressed {
                    if wifi_login_password.len() < 63 {
                        wifi_login_password.push(wifi_login_current_char(wifi_login_char_idx));
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_char_del && !was_pressed {
                    let _ = wifi_login_password.pop();
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_default && !was_pressed {
                    let default_ssid = wifi_default_ssid();
                    let default_password = wifi_default_password();
                    if default_ssid.is_empty() {
                        wifi_login_status = String::from("Default SSID fehlt (wifi_secrets.local)");
                        draw_wifi_login_screen(
                            panel,
                            &wifi_login_items,
                            wifi_login_selected,
                            wifi_login_offset,
                            &wifi_login_status,
                            &wifi_login_password,
                            wifi_login_char_idx,
                        )?;
                        was_pressed = true;
                        continue;
                    }
                    wifi_login_status = format!("Default connect {}...", default_ssid);
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;

                    let mut connected_info: Option<wifi::WifiConnectionInfo> = None;
                    let mut connect_error = String::new();
                    let default_ap = wifi::wifi_scan_records()
                        .ok()
                        .and_then(|items| items.into_iter().find(|ap| ap.ssid == default_ssid));
                    if let Some(ap) = default_ap.as_ref() {
                        println!(
                            "[WIFI] default AP gefunden ssid='{}' ch={} auth={:?}",
                            ap.ssid, ap.channel, ap.authmode
                        );
                    } else {
                        println!(
                            "[WIFI] default AP '{}' nicht im Scan gefunden, nutze direkten Connect",
                            default_ssid
                        );
                    }

                    let connect_result = if let Some(ap) = default_ap.as_ref() {
                        wifi::wifi_connect_ap(ap, default_password)
                    } else {
                        wifi::wifi_connect(default_ssid, default_password)
                    };

                    match connect_result {
                        Ok(()) => match wifi::wifi_wait_connected(12_000) {
                            Ok(info) => connected_info = Some(info),
                            Err(err) => {
                                connect_error = format!("default wait: {}", err);
                                println!("[WIFI] default wait failed: {}", err);
                            }
                        },
                        Err(err) => {
                            connect_error = format!("default connect: {}", err);
                            println!("[WIFI] default connect failed: {}", err);
                        }
                    }

                    if let Some(info) = connected_info {
                        wifi_connected_ssid = if info.ssid.is_empty() {
                            default_ssid.to_string()
                        } else {
                            info.ssid.clone()
                        };
                        wifi_connected_rssi = info.rssi;
                        wifi_connected_channel = info.channel;
                        wifi_connected_ip =
                            wifi::wifi_wait_ipv4(8_000).unwrap_or_else(|_| "-".to_string());
                        screen = Screen::WifiConnected;
                        draw_wifi_connected_screen(
                            panel,
                            &wifi_connected_ssid,
                            wifi_connected_rssi,
                            wifi_connected_channel,
                            &wifi_connected_ip,
                        )?;
                        was_pressed = true;
                        continue;
                    } else if connect_error.is_empty() {
                        wifi_login_status = String::from("Default Connect fehlgeschlagen");
                    } else {
                        wifi_login_status = format!("Default Fehler: {}", connect_error);
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                } else if hit_connect && !was_pressed {
                    if let Some(idx) = wifi_login_selected {
                        let ap = &wifi_login_items[idx];
                        let default_ssid = wifi_default_ssid();
                        let default_password = wifi_default_password();
                        if !default_ssid.is_empty()
                            && ap.ssid != default_ssid
                            && !default_password.is_empty()
                            && wifi_login_password == default_password
                        {
                            wifi_login_status =
                                format!("Achtung: Default-PWD ist fuer {}", default_ssid);
                            draw_wifi_login_screen(
                                panel,
                                &wifi_login_items,
                                wifi_login_selected,
                                wifi_login_offset,
                                &wifi_login_status,
                                &wifi_login_password,
                                wifi_login_char_idx,
                            )?;
                            was_pressed = true;
                            continue;
                        }
                        let password = if ap.authmode == sys::wifi_auth_mode_t_WIFI_AUTH_OPEN {
                            ""
                        } else {
                            wifi_login_password.as_str()
                        };
                        if ap.authmode != sys::wifi_auth_mode_t_WIFI_AUTH_OPEN
                            && wifi_login_password.is_empty()
                        {
                            wifi_login_status = String::from("Passwort eingeben (Add/Del)");
                            draw_wifi_login_screen(
                                panel,
                                &wifi_login_items,
                                wifi_login_selected,
                                wifi_login_offset,
                                &wifi_login_status,
                                &wifi_login_password,
                                wifi_login_char_idx,
                            )?;
                            was_pressed = true;
                            continue;
                        };
                        wifi_login_status = format!("Verbinde mit {}...", ap.ssid);
                        draw_wifi_login_screen(
                            panel,
                            &wifi_login_items,
                            wifi_login_selected,
                            wifi_login_offset,
                            &wifi_login_status,
                            &wifi_login_password,
                            wifi_login_char_idx,
                        )?;

                        let selected_ssid = ap.ssid.clone();
                        let mut connect_error = String::new();

                        let mut connected_info: Option<wifi::WifiConnectionInfo> = None;
                        match wifi::wifi_connect_ap(ap, password) {
                            Ok(()) => match wifi::wifi_wait_connected(10_000) {
                                Ok(info) => connected_info = Some(info),
                                Err(err) => {
                                    connect_error = format!("connect_ap wait: {}", err);
                                    println!("[WIFI] connect_ap wait failed: {}", err);
                                }
                            },
                            Err(err) => {
                                connect_error = format!("connect_ap: {}", err);
                                println!("[WIFI] connect_ap failed: {}", err);
                            }
                        }

                        if connected_info.is_none() {
                            println!("[WIFI] fallback connect ssid='{}'", ap.ssid);
                            match wifi::wifi_connect(&ap.ssid, password) {
                                Ok(()) => match wifi::wifi_wait_connected(10_000) {
                                    Ok(info) => connected_info = Some(info),
                                    Err(err) => {
                                        if !connect_error.is_empty() {
                                            connect_error.push_str(" | ");
                                        }
                                        connect_error.push_str(&format!("fallback wait: {}", err));
                                        println!("[WIFI] fallback wait failed: {}", err);
                                    }
                                },
                                Err(err) => {
                                    if !connect_error.is_empty() {
                                        connect_error.push_str(" | ");
                                    }
                                    connect_error.push_str(&format!("fallback: {}", err));
                                    println!("[WIFI] fallback connect failed: {}", err);
                                }
                            }
                        }

                        if let Some(info) = connected_info {
                            wifi_connected_ssid = if info.ssid.is_empty() {
                                selected_ssid
                            } else {
                                info.ssid.clone()
                            };
                            wifi_connected_rssi = info.rssi;
                            wifi_connected_channel = info.channel;
                            wifi_connected_ip =
                                wifi::wifi_wait_ipv4(8_000).unwrap_or_else(|_| "-".to_string());
                            screen = Screen::WifiConnected;
                            draw_wifi_connected_screen(
                                panel,
                                &wifi_connected_ssid,
                                wifi_connected_rssi,
                                wifi_connected_channel,
                                &wifi_connected_ip,
                            )?;
                            was_pressed = true;
                            continue;
                        } else if connect_error.is_empty() {
                            wifi_login_status = String::from("Kein Connect");
                        } else {
                            wifi_login_status = format!("Kein Connect: {}", connect_error);
                        }
                    } else {
                        wifi_login_status = String::from("Bitte zuerst ein Netz waehlen");
                    }
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                }
                was_pressed =
                    hit_back || hit_scan || hit_connect || hit_default || hit_list.is_some();
                was_pressed = was_pressed
                    || hit_char_prev
                    || hit_char_next
                    || hit_char_add
                    || hit_char_del
                    || hit_scroll_up
                    || hit_scroll_down;
            }
            Screen::WifiConnected => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::WifiLogin;
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;
                }
                was_pressed = hit_back;
            }
        }
        FreeRtos::delay_ms(20);
        tick_ms = tick_ms.wrapping_add(20);
    }
}
