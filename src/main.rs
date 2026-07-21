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
use std::{
    convert::Infallible,
    ffi::c_char,
    ffi::CString,
    ptr,
    time::{SystemTime, UNIX_EPOCH},
};

pub mod device;
pub mod gps;
pub mod http_server;
pub mod lora;
pub mod remote;
pub mod sdcard;
pub mod ui;
pub mod wifi;

use device::read_battery_once;
use ui::{
    clear_screen, draw_battery_status_frame, draw_battery_status_values, draw_battery_unavailable,
    draw_bitmap, draw_device_menu, draw_header, draw_http_menu, draw_menu, draw_text_box,
    draw_text_box_small, draw_wifi_channel_monitor, draw_wifi_frame, draw_wifi_menu,
    draw_wifi_message, draw_wifi_monitor, draw_wifi_screen, set_header_status_icons, BatteryLabels,
    PacketSample, RssiSeries, BACK_BTN_H, BACK_BTN_W, BACK_BTN_X, BACK_BTN_Y, DEVICE_MENU_BTN1_Y,
    DEVICE_MENU_BTN2_Y, DEVICE_MENU_BTN3_Y, DEVICE_MENU_BTN4_Y, DEVICE_MENU_BTN5_Y,
    DEVICE_MENU_BTN6_Y, DEVICE_MENU_BTN7_Y, DEVICE_MENU_BTN8_Y, HTTP_MENU_BTN1_Y, MENU_BTN1_Y,
    MENU_BTN2_Y, MENU_BTN_H, MENU_BTN_W, MENU_BTN_X, WIFI_CH_BTN_H, WIFI_CH_BTN_LEFT_X,
    WIFI_CH_BTN_RIGHT_X, WIFI_CH_BTN_W, WIFI_CH_BTN_Y, WIFI_MENU_BTN1_Y, WIFI_MENU_BTN2_Y,
    WIFI_MENU_BTN3_Y,
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
// Board reference examples use BAT_ADC on GPIO5.
const BAT_ADC_GPIO: i32 = 5;
const PWR_EN_GPIO: i32 = 10;
const PWR_ON_GPIO: i32 = 14;
const BL_GPIO: i32 = 38;
// External/reed switch input; Arduino example treats it as active-high.
const POWER_BTN_GPIO: i32 = 21;

#[derive(Clone, Debug)]
struct DefaultWifiCred {
    ssid: String,
    password: String,
}

fn push_default_cred(
    out: &mut Vec<DefaultWifiCred>,
    ssid: Option<&'static str>,
    password: Option<&'static str>,
) {
    let ssid = ssid.unwrap_or("").trim();
    if ssid.is_empty() {
        return;
    }
    let password = password.unwrap_or("").trim();
    if out.iter().any(|e| e.ssid == ssid && e.password == password) {
        return;
    }
    out.push(DefaultWifiCred {
        ssid: ssid.to_string(),
        password: password.to_string(),
    });
}

fn wifi_default_networks() -> Vec<DefaultWifiCred> {
    let mut out = Vec::new();
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_1"),
        option_env!("WIFI_DEFAULT_PASSWORD_1"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_2"),
        option_env!("WIFI_DEFAULT_PASSWORD_2"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_3"),
        option_env!("WIFI_DEFAULT_PASSWORD_3"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_4"),
        option_env!("WIFI_DEFAULT_PASSWORD_4"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_5"),
        option_env!("WIFI_DEFAULT_PASSWORD_5"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_6"),
        option_env!("WIFI_DEFAULT_PASSWORD_6"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_7"),
        option_env!("WIFI_DEFAULT_PASSWORD_7"),
    );
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID_8"),
        option_env!("WIFI_DEFAULT_PASSWORD_8"),
    );

    // Backward compatibility with old single-entry config.
    push_default_cred(
        &mut out,
        option_env!("WIFI_DEFAULT_SSID"),
        option_env!("WIFI_DEFAULT_PASSWORD"),
    );

    out
}

fn connect_with_fallback(
    ssid: &str,
    password: &str,
    wait_ms: u32,
) -> Result<wifi::WifiConnectionInfo> {
    let scanned_ap = wifi::wifi_scan_records()
        .ok()
        .and_then(|items| items.into_iter().find(|ap| ap.ssid == ssid));

    if let Some(ap) = scanned_ap.as_ref() {
        println!(
            "[WIFI_PATH] scan-hit -> wifi_connect_ap possible (ssid='{}', ch={})",
            ap.ssid, ap.channel
        );
    } else {
        println!(
            "[WIFI_PATH] scan-miss -> wifi_connect only (ssid='{}')",
            ssid
        );
    }

    match wifi::wifi_connect(ssid, password) {
        Ok(()) => match wifi::wifi_wait_connected(wait_ms) {
            Ok(info) => return Ok(info),
            Err(err) => println!("[WIFI] wait after wifi_connect failed: {}", err),
        },
        Err(err) => println!("[WIFI] wifi_connect failed: {}", err),
    }

    let ap = scanned_ap.ok_or_else(|| anyhow!("kein AP scan-hit fuer {}", ssid))?;
    wifi::wifi_connect_ap(&ap, password)?;
    wifi::wifi_wait_connected(wait_ms)
}

fn connect_default_sequence(
    defaults: &[DefaultWifiCred],
) -> Result<(wifi::WifiConnectionInfo, usize)> {
    let mut errors: Vec<String> = Vec::new();
    for (idx, item) in defaults.iter().enumerate() {
        println!(
            "[WIFI_BOOT] versuche default {}: '{}' (pass_len={})",
            idx + 1,
            item.ssid,
            item.password.len()
        );
        match connect_with_fallback(&item.ssid, &item.password, 12_000) {
            Ok(info) => return Ok((info, idx)),
            Err(err) => {
                println!(
                    "[WIFI_BOOT] default {} fehlgeschlagen (ssid='{}'): {}",
                    idx + 1,
                    item.ssid,
                    err
                );
                errors.push(format!("{}: {}", item.ssid, err));
            }
        }
    }
    Err(anyhow!(
        "kein Default-WLAN verbunden ({} Versuche): {}",
        defaults.len(),
        errors.join(" | ")
    ))
}

fn format_battery_dashboard(status: Option<&device::BatteryStatus>) -> String {
    let Some(status) = status else {
        return "ADC disabled/NA".to_string();
    };
    let source = match status.source {
        device::PowerSource::Usb => "USB",
        device::PowerSource::Battery => "Akku",
        device::PowerSource::Unknown => "Unbekannt",
    };
    format!("{} | {} mV | {} %", source, status.v_bat_mv, status.percent)
}

fn enter_deep_sleep() -> ! {
    unsafe {
        sys::gpio_hold_en(PWR_ON_GPIO);
        sys::gpio_hold_en(PWR_EN_GPIO);
        sys::gpio_hold_en(BL_GPIO);
        sys::gpio_deep_sleep_hold_en();
        sys::esp_sleep_enable_ext0_wakeup(POWER_BTN_GPIO, 1);
        sys::esp_deep_sleep_start()
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
    LoRaTest,
    GpsLogging,
    LoRaLogging,
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
const GPS_LOG_INTERVAL_MS: u32 = 200;
const LORA_LOG_INTERVAL_MS: u32 = 250;
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

struct GpsSdLogger {
    session: Option<sdcard::SdCardSession>,
    file_name: Option<String>,
    last_log_ms: u32,
}

impl GpsSdLogger {
    fn new() -> Self {
        Self {
            session: None,
            file_name: None,
            last_log_ms: 0,
        }
    }

    fn current_unix_secs_system() -> Option<u64> {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .ok()
            .map(|d| d.as_secs())
    }

    fn is_plausible_unix(unix_secs: u64) -> bool {
        // 2023-01-01T00:00:00Z to reject boot-default epoch.
        unix_secs >= 1_672_531_200
    }

    fn resolve_unix_secs(gps: &gps::GpsReader<'_>) -> u64 {
        if let Some(secs) = Self::current_unix_secs_system().filter(|v| Self::is_plausible_unix(*v))
        {
            return secs;
        }
        if let Some(secs) = gps.current_utc_unix_secs() {
            return secs;
        }
        Self::current_unix_secs_system().unwrap_or(0)
    }

    fn make_file_name(unix_secs: u64, run_id: u32) -> String {
        let (year, month, day, hour, minute, second) = unix_to_utc_ymdhms(unix_secs);
        format!("gps_{year:04}{month:02}{day:02}_{hour:02}{minute:02}{second:02}_{run_id:08}.txt")
    }

    fn make_file_name_83(unix_secs: u64, run_id: u32) -> String {
        // 8.3 fallback for FAT configurations without long file names.
        // Name is G + DDHHMM + N + ".txt" (8 chars base + 3 chars ext).
        let (_year, _month, day, hour, minute, _second) = unix_to_utc_ymdhms(unix_secs);
        let n = run_id % 10;
        format!("g{day:02}{hour:02}{minute:02}{n}.txt")
    }

    fn start(&mut self, now_ms: u32, gps: &gps::GpsReader<'_>) -> Result<()> {
        self.stop();
        let session = sdcard::SdCardSession::mount()?;
        let unix_secs = Self::resolve_unix_secs(gps);
        let run_id = now_ms;
        let long_file_name = Self::make_file_name(unix_secs, run_id);
        let short_file_name = Self::make_file_name_83(unix_secs, run_id);
        let file_name = match session.append_line(
            &long_file_name,
            "ts_ms;fix;sats;lat;lon;baud;raw_bps;last_nmea",
        ) {
            Ok(()) => long_file_name,
            Err(err) => {
                println!(
                    "[GPS_LOG] long filename failed ({}), fallback to /sdcard/{}",
                    err, short_file_name
                );
                session.append_line(
                    &short_file_name,
                    "ts_ms;fix;sats;lat;lon;baud;raw_bps;last_nmea",
                )?;
                short_file_name
            }
        };
        println!("[GPS_LOG] start file=/sdcard/{}", file_name);
        self.session = Some(session);
        self.file_name = Some(file_name);
        self.last_log_ms = now_ms.wrapping_sub(GPS_LOG_INTERVAL_MS);
        Ok(())
    }

    fn stop(&mut self) {
        if let Some(file_name) = self.file_name.as_deref() {
            println!("[GPS_LOG] close file=/sdcard/{}", file_name);
        }
        self.session = None;
        self.file_name = None;
        self.last_log_ms = 0;
    }

    fn is_active(&self) -> bool {
        self.session.is_some() && self.file_name.is_some()
    }

    fn file_name(&self) -> Option<&str> {
        self.file_name.as_deref()
    }

    fn poll_log(&mut self, now_ms: u32, gps: &gps::GpsReader<'_>) {
        let Some(session) = self.session.as_ref() else {
            return;
        };
        let Some(file_name) = self.file_name.as_deref() else {
            return;
        };
        if now_ms.wrapping_sub(self.last_log_ms) < GPS_LOG_INTERVAL_MS {
            return;
        }
        self.last_log_ms = now_ms;
        let sats = gps
            .sats()
            .map(|v| v.to_string())
            .unwrap_or_else(|| "-".to_string());
        let lat = gps
            .lat()
            .map(|v| format!("{:.6}", v))
            .unwrap_or_else(|| "-".to_string());
        let lon = gps
            .lon()
            .map(|v| format!("{:.6}", v))
            .unwrap_or_else(|| "-".to_string());
        let last_nmea = gps
            .last_sentence()
            .replace(';', ",")
            .replace('\r', " ")
            .replace('\n', " ");
        let line = format!(
            "{};{};{};{};{};{};{};{}",
            now_ms,
            if gps.fix() { "1" } else { "0" },
            sats,
            lat,
            lon,
            gps.current_baud(),
            gps.raw_bytes_per_sec(),
            last_nmea
        );
        if let Err(err) = session.append_line(file_name, &line) {
            println!("[GPS_LOG] write failed: {}", err);
            self.stop();
        }
    }
}

struct LoraSdLogger {
    session: Option<sdcard::SdCardSession>,
    file_name: Option<String>,
    last_log_ms: u32,
}

impl LoraSdLogger {
    fn new() -> Self {
        Self {
            session: None,
            file_name: None,
            last_log_ms: 0,
        }
    }

    fn is_active(&self) -> bool {
        self.session.is_some() && self.file_name.is_some()
    }

    fn file_name(&self) -> Option<&str> {
        self.file_name.as_deref()
    }

    fn current_unix_secs_system() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .ok()
            .map(|d| d.as_secs())
            .unwrap_or(0)
    }

    fn make_file_name(unix_secs: u64, run_id: u32) -> String {
        let (year, month, day, hour, minute, second) = unix_to_utc_ymdhms(unix_secs);
        format!("lora_{year:04}{month:02}{day:02}_{hour:02}{minute:02}{second:02}_{run_id:08}.txt")
    }

    fn make_file_name_83(unix_secs: u64, run_id: u32) -> String {
        let (_year, _month, day, hour, minute, _second) = unix_to_utc_ymdhms(unix_secs);
        let n = run_id % 10;
        format!("l{day:02}{hour:02}{minute:02}{n}.txt")
    }

    fn start(&mut self, now_ms: u32) -> Result<()> {
        self.stop();
        let session = sdcard::SdCardSession::mount()?;
        let unix_secs = Self::current_unix_secs_system();
        let run_id = now_ms;
        let long_file_name = Self::make_file_name(unix_secs, run_id);
        let short_file_name = Self::make_file_name_83(unix_secs, run_id);
        let file_name = match session.append_line(
            &long_file_name,
            "ts_ms;profile;baud;tx_packets;tx_bytes;rx_bytes;rx_lines;ping_seen;last_ping_seq;ff_only;cmd_status;cmd_hex;last_line;rx_hex",
        ) {
            Ok(()) => long_file_name,
            Err(err) => {
                println!(
                    "[LORA_LOG] long filename failed ({}), fallback to /sdcard/{}",
                    err, short_file_name
                );
                session.append_line(
                    &short_file_name,
                    "ts_ms;profile;baud;tx_packets;tx_bytes;rx_bytes;rx_lines;ping_seen;last_ping_seq;ff_only;cmd_status;cmd_hex;last_line;rx_hex",
                )?;
                short_file_name
            }
        };
        println!("[LORA_LOG] start file=/sdcard/{}", file_name);
        self.session = Some(session);
        self.file_name = Some(file_name);
        self.last_log_ms = now_ms.wrapping_sub(LORA_LOG_INTERVAL_MS);
        Ok(())
    }

    fn stop(&mut self) {
        if let Some(file_name) = self.file_name.as_deref() {
            println!("[LORA_LOG] close file=/sdcard/{}", file_name);
        }
        self.session = None;
        self.file_name = None;
        self.last_log_ms = 0;
    }

    fn poll_log(&mut self, now_ms: u32, lora: &lora::LoRaTester<'_>) {
        let Some(session) = self.session.as_ref() else {
            return;
        };
        let Some(file_name) = self.file_name.as_deref() else {
            return;
        };
        if now_ms.wrapping_sub(self.last_log_ms) < LORA_LOG_INTERVAL_MS {
            return;
        }
        self.last_log_ms = now_ms;
        let last_line = lora
            .last_line()
            .replace(';', ",")
            .replace('\r', " ")
            .replace('\n', " ");
        let cmd_hex = lora.cmd_probe_hex().replace(';', ",");
        let rx_hex = lora.recent_hex().replace(';', ",");
        let last_ping_seq = lora
            .last_ping_seq()
            .map(|v| format!("{v:03}"))
            .unwrap_or_else(|| "-".to_string());
        let line = format!(
            "{};{};{};{};{};{};{};{};{};{};{};{};{};{}",
            now_ms,
            "E220-default/873.125MHz/Air2.4k",
            lora.current_baud(),
            lora.tx_packets(),
            lora.tx_bytes(),
            lora.rx_bytes(),
            lora.rx_lines(),
            lora.ping_seen(),
            last_ping_seq,
            if lora.ff_only_alert() { 1 } else { 0 },
            lora.cmd_probe_status(),
            cmd_hex,
            last_line,
            rx_hex
        );
        if let Err(err) = session.append_line(file_name, &line) {
            println!("[LORA_LOG] write failed: {}", err);
            self.stop();
        }
    }
}

fn format_lora_log_info(logger: &LoraSdLogger) -> String {
    if let Some(file_name) = logger.file_name() {
        return format!("LOG on /sdcard/{}", file_name);
    }
    "LOG off".to_string()
}

fn draw_logging_screen(
    panel: sys::esp_lcd_panel_handle_t,
    title: &str,
    enabled: bool,
    file_name: Option<&str>,
) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, title, true)?;
    draw_text_box(
        panel,
        MENU_BTN_X,
        90,
        MENU_BTN_W,
        28,
        if enabled {
            "Stop Logging"
        } else {
            "Start Logging"
        },
        if enabled {
            Rgb565::new(63, 16, 0)
        } else {
            Rgb565::new(0, 63, 0)
        },
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        136,
        LCD_H_RES,
        12,
        if enabled { "Status: ON" } else { "Status: OFF" },
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    let file = file_name.unwrap_or("-");
    draw_text_box_small(
        panel,
        0,
        152,
        LCD_H_RES,
        12,
        &format!("File: {}", truncate_chars(file, 34)),
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    Ok(())
}

fn draw_gps_logging_screen(
    panel: sys::esp_lcd_panel_handle_t,
    enabled: bool,
    file_name: Option<&str>,
    gps: &gps::GpsReader<'_>,
) -> Result<()> {
    draw_logging_screen(panel, "GPS Logging", enabled, file_name)?;
    let sats = gps
        .sats()
        .map(|v| v.to_string())
        .unwrap_or_else(|| "-".to_string());
    let lat = gps
        .lat()
        .map(|v| format!("{:.6}", v))
        .unwrap_or_else(|| "-".to_string());
    let lon = gps
        .lon()
        .map(|v| format!("{:.6}", v))
        .unwrap_or_else(|| "-".to_string());
    draw_text_box_small(
        panel,
        0,
        178,
        LCD_H_RES,
        12,
        &format!(
            "Fix:{} Sats:{} Baud:{}",
            if gps.fix() { 1 } else { 0 },
            sats,
            gps.current_baud()
        ),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        194,
        LCD_H_RES,
        12,
        &format!(
            "Lat:{} Lon:{}",
            truncate_chars(&lat, 10),
            truncate_chars(&lon, 10)
        ),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        210,
        LCD_H_RES,
        12,
        &format!(
            "RawBps:{} Last:{}",
            gps.raw_bytes_per_sec(),
            truncate_chars(gps.last_sentence(), 16)
        ),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}

fn draw_lora_logging_screen(
    panel: sys::esp_lcd_panel_handle_t,
    enabled: bool,
    file_name: Option<&str>,
    lora: &lora::LoRaTester<'_>,
) -> Result<()> {
    draw_logging_screen(panel, "LoRa Logging", enabled, file_name)?;
    draw_text_box_small(
        panel,
        0,
        178,
        LCD_H_RES,
        12,
        "E220 default: 873.125MHz Air 2.4k",
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        194,
        LCD_H_RES,
        12,
        &format!(
            "TX pkt:{} TX bytes:{} baud:{}",
            lora.tx_packets(),
            lora.tx_bytes(),
            lora.current_baud()
        ),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        210,
        LCD_H_RES,
        12,
        &format!(
            "RX bytes:{} lines:{} ping:{} last:#{}",
            lora.rx_bytes(),
            lora.rx_lines(),
            lora.ping_seen(),
            lora.last_ping_seq()
                .map(|v| format!("{v:03}"))
                .unwrap_or_else(|| "---".to_string())
        ),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    draw_text_box_small(
        panel,
        0,
        226,
        LCD_H_RES,
        12,
        &format!(
            "Last:{} ff:{}",
            truncate_chars(lora.last_line(), 20),
            if lora.ff_only_alert() { "ERR" } else { "OK" }
        ),
        Rgb565::new(0, 63, 0),
        Rgb565::BLACK,
    )?;
    Ok(())
}

fn ntp_sync_time(timeout_ms: u32) -> Result<bool> {
    let server = b"pool.ntp.org\0";
    let mut cfg = sys::esp_sntp_config_t::default();
    cfg.smooth_sync = false;
    cfg.server_from_dhcp = false;
    cfg.wait_for_sync = true;
    cfg.start = true;
    cfg.sync_cb = None;
    cfg.renew_servers_after_new_IP = false;
    cfg.ip_event_to_renew = sys::ip_event_t_IP_EVENT_STA_GOT_IP;
    cfg.index_of_first_server = 0;
    cfg.num_of_servers = 1;
    cfg.servers[0] = server.as_ptr() as *const c_char;

    unsafe {
        // Reinit to ensure clean state when reconnecting.
        sys::esp_netif_sntp_deinit();
    }
    esp_ok(unsafe { sys::esp_netif_sntp_init(&cfg) })?;
    let wait_res = unsafe { sys::esp_netif_sntp_sync_wait(timeout_ms as _) };
    unsafe { sys::esp_netif_sntp_deinit() };

    if wait_res == sys::ESP_OK {
        let secs = GpsSdLogger::current_unix_secs_system().unwrap_or(0);
        println!("[TIME] NTP sync ok unix={}", secs);
        return Ok(true);
    }
    println!("[TIME] NTP sync failed code={}", wait_res);
    Ok(false)
}

fn unix_to_utc_ymdhms(unix_secs: u64) -> (i32, u32, u32, u32, u32, u32) {
    let day_secs = 86_400u64;
    let days = (unix_secs / day_secs) as i64;
    let sec_of_day = (unix_secs % day_secs) as u32;
    let hour = sec_of_day / 3_600;
    let minute = (sec_of_day % 3_600) / 60;
    let second = sec_of_day % 60;
    let (year, month, day) = civil_from_days(days);
    (year, month, day, hour, minute, second)
}

fn civil_from_days(days_since_unix_epoch: i64) -> (i32, u32, u32) {
    // Howard Hinnant's civil-from-days algorithm; epoch is 1970-01-01.
    let z = days_since_unix_epoch + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = mp + if mp < 10 { 3 } else { -9 };
    let year = y + if m <= 2 { 1 } else { 0 };
    (year as i32, m as u32, d as u32)
}

fn draw_boot_screen(panel: sys::esp_lcd_panel_handle_t, lines: &[String]) -> Result<()> {
    clear_screen(panel, Rgb565::BLACK)?;
    draw_header(panel, "Boot", false)?;
    let max_lines = 16usize;
    let start = lines.len().saturating_sub(max_lines);
    for (row, line) in lines[start..].iter().enumerate() {
        draw_text_box_small(
            panel,
            0,
            36 + (row as i32 * 14),
            LCD_H_RES,
            12,
            &truncate_chars(line, 38),
            Rgb565::WHITE,
            Rgb565::BLACK,
        )?;
    }
    Ok(())
}

fn push_boot_log(
    panel: sys::esp_lcd_panel_handle_t,
    logs: &mut Vec<String>,
    line: impl Into<String>,
) -> Result<()> {
    logs.push(line.into());
    draw_boot_screen(panel, logs)
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

    // Read battery once at boot for dashboard and battery screen.
    let mut battery_cache = if BAT_ADC_GPIO >= 0 {
        read_battery_once(BAT_ADC_GPIO).ok()
    } else {
        None
    };
    remote::set_battery(format_battery_dashboard(battery_cache.as_ref()));

    let panel = init_lcd()?;
    set_header_status_icons(false, false);
    let mut boot_logs: Vec<String> = Vec::new();
    push_boot_log(panel, &mut boot_logs, "Display initialisiert")?;

    push_boot_log(panel, &mut boot_logs, "Init NVS...")?;
    nvs_init()?;
    push_boot_log(panel, &mut boot_logs, "NVS OK")?;
    push_boot_log(panel, &mut boot_logs, "Init Touch...")?;
    let mut touch = Xpt2046::new()?;
    push_boot_log(panel, &mut boot_logs, "Touch OK")?;
    push_boot_log(panel, &mut boot_logs, "Lade Kalibrierung...")?;
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
    push_boot_log(panel, &mut boot_logs, "Kalibrierung bereit")?;
    let calibration = Calibration::from_points(&points);
    let mut touch = Some(touch);
    push_boot_log(panel, &mut boot_logs, "Init GPS...")?;
    let mut gps_reader = gps::GpsReader::new(p.uart1, p.pins.gpio16, p.pins.gpio15)?;
    let mut gps_sd_logger = GpsSdLogger::new();
    push_boot_log(panel, &mut boot_logs, "GPS OK")?;
    push_boot_log(panel, &mut boot_logs, "Init LoRa UART...")?;
    let mut lora_tester = lora::LoRaTester::new(p.uart2, p.pins.gpio17, p.pins.gpio18)?;
    let mut lora_sd_logger = LoraSdLogger::new();
    push_boot_log(panel, &mut boot_logs, "LoRa UART OK")?;
    let default_wifi = wifi_default_networks();
    let default_primary_ssid = default_wifi
        .first()
        .map(|x| x.ssid.clone())
        .unwrap_or_default();
    let default_primary_password = default_wifi
        .first()
        .map(|x| x.password.clone())
        .unwrap_or_default();

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
    let mut lora_last_redraw: u32 = 0;
    let mut gps_log_last_redraw: u32 = 0;
    let mut lora_log_last_redraw: u32 = 0;
    let mut http_server: Option<http_server::MiniHttpServer> = None;
    let mut http_status = String::from("Aus");
    let mut wifi_login_items: Vec<wifi::WifiAp> = Vec::new();
    let mut wifi_login_selected: Option<usize> = None;
    let mut wifi_login_offset: usize = 0;
    let mut wifi_login_status = String::from("Scan und Netz waehlen");
    let mut wifi_login_password = default_primary_password.clone();
    let mut wifi_login_char_idx: usize = 0;
    let mut gps_log_background = false;
    let mut lora_log_background = false;
    if !default_wifi.is_empty() {
        push_boot_log(
            panel,
            &mut boot_logs,
            format!("Auto WLAN: {} Netze", default_wifi.len()),
        )?;
        match connect_default_sequence(&default_wifi) {
            Ok((_info, used_idx)) => {
                push_boot_log(
                    panel,
                    &mut boot_logs,
                    format!("WLAN verbunden: {}", default_wifi[used_idx].ssid),
                )?;
                let wifi_connected_ip =
                    wifi::wifi_wait_ipv4(10_000).unwrap_or_else(|_| "-".to_string());
                set_header_status_icons(true, false);

                if wifi_connected_ip != "-" {
                    let _ = ntp_sync_time(10_000);
                    push_boot_log(
                        panel,
                        &mut boot_logs,
                        format!("IPv4: {}", wifi_connected_ip),
                    )?;
                    match http_server::MiniHttpServer::start(8080) {
                        Ok(server) => {
                            let port = server.port();
                            http_server = Some(server);
                            http_status = format!("AN {}:{}", wifi_connected_ip, port);
                            println!(
                                "[HTTP] auto-started at http://{}:{}/sd (default #{})",
                                wifi_connected_ip,
                                port,
                                used_idx + 1
                            );
                            set_header_status_icons(true, true);
                            push_boot_log(panel, &mut boot_logs, "HTTPD gestartet")?;
                        }
                        Err(err) => {
                            http_status = format!("AutoStart Fehler: {}", err);
                            println!("[HTTP] auto-start failed: {}", err);
                            set_header_status_icons(true, false);
                            push_boot_log(panel, &mut boot_logs, format!("HTTPD Fehler: {}", err))?;
                        }
                    }
                } else {
                    http_status = String::from("Kein IPv4 auf STA");
                    push_boot_log(panel, &mut boot_logs, "IPv4 fehlt")?;
                }
            }
            Err(err) => {
                println!("[WIFI_BOOT] auto default connect failed: {}", err);
                http_status = String::from("Aus");
                set_header_status_icons(false, false);
                push_boot_log(panel, &mut boot_logs, format!("WLAN Fehler: {}", err))?;
            }
        }
    } else {
        push_boot_log(panel, &mut boot_logs, "Kein Default WLAN konfiguriert")?;
    }
    push_boot_log(panel, &mut boot_logs, "Start Menu...")?;
    draw_menu(panel)?;

    loop {
        for cmd in remote::drain() {
            match cmd {
                remote::RemoteCommand::GotoMenu => {
                    screen = Screen::Menu;
                    draw_menu(panel)?;
                    remote::set_status("ok: menu");
                }
                remote::RemoteCommand::GotoWifiMenu => {
                    screen = Screen::WifiMenu;
                    draw_wifi_menu(panel)?;
                    remote::set_status("ok: wifi_menu");
                }
                remote::RemoteCommand::GotoDeviceMenu => {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                    remote::set_status("ok: device_menu");
                }
                remote::RemoteCommand::GotoGps => {
                    screen = Screen::Gps;
                    gps_back_to_device = true;
                    gps_last_redraw = tick_ms;
                    gps_reader.set_host_log_enabled(true);
                    if !gps_sd_logger.is_active() {
                        if let Err(err) = gps_sd_logger.start(tick_ms, &gps_reader) {
                            remote::set_status(format!("error: gps log start: {}", err));
                            continue;
                        }
                    }
                    gps::draw_gps_frame(panel)?;
                    gps::draw_gps_values(panel, &gps_reader)?;
                    remote::set_status("ok: gps");
                }
                remote::RemoteCommand::GotoBattery => {
                    screen = Screen::BatteryStatus;
                    battery_cache = if BAT_ADC_GPIO >= 0 {
                        read_battery_once(BAT_ADC_GPIO).ok()
                    } else {
                        None
                    };
                    remote::set_battery(format_battery_dashboard(battery_cache.as_ref()));
                    battery_last_read = 0;
                    battery_ignore_back_until = tick_ms.wrapping_add(1200);
                    battery_ignore_back = true;
                    battery_labels = Some(draw_battery_status_frame(panel)?);
                    remote::set_status("ok: battery");
                }
                remote::RemoteCommand::GotoLoopback => {
                    screen = Screen::UartLoopback;
                    gps_reader.set_host_log_enabled(false);
                    gps_reader.loopback_reset();
                    loopback_last_redraw = tick_ms;
                    gps::draw_uart_loopback_frame(panel)?;
                    gps::draw_uart_loopback_values(panel, &gps_reader)?;
                    remote::set_status("ok: loopback");
                }
                remote::RemoteCommand::GotoHttpMenu => {
                    screen = Screen::HttpMenu;
                    let ip_hint = wifi::wifi_sta_ipv4().ok().filter(|ip| ip != "0.0.0.0");
                    draw_http_menu(
                        panel,
                        http_server.is_some(),
                        &http_status,
                        ip_hint.as_deref(),
                    )?;
                    remote::set_status("ok: http_menu");
                }
                remote::RemoteCommand::GotoWifiLogin => {
                    screen = Screen::WifiLogin;
                    match wifi::wifi_scan_records() {
                        Ok(items) => {
                            wifi_login_items = items;
                            wifi_login_selected = wifi_login_items
                                .iter()
                                .position(|ap| {
                                    !default_primary_ssid.is_empty()
                                        && ap.ssid == default_primary_ssid
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
                    remote::set_status("ok: wifi_login");
                }
                remote::RemoteCommand::RunSdFormat => {
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
                    let (sd_result_ok, sd_result_msg) =
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
                            Ok(()) => (true, String::from("OK")),
                            Err(err) => (false, format!("{}", err)),
                        };
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
                        remote::set_status("ok: sd_format");
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
                        remote::set_status(format!("error: sd_format: {}", sd_result_msg));
                    }
                }
                remote::RemoteCommand::ToggleHttp => {
                    if http_server.is_none() {
                        match wifi::wifi_sta_ipv4() {
                            Ok(ip) if ip != "0.0.0.0" => {
                                match http_server::MiniHttpServer::start(8080) {
                                    Ok(server) => {
                                        let port = server.port();
                                        http_server = Some(server);
                                        http_status = format!("AN {}:{}", ip, port);
                                        set_header_status_icons(true, true);
                                        remote::set_status("ok: http start");
                                    }
                                    Err(err) => {
                                        http_status = format!("Fehler: {}", err);
                                        set_header_status_icons(true, false);
                                        remote::set_status(format!("error: http start: {}", err));
                                    }
                                }
                            }
                            Ok(_) => {
                                http_status = String::from("Kein IPv4 auf STA");
                                set_header_status_icons(false, http_server.is_some());
                                remote::set_status("error: no ipv4");
                            }
                            Err(err) => {
                                http_status = format!("IP Fehler: {}", err);
                                set_header_status_icons(false, http_server.is_some());
                                remote::set_status(format!("error: ip check: {}", err));
                            }
                        }
                    } else if let Some(mut server) = http_server.take() {
                        server.stop();
                        http_status = String::from("Aus");
                        let wifi_on = wifi::wifi_sta_ipv4()
                            .ok()
                            .map(|ip| ip != "0.0.0.0")
                            .unwrap_or(false);
                        set_header_status_icons(wifi_on, false);
                        remote::set_status("ok: http stop");
                    }
                }
                remote::RemoteCommand::WifiScan => match wifi::wifi_scan() {
                    Ok(items) => {
                        screen = Screen::WifiList;
                        draw_wifi_screen(panel, &items)?;
                        remote::set_status(format!("ok: wifi_scan {} ap", items.len()));
                    }
                    Err(err) => remote::set_status(format!("error: wifi_scan: {}", err)),
                },
                remote::RemoteCommand::WifiMonitor => {
                    screen = Screen::WifiMonitor;
                    monitor_series.clear();
                    monitor_last_scan = 0;
                    draw_wifi_monitor(panel, &[], RSSI_MIN, RSSI_MAX)?;
                    remote::set_status("ok: wifi_monitor");
                }
                remote::RemoteCommand::WifiChannelMonitor => {
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
                    remote::set_status(format!(
                        "ok: wifi_channel_monitor ch{}",
                        channel_monitor_channel
                    ));
                }
                remote::RemoteCommand::WifiMonitorStop => {
                    let _ = wifi::wifi_monitor_stop();
                    screen = Screen::WifiMenu;
                    draw_wifi_menu(panel)?;
                    remote::set_status("ok: wifi_monitor_stop");
                }
                remote::RemoteCommand::WifiConnectDefault => {
                    match connect_default_sequence(&default_wifi) {
                        Ok((info, used_idx)) => {
                            let ssid = if info.ssid.is_empty() {
                                default_wifi[used_idx].ssid.clone()
                            } else {
                                info.ssid
                            };
                            let ip =
                                wifi::wifi_wait_ipv4(8_000).unwrap_or_else(|_| "-".to_string());
                            if ip != "-" {
                                let _ = ntp_sync_time(10_000);
                            }
                            set_header_status_icons(true, http_server.is_some());
                            screen = Screen::WifiConnected;
                            draw_wifi_connected_screen(panel, &ssid, info.rssi, info.channel, &ip)?;
                            remote::set_status(format!("ok: wifi_default {} {}", ssid, ip));
                        }
                        Err(err) => remote::set_status(format!("error: wifi_default: {}", err)),
                    }
                }
                remote::RemoteCommand::WifiConnect { ssid, password } => {
                    match connect_with_fallback(&ssid, &password, 12_000) {
                        Ok(info) => {
                            let shown_ssid = if info.ssid.is_empty() {
                                ssid
                            } else {
                                info.ssid
                            };
                            let ip =
                                wifi::wifi_wait_ipv4(8_000).unwrap_or_else(|_| "-".to_string());
                            if ip != "-" {
                                let _ = ntp_sync_time(10_000);
                            }
                            set_header_status_icons(true, http_server.is_some());
                            screen = Screen::WifiConnected;
                            draw_wifi_connected_screen(
                                panel,
                                &shown_ssid,
                                info.rssi,
                                info.channel,
                                &ip,
                            )?;
                            remote::set_status(format!("ok: wifi_connect {} {}", shown_ssid, ip));
                        }
                        Err(err) => remote::set_status(format!("error: wifi_connect: {}", err)),
                    }
                }
            }
            was_pressed = false;
        }
        if !lora_log_background && !matches!(screen, Screen::LoRaTest) && lora_sd_logger.is_active()
        {
            lora_sd_logger.stop();
        }
        if !gps_log_background && !matches!(screen, Screen::Gps) && gps_sd_logger.is_active() {
            gps_sd_logger.stop();
        }

        if gps_log_background && !matches!(screen, Screen::Gps | Screen::UartLoopback) {
            let _ = gps_reader.poll(tick_ms)?;
            gps_sd_logger.poll_log(tick_ms, &gps_reader);
        }
        if lora_log_background && !matches!(screen, Screen::LoRaTest) {
            let _ = lora_tester.poll(tick_ms)?;
            lora_sd_logger.poll_log(tick_ms, &lora_tester);
        }

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
                    if let Err(err) = gps_sd_logger.start(tick_ms, &gps_reader) {
                        println!("[GPS_LOG] start failed: {}", err);
                    }
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
                let hit_gps_log = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN7_Y
                    && y < DEVICE_MENU_BTN7_Y + MENU_BTN_H;
                let hit_lora_log = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= DEVICE_MENU_BTN8_Y
                    && y < DEVICE_MENU_BTN8_Y + MENU_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::Menu;
                    draw_menu(panel)?;
                } else if hit_batt && !was_pressed {
                    screen = Screen::BatteryStatus;
                    battery_cache = if BAT_ADC_GPIO >= 0 {
                        read_battery_once(BAT_ADC_GPIO).ok()
                    } else {
                        None
                    };
                    remote::set_battery(format_battery_dashboard(battery_cache.as_ref()));
                    battery_last_read = 0;
                    battery_ignore_back_until = tick_ms.wrapping_add(1200);
                    battery_ignore_back = true;
                    battery_labels = Some(draw_battery_status_frame(panel)?);
                } else if hit_gps && !was_pressed {
                    screen = Screen::Gps;
                    gps_back_to_device = true;
                    gps_last_redraw = tick_ms;
                    gps_reader.set_host_log_enabled(true);
                    if !gps_sd_logger.is_active() {
                        if let Err(err) = gps_sd_logger.start(tick_ms, &gps_reader) {
                            println!("[GPS_LOG] start failed: {}", err);
                        }
                    }
                    gps::draw_gps_frame(panel)?;
                    gps::draw_gps_values(panel, &gps_reader)?;
                } else if hit_loopback && !was_pressed {
                    screen = Screen::LoRaTest;
                    lora_tester.reset();
                    if !lora_sd_logger.is_active() {
                        if let Err(err) = lora_sd_logger.start(tick_ms) {
                            println!("[LORA_LOG] start failed: {}", err);
                        }
                    }
                    lora_last_redraw = tick_ms;
                    lora::draw_lora_test_frame(panel)?;
                    lora::draw_lora_test_values(
                        panel,
                        &lora_tester,
                        &format_lora_log_info(&lora_sd_logger),
                    )?;
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
                    let (sd_result_ok, sd_result_msg) =
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
                            Ok(()) => (true, String::from("OK")),
                            Err(err) => (false, format!("{}", err)),
                        };
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
                    let ip_hint = wifi::wifi_sta_ipv4().ok().filter(|ip| ip != "0.0.0.0");
                    draw_http_menu(
                        panel,
                        http_server.is_some(),
                        &http_status,
                        ip_hint.as_deref(),
                    )?;
                } else if hit_wifi_login && !was_pressed {
                    screen = Screen::WifiLogin;
                    match wifi::wifi_scan_records() {
                        Ok(items) => {
                            wifi_login_items = items;
                            wifi_login_selected = wifi_login_items
                                .iter()
                                .position(|ap| {
                                    !default_primary_ssid.is_empty()
                                        && ap.ssid == default_primary_ssid
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
                } else if hit_gps_log && !was_pressed {
                    screen = Screen::GpsLogging;
                    gps_log_last_redraw = tick_ms;
                    draw_gps_logging_screen(
                        panel,
                        gps_log_background,
                        gps_sd_logger.file_name(),
                        &gps_reader,
                    )?;
                } else if hit_lora_log && !was_pressed {
                    screen = Screen::LoRaLogging;
                    lora_log_last_redraw = tick_ms;
                    draw_lora_logging_screen(
                        panel,
                        lora_log_background,
                        lora_sd_logger.file_name(),
                        &lora_tester,
                    )?;
                }
                was_pressed = hit_back
                    || hit_batt
                    || hit_gps
                    || hit_loopback
                    || hit_sd_format
                    || hit_http_menu
                    || hit_wifi_login
                    || hit_gps_log
                    || hit_lora_log;
            }
            Screen::Gps => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    gps_reader.set_host_log_enabled(false);
                    if !gps_log_background {
                        gps_sd_logger.stop();
                    }
                    if gps_back_to_device {
                        screen = Screen::DeviceMenu;
                        draw_device_menu(panel)?;
                    } else {
                        screen = Screen::Menu;
                        draw_menu(panel)?;
                    }
                } else {
                    let gps_changed = gps_reader.poll(tick_ms)?;
                    gps_sd_logger.poll_log(tick_ms, &gps_reader);
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
            Screen::LoRaTest => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    if !lora_log_background {
                        lora_sd_logger.stop();
                    }
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                } else {
                    let changed = lora_tester.poll(tick_ms)?;
                    lora_sd_logger.poll_log(tick_ms, &lora_tester);
                    if changed || tick_ms.wrapping_sub(lora_last_redraw) >= 300 {
                        lora::draw_lora_test_values(
                            panel,
                            &lora_tester,
                            &format_lora_log_info(&lora_sd_logger),
                        )?;
                        lora_last_redraw = tick_ms;
                    }
                }
                was_pressed = hit_back;
            }
            Screen::GpsLogging => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_toggle = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= 90
                    && y < 118;
                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                } else if hit_toggle && !was_pressed {
                    if gps_log_background {
                        gps_log_background = false;
                        if !matches!(screen, Screen::Gps) {
                            gps_sd_logger.stop();
                        }
                        remote::set_status("ok: gps_log off");
                    } else {
                        gps_log_background = true;
                        if !gps_sd_logger.is_active() {
                            match gps_sd_logger.start(tick_ms, &gps_reader) {
                                Ok(()) => remote::set_status("ok: gps_log on"),
                                Err(err) => {
                                    gps_log_background = false;
                                    remote::set_status(format!("error: gps_log start: {}", err));
                                }
                            }
                        } else {
                            remote::set_status("ok: gps_log on");
                        }
                    }
                    draw_gps_logging_screen(
                        panel,
                        gps_log_background,
                        gps_sd_logger.file_name(),
                        &gps_reader,
                    )?;
                    gps_log_last_redraw = tick_ms;
                } else if tick_ms.wrapping_sub(gps_log_last_redraw) >= 300 {
                    draw_gps_logging_screen(
                        panel,
                        gps_log_background,
                        gps_sd_logger.file_name(),
                        &gps_reader,
                    )?;
                    gps_log_last_redraw = tick_ms;
                }
                was_pressed = hit_back || hit_toggle;
            }
            Screen::LoRaLogging => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                let hit_toggle = touch_down
                    && x >= MENU_BTN_X
                    && x < MENU_BTN_X + MENU_BTN_W
                    && y >= 90
                    && y < 118;
                if hit_back && !was_pressed {
                    screen = Screen::DeviceMenu;
                    draw_device_menu(panel)?;
                } else if hit_toggle && !was_pressed {
                    if lora_log_background {
                        lora_log_background = false;
                        if !matches!(screen, Screen::LoRaTest) {
                            lora_sd_logger.stop();
                        }
                        remote::set_status("ok: lora_log off");
                    } else {
                        lora_log_background = true;
                        if !lora_sd_logger.is_active() {
                            match lora_sd_logger.start(tick_ms) {
                                Ok(()) => remote::set_status("ok: lora_log on"),
                                Err(err) => {
                                    lora_log_background = false;
                                    remote::set_status(format!("error: lora_log start: {}", err));
                                }
                            }
                        } else {
                            remote::set_status("ok: lora_log on");
                        }
                    }
                    draw_lora_logging_screen(
                        panel,
                        lora_log_background,
                        lora_sd_logger.file_name(),
                        &lora_tester,
                    )?;
                    lora_log_last_redraw = tick_ms;
                } else if tick_ms.wrapping_sub(lora_log_last_redraw) >= 300 {
                    draw_lora_logging_screen(
                        panel,
                        lora_log_background,
                        lora_sd_logger.file_name(),
                        &lora_tester,
                    )?;
                    lora_log_last_redraw = tick_ms;
                }
                was_pressed = hit_back || hit_toggle;
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
                        match wifi::wifi_sta_ipv4() {
                            Ok(ip) if ip != "0.0.0.0" => {
                                match http_server::MiniHttpServer::start(8080) {
                                    Ok(server) => {
                                        let port = server.port();
                                        http_server = Some(server);
                                        http_status = format!("AN {}:{}", ip, port);
                                        set_header_status_icons(true, true);
                                        println!(
                                            "[HTTP] server started at http://{}:{}/sd",
                                            ip, port
                                        );
                                    }
                                    Err(err) => {
                                        http_status = format!("Fehler: {}", err);
                                        set_header_status_icons(true, false);
                                        println!("[HTTP] start failed: {}", err);
                                    }
                                }
                            }
                            Ok(_) => {
                                http_status = String::from("Kein IPv4 auf STA");
                                set_header_status_icons(false, http_server.is_some());
                                println!("[HTTP] start blocked: no IPv4 on STA");
                            }
                            Err(err) => {
                                http_status = format!("IP Fehler: {}", err);
                                set_header_status_icons(false, http_server.is_some());
                                println!("[HTTP] IP check failed: {}", err);
                            }
                        }
                    } else if let Some(mut server) = http_server.take() {
                        server.stop();
                        http_status = String::from("Aus");
                        let wifi_on = wifi::wifi_sta_ipv4()
                            .ok()
                            .map(|ip| ip != "0.0.0.0")
                            .unwrap_or(false);
                        set_header_status_icons(wifi_on, false);
                    }

                    let ip_hint = wifi::wifi_sta_ipv4().ok().filter(|ip| ip != "0.0.0.0");
                    draw_http_menu(
                        panel,
                        http_server.is_some(),
                        &http_status,
                        ip_hint.as_deref(),
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
                                    !default_primary_ssid.is_empty()
                                        && ap.ssid == default_primary_ssid
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
                    if default_wifi.is_empty() {
                        wifi_login_status = String::from("Kein Default WLAN (wifi_secrets.local)");
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
                    wifi_login_status = format!("Default connect: {} Netze...", default_wifi.len());
                    draw_wifi_login_screen(
                        panel,
                        &wifi_login_items,
                        wifi_login_selected,
                        wifi_login_offset,
                        &wifi_login_status,
                        &wifi_login_password,
                        wifi_login_char_idx,
                    )?;

                    match connect_default_sequence(&default_wifi) {
                        Ok((info, used_idx)) => {
                            let wifi_connected_ssid = if info.ssid.is_empty() {
                                default_wifi[used_idx].ssid.clone()
                            } else {
                                info.ssid.clone()
                            };
                            let wifi_connected_rssi = info.rssi;
                            let wifi_connected_channel = info.channel;
                            let wifi_connected_ip =
                                wifi::wifi_wait_ipv4(8_000).unwrap_or_else(|_| "-".to_string());
                            if wifi_connected_ip != "-" {
                                let _ = ntp_sync_time(10_000);
                            }
                            set_header_status_icons(true, http_server.is_some());
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
                        }
                        Err(err) => {
                            wifi_login_status = format!("Default Fehler: {}", err);
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
                } else if hit_connect && !was_pressed {
                    if let Some(idx) = wifi_login_selected {
                        let ap = &wifi_login_items[idx];
                        if let Some(conflict) = default_wifi.iter().find(|d| {
                            !d.password.is_empty()
                                && ap.ssid != d.ssid
                                && wifi_login_password == d.password
                        }) {
                            wifi_login_status =
                                format!("Achtung: Default-PWD ist fuer {}", conflict.ssid);
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
                        println!(
                            "[WIFI_PATH] manual: selected AP -> wifi_connect (ssid='{}')",
                            ap.ssid
                        );
                        match wifi::wifi_connect(&ap.ssid, password) {
                            Ok(()) => match wifi::wifi_wait_connected(10_000) {
                                Ok(info) => connected_info = Some(info),
                                Err(err) => {
                                    connect_error = format!("connect wait: {}", err);
                                    println!("[WIFI] connect wait failed: {}", err);
                                }
                            },
                            Err(err) => {
                                connect_error = format!("connect: {}", err);
                                println!("[WIFI] connect failed: {}", err);
                            }
                        }

                        if connected_info.is_none() {
                            println!(
                                "[WIFI_PATH] manual: connect not connected -> fallback wifi_connect_ap (ssid='{}', ch={})",
                                ap.ssid, ap.channel
                            );
                            match wifi::wifi_connect_ap(ap, password) {
                                Ok(()) => match wifi::wifi_wait_connected(10_000) {
                                    Ok(info) => connected_info = Some(info),
                                    Err(err) => {
                                        if !connect_error.is_empty() {
                                            connect_error.push_str(" | ");
                                        }
                                        connect_error.push_str(&format!(
                                            "fallback connect_ap wait: {}",
                                            err
                                        ));
                                        println!("[WIFI] fallback connect_ap wait failed: {}", err);
                                    }
                                },
                                Err(err) => {
                                    if !connect_error.is_empty() {
                                        connect_error.push_str(" | ");
                                    }
                                    connect_error
                                        .push_str(&format!("fallback connect_ap: {}", err));
                                    println!("[WIFI] fallback connect_ap failed: {}", err);
                                }
                            }
                        }

                        if let Some(info) = connected_info {
                            let wifi_connected_ssid = if info.ssid.is_empty() {
                                selected_ssid
                            } else {
                                info.ssid.clone()
                            };
                            let wifi_connected_rssi = info.rssi;
                            let wifi_connected_channel = info.channel;
                            let wifi_connected_ip =
                                wifi::wifi_wait_ipv4(8_000).unwrap_or_else(|_| "-".to_string());
                            if wifi_connected_ip != "-" {
                                let _ = ntp_sync_time(10_000);
                            }
                            set_header_status_icons(true, http_server.is_some());
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
