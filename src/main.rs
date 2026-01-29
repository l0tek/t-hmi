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

mod ui;
mod wifi;

use ui::{
    clear_screen, draw_bitmap, draw_menu, draw_text_box, draw_wifi_frame, draw_wifi_message,
    draw_wifi_screen, BACK_BTN_H, BACK_BTN_W, BACK_BTN_X, BACK_BTN_Y, MENU_BTN1_Y, MENU_BTN_H,
    MENU_BTN_W, MENU_BTN_X,
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
    WifiList,
}

fn main() -> Result<()> {
    sys::link_patches();

    let p = Peripherals::take().unwrap();

    // Power and backlight control.
    let mut pwr_en = PinDriver::output(p.pins.gpio10)?;
    let mut pwr_on = PinDriver::output(p.pins.gpio14)?;
    let mut bl = PinDriver::output(p.pins.gpio38)?;
    pwr_en.set_high()?;
    pwr_on.set_high()?;
    bl.set_high()?;

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

    let mut was_pressed = false;
    let mut screen = Screen::Menu;
    loop {
        let raw = touch.read_raw()?;
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
                if hit_menu && !was_pressed {
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
                }
                was_pressed = hit_menu;
            }
            Screen::WifiList => {
                let hit_back = touch_down
                    && x >= BACK_BTN_X
                    && x < BACK_BTN_X + BACK_BTN_W
                    && y >= BACK_BTN_Y
                    && y < BACK_BTN_Y + BACK_BTN_H;
                if hit_back && !was_pressed {
                    screen = Screen::Menu;
                    draw_menu(panel)?;
                }
                was_pressed = hit_back;
            }
        }
        FreeRtos::delay_ms(20);
    }
}
