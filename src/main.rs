use anyhow::{anyhow, Result};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use esp_idf_hal::{
    delay::Ets,
    gpio::{PinDriver, Pull},
    peripherals::Peripherals,
};
use esp_idf_sys as sys;
use std::{convert::Infallible, ptr};

const LCD_H_RES: i32 = 240;
const LCD_V_RES: i32 = 320;

const LCD_DATA_PINS: [i32; 8] = [48, 47, 39, 40, 41, 42, 45, 46];
const LCD_SCLK: i32 = 8;
const LCD_CS: i32 = 6;
const LCD_DC: i32 = 7;

const BUTTON_X: i32 = 20;
const BUTTON_Y: i32 = 220;
const BUTTON_W: i32 = 200;
const BUTTON_H: i32 = 70;

fn esp_ok(code: sys::esp_err_t) -> Result<()> {
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

struct FrameBuffer<'a> {
    width: i32,
    height: i32,
    data: &'a mut [u16],
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
    panel_config.data_endian = sys::lcd_rgb_data_endian_t_LCD_RGB_DATA_ENDIAN_BIG;
    panel_config.bits_per_pixel = 16;
    panel_config.flags = sys::esp_lcd_panel_dev_config_t__bindgen_ty_2::default();
    panel_config.vendor_config = ptr::null_mut();

    esp_ok(unsafe { sys::esp_lcd_new_panel_st7789(io_handle, &panel_config, &mut panel_handle) })?;
    esp_ok(unsafe { sys::esp_lcd_panel_reset(panel_handle) })?;
    esp_ok(unsafe { sys::esp_lcd_panel_init(panel_handle) })?;
    esp_ok(unsafe { sys::esp_lcd_panel_disp_on_off(panel_handle, true) })?;

    Ok(panel_handle)
}

fn draw_bitmap(
    panel: sys::esp_lcd_panel_handle_t,
    x: i32,
    y: i32,
    w: i32,
    h: i32,
    buffer: &[u16],
) -> Result<()> {
    let x_end = x + w;
    let y_end = y + h;
    esp_ok(unsafe {
        sys::esp_lcd_panel_draw_bitmap(panel, x, y, x_end, y_end, buffer.as_ptr() as *const _)
    })
}

fn draw_text_box(
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

fn draw_button(panel: sys::esp_lcd_panel_handle_t) -> Result<()> {
    draw_text_box(
        panel,
        BUTTON_X,
        BUTTON_Y,
        BUTTON_W,
        BUTTON_H,
        "TOUCH",
        Rgb565::WHITE,
        Rgb565::BLUE,
    )
}

fn clear_screen(panel: sys::esp_lcd_panel_handle_t, color: Rgb565) -> Result<()> {
    let buffer = vec![color.into_storage(); (LCD_H_RES * LCD_V_RES) as usize];
    draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, &buffer)
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

    let mut irq = PinDriver::input(p.pins.gpio9)?;
    irq.set_pull(Pull::Up)?;

    let panel = init_lcd()?;

    clear_screen(panel, Rgb565::BLACK)?;
    draw_text_box(
        panel,
        10,
        20,
        220,
        40,
        "Touch the button",
        Rgb565::WHITE,
        Rgb565::BLACK,
    )?;
    draw_button(panel)?;

    let mut was_pressed = false;
    loop {
        let pressed = irq.is_low();
        if pressed && !was_pressed {
            draw_text_box(
                panel,
                10,
                80,
                220,
                40,
                "Hello World",
                Rgb565::WHITE,
                Rgb565::BLACK,
            )?;
        }
        was_pressed = pressed;
        Ets::delay_ms(20);
    }
}
