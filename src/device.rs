use anyhow::{anyhow, Result};
use esp_idf_sys as sys;
use std::ptr;

pub const BATTERY_DIVIDER_SCALE: f32 = 3.0;
pub const BATTERY_EMPTY_V: f32 = 3.30;
pub const BATTERY_FULL_V: f32 = 4.20;

pub struct BatteryStatus {
    pub raw: i32,
    pub v_adc_mv: i32,
    pub v_bat_mv: i32,
    pub percent: u8,
    pub calibrated: bool,
}

pub struct BatteryReader {
    unit: sys::adc_oneshot_unit_handle_t,
    chan: sys::adc_channel_t,
    cali: sys::adc_cali_handle_t,
    calibrated: bool,
}

impl BatteryReader {
    pub fn new(bat_gpio: i32) -> Result<Self> {
        if bat_gpio < 0 {
            return Err(anyhow!("BAT_ADC disabled"));
        }
        let mut unit_id = sys::adc_unit_t_ADC_UNIT_1;
        let mut chan = sys::adc_channel_t_ADC_CHANNEL_0;
        let io_err = unsafe { sys::adc_oneshot_io_to_channel(bat_gpio, &mut unit_id, &mut chan) };
        if io_err != sys::ESP_OK {
            return Err(anyhow!("BAT_ADC GPIO{} is not an ADC pin", bat_gpio));
        }

        let mut unit: sys::adc_oneshot_unit_handle_t = ptr::null_mut();
        let mut cfg = sys::adc_oneshot_unit_init_cfg_t::default();
        cfg.unit_id = unit_id;
        cfg.clk_src =
            sys::soc_periph_adc_rtc_clk_src_t_ADC_RTC_CLK_SRC_DEFAULT as sys::adc_oneshot_clk_src_t;
        cfg.ulp_mode = sys::adc_ulp_mode_t_ADC_ULP_MODE_DISABLE;
        esp_ok(unsafe { sys::adc_oneshot_new_unit(&cfg, &mut unit) })?;

        let mut ch_cfg = sys::adc_oneshot_chan_cfg_t::default();
        ch_cfg.atten = sys::adc_atten_t_ADC_ATTEN_DB_11;
        ch_cfg.bitwidth = sys::adc_bitwidth_t_ADC_BITWIDTH_DEFAULT;
        esp_ok(unsafe { sys::adc_oneshot_config_channel(unit, chan, &ch_cfg) })?;

        let mut cali: sys::adc_cali_handle_t = ptr::null_mut();
        let mut calibrated = false;
        let mut mask: sys::adc_cali_scheme_ver_t = 0;
        if unsafe { sys::adc_cali_check_scheme(&mut mask) } == sys::ESP_OK
            && (mask & sys::adc_cali_scheme_ver_t_ADC_CALI_SCHEME_VER_CURVE_FITTING) != 0
        {
            let mut cal_cfg = sys::adc_cali_curve_fitting_config_t::default();
            cal_cfg.unit_id = unit_id;
            cal_cfg.chan = chan;
            cal_cfg.atten = ch_cfg.atten;
            cal_cfg.bitwidth = ch_cfg.bitwidth;
            if unsafe { sys::adc_cali_create_scheme_curve_fitting(&cal_cfg, &mut cali) }
                == sys::ESP_OK
            {
                calibrated = true;
            }
        }

        Ok(Self {
            unit,
            chan,
            cali,
            calibrated,
        })
    }

    pub fn read_status(&mut self) -> Result<BatteryStatus> {
        let raw = self.read_raw()?;
        let v_adc_mv = self.read_adc_mv_from_raw(raw)?;
        let v_bat_mv = (v_adc_mv as f32 * BATTERY_DIVIDER_SCALE) as i32;
        let v_bat_v = v_bat_mv as f32 / 1000.0;
        let percent = battery_percent(v_bat_v);
        Ok(BatteryStatus {
            raw,
            v_adc_mv,
            v_bat_mv,
            percent,
            calibrated: self.calibrated,
        })
    }

    fn read_raw(&mut self) -> Result<i32> {
        let mut raw: i32 = 0;
        esp_ok(unsafe { sys::adc_oneshot_read(self.unit, self.chan, &mut raw) })?;
        Ok(raw)
    }

    fn read_adc_mv_from_raw(&mut self, raw: i32) -> Result<i32> {
        if self.calibrated {
            let mut mv: i32 = 0;
            let err = unsafe {
                sys::adc_oneshot_get_calibrated_result(self.unit, self.cali, self.chan, &mut mv)
            };
            if err == sys::ESP_OK {
                return Ok(mv);
            }
        }
        Ok((raw * 3300) / 4095)
    }
}

impl Drop for BatteryReader {
    fn drop(&mut self) {
        if self.calibrated && !self.cali.is_null() {
            unsafe {
                sys::adc_cali_delete_scheme_curve_fitting(self.cali);
            }
        }
        if !self.unit.is_null() {
            unsafe {
                sys::adc_oneshot_del_unit(self.unit);
            }
        }
    }
}

pub fn read_battery_once(bat_gpio: i32) -> Result<BatteryStatus> {
    let mut reader = BatteryReader::new(bat_gpio)?;
    reader.read_status()
}

fn battery_percent(vbat_v: f32) -> u8 {
    let pct = ((vbat_v - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)) * 100.0;
    pct.round().clamp(0.0, 100.0) as u8
}

fn esp_ok(code: sys::esp_err_t) -> Result<()> {
    if code == sys::ESP_OK {
        Ok(())
    } else if let Some(err) = sys::EspError::from(code) {
        Err(err.into())
    } else {
        Err(anyhow!("ESP error code: {}", code))
    }
}
