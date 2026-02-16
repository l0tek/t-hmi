use anyhow::{anyhow, Result};
use esp_idf_sys as sys;
use std::{ffi::CString, ptr};

const SD_CLK_GPIO: i32 = 12;
const SD_CMD_GPIO: i32 = 11;
const SD_D0_GPIO: i32 = 13;
const SDMMC_HOST_FLAG_1BIT: u32 = 1 << 0;
const SDMMC_HOST_FLAG_4BIT: u32 = 1 << 1;
const SDMMC_HOST_FLAG_8BIT: u32 = 1 << 2;
const SDMMC_HOST_FLAG_DDR: u32 = 1 << 4;
const SDMMC_SLOT_FLAG_INTERNAL_PULLUP: u32 = 1 << 0;

fn sdmmc_host_default() -> sys::sdmmc_host_t {
    let mut host = sys::sdmmc_host_t::default();
    host.flags =
        SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_4BIT | SDMMC_HOST_FLAG_8BIT | SDMMC_HOST_FLAG_DDR;
    host.slot = sys::SDMMC_HOST_SLOT_1 as i32;
    host.max_freq_khz = sys::SDMMC_FREQ_DEFAULT as i32;
    host.io_voltage = 3.3;
    host.init = Some(sys::sdmmc_host_init);
    host.set_bus_width = Some(sys::sdmmc_host_set_bus_width);
    host.get_bus_width = Some(sys::sdmmc_host_get_slot_width);
    host.set_bus_ddr_mode = Some(sys::sdmmc_host_set_bus_ddr_mode);
    host.set_card_clk = Some(sys::sdmmc_host_set_card_clk);
    host.set_cclk_always_on = Some(sys::sdmmc_host_set_cclk_always_on);
    host.do_transaction = Some(sys::sdmmc_host_do_transaction);
    host.__bindgen_anon_1 = sys::sdmmc_host_t__bindgen_ty_1 {
        deinit: Some(sys::sdmmc_host_deinit),
    };
    host.io_int_enable = Some(sys::sdmmc_host_io_int_enable);
    host.io_int_wait = Some(sys::sdmmc_host_io_int_wait);
    host.command_timeout_ms = 0;
    host.get_real_freq = Some(sys::sdmmc_host_get_real_freq);
    host
}

pub fn run_sdcard_format_test<F>(mut progress: F) -> Result<()>
where
    F: FnMut(u8, &str) -> Result<()>,
{
    let base_path = CString::new("/sdcard")?;
    let mut card: *mut sys::sdmmc_card_t = ptr::null_mut();

    let host = sdmmc_host_default();

    let mut slot = sys::sdmmc_slot_config_t::default();
    slot.clk = SD_CLK_GPIO;
    slot.cmd = SD_CMD_GPIO;
    slot.d0 = SD_D0_GPIO;
    slot.d1 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.d2 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.d3 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.d4 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.d5 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.d6 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.d7 = sys::gpio_num_t_GPIO_NUM_NC;
    slot.__bindgen_anon_1 = sys::sdmmc_slot_config_t__bindgen_ty_1 {
        cd: sys::gpio_num_t_GPIO_NUM_NC,
    };
    slot.__bindgen_anon_2 = sys::sdmmc_slot_config_t__bindgen_ty_2 {
        wp: sys::gpio_num_t_GPIO_NUM_NC,
    };
    slot.width = 1;
    slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    let mount_cfg = sys::esp_vfs_fat_mount_config_t {
        format_if_mount_failed: false,
        max_files: 4,
        allocation_unit_size: 0,
        disk_status_check_enable: false,
    };

    progress(5, "Initialisiere...")?;
    crate::esp_ok(unsafe {
        sys::esp_vfs_fat_sdmmc_mount(
            base_path.as_ptr(),
            &host,
            (&slot as *const _) as *const _,
            &mount_cfg,
            &mut card,
        )
    })?;
    progress(25, "SD erkannt")?;

    let test_result = (|| -> Result<()> {
        progress(40, "Formatiere...")?;
        crate::esp_ok(unsafe { sys::esp_vfs_fat_sdcard_format(base_path.as_ptr(), card) })?;
        progress(75, "Schreibe test.txt...")?;
        std::fs::write("/sdcard/test.txt", "OK")?;
        progress(90, "Lese test.txt...")?;
        let content = std::fs::read_to_string("/sdcard/test.txt")?;
        if content.trim() == "OK" {
            Ok(())
        } else {
            Err(anyhow!("test.txt Inhalt ungueltig: {}", content.trim()))
        }
    })();

    let unmount_result =
        crate::esp_ok(unsafe { sys::esp_vfs_fat_sdcard_unmount(base_path.as_ptr(), card) });
    if let Err(err) = test_result {
        let _ = unmount_result;
        return Err(err);
    }
    unmount_result?;
    progress(100, "Fertig")?;
    Ok(())
}
