use anyhow::{anyhow, Result};
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};
use esp_idf_sys as sys;

fn esp_ok_ctx(code: sys::esp_err_t, ctx: &str) -> Result<()> {
    esp_ok_ctx_allow(code, ctx, &[])
}

fn wifi_init_cfg_default() -> sys::wifi_init_config_t {
    sys::wifi_init_config_t {
        osi_funcs: unsafe { &mut sys::g_wifi_osi_funcs },
        wpa_crypto_funcs: unsafe { sys::g_wifi_default_wpa_crypto_funcs },
        static_rx_buf_num: sys::CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM as i32,
        dynamic_rx_buf_num: sys::CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM as i32,
        tx_buf_type: sys::CONFIG_ESP32_WIFI_TX_BUFFER_TYPE as i32,
        static_tx_buf_num: sys::WIFI_STATIC_TX_BUFFER_NUM as i32,
        dynamic_tx_buf_num: sys::WIFI_DYNAMIC_TX_BUFFER_NUM as i32,
        rx_mgmt_buf_type: sys::CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF as i32,
        rx_mgmt_buf_num: sys::WIFI_RX_MGMT_BUF_NUM_DEF as i32,
        cache_tx_buf_num: sys::WIFI_CACHE_TX_BUFFER_NUM as i32,
        csi_enable: sys::WIFI_CSI_ENABLED as i32,
        ampdu_rx_enable: sys::WIFI_AMPDU_RX_ENABLED as i32,
        ampdu_tx_enable: sys::WIFI_AMPDU_TX_ENABLED as i32,
        amsdu_tx_enable: sys::WIFI_AMSDU_TX_ENABLED as i32,
        nvs_enable: sys::WIFI_NVS_ENABLED as i32,
        nano_enable: sys::WIFI_NANO_FORMAT_ENABLED as i32,
        rx_ba_win: sys::WIFI_DEFAULT_RX_BA_WIN as i32,
        wifi_task_core_id: sys::WIFI_TASK_CORE_ID as i32,
        beacon_max_len: sys::WIFI_SOFTAP_BEACON_MAX_LEN as i32,
        mgmt_sbuf_num: sys::WIFI_MGMT_SBUF_NUM as i32,
        feature_caps: sys::WIFI_FEATURE_CAPS as u64,
        sta_disconnected_pm: sys::WIFI_STA_DISCONNECTED_PM_ENABLED != 0,
        espnow_max_encrypt_num: sys::CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM as i32,
        magic: sys::WIFI_INIT_CONFIG_MAGIC as i32,
    }
}

fn esp_ok_ctx_allow(code: sys::esp_err_t, ctx: &str, allow: &[sys::esp_err_t]) -> Result<()> {
    if code == sys::ESP_OK || allow.iter().any(|&c| c == code) {
        Ok(())
    } else if let Some(err) = sys::EspError::from(code) {
        Err(anyhow!("{}: {}", ctx, err))
    } else {
        Err(anyhow!("{}: ESP error code {}", ctx, code))
    }
}

pub fn wifi_init_once() -> Result<()> {
    static mut INIT: bool = false;
    unsafe {
        if INIT {
            return Ok(());
        }
    }

    esp_ok_ctx_allow(
        unsafe { sys::esp_netif_init() },
        "esp_netif_init",
        &[sys::ESP_ERR_INVALID_STATE],
    )?;
    esp_ok_ctx_allow(
        unsafe { sys::esp_event_loop_create_default() },
        "esp_event_loop_create_default",
        &[sys::ESP_ERR_INVALID_STATE],
    )?;
    let sta = unsafe { sys::esp_netif_create_default_wifi_sta() };
    if sta.is_null() {
        return Err(anyhow!("esp_netif_create_default_wifi_sta failed"));
    }

    let mut cfg = wifi_init_cfg_default();
    esp_ok_ctx_allow(
        unsafe { sys::esp_wifi_init(&mut cfg) },
        "esp_wifi_init",
        &[sys::ESP_ERR_INVALID_STATE],
    )?;
    esp_ok_ctx(
        unsafe { sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA) },
        "esp_wifi_set_mode",
    )?;
    esp_ok_ctx(unsafe { sys::esp_wifi_start() }, "esp_wifi_start")?;

    unsafe {
        INIT = true;
    }
    Ok(())
}

pub fn wifi_scan() -> Result<Vec<(String, i8)>> {
    let records = wifi_scan_records()?;
    Ok(records.into_iter().map(|r| (r.ssid, r.rssi)).collect())
}

pub struct WifiAp {
    pub ssid: String,
    pub rssi: i8,
    pub channel: u8,
}

pub fn wifi_scan_records() -> Result<Vec<WifiAp>> {
    wifi_init_once()?;

    let mut scan_cfg: sys::wifi_scan_config_t = unsafe { core::mem::zeroed() };
    scan_cfg.show_hidden = true;
    scan_cfg.scan_type = sys::wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE;
    // Explicit scan times to avoid ESP_ERR_INVALID_ARG on some IDF versions.
    scan_cfg.scan_time.active.min = 10;
    scan_cfg.scan_time.active.max = 30;
    scan_cfg.scan_time.passive = 360;

    let res = unsafe { sys::esp_wifi_scan_start(&mut scan_cfg, true) };
    if res == sys::ESP_ERR_INVALID_ARG {
        // Fallback to default config if IDF rejects the explicit struct.
        esp_ok_ctx(
            unsafe { sys::esp_wifi_scan_start(ptr::null_mut(), true) },
            "esp_wifi_scan_start(default)",
        )?;
    } else {
        esp_ok_ctx(res, "esp_wifi_scan_start")?;
    }

    let mut count: u16 = 0;
    esp_ok_ctx(
        unsafe { sys::esp_wifi_scan_get_ap_num(&mut count) },
        "esp_wifi_scan_get_ap_num",
    )?;
    if count == 0 {
        return Ok(Vec::new());
    }

    let mut records: Vec<sys::wifi_ap_record_t> = Vec::with_capacity(count as usize);
    unsafe {
        records.set_len(count as usize);
    }
    let mut num = count;
    esp_ok_ctx(
        unsafe { sys::esp_wifi_scan_get_ap_records(&mut num, records.as_mut_ptr()) },
        "esp_wifi_scan_get_ap_records",
    )?;

    let mut out = Vec::with_capacity(num as usize);
    for rec in records.into_iter().take(num as usize) {
        let ssid_bytes = &rec.ssid;
        let len = ssid_bytes
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(ssid_bytes.len());
        let ssid = String::from_utf8_lossy(&ssid_bytes[..len]).to_string();
        out.push(WifiAp {
            ssid,
            rssi: rec.rssi,
            channel: rec.primary,
        });
    }
    Ok(out)
}

static PACKET_MGMT: AtomicU32 = AtomicU32::new(0);
static PACKET_DATA: AtomicU32 = AtomicU32::new(0);
static PACKET_CTRL: AtomicU32 = AtomicU32::new(0);
static mut PROMISC_ACTIVE: bool = false;

unsafe extern "C" fn promisc_rx_cb(
    _buf: *mut core::ffi::c_void,
    _type: sys::wifi_promiscuous_pkt_type_t,
) {
    match _type {
        sys::wifi_promiscuous_pkt_type_t_WIFI_PKT_MGMT => {
            PACKET_MGMT.fetch_add(1, Ordering::Relaxed);
        }
        sys::wifi_promiscuous_pkt_type_t_WIFI_PKT_DATA => {
            PACKET_DATA.fetch_add(1, Ordering::Relaxed);
        }
        sys::wifi_promiscuous_pkt_type_t_WIFI_PKT_CTRL => {
            PACKET_CTRL.fetch_add(1, Ordering::Relaxed);
        }
        _ => {}
    }
}

pub fn wifi_set_channel(channel: u8) -> Result<()> {
    esp_ok_ctx(
        unsafe {
            sys::esp_wifi_set_channel(channel, sys::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE)
        },
        "esp_wifi_set_channel",
    )
}

pub fn wifi_monitor_start(channel: u8) -> Result<()> {
    wifi_init_once()?;
    wifi_set_channel(channel)?;
    unsafe {
        sys::esp_wifi_set_promiscuous_rx_cb(Some(promisc_rx_cb));
        esp_ok_ctx(
            unsafe { sys::esp_wifi_set_promiscuous(true) },
            "esp_wifi_set_promiscuous",
        )?;
        PROMISC_ACTIVE = true;
    }
    Ok(())
}

pub fn wifi_monitor_stop() -> Result<()> {
    unsafe {
        if PROMISC_ACTIVE {
            esp_ok_ctx(
                sys::esp_wifi_set_promiscuous(false),
                "esp_wifi_set_promiscuous(false)",
            )?;
            PROMISC_ACTIVE = false;
        }
    }
    Ok(())
}

pub struct PacketCounts {
    pub mgmt: u32,
    pub data: u32,
    pub ctrl: u32,
}

pub fn wifi_monitor_take_counts() -> PacketCounts {
    PacketCounts {
        mgmt: PACKET_MGMT.swap(0, Ordering::Relaxed),
        data: PACKET_DATA.swap(0, Ordering::Relaxed),
        ctrl: PACKET_CTRL.swap(0, Ordering::Relaxed),
    }
}
