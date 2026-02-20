use anyhow::{anyhow, Result};
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};
use esp_idf_sys as sys;
use std::ffi::{CStr, CString};
use std::time::{Duration, Instant};

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

fn wifi_force_sta_ready() -> Result<()> {
    // Bring driver into a clean STA state before scan/connect.
    let _ = unsafe { sys::esp_wifi_scan_stop() };
    let _ = unsafe { sys::esp_wifi_disconnect() };
    let _ = esp_ok_ctx_allow(
        unsafe { sys::esp_wifi_stop() },
        "esp_wifi_stop",
        &[sys::ESP_ERR_WIFI_NOT_STARTED],
    );
    esp_ok_ctx(
        unsafe { sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA) },
        "esp_wifi_set_mode(STA)",
    )?;
    esp_ok_ctx_allow(
        unsafe { sys::esp_wifi_start() },
        "esp_wifi_start",
        &[sys::ESP_ERR_WIFI_CONN],
    )?;
    let _ = unsafe { sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE) };
    Ok(())
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
    // Reset persisted WiFi config that may have been left by previous firmware.
    let _ = unsafe { sys::esp_wifi_restore() };
    // Force common 2.4GHz legacy protocol set (ESP32-S3 compatible).
    let _ = unsafe {
        sys::esp_wifi_set_protocol(
            sys::wifi_interface_t_WIFI_IF_STA,
            (sys::WIFI_PROTOCOL_11B | sys::WIFI_PROTOCOL_11G | sys::WIFI_PROTOCOL_11N) as u8,
        )
    };
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
    pub authmode: sys::wifi_auth_mode_t,
    pub bssid: [u8; 6],
}

fn wifi_auth_name(auth: sys::wifi_auth_mode_t) -> &'static str {
    match auth {
        sys::wifi_auth_mode_t_WIFI_AUTH_OPEN => "OPEN",
        sys::wifi_auth_mode_t_WIFI_AUTH_WEP => "WEP",
        sys::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => "WPA_PSK",
        sys::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => "WPA2_PSK",
        sys::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => "WPA_WPA2_PSK",
        sys::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => "WPA2_ENTERPRISE",
        sys::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => "WPA3_PSK",
        sys::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => "WPA2_WPA3_PSK",
        sys::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => "WAPI_PSK",
        _ => "UNKNOWN",
    }
}

pub fn wifi_scan_records() -> Result<Vec<WifiAp>> {
    wifi_init_once()?;
    let _ = wifi_monitor_stop();
    wifi_force_sta_ready()?;

    let mut count: u16 = 0;
    for attempt in 0..3 {
        let mut scan_cfg: sys::wifi_scan_config_t = unsafe { core::mem::zeroed() };
        scan_cfg.show_hidden = true;
        scan_cfg.scan_type = sys::wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE;
        // Slightly larger windows for better reliability.
        scan_cfg.scan_time.active.min = 20;
        scan_cfg.scan_time.active.max = 80;
        scan_cfg.scan_time.passive = 400;

        let res = unsafe { sys::esp_wifi_scan_start(&mut scan_cfg, true) };
        if res == sys::ESP_ERR_INVALID_ARG {
            esp_ok_ctx(
                unsafe { sys::esp_wifi_scan_start(ptr::null_mut(), true) },
                "esp_wifi_scan_start(default)",
            )?;
        } else {
            esp_ok_ctx(res, "esp_wifi_scan_start")?;
        }

        count = 0;
        esp_ok_ctx(
            unsafe { sys::esp_wifi_scan_get_ap_num(&mut count) },
            "esp_wifi_scan_get_ap_num",
        )?;
        if count > 0 || attempt == 2 {
            break;
        }
    }
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
            authmode: rec.authmode,
            bssid: rec.bssid,
        });
    }
    Ok(out)
}

pub struct WifiConnectionInfo {
    pub ssid: String,
    pub rssi: i8,
    pub channel: u8,
}

fn copy_cstr_bytes(dst: &mut [u8], src: &[u8], field: &str) -> Result<()> {
    if src.len() >= dst.len() {
        return Err(anyhow!("{} zu lang (max {} Bytes)", field, dst.len() - 1));
    }
    dst.fill(0);
    dst[..src.len()].copy_from_slice(src);
    Ok(())
}

pub fn wifi_connect(ssid: &str, password: &str) -> Result<()> {
    wifi_init_once()?;
    let _ = wifi_monitor_stop();
    wifi_force_sta_ready()?;
    println!(
        "[WIFI_CALL] wifi_connect ssid='{}' pass_len={}",
        ssid,
        password.len()
    );

    let mut cfg = sys::wifi_config_t::default();
    unsafe {
        let sta = &mut cfg.sta;
        copy_cstr_bytes(&mut sta.ssid, ssid.as_bytes(), "SSID")?;
        copy_cstr_bytes(&mut sta.password, password.as_bytes(), "Passwort")?;
        sta.scan_method = sys::wifi_scan_method_t_WIFI_ALL_CHANNEL_SCAN;
        sta.sort_method = sys::wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL;
        sta.threshold.rssi = -127;
        // Use broad auth threshold for protected networks.
        sta.threshold.authmode = if password.is_empty() {
            sys::wifi_auth_mode_t_WIFI_AUTH_OPEN
        } else {
            sys::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK
        };
        sta.channel = 0;
        sta.bssid_set = false;
        sta.pmf_cfg.capable = false;
        sta.pmf_cfg.required = false;
    }

    esp_ok_ctx(
        unsafe { sys::esp_wifi_set_config(sys::wifi_interface_t_WIFI_IF_STA, &mut cfg) },
        "esp_wifi_set_config",
    )?;
    let _ = unsafe { sys::esp_wifi_disconnect() };
    esp_ok_ctx(unsafe { sys::esp_wifi_connect() }, "esp_wifi_connect")
}

pub fn wifi_connect_ap(ap: &WifiAp, password: &str) -> Result<()> {
    wifi_init_once()?;
    let _ = wifi_monitor_stop();
    wifi_force_sta_ready()?;

    let mut cfg = sys::wifi_config_t::default();
    unsafe {
        let sta = &mut cfg.sta;
        copy_cstr_bytes(&mut sta.ssid, ap.ssid.as_bytes(), "SSID")?;
        copy_cstr_bytes(&mut sta.password, password.as_bytes(), "Passwort")?;
        sta.scan_method = sys::wifi_scan_method_t_WIFI_ALL_CHANNEL_SCAN;
        sta.sort_method = sys::wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL;
        sta.threshold.rssi = -127;
        sta.threshold.authmode = if password.is_empty() {
            sys::wifi_auth_mode_t_WIFI_AUTH_OPEN
        } else {
            sys::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK
        };
        sta.channel = ap.channel;
        sta.bssid = ap.bssid;
        sta.bssid_set = true;
        sta.pmf_cfg.capable = false;
        sta.pmf_cfg.required = false;
    }

    println!(
        "[WIFI_CALL] wifi_connect_ap ssid='{}' ch={} auth={} bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} pass_len={}",
        ap.ssid,
        ap.channel,
        wifi_auth_name(ap.authmode),
        ap.bssid[0],
        ap.bssid[1],
        ap.bssid[2],
        ap.bssid[3],
        ap.bssid[4],
        ap.bssid[5],
        password.len()
    );

    esp_ok_ctx(
        unsafe { sys::esp_wifi_set_config(sys::wifi_interface_t_WIFI_IF_STA, &mut cfg) },
        "esp_wifi_set_config",
    )?;
    let _ = unsafe { sys::esp_wifi_disconnect() };
    esp_ok_ctx(unsafe { sys::esp_wifi_connect() }, "esp_wifi_connect")
}

pub fn wifi_connected_ap() -> Result<WifiConnectionInfo> {
    wifi_init_once()?;
    let mut rec: sys::wifi_ap_record_t = unsafe { core::mem::zeroed() };
    esp_ok_ctx(
        unsafe { sys::esp_wifi_sta_get_ap_info(&mut rec) },
        "esp_wifi_sta_get_ap_info",
    )?;

    let len = rec
        .ssid
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(rec.ssid.len());
    let ssid = String::from_utf8_lossy(&rec.ssid[..len]).to_string();

    Ok(WifiConnectionInfo {
        ssid,
        rssi: rec.rssi,
        channel: rec.primary,
    })
}

pub fn wifi_wait_connected(timeout_ms: u32) -> Result<WifiConnectionInfo> {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
    let mut last_err: Option<anyhow::Error> = None;

    while Instant::now() < deadline {
        match wifi_connected_ap() {
            Ok(info) if !info.ssid.is_empty() => return Ok(info),
            Ok(_) => last_err = Some(anyhow!("AP info leer")),
            Err(err) => last_err = Some(err),
        }
        std::thread::sleep(Duration::from_millis(250));
    }

    if let Some(err) = last_err {
        Err(anyhow!("Connect timeout: {}", err))
    } else {
        Err(anyhow!("Connect timeout"))
    }
}

pub fn wifi_wait_ipv4(timeout_ms: u32) -> Result<String> {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
    let mut last_err: Option<anyhow::Error> = None;

    while Instant::now() < deadline {
        match wifi_sta_ipv4() {
            Ok(ip) if ip != "0.0.0.0" => return Ok(ip),
            Ok(_) => last_err = Some(anyhow!("IP noch nicht vergeben")),
            Err(err) => last_err = Some(err),
        }
        std::thread::sleep(Duration::from_millis(250));
    }

    if let Some(err) = last_err {
        Err(anyhow!("IP timeout: {}", err))
    } else {
        Err(anyhow!("IP timeout"))
    }
}

pub fn wifi_sta_ipv4() -> Result<String> {
    wifi_init_once()?;

    let if_key = CString::new("WIFI_STA_DEF")?;
    let netif = unsafe { sys::esp_netif_get_handle_from_ifkey(if_key.as_ptr()) };
    if netif.is_null() {
        return Err(anyhow!("esp_netif_get_handle_from_ifkey returned null"));
    }

    let mut ip_info = sys::esp_netif_ip_info_t::default();
    esp_ok_ctx(
        unsafe { sys::esp_netif_get_ip_info(netif, &mut ip_info) },
        "esp_netif_get_ip_info",
    )?;

    let mut ip_buf = [0u8; 16];
    let ip_ptr =
        unsafe { sys::esp_ip4addr_ntoa(&ip_info.ip, ip_buf.as_mut_ptr(), ip_buf.len() as i32) };
    if ip_ptr.is_null() {
        return Err(anyhow!("esp_ip4addr_ntoa failed"));
    }

    let ip = unsafe { CStr::from_ptr(ip_buf.as_ptr()) }
        .to_string_lossy()
        .to_string();
    Ok(ip)
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
        // Force-disable promiscuous mode even if our local state flag drifted.
        let _ = sys::esp_wifi_set_promiscuous(false);
        sys::esp_wifi_set_promiscuous_rx_cb(None);
        PROMISC_ACTIVE = false;
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
