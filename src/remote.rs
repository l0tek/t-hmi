use anyhow::{anyhow, Result};
use std::collections::VecDeque;
use std::sync::{Mutex, OnceLock};

#[derive(Clone, Debug)]
pub enum RemoteCommand {
    GotoMenu,
    GotoWifiMenu,
    GotoDeviceMenu,
    GotoGps,
    GotoBattery,
    GotoLoopback,
    GotoHttpMenu,
    GotoWifiLogin,
    RunSdFormat,
    ToggleHttp,
    WifiScan,
    WifiMonitor,
    WifiChannelMonitor,
    WifiMonitorStop,
    WifiConnectDefault,
    WifiConnect { ssid: String, password: String },
}

fn queue() -> &'static Mutex<VecDeque<RemoteCommand>> {
    static Q: OnceLock<Mutex<VecDeque<RemoteCommand>>> = OnceLock::new();
    Q.get_or_init(|| Mutex::new(VecDeque::new()))
}

fn status() -> &'static Mutex<String> {
    static S: OnceLock<Mutex<String>> = OnceLock::new();
    S.get_or_init(|| Mutex::new(String::from("idle")))
}

pub fn enqueue(cmd: RemoteCommand) -> Result<()> {
    let mut q = queue()
        .lock()
        .map_err(|_| anyhow!("remote queue lock poisoned"))?;
    q.push_back(cmd);
    Ok(())
}

pub fn drain() -> Vec<RemoteCommand> {
    let Ok(mut q) = queue().lock() else {
        return Vec::new();
    };
    let mut out = Vec::with_capacity(q.len());
    while let Some(cmd) = q.pop_front() {
        out.push(cmd);
    }
    out
}

pub fn set_status(msg: impl Into<String>) {
    if let Ok(mut s) = status().lock() {
        *s = msg.into();
    }
}

pub fn get_status() -> String {
    status()
        .lock()
        .map(|s| s.clone())
        .unwrap_or_else(|_| "status unavailable".to_string())
}
