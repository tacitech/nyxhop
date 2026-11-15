//! The board's Ethernet port given to the camera (`--camport <interface>`, or NYX_CAMPORT; 14/11).
//!
//! On the NyxHop baseboard the SoM's own Ethernet jack is the camera's: plug a camera in and it is
//! found and set up with nothing typed. This module owns that port:
//!   * the port gets a subnet of its own, 172.31.77.1/24 (any other IPv4 address on it is removed:
//!     the SoM came with a static bench-LAN address there), and a small DHCP server hands each
//!     camera an address made from its MAC, so it is the same one after every restart;
//!   * a camera with a fixed address of its own (no DHCP, or its fallback before it asks) is heard
//!     in its ARP traffic; after STATIC_AFTER the board takes an address beside it on the port, so
//!     nothing on the camera has to change. Where a camera is is what it is heard at, not what DHCP
//!     acknowledged: an XM camera asks for a lease and goes on using its own address (14/11);
//!   * when the camera stream does not run (no URL, or no picture for STUCK), the camera on the
//!     port is set up over ONVIF (`nyx_common::onvif::auto_setup`: device service, logins, H.264
//!     for the link) and its RTSP address goes into the settings and is saved. The logins tried:
//!     the user's (`camlogin`, from the pilot app's camera page over the link), the one in the
//!     current address, the factory ones. A new `camlogin` sets the camera up at once;
//!   * what it is doing goes to the far app as `air_camport` (Shared::cam_port): none, setup,
//!     ready, login (the user's login is needed), fail.
//! A PC plugged into the port instead gets an address as well and reaches the board at 172.31.77.1.

use std::collections::HashMap;
use std::ffi::CString;
use std::net::{Ipv4Addr, UdpSocket};
use std::os::fd::AsRawFd;
use std::process::Command;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nyx_common::logging::log;
use nyx_common::onvif;
use nyx_common::source::SourceKind;
use nyx_tx::Shared;

/// The port's subnet; the board is .1, cameras get .100-.199.
const NET: [u8; 3] = [172, 31, 77];
const POOL_FIRST: u8 = 100;
const POOL_SIZE: u32 = 100;
const LEASE_S: u32 = 3600;
/// A host heard with an address of its own (not the port's) is met on its own subnet after this
/// long: a camera that speaks from an address uses it (one that takes a lease later moves over).
const STATIC_AFTER: Duration = Duration::from_secs(5);
/// A stream that gave no picture for this long has the camera on the port set up again.
const STUCK: Duration = Duration::from_secs(10);
/// One camera is not set up again sooner than this.
const RETRY: Duration = Duration::from_secs(30);
/// After a set-up, the stream gets this long to start before the camera is looked at again...
const QUIET: Duration = Duration::from_secs(20);
/// ...and this long when its encoder was changed (an XM camera restarts all of itself, ~40 s).
const QUIET_CHANGED: Duration = Duration::from_secs(75);
/// A camera first heard this recently is still starting: its ONVIF not answering is not a failure.
const STARTING: Duration = Duration::from_secs(150);

type Mac = [u8; 6];

fn me() -> Ipv4Addr {
    Ipv4Addr::new(NET[0], NET[1], NET[2], 1)
}

fn on_net(ip: Ipv4Addr) -> bool {
    ip.octets()[..3] == NET
}

fn mac_s(m: &Mac) -> String {
    m.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

/// When a host was first and last heard at one address.
#[derive(Clone, Copy)]
struct Heard {
    first: Instant,
    last: Instant,
}

#[derive(Default)]
struct State {
    /// every host heard on the port (ARP), at every address it used
    hosts: HashMap<Mac, HashMap<Ipv4Addr, Heard>>,
    /// addresses the DHCP server offered, by MAC
    leases: HashMap<Mac, Ipv4Addr>,
    /// hosts whose lease was acknowledged (for the log: a camera may still not use it)
    bound: HashMap<Mac, Ipv4Addr>,
    /// addresses a host said were in use (DHCPDECLINE)
    declined: Vec<Ipv4Addr>,
}

/// An on-port address heard this recently is where the host is, whatever else it used.
const ON_NET_FRESH: Duration = Duration::from_secs(90);
/// An address not heard for this long is forgotten (unless the camera stream uses it).
const FORGET: Duration = Duration::from_secs(600);

/// Where a host is: an address on the port's subnet heard lately, else the one heard last.
fn whereabouts(addrs: &HashMap<Ipv4Addr, Heard>) -> Option<(Ipv4Addr, Heard)> {
    let fresh_on = addrs.iter().filter(|(a, h)| on_net(**a) && h.last.elapsed() < ON_NET_FRESH).max_by_key(|(_, h)| h.last);
    fresh_on.or_else(|| addrs.iter().max_by_key(|(_, h)| h.last)).map(|(a, h)| (*a, *h))
}

/// Take the port and start its threads.
pub fn spawn(shared: Arc<Shared>, iface: String) {
    let st = Arc::new(Mutex::new(State::default()));
    let own = read_mac(&iface);
    log(&format!(
        "camera port: {iface} ({}) is the camera's: {}/24, DHCP for cameras, ONVIF set-up",
        own.map_or("no MAC".into(), |m| mac_s(&m)),
        me()
    ));
    release_from_dhcpcd(&iface);
    let _ = ip(&["link", "set", &iface, "up"]);
    let mut said = Vec::new();
    ensure_addresses(&iface, &[], &mut said);
    {
        let (st, i) = (st.clone(), iface.clone());
        std::thread::Builder::new().name("campt-dhcp".into()).spawn(move || dhcp(&i, &st)).expect("spawn campt-dhcp");
    }
    {
        let (st, i) = (st.clone(), iface.clone());
        std::thread::Builder::new()
            .name("campt-arp".into())
            .spawn(move || arp(&i, own.unwrap_or_default(), &st))
            .expect("spawn campt-arp");
    }
    std::thread::Builder::new().name("campt".into()).spawn(move || run(&shared, &iface, &st)).expect("spawn campt");
}

fn read_mac(iface: &str) -> Option<Mac> {
    let s = std::fs::read_to_string(format!("/sys/class/net/{iface}/address")).ok()?;
    let v: Vec<u8> = s.trim().split(':').filter_map(|h| u8::from_str_radix(h, 16).ok()).collect();
    v.try_into().ok()
}

/// `ip <args>`: its output, or what it said on failure.
fn ip(args: &[&str]) -> Result<String, String> {
    let o = Command::new("ip").args(args).output().map_err(|e| e.to_string())?;
    if o.status.success() {
        Ok(String::from_utf8_lossy(&o.stdout).into_owned())
    } else {
        Err(String::from_utf8_lossy(&o.stderr).trim().to_string())
    }
}

/// dhcpcd (Kuiper's network manager) leaves the port alone: `denyinterfaces` in its configuration
/// for the next boot, and told now. Its static address or link-local one would come back on the
/// port every few seconds otherwise (14/11: a static 192.168.0.22 kept reappearing).
fn release_from_dhcpcd(iface: &str) {
    const CONF: &str = "/etc/dhcpcd.conf";
    let Ok(conf) = std::fs::read_to_string(CONF) else { return };
    let line = format!("denyinterfaces {iface}");
    if !conf.lines().any(|l| l.trim() == line) {
        // in the global part, before the first `interface`/`profile` block: dhcpcd ignores it
        // inside one, and a board flashed with a fixed address ends with `interface eth0`
        let add = format!("# {iface} is the camera port (nyx-ipcam): its own subnet and DHCP\n{line}\n\n");
        let mut out = String::with_capacity(conf.len() + add.len());
        let mut done = false;
        for l in conf.split_inclusive('\n') {
            let t = l.trim_start();
            if !done && (t.starts_with("interface ") || t.starts_with("profile ")) {
                out.push_str(&add);
                done = true;
            }
            out.push_str(l);
        }
        if !done {
            if !out.ends_with('\n') {
                out.push('\n');
            }
            out.push('\n');
            out.push_str(&add);
        }
        let tmp = format!("{CONF}.camport");
        // (written beside it and moved over, with its mode and owner)
        let put = || -> std::io::Result<()> {
            use std::os::unix::fs::MetadataExt;
            let meta = std::fs::metadata(CONF)?;
            std::fs::write(&tmp, &out)?;
            std::fs::set_permissions(&tmp, meta.permissions())?;
            let c = CString::new(tmp.as_str()).map_err(std::io::Error::other)?;
            // SAFETY: chown on a path string that lives through the call
            unsafe { libc::chown(c.as_ptr(), meta.uid(), meta.gid()) };
            std::fs::rename(&tmp, CONF)
        };
        match put() {
            Ok(()) => log(&format!("camera port: {CONF}: dhcpcd leaves {iface} alone from now on")),
            Err(e) => log(&format!("camera port: cannot write {CONF}: {e}")),
        }
    }
    // (for the dhcpcd running now; it keeps managing the other interfaces)
    let _ = Command::new("dhcpcd").args(["-x", iface]).output();
}

/// The port carries the board's address on its subnet and no other IPv4 address but `keep` (the
/// addresses beside fixed-address cameras). `said`: removals already logged.
fn ensure_addresses(iface: &str, keep: &[Ipv4Addr], said: &mut Vec<String>) {
    let out = ip(&["-4", "-o", "addr", "show", "dev", iface]).unwrap_or_default();
    let mut have_me = false;
    for l in out.lines() {
        let Some(cidr) = l.split_whitespace().skip_while(|w| *w != "inet").nth(1) else { continue };
        let Some(a) = cidr.split('/').next().and_then(|s| s.parse::<Ipv4Addr>().ok()) else { continue };
        if a == me() && cidr.ends_with("/24") {
            have_me = true;
            continue;
        }
        if keep.contains(&a) {
            continue;
        }
        let r = ip(&["addr", "del", cidr, "dev", iface]);
        if !said.iter().any(|c| c == cidr) {
            said.push(cidr.to_string());
            match r {
                Ok(_) => log(&format!("camera port: removed {cidr} from {iface} (the port is the camera's)")),
                Err(e) => log(&format!("camera port: cannot remove {cidr} from {iface}: {e}")),
            }
        }
    }
    if !have_me {
        let cidr = format!("{}/24", me());
        match ip(&["addr", "add", &cidr, "brd", "+", "dev", iface]) {
            Ok(_) => log(&format!("camera port: {iface} is {cidr}")),
            Err(e) => log(&format!("camera port: cannot give {iface} {cidr}: {e}")),
        }
    }
}

// --------------------------------------------------------------------- DHCP --

fn bind_to_device(fd: i32, iface: &str) -> Result<(), String> {
    let name = CString::new(iface).map_err(|e| e.to_string())?;
    let b = name.as_bytes_with_nul();
    // SAFETY: setsockopt reads `b.len()` bytes from a live buffer
    let r = unsafe {
        libc::setsockopt(fd, libc::SOL_SOCKET, libc::SO_BINDTODEVICE, b.as_ptr().cast(), b.len() as libc::socklen_t)
    };
    if r == 0 { Ok(()) } else { Err(std::io::Error::last_os_error().to_string()) }
}

fn bind_dhcp(iface: &str) -> Result<UdpSocket, String> {
    let s = UdpSocket::bind("0.0.0.0:67").map_err(|e| format!("UDP 67: {e}"))?;
    s.set_broadcast(true).map_err(|e| e.to_string())?;
    bind_to_device(s.as_raw_fd(), iface)?;
    Ok(s)
}

/// Option `code` of a DHCP message (the bytes after the magic cookie).
fn opt(opts: &[u8], code: u8) -> Option<&[u8]> {
    let mut i = 0;
    while i < opts.len() {
        match opts[i] {
            0 => i += 1,
            255 => break,
            c => {
                let n = *opts.get(i + 1)? as usize;
                let v = opts.get(i + 2..i + 2 + n)?;
                if c == code {
                    return Some(v);
                }
                i += 2 + n;
            }
        }
    }
    None
}

fn ip4(v: &[u8]) -> Option<Ipv4Addr> {
    <[u8; 4]>::try_from(v).ok().map(Ipv4Addr::from)
}

/// The address a MAC gets: the same every time (a hash of it in the pool), the next free one if
/// another host holds it.
fn assign(st: &mut State, mac: Mac) -> Ipv4Addr {
    if let Some(&a) = st.leases.get(&mac) {
        return a;
    }
    let h = mac.iter().fold(0x811c_9dc5u32, |h, &b| (h ^ u32::from(b)).wrapping_mul(0x0100_0193));
    let a = (0..POOL_SIZE)
        .map(|k| Ipv4Addr::new(NET[0], NET[1], NET[2], POOL_FIRST + ((h.wrapping_add(k)) % POOL_SIZE) as u8))
        .find(|a| !st.leases.values().any(|x| x == a) && !st.declined.contains(a))
        .unwrap_or(Ipv4Addr::new(NET[0], NET[1], NET[2], POOL_FIRST));
    st.leases.insert(mac, a);
    a
}

/// A BOOTP reply to `req` (2 offer, 5 ack, 6 nak), broadcast on the port.
fn reply(sock: &UdpSocket, req: &[u8], kind: u8, yi: Ipv4Addr) {
    let mut r = vec![0u8; 240];
    r[0] = 2;
    r[1] = 1;
    r[2] = 6;
    r[4..8].copy_from_slice(&req[4..8]);
    r[10..12].copy_from_slice(&req[10..12]);
    if kind != 6 {
        r[16..20].copy_from_slice(&yi.octets());
        r[20..24].copy_from_slice(&me().octets());
    }
    r[28..44].copy_from_slice(&req[28..44]);
    r[236..240].copy_from_slice(&[99, 130, 83, 99]);
    r.extend([53, 1, kind, 54, 4]);
    r.extend(me().octets());
    if kind != 6 {
        r.extend([51, 4]);
        r.extend(LEASE_S.to_be_bytes());
        r.extend([58, 4]);
        r.extend((LEASE_S / 2).to_be_bytes());
        r.extend([59, 4]);
        r.extend((LEASE_S / 8 * 7).to_be_bytes());
        r.extend([1, 4, 255, 255, 255, 0, 3, 4]);
        r.extend(me().octets());
        r.extend([28, 4, NET[0], NET[1], NET[2], 255]);
    }
    r.push(255);
    r.resize(r.len().max(300), 0);
    let _ = sock.send_to(&r, (Ipv4Addr::BROADCAST, 68));
}

fn dhcp(iface: &str, st: &Mutex<State>) {
    let mut said = String::new();
    let sock = loop {
        match bind_dhcp(iface) {
            Ok(s) => break s,
            Err(e) => {
                if e != said {
                    log(&format!("camera port: no DHCP server on {iface}: {e} (trying again)"));
                    said = e;
                }
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    };
    let mut buf = [0u8; 1500];
    loop {
        let Ok((n, _)) = sock.recv_from(&mut buf) else { continue };
        let p = &buf[..n];
        if n < 240 || p[0] != 1 || p[1] != 1 || p[2] != 6 || p[236..240] != [99, 130, 83, 99] {
            continue;
        }
        let mac: Mac = p[28..34].try_into().unwrap_or_default();
        let opts = &p[240..];
        let Some(&[kind]) = opt(opts, 53) else { continue };
        let mut s = st.lock().unwrap();
        match kind {
            // DISCOVER
            1 => {
                let a = assign(&mut s, mac);
                drop(s);
                reply(&sock, p, 2, a);
            }
            // REQUEST
            3 => {
                if let Some(sid) = opt(opts, 54).and_then(ip4)
                    && sid != me()
                {
                    continue; // it took another server's offer
                }
                let ciaddr = Ipv4Addr::new(p[12], p[13], p[14], p[15]);
                let want = opt(opts, 50).and_then(ip4).unwrap_or(ciaddr);
                let a = assign(&mut s, mac);
                if want == a {
                    let new = s.bound.insert(mac, a) != Some(a);
                    drop(s);
                    reply(&sock, p, 5, a);
                    if new {
                        log(&format!("camera port: {} took {a} (DHCP)", mac_s(&mac)));
                    }
                } else {
                    // an address from another network (the camera's last lease, or its own): no,
                    // and it asks again from the start
                    drop(s);
                    reply(&sock, p, 6, Ipv4Addr::UNSPECIFIED);
                }
            }
            // DECLINE: the address is in use on the port
            4 => {
                if let Some(a) = s.leases.remove(&mac) {
                    s.declined.push(a);
                    s.bound.remove(&mac);
                    log(&format!("camera port: {} says {a} is taken; it gets another", mac_s(&mac)));
                }
            }
            // RELEASE
            7 => {
                s.bound.remove(&mac);
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------- ARP --

/// Every ARP frame on the port: who is there, at which address (a camera with an address of its
/// own says it there, asking for its gateway or announcing itself).
fn arp(iface: &str, own: Mac, st: &Mutex<State>) {
    const ETH_P_ARP: u16 = 0x0806;
    // SAFETY: plain socket calls on a descriptor this function owns; the buffers outlive them
    let fd = unsafe { libc::socket(libc::AF_PACKET, libc::SOCK_RAW, i32::from(ETH_P_ARP.to_be())) };
    if fd < 0 {
        log(&format!("camera port: no ARP listener: {}", std::io::Error::last_os_error()));
        return;
    }
    let Ok(name) = CString::new(iface) else { return };
    let idx = unsafe { libc::if_nametoindex(name.as_ptr()) };
    let mut sll: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
    sll.sll_family = libc::AF_PACKET as u16;
    sll.sll_protocol = ETH_P_ARP.to_be();
    sll.sll_ifindex = idx as i32;
    let r = unsafe {
        libc::bind(fd, (&raw const sll).cast(), std::mem::size_of::<libc::sockaddr_ll>() as libc::socklen_t)
    };
    if r != 0 {
        log(&format!("camera port: no ARP listener on {iface}: {}", std::io::Error::last_os_error()));
        unsafe { libc::close(fd) };
        return;
    }
    let mut buf = [0u8; 256];
    loop {
        let n = unsafe { libc::recv(fd, buf.as_mut_ptr().cast(), buf.len(), 0) };
        if n < 42 {
            if n < 0 {
                std::thread::sleep(Duration::from_millis(500));
            }
            continue;
        }
        let f = &buf[..n as usize];
        if f[12..14] != [0x08, 0x06] {
            continue;
        }
        let sha: Mac = f[22..28].try_into().unwrap_or_default();
        let spa = Ipv4Addr::new(f[28], f[29], f[30], f[31]);
        if sha == own || spa.is_unspecified() || spa == me() {
            continue;
        }
        let mut s = st.lock().unwrap();
        let now = Instant::now();
        let addrs = s.hosts.entry(sha).or_default();
        let new = !addrs.contains_key(&spa);
        addrs.entry(spa).or_insert(Heard { first: now, last: now }).last = now;
        drop(s);
        if new {
            log(&format!("camera port: heard {} at {spa}", mac_s(&sha)));
        }
    }
}

// --------------------------------------------------------------------- main --

/// The IPv4 host of an rtsp:// address.
fn host_of(url: &str) -> Option<Ipv4Addr> {
    let after = url.trim().strip_prefix("rtsp://")?;
    let auth = after.split('/').next()?;
    let hostport = auth.rsplit_once('@').map_or(auth, |(_, h)| h);
    hostport.split(':').next()?.parse().ok()
}

/// An address without its password, for the log.
fn redact(url: &str) -> String {
    match url.split_once("://") {
        Some((s, rest)) => {
            let (auth, path) = rest.find('/').map_or((rest, ""), |p| rest.split_at(p));
            match auth.rsplit_once('@') {
                Some((cred, host)) => {
                    let user = cred.split(':').next().unwrap_or("");
                    format!("{s}://{user}{}@{host}{path}", if cred.contains(':') { ":***" } else { "" })
                }
                None => url.to_string(),
            }
        }
        None => url.to_string(),
    }
}

/// The same camera stream (host and path; the login may differ).
fn same_stream(a: &str, b: &str) -> bool {
    let strip = |u: &str| {
        let rest = u.split_once("://").map_or(u, |(_, r)| r);
        let (auth, path) = rest.find('/').map_or((rest, ""), |p| rest.split_at(p));
        format!("{}{path}", auth.rsplit_once('@').map_or(auth, |(_, h)| h))
    };
    strip(a) == strip(b)
}

/// `camlogin` ("user:password", percent-encoded) as a login.
fn user_login(v: &str) -> Option<(String, String)> {
    let (u, p) = v.split_once(':').unwrap_or((v, ""));
    (!u.is_empty()).then(|| (onvif::pct_decode(u), onvif::pct_decode(p)))
}

fn set_state(shared: &Shared, s: &str) {
    let mut c = shared.cam_port.lock().unwrap();
    if *c != s {
        *c = s.to_string();
    }
}

/// The board's address beside a camera that keeps an address of its own (its /24's .254, or .253
/// for a camera at .254).
fn beside(a: Ipv4Addr) -> Ipv4Addr {
    let o = a.octets();
    Ipv4Addr::new(o[0], o[1], o[2], if o[3] == 254 { 253 } else { 254 })
}

fn run(shared: &Shared, iface: &str, st: &Mutex<State>) {
    let mut said = Vec::new();
    let mut last_login = shared.config.lock().unwrap().cam_login.clone();
    set_state(shared, "none");
    // camera address -> the board's address beside it
    let mut aliases: HashMap<Ipv4Addr, Ipv4Addr> = HashMap::new();
    // the saved camera has an address of its own: met there at once, not after STATIC_AFTER
    // (after a restart the stream came 6 s later than it could)
    let saved = host_of(&shared.config.lock().unwrap().rtsp_url);
    if let Some(cam) = saved
        && !on_net(cam)
    {
        let mine = beside(cam);
        let _ = ip(&["addr", "add", &format!("{mine}/32"), "dev", iface]);
        if ip(&["route", "replace", &format!("{cam}/32"), "dev", iface, "src", &mine.to_string()]).is_ok() {
            aliases.insert(cam, mine);
        }
    }
    let mut last_frames = shared.rtsp.frames_total.load(Ordering::Relaxed);
    let mut last_pic = Instant::now();
    // no setting up again before this: the camera restarts its stream, or all of itself, after a set-up
    let mut quiet_until = Instant::now();
    let mut last_url = String::new();
    let mut tried: HashMap<Ipv4Addr, Instant> = HashMap::new();
    let mut last_err: HashMap<Ipv4Addr, String> = HashMap::new();
    while !shared.stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_secs(2));
        let keep: Vec<Ipv4Addr> = aliases.values().copied().collect();
        ensure_addresses(iface, &keep, &mut said);

        let (url, login_cfg) = {
            let c = shared.config.lock().unwrap();
            (c.rtsp_url.clone(), c.cam_login.clone())
        };
        let named = host_of(&url);
        // the user gave the camera's login: tried at once, on every camera of the port
        let new_login = login_cfg != last_login;
        if new_login {
            last_login = login_cfg.clone();
            tried.clear();
            log(if login_cfg.is_empty() {
                "camera port: the camera login was cleared; setting the camera up again"
            } else {
                "camera port: a camera login was given; setting the camera up with it"
            });
        }
        // where each host on the port is
        let hosts: Vec<(Mac, Ipv4Addr, Heard)> = {
            let mut s = st.lock().unwrap();
            for addrs in s.hosts.values_mut() {
                addrs.retain(|a, h| h.last.elapsed() < FORGET || Some(*a) == named);
            }
            s.hosts.retain(|_, addrs| !addrs.is_empty());
            s.hosts.iter().filter_map(|(m, addrs)| whereabouts(addrs).map(|(a, h)| (*m, a, h))).collect()
        };
        // hosts with fixed addresses off the port's subnet: the board takes one beside each (and
        // keeps it while the camera stream uses it)
        let mut want: HashMap<Ipv4Addr, Ipv4Addr> = HashMap::new();
        for (_, a, h) in &hosts {
            if !on_net(*a) && h.first.elapsed() >= STATIC_AFTER {
                want.insert(*a, beside(*a));
            }
        }
        if let Some(n) = named
            && aliases.contains_key(&n)
        {
            want.insert(n, beside(n));
        }
        for (cam, mine) in aliases.clone() {
            if want.get(&cam) != Some(&mine) {
                let _ = ip(&["route", "del", &format!("{cam}/32"), "dev", iface]);
                let _ = ip(&["addr", "del", &format!("{mine}/32"), "dev", iface]);
                aliases.remove(&cam);
                log(&format!("camera port: {cam} gone or on DHCP now; {mine} dropped"));
            }
        }
        // the routes again: the kernel drops them when the port's link goes down (a camera
        // unplugged and plugged back, 14/11), the addresses stay
        for (cam, mine) in &aliases {
            let _ = ip(&["route", "replace", &format!("{cam}/32"), "dev", iface, "src", &mine.to_string()]);
        }
        for (cam, mine) in want {
            if aliases.contains_key(&cam) {
                continue;
            }
            let _ = ip(&["addr", "add", &format!("{mine}/32"), "dev", iface]);
            match ip(&["route", "replace", &format!("{cam}/32"), "dev", iface, "src", &mine.to_string()]) {
                Ok(_) => {
                    aliases.insert(cam, mine);
                    log(&format!("camera port: {cam} keeps an address of its own; the board meets it from {mine}"));
                }
                Err(e) => log(&format!("camera port: no route to {cam}: {e}")),
            }
        }

        // the stream: set the camera up when there is none, or it gives no picture
        let n = shared.rtsp.frames_total.load(Ordering::Relaxed);
        let mut streaming = false;
        if n != last_frames {
            last_frames = n;
            last_pic = Instant::now();
            streaming = true;
        }
        if url != last_url {
            last_url = url.clone();
            last_pic = Instant::now();
        }
        let need = new_login
            || url.trim().is_empty()
            || (!shared.standby.load(Ordering::Relaxed) && last_pic.elapsed() >= STUCK && Instant::now() >= quiet_until);
        if !need {
            if streaming {
                set_state(shared, "ready");
            }
            continue;
        }
        let mut cands: Vec<(Mac, Ipv4Addr, Heard)> =
            hosts.iter().filter(|(_, a, _)| on_net(*a) || aliases.contains_key(a)).copied().collect();
        // the camera the address names first (it may only need H.264 back)
        cands.sort_by_key(|(_, a, _)| Some(*a) != named);
        if cands.is_empty() {
            set_state(shared, "none");
        }
        for (mac, cam, heard) in cands {
            if tried.get(&cam).is_some_and(|t| t.elapsed() < RETRY) {
                continue;
            }
            tried.insert(cam, Instant::now());
            // the user's login first, else the one in the current address (factory ones after)
            let login = user_login(&login_cfg).or_else(|| onvif::device_for(&url, "").map(|(_, u, p)| (u, p)));
            set_state(shared, "setup");
            match onvif::auto_setup(cam, login.as_ref().map(|(u, p)| (u.as_str(), p.as_str()))) {
                Ok(a) => {
                    last_err.remove(&cam);
                    let new_cam = !same_stream(&url, &a.rtsp);
                    {
                        let mut c = shared.config.lock().unwrap();
                        c.source = SourceKind::Rtsp;
                        c.rtsp_pass = true;
                        c.rtsp_url = a.rtsp.clone();
                        c.onvif_url = a.device.clone();
                        // another camera: its own bitrate when first reached is its ceiling
                        if new_cam {
                            c.cam_max_kbps = 0;
                        }
                    }
                    if new_cam {
                        shared.rtsp.onvif_base_kbps.store(0, Ordering::Relaxed);
                    }
                    log(&format!(
                        "camera port: {} {} at {cam} ready{}: {}",
                        if a.model.is_empty() { "camera" } else { a.model.as_str() },
                        mac_s(&mac),
                        if a.changed.is_empty() { String::new() } else { format!(" ({})", a.changed) },
                        redact(&a.rtsp)
                    ));
                    let saved = nyx_tx::save_config(shared);
                    log(&format!("camera port: {saved}"));
                    set_state(shared, "ready");
                    last_url = a.rtsp;
                    last_pic = Instant::now();
                    // a camera whose encoder was changed may restart all of itself (an XM: ~40 s, its
                    // ONVIF silent meanwhile); setting it up again in the middle looped (14/11)
                    quiet_until = Instant::now() + if a.changed.is_empty() { QUIET } else { QUIET_CHANGED };
                    break;
                }
                Err(e) => {
                    // a camera just powered up talks on the network a minute before its ONVIF does (an XM:
                    // ~90 s): not a failure yet
                    let state = if e.starts_with(onvif::LOGIN_REFUSED) {
                        "login"
                    } else if heard.first.elapsed() < STARTING {
                        "setup"
                    } else {
                        "fail"
                    };
                    set_state(shared, state);
                    if last_err.get(&cam) != Some(&e) {
                        log(&format!("camera port: {} at {cam}: {e}", mac_s(&mac)));
                        last_err.insert(cam, e);
                    }
                }
            }
        }
    }
}
