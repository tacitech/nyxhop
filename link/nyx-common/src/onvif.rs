//! ONVIF, the part an IP camera needs to follow the radio link (camera pass-through, 17/9).
//!
//! The camera's H.264 goes on air as it is (pass-through), so the only way to make it fit a link
//! that changes is to change the camera: its encoder bitrate limit, through the ONVIF Media
//! service most IP cameras have. This module is the client (find the media profile behind the RTSP
//! URL in use, read its encoder configuration, set a new bitrate limit), the thread that applies
//! the bitrate the transmit worker asks for (`RtspShared::want_kbps`), and the small pieces the
//! fake camera uses to answer the same calls.
//!
//! SOAP 1.2 over plain HTTP. Authentication: WS-Security UsernameToken with a password digest,
//! timestamped with the camera's own clock (cameras refuse a skewed one), and HTTP Digest when the
//! camera answers 401 instead. No dependencies (SHA-1, MD5, base64 and the XML scanning are here),
//! so the board build stays pure Rust.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::logging::log;
use crate::source::RtspShared;

// ------------------------------------------------------------------ hashes --

pub fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [0x6745_2301, 0xEFCD_AB89, 0x98BA_DCFE, 0x1032_5476, 0xC3D2_E1F0];
    let mut msg = data.to_vec();
    let bits = (data.len() as u64).wrapping_mul(8);
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bits.to_be_bytes());
    for chunk in msg.chunks(64) {
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([chunk[4 * i], chunk[4 * i + 1], chunk[4 * i + 2], chunk[4 * i + 3]]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let (mut a, mut b, mut c, mut d, mut e) = (h[0], h[1], h[2], h[3], h[4]);
        for (i, &wi) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5A82_7999),
                20..=39 => (b ^ c ^ d, 0x6ED9_EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC),
                _ => (b ^ c ^ d, 0xCA62_C1D6),
            };
            let t = a.rotate_left(5).wrapping_add(f).wrapping_add(e).wrapping_add(k).wrapping_add(wi);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = t;
        }
        for (x, v) in h.iter_mut().zip([a, b, c, d, e]) {
            *x = x.wrapping_add(v);
        }
    }
    let mut out = [0u8; 20];
    for (i, v) in h.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&v.to_be_bytes());
    }
    out
}

pub fn md5(data: &[u8]) -> [u8; 16] {
    const S: [u32; 64] = [
        7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9,
        14, 20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15,
        21, 6, 10, 15, 21,
    ];
    let k: Vec<u32> = (0..64).map(|i| ((i as f64 + 1.0).sin().abs() * 4_294_967_296.0) as u32).collect();
    let (mut a0, mut b0, mut c0, mut d0) = (0x6745_2301u32, 0xEFCD_AB89u32, 0x98BA_DCFEu32, 0x1032_5476u32);
    let mut msg = data.to_vec();
    let bits = (data.len() as u64).wrapping_mul(8);
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bits.to_le_bytes());
    for chunk in msg.chunks(64) {
        let m: Vec<u32> = chunk.chunks(4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
        let (mut a, mut b, mut c, mut d) = (a0, b0, c0, d0);
        for i in 0..64 {
            let (f, g) = match i {
                0..=15 => ((b & c) | (!b & d), i),
                16..=31 => ((d & b) | (!d & c), (5 * i + 1) % 16),
                32..=47 => (b ^ c ^ d, (3 * i + 5) % 16),
                _ => (c ^ (b | !d), (7 * i) % 16),
            };
            let f = f.wrapping_add(a).wrapping_add(k[i]).wrapping_add(m[g]);
            a = d;
            d = c;
            c = b;
            b = b.wrapping_add(f.rotate_left(S[i]));
        }
        a0 = a0.wrapping_add(a);
        b0 = b0.wrapping_add(b);
        c0 = c0.wrapping_add(c);
        d0 = d0.wrapping_add(d);
    }
    let mut out = [0u8; 16];
    for (i, v) in [a0, b0, c0, d0].iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
    }
    out
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

pub fn b64_encode(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let v = (u32::from(c[0]) << 16) | (u32::from(*c.get(1).unwrap_or(&0)) << 8) | u32::from(*c.get(2).unwrap_or(&0));
        for k in 0..4 {
            out.push(if k <= c.len() { B64[((v >> (18 - 6 * k)) & 63) as usize] as char } else { '=' });
        }
    }
    out
}

pub fn b64_decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let (mut acc, mut n) = (0u32, 0u32);
    for ch in s.bytes().filter(|b| !b.is_ascii_whitespace()) {
        if ch == b'=' {
            break;
        }
        let v = B64.iter().position(|&x| x == ch)? as u32;
        acc = (acc << 6) | v;
        n += 6;
        if n >= 8 {
            n -= 8;
            out.push((acc >> n) as u8);
        }
    }
    Some(out)
}

/// The WS-Security password digest: base64(SHA-1(nonce + created + password)).
pub fn password_digest(nonce: &[u8], created: &str, password: &str) -> String {
    let mut buf = nonce.to_vec();
    buf.extend_from_slice(created.as_bytes());
    buf.extend_from_slice(password.as_bytes());
    b64_encode(&sha1(&buf))
}

// -------------------------------------------------------------------- time --

fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// `YYYY-MM-DDTHH:MM:SSZ` of a Unix time.
pub fn iso_utc(unix: i64) -> String {
    let (days, secs) = (unix.div_euclid(86_400), unix.rem_euclid(86_400));
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z", secs / 3600, secs / 60 % 60, secs % 60)
}

pub fn unix_now() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs() as i64)
}

// --------------------------------------------------------------------- xml --

/// Every element named `local` (whatever its prefix): (attributes text, inner text), outermost
/// first. Enough for SOAP answers; not a general XML parser.
pub fn elements<'a>(xml: &'a str, local: &str) -> Vec<(&'a str, &'a str)> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while let Some(off) = xml[i..].find('<') {
        let start = i + off;
        let rest = &xml[start + 1..];
        if rest.starts_with(['/', '?', '!']) {
            i = start + 1;
            continue;
        }
        let name_end = rest.find(|c: char| c.is_whitespace() || c == '>' || c == '/').unwrap_or(rest.len());
        let qname = &rest[..name_end];
        let Some(gt) = rest.find('>') else { break };
        if qname.rsplit(':').next() != Some(local) {
            i = start + 1;
            continue;
        }
        let tag = &rest[..gt];
        let content = start + 1 + gt + 1;
        if tag.ends_with('/') {
            out.push((tag[name_end..].trim_end_matches('/'), ""));
            i = content;
            continue;
        }
        let (open, close) = (format!("<{qname}"), format!("</{qname}"));
        let mut depth = 1;
        let mut j = content;
        let mut end = None;
        while depth > 0 {
            let c = xml[j..].find(&close).map(|p| j + p);
            let o = xml[j..].find(&open).map(|p| j + p).filter(|&p| {
                xml[p + open.len()..].starts_with(|ch: char| ch == '>' || ch.is_whitespace())
            });
            match (o, c) {
                (Some(o), Some(c)) if o < c => {
                    depth += 1;
                    j = o + open.len();
                }
                (_, Some(c)) => {
                    depth -= 1;
                    j = c + close.len();
                    if depth == 0 {
                        end = Some(c);
                    }
                }
                _ => break,
            }
        }
        let Some(end) = end else { break };
        out.push((&tag[name_end..], &xml[content..end]));
        i = j;
    }
    out
}

/// The inner text of the first element named `local`.
pub fn find<'a>(xml: &'a str, local: &str) -> Option<&'a str> {
    elements(xml, local).into_iter().next().map(|(_, inner)| inner)
}

/// `name="value"` in an element's attribute text.
pub fn attr<'a>(attrs: &'a str, name: &str) -> Option<&'a str> {
    let mut rest = attrs;
    while let Some(p) = rest.find(name) {
        let after = &rest[p + name.len()..];
        let before_ok = p == 0 || rest[..p].ends_with(|c: char| c.is_whitespace() || c == ':');
        if before_ok && let Some(after) = after.trim_start().strip_prefix('=') {
            let after = after.trim_start();
            let q = after.chars().next()?;
            if q == '"' || q == '\'' {
                let body = &after[1..];
                return body.find(q).map(|e| &body[..e]);
            }
        }
        rest = &rest[p + name.len()..];
    }
    None
}

pub fn unescape(s: &str) -> String {
    s.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
}

pub fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

fn text(xml: &str, local: &str) -> Option<String> {
    find(xml, local).map(|t| unescape(t.trim()))
}

// -------------------------------------------------------------------- http --

struct HttpUrl {
    host: String,
    port: u16,
    path: String,
}

fn parse_http(url: &str) -> Result<HttpUrl, String> {
    let rest = url.trim().strip_prefix("http://").ok_or_else(|| format!("not an http:// URL: {url}"))?;
    let (auth, path) = rest.split_once('/').map_or((rest, "/".to_string()), |(a, p)| (a, format!("/{p}")));
    let auth = auth.rsplit('@').next().unwrap_or(auth);
    let (host, port) = match auth.rsplit_once(':') {
        Some((h, p)) => (h.to_string(), p.parse().map_err(|_| format!("bad port in {url}"))?),
        None => (auth.to_string(), 80),
    };
    Ok(HttpUrl { host, port, path })
}

struct Reply {
    status: u16,
    headers: String,
    body: String,
}

fn http_post(url: &HttpUrl, action: &str, body: &str, authorization: Option<&str>) -> Result<Reply, String> {
    let addr = std::net::ToSocketAddrs::to_socket_addrs(&(url.host.as_str(), url.port))
        .map_err(|e| e.to_string())?
        .next()
        .ok_or("no address")?;
    let mut s = TcpStream::connect_timeout(&addr, Duration::from_secs(3)).map_err(|e| format!("{}:{}: {e}", url.host, url.port))?;
    let _ = s.set_read_timeout(Some(Duration::from_secs(6)));
    let _ = s.set_write_timeout(Some(Duration::from_secs(3)));
    let mut req = format!(
        "POST {} HTTP/1.1\r\nHost: {}:{}\r\nContent-Type: application/soap+xml; charset=utf-8; action=\"{action}\"\r\n\
         Content-Length: {}\r\nConnection: close\r\nUser-Agent: nyxhop\r\n",
        url.path, url.host, url.port, body.len()
    );
    if let Some(a) = authorization {
        req.push_str(&format!("Authorization: {a}\r\n"));
    }
    req.push_str("\r\n");
    req.push_str(body);
    s.write_all(req.as_bytes()).map_err(|e| e.to_string())?;
    let mut raw = Vec::new();
    if let Err(e) = s.read_to_end(&mut raw)
        && raw.is_empty()
    {
        return Err(e.to_string());
    }
    let text = String::from_utf8_lossy(&raw).into_owned();
    let (head, body) = text.split_once("\r\n\r\n").ok_or("no HTTP reply")?;
    let status = head.split_whitespace().nth(1).and_then(|c| c.parse().ok()).ok_or("bad HTTP status")?;
    let body = if head.to_ascii_lowercase().contains("transfer-encoding: chunked") {
        dechunk(body)
    } else {
        body.to_string()
    };
    Ok(Reply { status, headers: head.to_string(), body })
}

fn dechunk(mut s: &str) -> String {
    let mut out = String::new();
    while let Some((line, rest)) = s.split_once("\r\n") {
        let Ok(n) = usize::from_str_radix(line.trim().split(';').next().unwrap_or(""), 16) else { break };
        if n == 0 || rest.len() < n {
            break;
        }
        out.push_str(&rest[..n]);
        s = rest[n..].trim_start_matches("\r\n");
    }
    out
}

/// An HTTP Digest challenge (RFC 2617) remembered from a 401.
struct Digest {
    realm: String,
    nonce: String,
    opaque: Option<String>,
    qop: bool,
    nc: u32,
}

impl Digest {
    fn from_headers(headers: &str) -> Option<Digest> {
        let line = headers.lines().find(|l| l.to_ascii_lowercase().starts_with("www-authenticate:") && l.contains("Digest"))?;
        let params = &line[line.find("Digest")? + 6..];
        Some(Digest {
            realm: attr(params, "realm")?.to_string(),
            nonce: attr(params, "nonce")?.to_string(),
            opaque: attr(params, "opaque").map(str::to_string),
            qop: attr(params, "qop").is_some_and(|q| q.split(',').any(|v| v.trim() == "auth")),
            nc: 0,
        })
    }

    fn header(&mut self, user: &str, pass: &str, uri: &str) -> String {
        self.nc += 1;
        let ha1 = hex(&md5(format!("{user}:{}:{pass}", self.realm).as_bytes()));
        let ha2 = hex(&md5(format!("POST:{uri}").as_bytes()));
        let cnonce = hex(&random_bytes(8));
        let nc = format!("{:08x}", self.nc);
        let resp = if self.qop {
            hex(&md5(format!("{ha1}:{}:{nc}:{cnonce}:auth:{ha2}", self.nonce).as_bytes()))
        } else {
            hex(&md5(format!("{ha1}:{}:{ha2}", self.nonce).as_bytes()))
        };
        let mut h = format!(
            "Digest username=\"{user}\", realm=\"{}\", nonce=\"{}\", uri=\"{uri}\", response=\"{resp}\", algorithm=MD5",
            self.realm, self.nonce
        );
        if self.qop {
            h.push_str(&format!(", qop=auth, nc={nc}, cnonce=\"{cnonce}\""));
        }
        if let Some(o) = &self.opaque {
            h.push_str(&format!(", opaque=\"{o}\""));
        }
        h
    }
}

fn random_bytes(n: usize) -> Vec<u8> {
    let seed = SystemTime::now().duration_since(UNIX_EPOCH).map_or(1, |d| d.as_nanos() as u64) ^ (std::process::id() as u64) << 32;
    let mut r = crate::rng::Rng64::new(seed);
    (0..n).map(|_| r.next_u64() as u8).collect()
}

// ------------------------------------------------------------------ client --

const NS: &str = "xmlns:s=\"http://www.w3.org/2003/05/soap-envelope\" xmlns:tds=\"http://www.onvif.org/ver10/device/wsdl\" \
xmlns:trt=\"http://www.onvif.org/ver10/media/wsdl\" xmlns:tr2=\"http://www.onvif.org/ver20/media/wsdl\" \
xmlns:tt=\"http://www.onvif.org/ver10/schema\"";

/// The H.264 encoder configuration of one profile, kept as the camera wrote it so a change of the
/// bitrate sends everything else back unchanged.
#[derive(Clone, Debug, Default)]
pub struct EncoderConfig {
    pub token: String,
    pub name: String,
    pub use_count: String,
    pub encoding: String,
    pub width: u32,
    pub height: u32,
    pub quality: String,
    pub frame_rate_limit: String,
    pub encoding_interval: String,
    pub bitrate_kbps: u32,
    pub gov_length: Option<String>,
    pub h264_profile: Option<String>,
    pub multicast: Option<String>,
    pub session_timeout: String,
}

impl EncoderConfig {
    pub fn parse(token: &str, xml: &str) -> EncoderConfig {
        let rate = find(xml, "RateControl").unwrap_or("");
        let res = find(xml, "Resolution").unwrap_or("");
        let h264 = find(xml, "H264");
        EncoderConfig {
            token: token.to_string(),
            name: text(xml, "Name").unwrap_or_default(),
            use_count: text(xml, "UseCount").unwrap_or_else(|| "1".into()),
            encoding: text(xml, "Encoding").unwrap_or_default(),
            width: text(res, "Width").and_then(|v| v.parse().ok()).unwrap_or(0),
            height: text(res, "Height").and_then(|v| v.parse().ok()).unwrap_or(0),
            quality: text(xml, "Quality").unwrap_or_else(|| "5".into()),
            frame_rate_limit: text(rate, "FrameRateLimit").unwrap_or_else(|| "25".into()),
            encoding_interval: text(rate, "EncodingInterval").unwrap_or_else(|| "1".into()),
            bitrate_kbps: text(rate, "BitrateLimit").and_then(|v| v.parse().ok()).unwrap_or(0),
            gov_length: h264.and_then(|h| text(h, "GovLength")),
            h264_profile: h264.and_then(|h| text(h, "H264Profile")),
            multicast: find(xml, "Multicast").map(str::to_string),
            session_timeout: text(xml, "SessionTimeout").unwrap_or_else(|| "PT60S".into()),
        }
    }

    /// The `<tt:...>` children of a VideoEncoderConfiguration element.
    pub fn to_xml(&self) -> String {
        let mut x = format!(
            "<tt:Name>{}</tt:Name><tt:UseCount>{}</tt:UseCount><tt:Encoding>{}</tt:Encoding>\
             <tt:Resolution><tt:Width>{}</tt:Width><tt:Height>{}</tt:Height></tt:Resolution><tt:Quality>{}</tt:Quality>\
             <tt:RateControl><tt:FrameRateLimit>{}</tt:FrameRateLimit><tt:EncodingInterval>{}</tt:EncodingInterval>\
             <tt:BitrateLimit>{}</tt:BitrateLimit></tt:RateControl>",
            escape(&self.name), self.use_count, self.encoding, self.width, self.height, self.quality,
            self.frame_rate_limit, self.encoding_interval, self.bitrate_kbps
        );
        if self.gov_length.is_some() || self.h264_profile.is_some() {
            x.push_str("<tt:H264>");
            if let Some(g) = &self.gov_length {
                x.push_str(&format!("<tt:GovLength>{g}</tt:GovLength>"));
            }
            if let Some(p) = &self.h264_profile {
                x.push_str(&format!("<tt:H264Profile>{p}</tt:H264Profile>"));
            }
            x.push_str("</tt:H264>");
        }
        // Multicast and SessionTimeout are required by the schema; rebuild the first with our own
        // prefix (the camera's may differ from the one declared in this request)
        let mc = self.multicast.as_deref().unwrap_or("");
        let ip = find(mc, "IPv4Address").map(|v| v.trim().to_string()).unwrap_or_else(|| "0.0.0.0".into());
        x.push_str(&format!(
            "<tt:Multicast><tt:Address><tt:Type>IPv4</tt:Type><tt:IPv4Address>{ip}</tt:IPv4Address></tt:Address>\
             <tt:Port>{}</tt:Port><tt:TTL>{}</tt:TTL><tt:AutoStart>{}</tt:AutoStart></tt:Multicast>\
             <tt:SessionTimeout>{}</tt:SessionTimeout>",
            text(mc, "Port").unwrap_or_else(|| "0".into()),
            text(mc, "TTL").unwrap_or_else(|| "0".into()),
            text(mc, "AutoStart").unwrap_or_else(|| "false".into()),
            self.session_timeout
        ));
        x
    }
}

/// One camera: where its services are, which profile feeds our RTSP URL, and how to sign calls.
pub struct Camera {
    media: String,
    user: String,
    pass: String,
    clock_offset: i64,
    digest: Option<Digest>,
    pub profile: String,
    pub config: EncoderConfig,
    /// the bitrate limits the camera accepts (kbit/s), when it says
    pub range: Option<(u32, u32)>,
    /// Media2 (ver20) when the camera has it: the bitrate is set there, with the configuration as
    /// Media2 reads it. An XM camera's Media (ver10) says Main where Media2 says Baseline, and a
    /// change written back through ver10 would carry that.
    media2: String,
    enc2: Option<Enc2>,
}

impl Camera {
    /// A camera to call, signed with its own clock: a WS-Security timestamp off by more than a few
    /// seconds is refused, and a camera without NTP keeps whatever time it booted with.
    fn session(device_url: &str, user: &str, pass: &str) -> Camera {
        let mut cam = Camera {
            media: device_url.to_string(),
            user: user.to_string(),
            pass: pass.to_string(),
            clock_offset: 0,
            digest: None,
            profile: String::new(),
            config: EncoderConfig::default(),
            range: None,
            media2: String::new(),
            enc2: None,
        };
        // (no authentication needed for the clock)
        if let Ok(r) = cam.call(device_url, "http://www.onvif.org/ver10/device/wsdl/GetSystemDateAndTime", "<tds:GetSystemDateAndTime/>", false)
            && let Some(utc) = find(&r, "UTCDateTime")
        {
            let n = |x: &str, k: &str| text(x, k).and_then(|v| v.parse::<i64>().ok());
            let (d, t) = (find(utc, "Date").unwrap_or(""), find(utc, "Time").unwrap_or(""));
            if let (Some(y), Some(mo), Some(da), Some(h), Some(mi), Some(se)) =
                (n(d, "Year"), n(d, "Month"), n(d, "Day"), n(t, "Hour"), n(t, "Minute"), n(t, "Second"))
            {
                cam.clock_offset = days_from_civil(y, mo, da) * 86_400 + h * 3600 + mi * 60 + se - unix_now();
            }
        }
        cam
    }

    /// Find the camera's media service and the profile whose stream is `rtsp_url` (the profile
    /// with the same path and query, else the one of the same picture size, else the first).
    pub fn connect(device_url: &str, user: &str, pass: &str, rtsp_url: &str, dims: (u32, u32)) -> Result<Camera, String> {
        let mut cam = Camera::session(device_url, user, pass);
        let caps = cam.call(
            device_url,
            "http://www.onvif.org/ver10/device/wsdl/GetCapabilities",
            "<tds:GetCapabilities><tds:Category>Media</tds:Category></tds:GetCapabilities>",
            true,
        )?;
        if let Some(x) = find(&caps, "Media").and_then(|m| text(m, "XAddr")) {
            cam.media = same_host(device_url, &x);
        }
        if let Ok(r) = cam.call(
            device_url,
            "http://www.onvif.org/ver10/device/wsdl/GetServices",
            "<tds:GetServices><tds:IncludeCapability>false</tds:IncludeCapability></tds:GetServices>",
            true,
        ) && let Some((_, s)) = elements(&r, "Service").into_iter().find(|(_, s)| text(s, "Namespace").as_deref() == Some(MEDIA2))
            && let Some(x) = text(s, "XAddr")
        {
            cam.media2 = same_host(device_url, &x);
        }
        let media = cam.media.clone();
        let profiles = cam.call(&media, "http://www.onvif.org/ver10/media/wsdl/GetProfiles", "<trt:GetProfiles/>", true)?;
        let want = path_and_query(rtsp_url);
        let mut best: Option<(i32, String, EncoderConfig)> = None;
        for (attrs, inner) in elements(&profiles, "Profiles") {
            let Some(token) = attr(attrs, "token") else { continue };
            let Some((eattrs, enc)) = elements(inner, "VideoEncoderConfiguration").into_iter().next() else { continue };
            let cfg = EncoderConfig::parse(attr(eattrs, "token").unwrap_or(""), enc);
            if !cfg.encoding.eq_ignore_ascii_case("H264") {
                continue;
            }
            let mut score = 0;
            let body = format!(
                "<trt:GetStreamUri><trt:StreamSetup><tt:Stream>RTP-Unicast</tt:Stream><tt:Transport><tt:Protocol>RTSP</tt:Protocol>\
                 </tt:Transport></trt:StreamSetup><trt:ProfileToken>{}</trt:ProfileToken></trt:GetStreamUri>",
                escape(token)
            );
            if let Ok(r) = cam.call(&media, "http://www.onvif.org/ver10/media/wsdl/GetStreamUri", &body, true)
                && let Some(uri) = text(&r, "Uri")
            {
                let (p, q) = path_and_query(&uri);
                if p == want.0 {
                    score += 100;
                    // query parameters of our URL the profile's URI also has (channel=1, subtype=1)
                    score += want.1.iter().filter(|kv| q.contains(kv)).count() as i32 * 10;
                    score -= q.len() as i32; // ...and the fewest extras
                }
            }
            if dims != (0, 0) && (cfg.width, cfg.height) == dims {
                score += 5;
            }
            if best.as_ref().is_none_or(|b| score > b.0) {
                best = Some((score, token.to_string(), cfg));
            }
        }
        let (_, profile, cfg) = best.ok_or("no H.264 media profile")?;
        cam.profile = profile;
        cam.config = cfg;
        let body = format!(
            "<trt:GetVideoEncoderConfigurationOptions><trt:ConfigurationToken>{}</trt:ConfigurationToken>\
             <trt:ProfileToken>{}</trt:ProfileToken></trt:GetVideoEncoderConfigurationOptions>",
            escape(&cam.config.token),
            escape(&cam.profile)
        );
        if let Ok(r) = cam.call(&media, "http://www.onvif.org/ver10/media/wsdl/GetVideoEncoderConfigurationOptions", &body, true)
            && let Some(br) = find(&r, "BitrateRange")
            && let (Some(lo), Some(hi)) = (text(br, "Min").and_then(|v| v.parse().ok()), text(br, "Max").and_then(|v| v.parse().ok()))
        {
            cam.range = Some((lo, hi));
        }
        cam.refresh()?;
        Ok(cam)
    }

    /// Read the encoder configuration again (another client may have changed it).
    pub fn refresh(&mut self) -> Result<(), String> {
        if !self.media2.is_empty() {
            let m2 = self.media2.clone();
            match self.call(&m2, &format!("{MEDIA2}/GetVideoEncoderConfigurations"), "<tr2:GetVideoEncoderConfigurations/>", true) {
                Ok(r) => {
                    if let Some(e) = Enc2::all(&r).into_iter().find(|e| e.token == self.config.token) {
                        self.config.encoding = e.encoding.clone();
                        (self.config.width, self.config.height) = (e.w, e.h);
                        self.config.bitrate_kbps = e.bitrate.parse::<f32>().map_or(self.config.bitrate_kbps, |v| v as u32);
                        self.enc2 = Some(e);
                        return Ok(());
                    }
                    // (not the same tokens in both services: ver10 it is)
                    self.media2.clear();
                    self.enc2 = None;
                }
                Err(e) => return Err(e),
            }
        }
        let body = format!(
            "<trt:GetVideoEncoderConfiguration><trt:ConfigurationToken>{}</trt:ConfigurationToken></trt:GetVideoEncoderConfiguration>",
            escape(&self.config.token)
        );
        let media = self.media.clone();
        let r = self.call(&media, "http://www.onvif.org/ver10/media/wsdl/GetVideoEncoderConfiguration", &body, true)?;
        let (attrs, inner) = elements(&r, "Configuration").into_iter().next().ok_or("no encoder configuration in the answer")?;
        self.config = EncoderConfig::parse(attr(attrs, "token").unwrap_or(&self.config.token), inner);
        Ok(())
    }

    /// Set the bitrate limit (kbit/s, clamped to what the camera accepts); returns what was set.
    pub fn set_bitrate(&mut self, kbps: u32) -> Result<u32, String> {
        let kbps = match self.range {
            Some((lo, hi)) => kbps.clamp(lo, hi.max(lo)),
            None => kbps,
        };
        if let Some(mut e) = self.enc2.clone() {
            e.bitrate = kbps.to_string();
            let m2 = self.media2.clone();
            self.call(
                &m2,
                &format!("{MEDIA2}/SetVideoEncoderConfiguration"),
                &format!("<tr2:SetVideoEncoderConfiguration>{}</tr2:SetVideoEncoderConfiguration>", e.to_xml()),
                true,
            )?;
            self.enc2 = Some(e);
            self.config.bitrate_kbps = kbps;
            return Ok(kbps);
        }
        let mut cfg = self.config.clone();
        cfg.bitrate_kbps = kbps;
        let body = format!(
            "<trt:SetVideoEncoderConfiguration><trt:Configuration token=\"{}\">{}</trt:Configuration>\
             <trt:ForcePersistence>false</trt:ForcePersistence></trt:SetVideoEncoderConfiguration>",
            escape(&cfg.token),
            cfg.to_xml()
        );
        let media = self.media.clone();
        self.call(&media, "http://www.onvif.org/ver10/media/wsdl/SetVideoEncoderConfiguration", &body, true)?;
        self.config = cfg;
        Ok(kbps)
    }

    fn call(&mut self, url: &str, action: &str, body: &str, auth: bool) -> Result<String, String> {
        let u = parse_http(url)?;
        let mut tries = 0;
        loop {
            tries += 1;
            let header = if auth && !self.user.is_empty() { self.ws_security() } else { String::new() };
            let env = format!(
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?><s:Envelope {NS}><s:Header>{header}</s:Header><s:Body>{body}</s:Body></s:Envelope>"
            );
            let authz = match (&mut self.digest, auth) {
                (Some(d), true) => Some(d.header(&self.user, &self.pass, &u.path)),
                _ => None,
            };
            let r = http_post(&u, action, &env, authz.as_deref())?;
            if r.status == 401 && tries == 1 && auth {
                // the camera wants HTTP Digest (as well); take its challenge and try once more
                if let Some(d) = Digest::from_headers(&r.headers) {
                    self.digest = Some(d);
                    continue;
                }
            }
            if let Some(fault) = find(&r.body, "Fault") {
                let why = text(fault, "Text").or_else(|| text(fault, "Value")).unwrap_or_else(|| "fault".into());
                return Err(format!("{}: {why}", action.rsplit('/').next().unwrap_or(action)));
            }
            if r.status >= 300 {
                return Err(format!("{}: HTTP {}", action.rsplit('/').next().unwrap_or(action), r.status));
            }
            return Ok(r.body);
        }
    }

    fn ws_security(&self) -> String {
        let nonce = random_bytes(16);
        let created = iso_utc(unix_now() + self.clock_offset);
        format!(
            "<wsse:Security s:mustUnderstand=\"1\" xmlns:wsse=\"http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd\" \
             xmlns:wsu=\"http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd\"><wsse:UsernameToken>\
             <wsse:Username>{}</wsse:Username>\
             <wsse:Password Type=\"http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest\">{}</wsse:Password>\
             <wsse:Nonce EncodingType=\"http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary\">{}</wsse:Nonce>\
             <wsu:Created>{created}</wsu:Created></wsse:UsernameToken></wsse:Security>",
            escape(&self.user),
            password_digest(&nonce, &created, &self.pass),
            b64_encode(&nonce)
        )
    }
}

/// A service address the camera reported, on the host we reach it at (cameras behind a router
/// or with a second interface report an address we cannot use); its own port and path are kept.
fn same_host(device_url: &str, service: &str) -> String {
    match (parse_http(device_url), parse_http(service)) {
        (Ok(d), Ok(s)) => format!("http://{}:{}{}", d.host, s.port, s.path),
        _ => service.to_string(),
    }
}

/// Path and `key=value` query parts of a URL (the credentials and host do not matter).
fn path_and_query(url: &str) -> (String, Vec<String>) {
    let after = url.split_once("://").map_or(url, |(_, r)| r);
    let path = after.find('/').map_or("/", |p| &after[p..]);
    let (p, q) = path.split_once('?').unwrap_or((path, ""));
    (p.to_string(), q.split('&').filter(|s| !s.is_empty()).map(|s| s.to_ascii_lowercase()).collect())
}

/// The device service and credentials for a camera: the explicit ONVIF URL when there is one,
/// else port 80 of the RTSP host; the user and password of the RTSP URL either way.
pub fn device_for(rtsp_url: &str, explicit: &str) -> Option<(String, String, String)> {
    let after = rtsp_url.trim().strip_prefix("rtsp://")?;
    let auth = after.split('/').next()?;
    let (cred, hostport) = auth.rsplit_once('@').unwrap_or(("", auth));
    let (user, pass) = cred.split_once(':').unwrap_or((cred, ""));
    let host = hostport.rsplit_once(':').map_or(hostport, |(h, _)| h);
    let device = if explicit.trim().is_empty() {
        format!("http://{host}/onvif/device_service")
    } else {
        explicit.trim().to_string()
    };
    Some((device, pct_decode(user), pct_decode(pass)))
}

/// `%XX` back to the byte (a user name or password taken from a URL).
pub fn pct_decode(s: &str) -> String {
    let b = s.as_bytes();
    let hexd = |c: u8| (c as char).to_digit(16).map(|d| d as u8);
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%'
            && i + 2 < b.len()
            && let (Some(h), Some(l)) = (hexd(b[i + 1]), hexd(b[i + 2]))
        {
            out.push(h << 4 | l);
            i += 3;
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// A user name or password made safe for the user-info part of a URL.
pub fn pct_encode(s: &str) -> String {
    let mut o = String::new();
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || b"-._~!$&'()*+,;=".contains(&b) {
            o.push(b as char);
        } else {
            o.push_str(&format!("%{b:02X}"));
        }
    }
    o
}

// -------------------------------------------------------------- auto setup --
//
// The board's camera port (nyx-ipcam, 14/11): a camera plugged into it is found and made ready for
// the link without anyone typing a URL. Its device service comes from a WS-Discovery probe (or the
// usual ports), the factory logins are tried, the main stream is set to what the link carries best
// (H.264 Baseline: no B-frames, so no reordering delay; at most 1920x1080; a key frame every
// second, which is what a lost frame costs in pass-through) and its RTSP address is read back.
// Cheap cameras stream H.265 out of the box while their Media (ver10) service still says H.264 (an
// XM GK7205 on 14/11); Media2 (ver20) tells the truth, so it is used when the camera has it.

/// The logins cameras leave the factory with, tried after the one the user gave. Kept short: some
/// cameras lock the account after a few refusals.
pub const FACTORY_LOGINS: &[(&str, &str)] = &[("admin", ""), ("admin", "admin"), ("admin", "12345"), ("admin", "123456"), ("root", "")];

/// How an `auto_setup` error begins when the camera refused every login tried (the user's is
/// needed then).
pub const LOGIN_REFUSED: &str = "login refused";

/// The largest picture the link is given (every frame is decoded on the ground; more only costs bits).
const AUTO_MAX: (u32, u32) = (1920, 1080);

const MEDIA1: &str = "http://www.onvif.org/ver10/media/wsdl";
const MEDIA2: &str = "http://www.onvif.org/ver20/media/wsdl";

/// A camera made ready by `auto_setup`.
#[derive(Clone, Debug, Default)]
pub struct AutoCam {
    pub device: String,
    pub user: String,
    pub pass: String,
    /// the main stream's RTSP address, the login in it
    pub rtsp: String,
    pub model: String,
    /// what was changed on the camera ("" = it was ready)
    pub changed: String,
}

fn uuid() -> String {
    let h = hex(&random_bytes(16));
    format!("{}-{}-{}-{}-{}", &h[..8], &h[8..12], &h[12..16], &h[16..20], &h[20..])
}

/// A WS-Discovery probe sent straight to one address (UDP 3702): the service addresses it gives.
pub fn probe(ip: std::net::Ipv4Addr, wait: Duration) -> Vec<String> {
    let Ok(s) = std::net::UdpSocket::bind("0.0.0.0:0") else { return Vec::new() };
    let _ = s.set_read_timeout(Some(Duration::from_millis(250)));
    let msg = format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?><e:Envelope xmlns:e=\"http://www.w3.org/2003/05/soap-envelope\" \
         xmlns:w=\"http://schemas.xmlsoap.org/ws/2004/08/addressing\" xmlns:d=\"http://schemas.xmlsoap.org/ws/2005/04/discovery\" \
         xmlns:dn=\"http://www.onvif.org/ver10/network/wsdl\"><e:Header><w:MessageID>uuid:{}</w:MessageID>\
         <w:To e:mustUnderstand=\"true\">urn:schemas-xmlsoap-org:ws:2005:04:discovery</w:To>\
         <w:Action e:mustUnderstand=\"true\">http://schemas.xmlsoap.org/ws/2005/04/discovery/Probe</w:Action>\
         </e:Header><e:Body><d:Probe><d:Types>dn:NetworkVideoTransmitter</d:Types></d:Probe></e:Body></e:Envelope>",
        uuid()
    );
    let mut out = Vec::new();
    let mut buf = vec![0u8; 16384];
    let t0 = Instant::now();
    let mut sent = 0u32;
    while t0.elapsed() < wait && out.is_empty() {
        // twice, half the wait apart (UDP)
        if sent < 2 && t0.elapsed() >= wait / 2 * sent {
            let _ = s.send_to(msg.as_bytes(), (ip, 3702));
            sent += 1;
        }
        if let Ok((n, from)) = s.recv_from(&mut buf)
            && from.ip() == std::net::IpAddr::V4(ip)
            && let Some(x) = text(&String::from_utf8_lossy(&buf[..n]), "XAddrs")
        {
            out.extend(x.split_whitespace().filter(|a| a.starts_with("http://")).map(str::to_string));
        }
    }
    out
}

/// The camera's device service: what WS-Discovery says, else the usual addresses; the first one
/// that answers GetSystemDateAndTime (no login needed for it).
fn find_device(ip: std::net::Ipv4Addr) -> Option<String> {
    let base = format!("http://{ip}/");
    let mut cands: Vec<String> = probe(ip, Duration::from_millis(1500)).iter().map(|x| same_host(&base, x)).collect();
    for p in [80u16, 8899, 8000, 8080, 2020] {
        let c = format!("http://{ip}:{p}/onvif/device_service");
        if !cands.contains(&c) {
            cands.push(c);
        }
    }
    let env = format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?><s:Envelope {NS}><s:Header/><s:Body><tds:GetSystemDateAndTime/></s:Body></s:Envelope>"
    );
    cands.into_iter().find(|c| {
        parse_http(c)
            .ok()
            .and_then(|u| http_post(&u, "http://www.onvif.org/ver10/device/wsdl/GetSystemDateAndTime", &env, None).ok())
            .is_some_and(|r| r.body.contains("SystemDateAndTime"))
    })
}

/// A refusal of the login (rather than a camera that cannot do something).
fn refused_login(e: &str) -> bool {
    let l = e.to_ascii_lowercase();
    l.contains("401") || l.contains("auth") || l.contains("password") || l.contains("not permitted")
}

/// `uri` on the address we reach the camera at (cameras with a second interface report another),
/// its port and path kept, any login in it dropped.
fn on_host(uri: &str, ip: std::net::Ipv4Addr) -> String {
    let Some((scheme, rest)) = uri.split_once("://") else { return uri.to_string() };
    let (auth, path) = rest.find('/').map_or((rest, ""), |p| rest.split_at(p));
    let hostport = auth.rsplit_once('@').map_or(auth, |(_, h)| h);
    match hostport.rsplit_once(':').map(|(_, p)| p).filter(|p| p.parse::<u16>().is_ok()) {
        Some(p) => format!("{scheme}://{ip}:{p}{path}"),
        None => format!("{scheme}://{ip}{path}"),
    }
}

/// An RTSP address with the login in it (no password: the user name alone).
pub fn with_login(uri: &str, user: &str, pass: &str) -> String {
    let Some((scheme, rest)) = uri.split_once("://") else { return uri.to_string() };
    let (auth, path) = rest.find('/').map_or((rest, ""), |p| rest.split_at(p));
    let host = auth.rsplit_once('@').map_or(auth, |(_, h)| h);
    match (user.is_empty(), pass.is_empty()) {
        (true, _) => format!("{scheme}://{host}{path}"),
        (false, true) => format!("{scheme}://{}@{host}{path}", pct_encode(user)),
        (false, false) => format!("{scheme}://{}:{}@{host}{path}", pct_encode(user), pct_encode(pass)),
    }
}

/// Find the camera at `ip`, log in (the user's login first, then the factory ones), set its main
/// stream for the link and read back its RTSP address.
pub fn auto_setup(ip: std::net::Ipv4Addr, login: Option<(&str, &str)>) -> Result<AutoCam, String> {
    let device =
        find_device(ip).ok_or_else(|| "no ONVIF service answers (WS-Discovery, ports 80, 8899, 8000, 8080, 2020)".to_string())?;
    let mut logins: Vec<(String, String)> = Vec::new();
    if let Some((u, p)) = login
        && !u.is_empty()
    {
        logins.push((u.to_string(), p.to_string()));
    }
    for (u, p) in FACTORY_LOGINS {
        if !logins.iter().any(|(a, b)| a == u && b == p) {
            logins.push((u.to_string(), p.to_string()));
        }
    }
    let mut refused = String::new();
    for (user, pass) in logins {
        let mut cam = Camera::session(&device, &user, &pass);
        match cam.make_ready(&device, ip) {
            Ok(mut a) => {
                // An XM camera keeps what ONVIF set in RAM until it restarts on its own terms: power it off
                // and it is back at its factory H.265 (14/11). A soft restart writes it down (proved over a
                // power cut), so after a change the camera is restarted once (~20 s); from then on it starts
                // the way the link wants it and needs nothing.
                if !a.changed.is_empty() {
                    let ok = cam.call(&device, "http://www.onvif.org/ver10/device/wsdl/SystemReboot", "<tds:SystemReboot/>", true).is_ok();
                    a.changed.push_str(if ok { "; camera restarted to keep it" } else { "; the camera refused a restart (it may lose this when powered off)" });
                }
                a.rtsp = with_login(&a.rtsp, &user, &pass);
                a.device = device;
                a.user = user;
                a.pass = pass;
                return Ok(a);
            }
            Err(e) if refused_login(&e) => refused = e,
            Err(e) => return Err(format!("{e} ({device})")),
        }
    }
    Err(format!("{LOGIN_REFUSED}: the camera takes none of the logins tried ({refused}); its user and password are needed"))
}

/// A Media2 (ver20) video encoder configuration, kept as the camera wrote it so a change sends
/// everything else back unchanged.
#[derive(Clone, Debug, Default)]
struct Enc2 {
    token: String,
    name: String,
    use_count: String,
    encoding: String,
    w: u32,
    h: u32,
    fps: String,
    bitrate: String,
    cbr: String,
    quality: String,
    gov: Option<u32>,
    profile: Option<String>,
    multicast: Option<String>,
}

impl Enc2 {
    fn parse(attrs: &str, xml: &str) -> Enc2 {
        let rate = elements(xml, "RateControl").into_iter().next();
        let res = find(xml, "Resolution").unwrap_or("");
        Enc2 {
            token: attr(attrs, "token").unwrap_or("").to_string(),
            name: text(xml, "Name").unwrap_or_default(),
            use_count: text(xml, "UseCount").unwrap_or_else(|| "1".into()),
            encoding: text(xml, "Encoding").unwrap_or_default(),
            w: text(res, "Width").and_then(|v| v.parse().ok()).unwrap_or(0),
            h: text(res, "Height").and_then(|v| v.parse().ok()).unwrap_or(0),
            fps: rate.and_then(|(_, i)| text(i, "FrameRateLimit")).unwrap_or_else(|| "25".into()),
            bitrate: rate.and_then(|(_, i)| text(i, "BitrateLimit")).unwrap_or_else(|| "2048".into()),
            cbr: rate.and_then(|(a, _)| attr(a, "ConstantBitRate")).unwrap_or("false").to_string(),
            quality: text(xml, "Quality").unwrap_or_else(|| "4".into()),
            gov: attr(attrs, "GovLength").and_then(|v| v.parse().ok()),
            profile: attr(attrs, "Profile").map(str::to_string),
            multicast: find(xml, "Multicast").map(str::to_string),
        }
    }

    /// Every configuration in a GetVideoEncoderConfigurations answer.
    fn all(xml: &str) -> Vec<Enc2> {
        elements(xml, "Configurations").into_iter().map(|(a, i)| Enc2::parse(a, i)).collect()
    }

    fn is(&self, enc: &str) -> bool {
        self.encoding.eq_ignore_ascii_case(enc)
    }

    /// This configuration made right for the link, within what the camera offers for H.264 (the
    /// attributes and content of its H.264 `Options`).
    fn for_link(&self, oattrs: &str, oxml: &str) -> Enc2 {
        let mut e = self.clone();
        e.encoding = "H264".into();
        let sizes: Vec<(u32, u32)> = elements(oxml, "ResolutionsAvailable")
            .iter()
            .filter_map(|(_, r)| Some((text(r, "Width")?.parse().ok()?, text(r, "Height")?.parse().ok()?)))
            .collect();
        let fits = |s: &(u32, u32)| s.0 <= AUTO_MAX.0 && s.1 <= AUTO_MAX.1;
        if !sizes.is_empty() && !(sizes.contains(&(e.w, e.h)) && fits(&(e.w, e.h))) {
            let pick = sizes.iter().filter(|s| fits(s)).max_by_key(|s| s.0 * s.1).or_else(|| sizes.iter().min_by_key(|s| s.0 * s.1));
            if let Some(&(w, h)) = pick {
                (e.w, e.h) = (w, h);
            }
        }
        let profiles: Vec<&str> = attr(oattrs, "ProfilesSupported").unwrap_or("").split_whitespace().collect();
        if profiles.contains(&"Baseline") {
            e.profile = Some("Baseline".into());
        } else if let Some(p) = profiles.first()
            && e.profile.as_deref().is_none_or(|cur| !profiles.contains(&cur))
        {
            e.profile = Some(p.to_string());
        }
        let fps = e.fps.parse::<f32>().unwrap_or(25.0).round().max(1.0) as u32;
        let (lo, hi) = attr(oattrs, "GovLengthRange")
            .and_then(|r| {
                let mut it = r.split_whitespace().filter_map(|v| v.parse::<u32>().ok());
                Some((it.next()?, it.next()?))
            })
            .unwrap_or((1, 300));
        e.gov = Some(fps.clamp(lo, hi.max(lo)));
        e
    }

    /// Worth setting the camera again: the codec, the size or the key frame spacing. Not the H.264
    /// profile alone - an XM camera takes Baseline, keeps Main, says Main, and restarts itself (~40 s)
    /// on every change, so asking again each time looped (14/11).
    fn differs(&self, o: &Enc2) -> bool {
        !self.encoding.eq_ignore_ascii_case(&o.encoding) || (self.w, self.h) != (o.w, o.h) || self.gov != o.gov
    }

    fn to_xml(&self) -> String {
        let mut a = format!("token=\"{}\"", escape(&self.token));
        if let Some(g) = self.gov {
            a.push_str(&format!(" GovLength=\"{g}\""));
        }
        if let Some(p) = &self.profile {
            a.push_str(&format!(" Profile=\"{}\"", escape(p)));
        }
        let mc = self.multicast.as_deref().unwrap_or("");
        let ip = find(mc, "IPv4Address").map(|v| v.trim().to_string()).unwrap_or_else(|| "0.0.0.0".into());
        format!(
            "<tr2:Configuration {a}><tt:Name>{}</tt:Name><tt:UseCount>{}</tt:UseCount><tt:Encoding>{}</tt:Encoding>\
             <tt:Resolution><tt:Width>{}</tt:Width><tt:Height>{}</tt:Height></tt:Resolution>\
             <tt:RateControl ConstantBitRate=\"{}\"><tt:FrameRateLimit>{}</tt:FrameRateLimit><tt:BitrateLimit>{}</tt:BitrateLimit></tt:RateControl>\
             <tt:Multicast><tt:Address><tt:Type>IPv4</tt:Type><tt:IPv4Address>{ip}</tt:IPv4Address></tt:Address>\
             <tt:Port>{}</tt:Port><tt:TTL>{}</tt:TTL><tt:AutoStart>{}</tt:AutoStart></tt:Multicast>\
             <tt:Quality>{}</tt:Quality></tr2:Configuration>",
            escape(&self.name), self.use_count, self.encoding, self.w, self.h, self.cbr, self.fps, self.bitrate,
            text(mc, "Port").unwrap_or_else(|| "0".into()),
            text(mc, "TTL").unwrap_or_else(|| "0".into()),
            text(mc, "AutoStart").unwrap_or_else(|| "false".into()),
            self.quality
        )
    }
}

impl Camera {
    /// Log in, set the main stream for the link and read its RTSP address (no login added to it).
    fn make_ready(&mut self, device: &str, ip: std::net::Ipv4Addr) -> Result<AutoCam, String> {
        let (mut media1, mut media2) = (String::new(), String::new());
        if let Ok(r) = self.call(
            device,
            "http://www.onvif.org/ver10/device/wsdl/GetServices",
            "<tds:GetServices><tds:IncludeCapability>false</tds:IncludeCapability></tds:GetServices>",
            true,
        ) {
            for (_, s) in elements(&r, "Service") {
                let (Some(ns), Some(x)) = (text(s, "Namespace"), text(s, "XAddr")) else { continue };
                if ns == MEDIA1 {
                    media1 = same_host(device, &x);
                } else if ns == MEDIA2 {
                    media2 = same_host(device, &x);
                }
            }
        }
        if media1.is_empty() {
            let caps = self.call(
                device,
                "http://www.onvif.org/ver10/device/wsdl/GetCapabilities",
                "<tds:GetCapabilities><tds:Category>Media</tds:Category></tds:GetCapabilities>",
                true,
            )?;
            media1 = find(&caps, "Media").and_then(|m| text(m, "XAddr")).map_or(device.to_string(), |x| same_host(device, &x));
        }
        // the first call that needs the login: a refused one ends here
        let profiles = self.call(&media1, &format!("{MEDIA1}/GetProfiles"), "<trt:GetProfiles/>", true)?;
        let mut changed = Vec::new();
        let main = if media2.is_empty() {
            self.h264_media1(&media1, &profiles, &mut changed)?
        } else {
            self.h264_media2(&media2, &mut changed)?
        };
        let profiles = if changed.is_empty() {
            profiles
        } else {
            self.call(&media1, &format!("{MEDIA1}/GetProfiles"), "<trt:GetProfiles/>", true)?
        };
        // the profile on the main encoder configuration, else the largest video one
        let mut pick: Option<(u64, String)> = None;
        for (attrs, inner) in elements(&profiles, "Profiles") {
            let Some(token) = attr(attrs, "token") else { continue };
            let Some((ea, enc)) = elements(inner, "VideoEncoderConfiguration").into_iter().next() else { continue };
            let cfg = EncoderConfig::parse(attr(ea, "token").unwrap_or(""), enc);
            if cfg.encoding.eq_ignore_ascii_case("JPEG") {
                continue;
            }
            let score = if !main.is_empty() && cfg.token == main { u64::MAX } else { u64::from(cfg.width) * u64::from(cfg.height) };
            if pick.as_ref().is_none_or(|p| score > p.0) {
                pick = Some((score, token.to_string()));
            }
        }
        let (_, profile) = pick.ok_or("no video profile")?;
        let body = format!(
            "<trt:GetStreamUri><trt:StreamSetup><tt:Stream>RTP-Unicast</tt:Stream><tt:Transport><tt:Protocol>RTSP</tt:Protocol>\
             </tt:Transport></trt:StreamSetup><trt:ProfileToken>{}</trt:ProfileToken></trt:GetStreamUri>",
            escape(&profile)
        );
        let r = self.call(&media1, &format!("{MEDIA1}/GetStreamUri"), &body, true)?;
        let uri = text(&r, "Uri").ok_or("no stream address in the GetStreamUri answer")?;
        let model = self
            .call(device, "http://www.onvif.org/ver10/device/wsdl/GetDeviceInformation", "<tds:GetDeviceInformation/>", true)
            .ok()
            .map(|r| format!("{} {}", text(&r, "Manufacturer").unwrap_or_default(), text(&r, "Model").unwrap_or_default()).trim().to_string())
            .unwrap_or_default();
        Ok(AutoCam { rtsp: on_host(&uri, ip), model, changed: changed.join("; "), ..Default::default() })
    }

    /// Media2: the main stream (the largest video encoder) made right for the link; its token.
    fn h264_media2(&mut self, m2: &str, changed: &mut Vec<String>) -> Result<String, String> {
        let get = format!("{MEDIA2}/GetVideoEncoderConfigurations");
        let r = self.call(m2, &get, "<tr2:GetVideoEncoderConfigurations/>", true)?;
        let main = Enc2::all(&r).into_iter().filter(|e| !e.is("JPEG")).max_by_key(|e| u64::from(e.w) * u64::from(e.h)).ok_or("no video encoder")?;
        let body = format!(
            "<tr2:GetVideoEncoderConfigurationOptions><tr2:ConfigurationToken>{}</tr2:ConfigurationToken></tr2:GetVideoEncoderConfigurationOptions>",
            escape(&main.token)
        );
        let opts = self.call(m2, &format!("{MEDIA2}/GetVideoEncoderConfigurationOptions"), &body, true)?;
        let Some((oa, oi)) =
            elements(&opts, "Options").into_iter().find(|(_, i)| text(i, "Encoding").is_some_and(|e| e.eq_ignore_ascii_case("H264")))
        else {
            return if main.is("H264") { Ok(main.token) } else { Err(format!("the camera cannot send H.264 (it sends {})", main.encoding)) };
        };
        let want = main.for_link(oa, oi);
        if want.differs(&main) {
            self.call(
                m2,
                &format!("{MEDIA2}/SetVideoEncoderConfiguration"),
                &format!("<tr2:SetVideoEncoderConfiguration>{}</tr2:SetVideoEncoderConfiguration>", want.to_xml()),
                true,
            )
            .map_err(|e| format!("setting the main stream to H.264: {e}"))?;
            changed.push(format!(
                "main stream {} {}x{} {} key frame every {} -> H.264 {}x{} {} key frame every {}",
                main.encoding, main.w, main.h, main.profile.as_deref().unwrap_or(""), main.gov.unwrap_or(0),
                want.w, want.h, want.profile.as_deref().unwrap_or(""), want.gov.unwrap_or(0)
            ));
            // read back: some cameras answer the call and keep what they had
            let r = self.call(m2, &get, "<tr2:GetVideoEncoderConfigurations/>", true)?;
            if let Some(now) = Enc2::all(&r).into_iter().find(|e| e.token == main.token)
                && !now.is("H264")
            {
                return Err(format!("the camera stays at {} after being set to H.264", now.encoding));
            }
        }
        Ok(main.token)
    }

    /// Media (ver10) only: the main stream to H.264 with a key frame every second; its token.
    fn h264_media1(&mut self, m1: &str, profiles: &str, changed: &mut Vec<String>) -> Result<String, String> {
        let main = elements(profiles, "Profiles")
            .into_iter()
            .filter_map(|(_, inner)| {
                elements(inner, "VideoEncoderConfiguration").into_iter().next().map(|(a, i)| EncoderConfig::parse(attr(a, "token").unwrap_or(""), i))
            })
            .filter(|c| !c.encoding.eq_ignore_ascii_case("JPEG"))
            .max_by_key(|c| u64::from(c.width) * u64::from(c.height));
        let Some(mut cfg) = main else { return Ok(String::new()) };
        let fps = cfg.frame_rate_limit.parse::<f32>().unwrap_or(25.0).round().max(1.0) as u32;
        let gov_ok = cfg.gov_length.as_deref().and_then(|g| g.parse::<u32>().ok()).is_some_and(|g| g <= fps * 3 / 2);
        if cfg.encoding.eq_ignore_ascii_case("H264") && gov_ok {
            return Ok(cfg.token);
        }
        let before = format!("{} {}x{}", cfg.encoding, cfg.width, cfg.height);
        cfg.encoding = "H264".into();
        cfg.gov_length = Some(fps.to_string());
        if cfg.h264_profile.is_none() {
            cfg.h264_profile = Some("Baseline".into());
        }
        let body = format!(
            "<trt:SetVideoEncoderConfiguration><trt:Configuration token=\"{}\">{}</trt:Configuration>\
             <trt:ForcePersistence>true</trt:ForcePersistence></trt:SetVideoEncoderConfiguration>",
            escape(&cfg.token),
            cfg.to_xml()
        );
        self.call(m1, &format!("{MEDIA1}/SetVideoEncoderConfiguration"), &body, true)
            .map_err(|e| format!("setting the main stream to H.264: {e}"))?;
        changed.push(format!("main stream {before} -> H.264, key frame every {fps}"));
        Ok(cfg.token)
    }
}

// ------------------------------------------------------------------ thread --

/// Apply `RtspShared::want_kbps` to the camera while pass-through and adaptation are on.
pub fn spawn(shared: Arc<RtspShared>) {
    std::thread::Builder::new()
        .name("onvif".into())
        .spawn(move || run(shared))
        .expect("spawn onvif thread");
}

fn set_status(shared: &RtspShared, s: String) {
    let mut st = shared.onvif_status.lock().unwrap();
    if *st != s {
        log(&format!("onvif: {s}"));
        *st = s;
    }
}

fn run(shared: Arc<RtspShared>) {
    let mut cam: Option<(String, Camera)> = None;
    let mut retry_at = Instant::now();
    let mut last_set = Instant::now() - Duration::from_secs(60);
    let mut last_read = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(250));
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        let active = shared.wanted.load(Ordering::Relaxed)
            && shared.pass.load(Ordering::Relaxed)
            && shared.onvif_adapt.load(Ordering::Relaxed);
        if !active {
            if cam.take().is_some() || shared.onvif_ok.load(Ordering::Relaxed) {
                shared.onvif_ok.store(false, Ordering::Relaxed);
            }
            set_status(&shared, "off".into());
            continue;
        }
        let rtsp = shared.url.lock().unwrap().clone();
        let explicit = shared.onvif_url.lock().unwrap().clone();
        let Some((device, user, pass)) = device_for(&rtsp, &explicit) else {
            set_status(&shared, "no camera URL".into());
            continue;
        };
        let key = format!("{device}|{user}|{pass}|{rtsp}");
        if cam.as_ref().is_some_and(|(k, _)| *k != key) {
            cam = None;
            shared.onvif_ok.store(false, Ordering::Relaxed);
        }
        if cam.is_none() {
            if Instant::now() < retry_at {
                continue;
            }
            let dims = *shared.dims.lock().unwrap();
            match Camera::connect(&device, &user, &pass, &rtsp, dims) {
                Ok(c) => {
                    let range = c.range.map_or(String::new(), |(lo, hi)| format!(", accepts {lo}-{hi}"));
                    set_status(&shared, format!(
                        "profile {} {}x{}, bitrate limit {} kbps{range}",
                        c.profile, c.config.width, c.config.height, c.config.bitrate_kbps
                    ));
                    shared.onvif_kbps.store(c.config.bitrate_kbps, Ordering::Relaxed);
                    if shared.onvif_base_kbps.load(Ordering::Relaxed) == 0 {
                        shared.onvif_base_kbps.store(c.config.bitrate_kbps, Ordering::Relaxed);
                    }
                    *shared.onvif_range.lock().unwrap() = c.range.unwrap_or((0, 0));
                    shared.onvif_ok.store(true, Ordering::Relaxed);
                    cam = Some((key, c));
                }
                Err(e) => {
                    set_status(&shared, format!("error: {e} ({device})"));
                    retry_at = Instant::now() + Duration::from_secs(10);
                }
            }
            continue;
        }
        let want = shared.want_kbps.load(Ordering::Relaxed);
        let Some((_, c)) = cam.as_mut() else { continue };
        // read the camera back now and then: it may have restarted at its own bitrate, or
        // another client may have changed it, and then the decisions below would compare
        // against a value the camera no longer has
        if last_read.elapsed() >= Duration::from_secs(10) {
            last_read = Instant::now();
            if let Err(e) = c.refresh() {
                set_status(&shared, format!("error: {e}"));
                shared.onvif_ok.store(false, Ordering::Relaxed);
                cam = None;
                retry_at = Instant::now() + Duration::from_secs(5);
                continue;
            }
            shared.onvif_kbps.store(c.config.bitrate_kbps, Ordering::Relaxed);
        }
        let now = c.config.bitrate_kbps.max(1);
        if want == 0 || (want as f32 - now as f32).abs() < now as f32 * 0.10 {
            continue;
        }
        // down after 2 s, up after 5 s: a camera may restart its stream on every change
        let gap = if want < now { Duration::from_secs(2) } else { Duration::from_secs(5) };
        if last_set.elapsed() < gap {
            continue;
        }
        last_set = Instant::now();
        match c.set_bitrate(want) {
            Ok(k) => {
                log(&format!("onvif: bitrate limit {now} -> {k} kbps"));
                shared.onvif_kbps.store(k, Ordering::Relaxed);
                set_status(&shared, format!("profile {} {}x{}, bitrate limit {k} kbps (following the link)", c.profile, c.config.width, c.config.height));
            }
            Err(e) => {
                set_status(&shared, format!("error: {e}"));
                shared.onvif_ok.store(false, Ordering::Relaxed);
                cam = None;
                retry_at = Instant::now() + Duration::from_secs(5);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hashes_match_known_vectors() {
        assert_eq!(hex(&sha1(b"")), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
        assert_eq!(hex(&sha1(b"abc")), "a9993e364706816aba3e25717850c26c9cd0d89d");
        assert_eq!(hex(&md5(b"")), "d41d8cd98f00b204e9800998ecf8427e");
        assert_eq!(hex(&md5(b"abc")), "900150983cd24fb0d6963f7d28e17f72");
        let long = vec![b'a'; 1000];
        assert_eq!(hex(&sha1(&long)), "291e9a6c66994949b57ba5e650361e98fc36b1ba");
        assert_eq!(hex(&md5(&long)), "cabe45dcc9ae5b66ba86600cca6b8ba8");
    }

    #[test]
    fn base64_round_trip() {
        for s in [&b""[..], b"f", b"fo", b"foo", b"foobar", &[0u8, 255, 17, 3]] {
            assert_eq!(b64_decode(&b64_encode(s)).unwrap(), s);
        }
        assert_eq!(b64_encode(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn ws_security_digest_example() {
        // the example of the ONVIF Application Programmer's Guide
        let nonce = b64_decode("LKqI6G/AikKCQrN0zqZFlg==").unwrap();
        assert_eq!(password_digest(&nonce, "2010-09-16T07:50:45Z", "userpassword"), "tuOSpGlFlIXsozq4HFNeeGeFLEI=");
    }

    #[test]
    fn iso_time() {
        assert_eq!(iso_utc(0), "1970-01-01T00:00:00Z");
        assert_eq!(iso_utc(1_284_623_445), "2010-09-16T07:50:45Z");
        assert_eq!(days_from_civil(2010, 9, 16) * 86_400 + 7 * 3600 + 50 * 60 + 45, 1_284_623_445);
    }

    #[test]
    fn xml_scanning() {
        let x = r#"<env:Body><trt:GetProfilesResponse><trt:Profiles token="P1" fixed="true"><tt:Name>a</tt:Name>
            <tt:VideoEncoderConfiguration token="E1"><tt:Encoding>H264</tt:Encoding><tt:RateControl>
            <tt:BitrateLimit>2048</tt:BitrateLimit></tt:RateControl></tt:VideoEncoderConfiguration></trt:Profiles>
            <trt:Profiles token='P2'/></trt:GetProfilesResponse></env:Body>"#;
        let p = elements(x, "Profiles");
        assert_eq!(p.len(), 2);
        assert_eq!(attr(p[0].0, "token"), Some("P1"));
        assert_eq!(attr(p[1].0, "token"), Some("P2"));
        let (ea, e) = elements(p[0].1, "VideoEncoderConfiguration")[0];
        assert_eq!(attr(ea, "token"), Some("E1"));
        assert_eq!(EncoderConfig::parse("E1", e).bitrate_kbps, 2048);
        assert!(find(x, "Configuration").is_none());
    }

    #[test]
    fn device_from_rtsp() {
        let (d, u, p) = device_for("rtsp://admin:pw@192.168.1.108:554/cam/realmonitor?channel=1&subtype=1", "").unwrap();
        assert_eq!((d.as_str(), u.as_str(), p.as_str()), ("http://192.168.1.108/onvif/device_service", "admin", "pw"));
        assert_eq!(path_and_query("rtsp://h/cam/realmonitor?channel=1&subtype=1").1, vec!["channel=1", "subtype=1"]);
        assert_eq!(same_host("http://10.0.0.5/onvif/device_service", "http://192.168.1.1:8999/onvif/Media"), "http://10.0.0.5:8999/onvif/Media");
    }

    #[test]
    fn logins_in_urls() {
        let ip: std::net::Ipv4Addr = "172.31.77.187".parse().unwrap();
        // the XM camera's own stream address (its login hash in the path), reported on another host
        let uri = "rtsp://192.168.1.10:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream";
        let on = on_host(uri, ip);
        assert_eq!(on, "rtsp://172.31.77.187:554/user=admin_password=tlJwpbo6_channel=0_stream=0&onvif=0.sdp?real_stream");
        assert_eq!(with_login(&on, "admin", ""), on.replace("rtsp://", "rtsp://admin@"));
        let l = with_login("rtsp://u:old@10.0.0.2/live", "ad@m", "p:w/%");
        assert_eq!(l, "rtsp://ad%40m:p%3Aw%2F%25@10.0.0.2/live");
        let (_, u, p) = device_for(&l, "").unwrap();
        assert_eq!((u.as_str(), p.as_str()), ("ad@m", "p:w/%"));
        assert_eq!(on_host("rtsp://cam.local/x", ip), "rtsp://172.31.77.187/x");
    }

    #[test]
    fn media2_for_the_link() {
        // what the XM GK7205 answered on 14/11 (Media2): H.265 at 2560x1440, key frame every 40
        let cfgs = r#"<tr2:GetVideoEncoderConfigurationsResponse><tr2:Configurations token="000" GovLength="40" Profile="Main"><tt:Name>V_ENC_000</tt:Name><tt:UseCount>1</tt:UseCount><tt:Encoding>H265</tt:Encoding><tt:Resolution><tt:Width>2560</tt:Width><tt:Height>1440</tt:Height></tt:Resolution><tt:RateControl ConstantBitRate="false"><tt:FrameRateLimit>20.0</tt:FrameRateLimit><tt:BitrateLimit>2560</tt:BitrateLimit></tt:RateControl><tt:Multicast><tt:Address><tt:Type>IPv4</tt:Type><tt:IPv4Address>239.0.1.0</tt:IPv4Address></tt:Address><tt:Port>32002</tt:Port><tt:TTL>2</tt:TTL><tt:AutoStart>false</tt:AutoStart></tt:Multicast><tt:Quality>4.00</tt:Quality></tr2:Configurations><tr2:Configurations token="002"><tt:Name>V_ENC_002</tt:Name><tt:UseCount>1</tt:UseCount><tt:Encoding>JPEG</tt:Encoding><tt:Resolution><tt:Width>704</tt:Width><tt:Height>576</tt:Height></tt:Resolution></tr2:Configurations></tr2:GetVideoEncoderConfigurationsResponse>"#;
        let opts = r#"<tr2:Options GovLengthRange="1 300" FrameRatesSupported="19 18" ProfilesSupported="Baseline Main High" ConstantBitRateSupported="true"><tt:Encoding>H264</tt:Encoding><tt:ResolutionsAvailable><tt:Width>1280</tt:Width><tt:Height>720</tt:Height></tt:ResolutionsAvailable><tt:ResolutionsAvailable><tt:Width>1920</tt:Width><tt:Height>1080</tt:Height></tt:ResolutionsAvailable><tt:ResolutionsAvailable><tt:Width>2560</tt:Width><tt:Height>1440</tt:Height></tt:ResolutionsAvailable></tr2:Options>"#;
        let all = Enc2::all(cfgs);
        assert_eq!(all.len(), 2);
        let main = all.into_iter().filter(|e| !e.is("JPEG")).max_by_key(|e| e.w * e.h).unwrap();
        assert_eq!((main.token.as_str(), main.encoding.as_str(), main.w, main.gov), ("000", "H265", 2560, Some(40)));
        let (oa, oi) = elements(opts, "Options").into_iter().next().unwrap();
        let want = main.for_link(oa, oi);
        assert_eq!((want.encoding.as_str(), want.w, want.h, want.profile.as_deref(), want.gov), ("H264", 1920, 1080, Some("Baseline"), Some(20)));
        assert!(want.differs(&main));
        assert!(!want.differs(&want.for_link(oa, oi)));
        let x = want.to_xml();
        assert!(x.starts_with("<tr2:Configuration token=\"000\" GovLength=\"20\" Profile=\"Baseline\">"));
        assert!(x.contains("<tt:Encoding>H264</tt:Encoding>") && x.contains("<tt:IPv4Address>239.0.1.0</tt:IPv4Address>"));
        // what it writes reads back the same
        let back = Enc2::parse("token=\"000\" GovLength=\"20\" Profile=\"Baseline\"", &x);
        assert!(!back.differs(&want) && back.bitrate == "2560" && back.fps == "20.0");
    }
}
