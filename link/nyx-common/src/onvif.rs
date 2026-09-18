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
xmlns:trt=\"http://www.onvif.org/ver10/media/wsdl\" xmlns:tt=\"http://www.onvif.org/ver10/schema\"";

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
}

impl Camera {
    /// Find the camera's media service and the profile whose stream is `rtsp_url` (the profile
    /// with the same path and query, else the one of the same picture size, else the first).
    pub fn connect(device_url: &str, user: &str, pass: &str, rtsp_url: &str, dims: (u32, u32)) -> Result<Camera, String> {
        let mut cam = Camera {
            media: device_url.to_string(),
            user: user.to_string(),
            pass: pass.to_string(),
            clock_offset: 0,
            digest: None,
            profile: String::new(),
            config: EncoderConfig::default(),
            range: None,
        };
        // the camera's clock (no authentication needed): a WS-Security timestamp off by more than a
        // few seconds is refused
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
        let caps = cam.call(
            device_url,
            "http://www.onvif.org/ver10/device/wsdl/GetCapabilities",
            "<tds:GetCapabilities><tds:Category>Media</tds:Category></tds:GetCapabilities>",
            true,
        )?;
        if let Some(x) = find(&caps, "Media").and_then(|m| text(m, "XAddr")) {
            cam.media = same_host(device_url, &x);
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
    let dec = |s: &str| s.replace("%40", "@").replace("%3A", ":").replace("%3a", ":");
    Some((device, dec(user), dec(pass)))
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
}
