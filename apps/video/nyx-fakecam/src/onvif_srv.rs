//! The fake camera's ONVIF service: the Device and Media 1.0 calls the board's client uses
//! (nyx_common::onvif), answered the way cameras answer them, so the whole bitrate loop can be
//! tried on the bench. A SetVideoEncoderConfiguration changes the encoder's bitrate at once.
//!
//! Authentication when `--pass` is given: WS-Security UsernameToken with a password digest, or,
//! with `--http-digest`, HTTP Digest (the two ways real cameras ask for it).

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use nyx_common::logging::log;
use nyx_common::onvif::{self, EncoderConfig, attr, b64_decode, elements, find, hex, md5, password_digest};

pub struct State {
    /// the encoder's bitrate limit (kbit/s); the encoder loop follows it
    pub kbps: AtomicU32,
    pub width: u32,
    pub height: u32,
    pub fps: f32,
    pub gop: u32,
    pub rtsp_port: u16,
    pub user: String,
    pub pass: String,
    pub http_digest: bool,
    nonce: String,
}

impl State {
    #[allow(clippy::too_many_arguments)]
    pub fn new(kbps: u32, width: u32, height: u32, fps: f32, gop: u32, rtsp_port: u16, user: String, pass: String, http_digest: bool) -> Arc<State> {
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(7, |d| d.as_nanos());
        Arc::new(State {
            kbps: AtomicU32::new(kbps),
            width,
            height,
            fps,
            gop,
            rtsp_port,
            user,
            pass,
            http_digest,
            nonce: hex(&md5(seed.to_string().as_bytes())),
        })
    }

    fn config(&self) -> EncoderConfig {
        EncoderConfig {
            token: "enc_main".into(),
            name: "main stream".into(),
            use_count: "1".into(),
            encoding: "H264".into(),
            width: self.width,
            height: self.height,
            quality: "5".into(),
            frame_rate_limit: format!("{}", self.fps.round() as u32),
            encoding_interval: "1".into(),
            bitrate_kbps: self.kbps.load(Ordering::Relaxed),
            gov_length: Some(self.gop.to_string()),
            h264_profile: Some("Main".into()),
            multicast: None,
            session_timeout: "PT60S".into(),
        }
    }
}

/// Listen on `port` (another one if it is taken) and answer in a thread; the port in use.
pub fn spawn(state: Arc<State>, port: u16) -> Option<u16> {
    let listener = [port, 8080, 8000]
        .into_iter()
        .find_map(|p| TcpListener::bind(("0.0.0.0", p)).ok().map(|l| (l, p)));
    let Some((listener, bound)) = listener else {
        log(&format!("fakecam: onvif: cannot listen on {port}, 8080 or 8000"));
        return None;
    };
    std::thread::Builder::new()
        .name("onvif-srv".into())
        .spawn(move || {
            for conn in listener.incoming() {
                let Ok(s) = conn else { continue };
                let st = state.clone();
                std::thread::spawn(move || {
                    if let Err(e) = serve(s, &st) {
                        log(&format!("fakecam: onvif: {e}"));
                    }
                });
            }
        })
        .expect("spawn onvif-srv");
    Some(bound)
}

fn serve(stream: TcpStream, st: &State) -> std::io::Result<()> {
    let _ = stream.set_read_timeout(Some(std::time::Duration::from_secs(5)));
    let local = stream.local_addr()?;
    let mut rd = BufReader::new(stream.try_clone()?);
    let mut w = stream;
    let mut head = Vec::new();
    loop {
        let mut line = String::new();
        if rd.read_line(&mut line)? == 0 {
            return Ok(());
        }
        if line == "\r\n" || line == "\n" {
            break;
        }
        head.push(line.trim_end().to_string());
    }
    let header = |name: &str| {
        head.iter()
            .find(|l| l.to_ascii_lowercase().starts_with(&format!("{}:", name.to_ascii_lowercase())))
            .map(|l| l[name.len() + 1..].trim().to_string())
    };
    let len: usize = header("Content-Length").and_then(|v| v.parse().ok()).unwrap_or(0);
    let mut body = vec![0u8; len.min(1 << 20)];
    rd.read_exact(&mut body)?;
    let body = String::from_utf8_lossy(&body).into_owned();
    let host = header("Host").unwrap_or_else(|| local.to_string());
    let op = find(&body, "Body").and_then(first_element).unwrap_or_default();

    // authentication (the clock is open to everyone: clients need it to sign)
    if !st.pass.is_empty() && op != "GetSystemDateAndTime" {
        let ok = if st.http_digest { digest_ok(st, header("Authorization").as_deref(), &head) } else { ws_ok(st, &body) };
        if !ok {
            log(&format!("fakecam: onvif: {op} refused (authentication)"));
            if st.http_digest {
                let msg = format!(
                    "HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Digest realm=\"nyx-fakecam\", qop=\"auth\", nonce=\"{}\"\r\n\
                     Content-Length: 0\r\nConnection: close\r\n\r\n",
                    st.nonce
                );
                return w.write_all(msg.as_bytes());
            }
            return reply(&mut w, 400, &fault("ter:NotAuthorized", "Sender not authorized"));
        }
    }

    let cfg = st.config();
    let answer = match op.as_str() {
        "GetSystemDateAndTime" => {
            let iso = onvif::iso_utc(onvif::unix_now()); // YYYY-MM-DDTHH:MM:SSZ
            let n = |a: usize, b: usize| iso[a..b].trim_start_matches('0').parse::<u32>().unwrap_or(0);
            format!(
                "<tds:GetSystemDateAndTimeResponse><tds:SystemDateAndTime><tt:DateTimeType>NTP</tt:DateTimeType>\
                 <tt:DaylightSavings>false</tt:DaylightSavings><tt:UTCDateTime><tt:Time><tt:Hour>{}</tt:Hour><tt:Minute>{}</tt:Minute>\
                 <tt:Second>{}</tt:Second></tt:Time><tt:Date><tt:Year>{}</tt:Year><tt:Month>{}</tt:Month><tt:Day>{}</tt:Day></tt:Date>\
                 </tt:UTCDateTime></tds:SystemDateAndTime></tds:GetSystemDateAndTimeResponse>",
                n(11, 13), n(14, 16), n(17, 19), n(0, 4), n(5, 7), n(8, 10)
            )
        }
        "GetCapabilities" => format!(
            "<tds:GetCapabilitiesResponse><tds:Capabilities><tt:Device><tt:XAddr>http://{host}/onvif/device_service</tt:XAddr></tt:Device>\
             <tt:Media><tt:XAddr>http://{host}/onvif/media_service</tt:XAddr><tt:StreamingCapabilities><tt:RTPMulticast>false</tt:RTPMulticast>\
             <tt:RTP_TCP>true</tt:RTP_TCP><tt:RTP_RTSP_TCP>true</tt:RTP_RTSP_TCP></tt:StreamingCapabilities></tt:Media>\
             </tds:Capabilities></tds:GetCapabilitiesResponse>"
        ),
        "GetDeviceInformation" => "<tds:GetDeviceInformationResponse><tds:Manufacturer>NyxHop</tds:Manufacturer><tds:Model>nyx-fakecam</tds:Model>\
             <tds:FirmwareVersion>1</tds:FirmwareVersion><tds:SerialNumber>0</tds:SerialNumber><tds:HardwareId>pc</tds:HardwareId>\
             </tds:GetDeviceInformationResponse>"
            .to_string(),
        "GetProfiles" => format!(
            "<trt:GetProfilesResponse><trt:Profiles token=\"main\" fixed=\"true\"><tt:Name>main</tt:Name>\
             <tt:VideoEncoderConfiguration token=\"{}\">{}</tt:VideoEncoderConfiguration></trt:Profiles></trt:GetProfilesResponse>",
            cfg.token,
            cfg.to_xml()
        ),
        "GetStreamUri" => {
            let ip = host.rsplit_once(':').map_or(host.as_str(), |(h, _)| h);
            format!(
                "<trt:GetStreamUriResponse><trt:MediaUri><tt:Uri>rtsp://{ip}:{}/cam</tt:Uri><tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>\
                 <tt:InvalidAfterReboot>false</tt:InvalidAfterReboot><tt:Timeout>PT0S</tt:Timeout></trt:MediaUri></trt:GetStreamUriResponse>",
                st.rtsp_port
            )
        }
        "GetVideoEncoderConfiguration" => format!(
            "<trt:GetVideoEncoderConfigurationResponse><trt:Configuration token=\"{}\">{}</trt:Configuration>\
             </trt:GetVideoEncoderConfigurationResponse>",
            cfg.token,
            cfg.to_xml()
        ),
        "GetVideoEncoderConfigurations" => format!(
            "<trt:GetVideoEncoderConfigurationsResponse><trt:Configurations token=\"{}\">{}</trt:Configurations>\
             </trt:GetVideoEncoderConfigurationsResponse>",
            cfg.token,
            cfg.to_xml()
        ),
        "GetVideoEncoderConfigurationOptions" => format!(
            "<trt:GetVideoEncoderConfigurationOptionsResponse><trt:Options><tt:QualityRange><tt:Min>1</tt:Min><tt:Max>10</tt:Max></tt:QualityRange>\
             <tt:H264><tt:ResolutionsAvailable><tt:Width>{w}</tt:Width><tt:Height>{h}</tt:Height></tt:ResolutionsAvailable>\
             <tt:GovLengthRange><tt:Min>1</tt:Min><tt:Max>600</tt:Max></tt:GovLengthRange><tt:FrameRateRange><tt:Min>1</tt:Min><tt:Max>60</tt:Max></tt:FrameRateRange>\
             <tt:EncodingIntervalRange><tt:Min>1</tt:Min><tt:Max>1</tt:Max></tt:EncodingIntervalRange><tt:H264ProfilesSupported>Main</tt:H264ProfilesSupported></tt:H264>\
             <tt:Extension><tt:H264><tt:BitrateRange><tt:Min>100</tt:Min><tt:Max>8000</tt:Max></tt:BitrateRange></tt:H264></tt:Extension>\
             </trt:Options></trt:GetVideoEncoderConfigurationOptionsResponse>",
            w = st.width,
            h = st.height
        ),
        "SetVideoEncoderConfiguration" => {
            let conf = elements(&body, "Configuration").into_iter().next();
            let new = conf.map(|(a, inner)| EncoderConfig::parse(attr(a, "token").unwrap_or(""), inner));
            match new {
                Some(c) if c.token == cfg.token && (100..=8000).contains(&c.bitrate_kbps) => {
                    let old = st.kbps.swap(c.bitrate_kbps, Ordering::Relaxed);
                    log(&format!("fakecam: onvif: bitrate limit {old} -> {} kbps", c.bitrate_kbps));
                    "<trt:SetVideoEncoderConfigurationResponse/>".to_string()
                }
                Some(c) if c.token != cfg.token => return reply(&mut w, 400, &fault("ter:NoConfig", "no such configuration")),
                _ => return reply(&mut w, 400, &fault("ter:ConfigModify", "bitrate out of range")),
            }
        }
        _ => {
            log(&format!("fakecam: onvif: {op} not supported"));
            return reply(&mut w, 400, &fault("ter:ActionNotSupported", &format!("{op} is not supported")));
        }
    };
    reply(&mut w, 200, &answer)
}

/// The local name of the first element in a SOAP body.
fn first_element(body: &str) -> Option<String> {
    let start = body.find('<')? + 1;
    let rest = &body[start..];
    let end = rest.find(|c: char| c.is_whitespace() || c == '>' || c == '/')?;
    Some(rest[..end].rsplit(':').next()?.to_string())
}

fn ws_ok(st: &State, body: &str) -> bool {
    let Some(tok) = find(body, "UsernameToken") else { return false };
    let t = |k: &str| find(tok, k).map(|v| v.trim().to_string());
    let (Some(user), Some(pw), Some(nonce), Some(created)) = (t("Username"), t("Password"), t("Nonce"), t("Created")) else {
        return false;
    };
    let Some(nonce) = b64_decode(&nonce) else { return false };
    user == st.user && pw == password_digest(&nonce, &created, &st.pass)
}

fn digest_ok(st: &State, auth: Option<&str>, head: &[String]) -> bool {
    let Some(a) = auth.and_then(|a| a.strip_prefix("Digest")) else { return false };
    let method = head.first().and_then(|l| l.split_whitespace().next()).unwrap_or("POST");
    let get = |k: &str| attr(a, k).map(str::to_string).or_else(|| {
        // unquoted values: qop=auth, nc=00000001
        a.split(',').filter_map(|p| p.trim().split_once('=')).find(|(n, _)| *n == k).map(|(_, v)| v.trim().to_string())
    });
    let (Some(user), Some(realm), Some(nonce), Some(uri), Some(resp)) = (get("username"), get("realm"), get("nonce"), get("uri"), get("response")) else {
        return false;
    };
    if user != st.user || nonce != st.nonce {
        return false;
    }
    let ha1 = hex(&md5(format!("{user}:{realm}:{}", st.pass).as_bytes()));
    let ha2 = hex(&md5(format!("{method}:{uri}").as_bytes()));
    let want = match (get("qop"), get("nc"), get("cnonce")) {
        (Some(qop), Some(nc), Some(cn)) => hex(&md5(format!("{ha1}:{nonce}:{nc}:{cn}:{qop}:{ha2}").as_bytes())),
        _ => hex(&md5(format!("{ha1}:{nonce}:{ha2}").as_bytes())),
    };
    resp == want
}

fn fault(code: &str, why: &str) -> String {
    format!(
        "<s:Fault><s:Code><s:Value>s:Sender</s:Value><s:Subcode><s:Value>{code}</s:Value></s:Subcode></s:Code>\
         <s:Reason><s:Text xml:lang=\"en\">{why}</s:Text></s:Reason></s:Fault>"
    )
}

fn reply(w: &mut TcpStream, status: u16, body_inner: &str) -> std::io::Result<()> {
    let env = format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?><s:Envelope xmlns:s=\"http://www.w3.org/2003/05/soap-envelope\" \
         xmlns:tds=\"http://www.onvif.org/ver10/device/wsdl\" xmlns:trt=\"http://www.onvif.org/ver10/media/wsdl\" \
         xmlns:tt=\"http://www.onvif.org/ver10/schema\" xmlns:ter=\"http://www.onvif.org/ver10/error\">\
         <s:Body>{body_inner}</s:Body></s:Envelope>"
    );
    let reason = if status == 200 { "OK" } else { "Bad Request" };
    let msg = format!(
        "HTTP/1.1 {status} {reason}\r\nContent-Type: application/soap+xml; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{env}",
        env.len()
    );
    w.write_all(msg.as_bytes())?;
    w.flush()
}
