//! MAVLink v2 for a pilot's ground station: the dozen messages it sends and reads, coded here
//! (field order and CRC_EXTRA as in the common dialect; the tests hold bytes made by pymavlink),
//! and the vehicle as they describe it.

use std::collections::VecDeque;
use std::time::Instant;

pub const GCS_SYS: u8 = 255;
pub const GCS_COMP: u8 = 190;

pub const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
pub const MAV_CMD_NAV_LAND: u16 = 21;
pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;
pub const MAV_CMD_DO_SET_MODE: u16 = 176;
pub const MAV_CMD_COMPONENT_ARM_DISARM: u16 = 400;
pub const MAV_CMD_SET_MESSAGE_INTERVAL: u16 = 511;

fn crc_step(b: u8, crc: u16) -> u16 {
    let mut t = b ^ (crc as u8);
    t ^= t << 4;
    let t = t as u16;
    (crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)
}

/// X.25 over the frame after its start byte, then the message's CRC_EXTRA
fn crc(bytes: &[u8], extra: u8) -> u16 {
    crc_step(extra, bytes.iter().fold(0xFFFF, |c, &b| crc_step(b, c)))
}

/// CRC_EXTRA of the messages read here (a frame of any other id is stepped over unchecked)
fn crc_extra(id: u32) -> Option<u8> {
    Some(match id {
        0 => 50,    // HEARTBEAT
        1 => 124,   // SYS_STATUS
        24 => 24,   // GPS_RAW_INT
        30 => 39,   // ATTITUDE
        32 => 185,  // LOCAL_POSITION_NED
        33 => 104,  // GLOBAL_POSITION_INT
        74 => 20,   // VFR_HUD
        77 => 143,  // COMMAND_ACK
        111 => 34,  // TIMESYNC
        242 => 104, // HOME_POSITION
        245 => 130, // EXTENDED_SYS_STATE
        253 => 83,  // STATUSTEXT
        _ => return None,
    })
}

/// Frames out, numbered
#[derive(Default)]
pub struct Encoder {
    seq: u8,
}

impl Encoder {
    fn frame(&mut self, id: u32, extra: u8, payload: &[u8]) -> Vec<u8> {
        // v2: the payload's trailing zeros are left out (one byte at least stays)
        let mut n = payload.len();
        while n > 1 && payload[n - 1] == 0 {
            n -= 1;
        }
        let mut f = Vec::with_capacity(12 + n);
        f.extend_from_slice(&[0xFD, n as u8, 0, 0, self.seq, GCS_SYS, GCS_COMP]);
        f.extend_from_slice(&id.to_le_bytes()[..3]);
        f.extend_from_slice(&payload[..n]);
        let c = crc(&f[1..], extra);
        f.extend_from_slice(&c.to_le_bytes());
        self.seq = self.seq.wrapping_add(1);
        f
    }

    /// HEARTBEAT as a ground station (MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID)
    pub fn heartbeat(&mut self) -> Vec<u8> {
        let mut p = [0u8; 9];
        p[4] = 6;
        p[5] = 8;
        p[8] = 3;
        self.frame(0, 50, &p)
    }

    /// MANUAL_CONTROL: x pitch, y roll, r yaw in -1000..1000, z throttle 0..1000 (500 = hold)
    pub fn manual_control(&mut self, target: u8, x: i16, y: i16, z: i16, r: i16) -> Vec<u8> {
        let mut p = [0u8; 11];
        p[0..2].copy_from_slice(&x.to_le_bytes());
        p[2..4].copy_from_slice(&y.to_le_bytes());
        p[4..6].copy_from_slice(&z.to_le_bytes());
        p[6..8].copy_from_slice(&r.to_le_bytes());
        p[10] = target;
        self.frame(69, 243, &p)
    }

    pub fn command_long(&mut self, sys: u8, comp: u8, cmd: u16, params: [f32; 7]) -> Vec<u8> {
        let mut p = [0u8; 33];
        for (i, v) in params.iter().enumerate() {
            p[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        p[28..30].copy_from_slice(&cmd.to_le_bytes());
        p[30] = sys;
        p[31] = comp;
        self.frame(76, 152, &p)
    }

    /// TIMESYNC asking (tc1 0): the answer carries ts1 back, which times the link's round trip
    pub fn timesync(&mut self, ts1: i64) -> Vec<u8> {
        let mut p = [0u8; 16];
        p[8..16].copy_from_slice(&ts1.to_le_bytes());
        self.frame(111, 34, &p)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Msg {
    Heartbeat { custom_mode: u32, autopilot: u8, base_mode: u8 },
    SysStatus { load: u16, voltage_mv: u16, remaining: i8 },
    /// the receiver's fix (0, 1 none; 2 2D; 3 and up 3D), its satellites (255 = not told) and
    /// where it is, 1e-7 degrees
    GpsRaw { fix: u8, sats: u8, lat: i32, lon: i32 },
    Attitude { roll: f32, pitch: f32, yaw: f32 },
    LocalPos { x: f32, y: f32, z: f32, vx: f32, vy: f32, vz: f32 },
    GlobalPos { lat: i32, lon: i32, alt_mm: i32, rel_mm: i32 },
    VfrHud { groundspeed: f32, climb: f32, heading: i16, throttle: u16 },
    CommandAck { command: u16, result: u8 },
    Timesync { tc1: i64, ts1: i64 },
    Home { lat: i32, lon: i32, alt_mm: i32 },
    ExtSys { landed: u8 },
    Text { severity: u8, text: String },
}

fn f32_at(p: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(p[o..o + 4].try_into().unwrap())
}
fn i32_at(p: &[u8], o: usize) -> i32 {
    i32::from_le_bytes(p[o..o + 4].try_into().unwrap())
}
fn u16_at(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(p[o..o + 2].try_into().unwrap())
}

fn decode(id: u32, p: &[u8]) -> Option<Msg> {
    Some(match id {
        0 => Msg::Heartbeat { custom_mode: u32::from_le_bytes(p[0..4].try_into().unwrap()), autopilot: p[5], base_mode: p[6] },
        1 => Msg::SysStatus { load: u16_at(p, 12), voltage_mv: u16_at(p, 14), remaining: p[30] as i8 },
        24 => Msg::GpsRaw { fix: p[28], sats: p[29], lat: i32_at(p, 8), lon: i32_at(p, 12) },
        30 => Msg::Attitude { roll: f32_at(p, 4), pitch: f32_at(p, 8), yaw: f32_at(p, 12) },
        32 => Msg::LocalPos { x: f32_at(p, 4), y: f32_at(p, 8), z: f32_at(p, 12), vx: f32_at(p, 16), vy: f32_at(p, 20), vz: f32_at(p, 24) },
        33 => Msg::GlobalPos { lat: i32_at(p, 4), lon: i32_at(p, 8), alt_mm: i32_at(p, 12), rel_mm: i32_at(p, 16) },
        74 => Msg::VfrHud { groundspeed: f32_at(p, 4), climb: f32_at(p, 12), heading: u16_at(p, 16) as i16, throttle: u16_at(p, 18) },
        77 => Msg::CommandAck { command: u16_at(p, 0), result: p[2] },
        111 => Msg::Timesync {
            tc1: i64::from_le_bytes(p[0..8].try_into().unwrap()),
            ts1: i64::from_le_bytes(p[8..16].try_into().unwrap()),
        },
        242 => Msg::Home { lat: i32_at(p, 0), lon: i32_at(p, 4), alt_mm: i32_at(p, 8) },
        245 => Msg::ExtSys { landed: p[1] },
        253 => {
            let t = &p[1..51];
            let end = t.iter().position(|&b| b == 0).unwrap_or(t.len());
            Msg::Text { severity: p[0], text: String::from_utf8_lossy(&t[..end]).into_owned() }
        }
        _ => return None,
    })
}

/// The messages of one datagram (v2 frames; anything else skipped): (system, component, message)
pub fn parse(buf: &[u8]) -> Vec<(u8, u8, Msg)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 12 <= buf.len() {
        if buf[i] != 0xFD {
            i += 1;
            continue;
        }
        let n = buf[i + 1] as usize;
        let signed = buf[i + 2] & 1 != 0;
        let total = 12 + n + if signed { 13 } else { 0 };
        if i + total > buf.len() {
            break;
        }
        let f = &buf[i..i + total];
        let id = u32::from_le_bytes([f[7], f[8], f[9], 0]);
        if let Some(extra) = crc_extra(id) {
            let want = u16::from_le_bytes([f[10 + n], f[11 + n]]);
            if crc(&f[1..10 + n], extra) == want {
                // (the payload as long as the message: v2 left its trailing zeros out)
                let mut p = [0u8; 64];
                p[..n.min(64)].copy_from_slice(&f[10..10 + n.min(64)]);
                if let Some(m) = decode(id, &p) {
                    out.push((f[5], f[6], m));
                }
            } else {
                i += 1;
                continue;
            }
        }
        i += total;
    }
    out
}

/// The flight controller on the other end (its heartbeat's autopilot)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Autopilot {
    #[default]
    Px4,
    ArduPilot,
}

/// The flight modes as the pilot picks them; each autopilot has its own numbers for them
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightMode {
    Position,
    Altitude,
    Stabilized,
    Hold,
    /// ArduPilot's GUIDED (its takeoff command needs it)
    Guided,
}

impl FlightMode {
    /// DO_SET_MODE's param2 and param3 (param1 = custom mode enabled): PX4's main and sub mode,
    /// ArduCopter's mode number (LOITER, ALT_HOLD, STABILIZE, BRAKE, GUIDED)
    pub fn params(self, ap: Autopilot) -> (f32, f32) {
        match (ap, self) {
            (Autopilot::Px4, FlightMode::Position) => (3.0, 0.0),
            (Autopilot::Px4, FlightMode::Altitude) => (2.0, 0.0),
            (Autopilot::Px4, FlightMode::Stabilized) => (7.0, 0.0),
            (Autopilot::Px4, FlightMode::Hold | FlightMode::Guided) => (4.0, 3.0),
            (Autopilot::ArduPilot, FlightMode::Position) => (5.0, 0.0),
            (Autopilot::ArduPilot, FlightMode::Altitude) => (2.0, 0.0),
            (Autopilot::ArduPilot, FlightMode::Stabilized) => (0.0, 0.0),
            (Autopilot::ArduPilot, FlightMode::Hold) => (17.0, 0.0),
            (Autopilot::ArduPilot, FlightMode::Guided) => (4.0, 0.0),
        }
    }
}

/// Where a place on the globe comes from
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Place {
    /// the autopilot's own position (GLOBAL_POSITION_INT): it flies by it
    Autopilot,
    /// the satellite receiver's alone (GPS_RAW_INT): it has a fix, the autopilot no position of
    /// its own yet (indoors, before its compass and its estimator agree)
    Receiver,
    /// neither tells now: the last place either told, this run or the one before
    Last,
}

/// The vehicle as its messages tell it
#[derive(Clone, Default)]
pub struct Vehicle {
    pub last_hb: Option<Instant>,
    pub sys: u8,
    pub comp: u8,
    pub armed: bool,
    pub autopilot: Autopilot,
    /// PX4: main mode (custom_mode bits 16..23); ArduPilot: the mode number
    pub main_mode: u8,
    pub sub_mode: u8,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    /// when its local position came last (an autopilot with none sends none)
    pub local_at: Option<Instant>,
    /// where it is on the globe, 1e-7 degrees (GLOBAL_POSITION_INT), and when that came last
    pub lat: i32,
    pub lon: i32,
    pub pos_at: Option<Instant>,
    pub amsl: f32,
    pub alt: f32,
    pub home_amsl: Option<f32>,
    /// home on the globe, 1e-7 degrees (HOME_POSITION)
    pub home: Option<(i32, i32)>,
    /// the satellite receiver (GPS_RAW_INT): when, its fix, its satellites; where it is while it
    /// has a fix (1e-7 degrees)
    pub gps: Option<(Instant, u8, u8)>,
    pub gps_ll: Option<(i32, i32)>,
    /// the last place on the globe the autopilot or the receiver told (1e-7 degrees), and its
    /// local position (north, east) there: from that place the local frame carries it on
    pub last_ll: Option<(i32, i32)>,
    last_xy: Option<(f32, f32)>,
    pub load: f32,
    pub batt_v: f32,
    pub batt_pct: i8,
    pub landed: u8,
    /// the throttle it flies with, % (VFR_HUD)
    pub throttle_pct: u16,
    pub rtt_ms: Option<f32>,
    pub rate: f32,
    pub texts: VecDeque<(Instant, u8, String)>,
    /// the flight's track, from where it armed: local (north, east in metres) and on the globe
    /// (1e-7 degrees)
    pub track: VecDeque<(f32, f32)>,
    pub gtrack: VecDeque<(i32, i32)>,
    /// how many times each track was thinned
    thinned: (u8, u8),
    /// where it armed: local, and on the globe if it knew
    pub takeoff: Option<(f32, f32)>,
    pub takeoff_ll: Option<(i32, i32)>,
}

/// A track's room: full, every other point goes and the spacing doubles, so a long flight stays
/// whole, only coarser
const TRACK_MAX: usize = 4000;

fn track_push<T: Copy>(track: &mut VecDeque<T>, thinned: &mut u8, gap: f32, p: T, dist: impl Fn(T, T) -> f32) {
    let gap = gap * (1u32 << (*thinned).min(16)) as f32;
    if track.back().is_some_and(|&q| dist(q, p) <= gap) {
        return;
    }
    track.push_back(p);
    if track.len() > TRACK_MAX {
        let mut i = 0usize;
        track.retain(|_| {
            i += 1;
            i % 2 == 1
        });
        *thinned += 1;
    }
}

/// Metres between two points on the globe a short way apart (1e-7 degrees)
fn ll_dist(a: (i32, i32), b: (i32, i32)) -> f32 {
    const M: f64 = 0.011_132; // a 1e-7 degree of latitude
    let dn = (b.0 - a.0) as f64 * M;
    let de = (b.1 - a.1) as f64 * M * (a.0 as f64 * 1e-7).to_radians().cos();
    dn.hypot(de) as f32
}

impl Vehicle {
    pub fn link(&self) -> bool {
        self.last_hb.is_some_and(|t| t.elapsed().as_secs_f32() < 3.0)
    }

    /// Where it is on the globe (latitude, longitude in degrees), while it tells: PX4 says nothing
    /// without a position, ArduPilot says 0, 0
    pub fn global(&self) -> Option<(f64, f64)> {
        let fresh = self.pos_at.is_some_and(|t| t.elapsed().as_secs_f32() < 3.0);
        fresh.then_some((self.lat as f64 * 1e-7, self.lon as f64 * 1e-7))
    }

    fn local(&self) -> bool {
        self.local_at.is_some_and(|t| t.elapsed().as_secs_f32() < 3.0)
    }

    /// A point of the local frame (north, east in metres) on the globe, carried from the last
    /// place known by the way the local frame has gone since: where an aircraft with no fix
    /// (indoors, flying by its own estimate) and its track are drawn
    pub fn carried(&self, n: f32, e: f32) -> Option<(f64, f64)> {
        let (lat, lon) = self.last_ll.map(|(lat, lon)| (lat as f64 * 1e-7, lon as f64 * 1e-7))?;
        let (dn, de) = match self.last_xy {
            Some((n0, e0)) if self.local() => ((n - n0) as f64, (e - e0) as f64),
            _ => (0.0, 0.0),
        };
        Some((lat + dn / 111_320.0, lon + de / (111_320.0 * lat.to_radians().cos())))
    }

    /// Where to draw it on the globe (degrees) and where that comes from: the autopilot's own
    /// position, else the receiver's while it has a fix, else the last place known
    pub fn place(&self) -> Option<((f64, f64), Place)> {
        let deg = |(lat, lon): (i32, i32)| (lat as f64 * 1e-7, lon as f64 * 1e-7);
        if let Some(ll) = self.global() {
            return Some((ll, Place::Autopilot));
        }
        if let (Some((fix, _)), Some(ll)) = (self.gps_fix(), self.gps_ll) {
            if fix >= 2 {
                return Some((deg(ll), Place::Receiver));
            }
        }
        self.carried(self.x, self.y).map(|ll| (ll, Place::Last))
    }

    /// Home on the globe (degrees): the autopilot's, else where it armed
    pub fn home_global(&self) -> Option<(f64, f64)> {
        self.home.or(self.takeoff_ll).map(|(lat, lon)| (lat as f64 * 1e-7, lon as f64 * 1e-7))
    }

    /// Home in the local frame (north, east): where it armed, else the frame's origin
    pub fn home_local(&self) -> (f32, f32) {
        self.takeoff.unwrap_or((0.0, 0.0))
    }

    /// The satellite receiver's fix (3 and up: 3D) and satellites (255: not told), while it tells
    pub fn gps_fix(&self) -> Option<(u8, u8)> {
        self.gps.filter(|g| g.0.elapsed().as_secs_f32() < 5.0).map(|g| (g.1, g.2))
    }

    /// A mode where the throttle stick is the thrust itself (no height hold): PX4 Manual, Acro,
    /// Stabilized; ArduPilot Stabilize, Acro
    pub fn manual_thrust(&self) -> bool {
        match self.autopilot {
            Autopilot::Px4 => matches!(self.main_mode, 1 | 5 | 7),
            Autopilot::ArduPilot => matches!(self.main_mode, 0 | 1),
        }
    }

    /// The pilot's mode the vehicle is in, if it is one of them
    pub fn flight_mode(&self) -> Option<FlightMode> {
        Some(match (self.autopilot, self.main_mode, self.sub_mode) {
            (Autopilot::Px4, 3, _) | (Autopilot::ArduPilot, 5 | 16, _) => FlightMode::Position,
            (Autopilot::Px4, 2, _) | (Autopilot::ArduPilot, 2, _) => FlightMode::Altitude,
            (Autopilot::Px4, 7, _) | (Autopilot::ArduPilot, 0, _) => FlightMode::Stabilized,
            _ => return None,
        })
    }

    pub fn mode_name(&self) -> &'static str {
        if self.autopilot == Autopilot::ArduPilot {
            return match self.main_mode {
                0 => "STABILIZE",
                1 => "ACRO",
                2 => "ALT HOLD",
                3 => "AUTO",
                4 => "GUIDED",
                5 => "LOITER",
                6 => "RTL",
                7 => "CIRCLE",
                9 => "LAND",
                16 => "POSHOLD",
                17 => "BRAKE",
                21 => "SMART RTL",
                _ => "-",
            };
        }
        match (self.main_mode, self.sub_mode) {
            (1, _) => "MANUAL",
            (2, _) => "ALTITUDE",
            (3, _) => "POSITION",
            (4, 2) => "TAKEOFF",
            (4, 3) => "HOLD",
            (4, 4) => "MISSION",
            (4, 5) => "RETURN",
            (4, 6) => "LAND",
            (4, _) => "AUTO",
            (5, _) => "ACRO",
            (6, _) => "OFFBOARD",
            (7, _) => "STABILIZED",
            _ => "-",
        }
    }

    pub fn update(&mut self, sys: u8, comp: u8, m: &Msg) {
        match *m {
            // (PX4 12, ArduPilot 3; a ground station's or a camera's heartbeat is not the vehicle)
            Msg::Heartbeat { custom_mode, autopilot, base_mode } if autopilot == 12 || autopilot == 3 => {
                let armed = base_mode & 0x80 != 0;
                // it arms: a new flight, its track starts here
                if armed && !self.armed && self.last_hb.is_some() {
                    self.track.clear();
                    self.gtrack.clear();
                    self.thinned = (0, 0);
                    self.takeoff = Some((self.x, self.y));
                    self.takeoff_ll = match self.place() {
                        Some((_, Place::Autopilot)) => Some((self.lat, self.lon)),
                        Some((_, Place::Receiver)) => self.gps_ll,
                        _ => None,
                    };
                }
                self.last_hb = Some(Instant::now());
                self.sys = sys;
                self.comp = comp;
                self.armed = armed;
                if autopilot == 3 {
                    self.autopilot = Autopilot::ArduPilot;
                    self.main_mode = custom_mode as u8;
                    self.sub_mode = 0;
                } else {
                    self.autopilot = Autopilot::Px4;
                    self.main_mode = ((custom_mode >> 16) & 0xFF) as u8;
                    self.sub_mode = ((custom_mode >> 24) & 0xFF) as u8;
                }
            }
            Msg::SysStatus { load, voltage_mv, remaining } => {
                self.load = load as f32 / 10.0;
                self.batt_v = voltage_mv as f32 / 1000.0;
                self.batt_pct = remaining;
            }
            Msg::Attitude { roll, pitch, yaw } => {
                self.roll = roll;
                self.pitch = pitch;
                self.yaw = yaw;
            }
            Msg::LocalPos { x, y, z, vx, vy, vz } => {
                (self.x, self.y, self.z, self.vx, self.vy, self.vz) = (x, y, z, vx, vy, vz);
                self.local_at = Some(Instant::now());
                // (a last place told before any local position, or kept from an earlier run: the
                // local frame carries it on from here)
                if self.last_ll.is_some() && self.last_xy.is_none() {
                    self.last_xy = Some((x, y));
                }
                if self.armed {
                    track_push(&mut self.track, &mut self.thinned.0, 0.3, (x, y), |a, b| (a.0 - b.0).hypot(a.1 - b.1));
                }
            }
            Msg::GlobalPos { lat, lon, alt_mm, rel_mm } => {
                self.amsl = alt_mm as f32 / 1000.0;
                self.alt = rel_mm as f32 / 1000.0;
                // (0, 0: ArduPilot with no position yet)
                if lat != 0 || lon != 0 {
                    self.lat = lat;
                    self.lon = lon;
                    self.pos_at = Some(Instant::now());
                    self.last_ll = Some((lat, lon));
                    self.last_xy = self.local().then_some((self.x, self.y));
                    if self.armed {
                        track_push(&mut self.gtrack, &mut self.thinned.1, 1.0, (lat, lon), ll_dist);
                    }
                }
            }
            Msg::GpsRaw { fix, sats, lat, lon } => {
                // (the autopilot's own position, while it tells one, is the one drawn and tracked)
                let own = self.global().is_some();
                self.gps = Some((Instant::now(), fix, sats));
                self.gps_ll = (fix >= 2 && (lat != 0 || lon != 0)).then_some((lat, lon));
                if let (Some(ll), false) = (self.gps_ll, own) {
                    self.last_ll = Some(ll);
                    self.last_xy = self.local().then_some((self.x, self.y));
                    if self.armed {
                        track_push(&mut self.gtrack, &mut self.thinned.1, 1.0, ll, ll_dist);
                    }
                }
            }
            Msg::Home { lat, lon, alt_mm } => {
                self.home_amsl = Some(alt_mm as f32 / 1000.0);
                if lat != 0 || lon != 0 {
                    self.home = Some((lat, lon));
                }
            }
            Msg::ExtSys { landed } => self.landed = landed,
            Msg::VfrHud { throttle, .. } => self.throttle_pct = throttle,
            Msg::Text { severity, ref text } => {
                self.texts.push_back((Instant::now(), severity, text.clone()));
                if self.texts.len() > 6 {
                    self.texts.pop_front();
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    // frames from pymavlink (common dialect), same fields and sequence numbers
    #[test]
    fn encodes_as_pymavlink() {
        let mut e = Encoder { seq: 7 };
        assert_eq!(e.heartbeat(), hex("fd09000007ffbe0000000000000006080000031f99"));
        assert_eq!(e.manual_control(1, -250, 400, 700, -1000), hex("fd0b000008ffbe45000006ff9001bc0218fc000001becd"));
        assert_eq!(
            e.command_long(1, 1, 400, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            hex("fd20000009ffbe4c00000000803f000000000000000000000000000000000000000000000000900101012243")
        );
    }

    #[test]
    fn decodes_pymavlink_frames() {
        let mut buf = hex("fd1c00000301011e0000d2040000cdcccc3dcdcc4cbe0000c03f0ad7233c0ad7a33c8fc2f53c6744");
        buf.extend(hex("fd09000004010100000000000300020c91040370ed"));
        let m = parse(&buf);
        assert_eq!(m.len(), 2);
        assert_eq!(m[0].0, 1);
        match m[0].2 {
            Msg::Attitude { roll, pitch, yaw } => {
                assert!((roll - 0.1).abs() < 1e-6 && (pitch + 0.2).abs() < 1e-6 && (yaw - 1.5).abs() < 1e-6)
            }
            ref o => panic!("{o:?}"),
        }
        let mut v = Vehicle::default();
        v.update(1, 1, &m[1].2);
        assert!(v.armed && v.main_mode == 3 && v.mode_name() == "POSITION");
    }

    // the position messages, from pymavlink as well: GLOBAL_POSITION_INT, GPS_RAW_INT (as PX4
    // sends it and with the extensions ArduPilot fills), HOME_POSITION
    #[test]
    fn decodes_the_position_messages() {
        let one = |s: &str| {
            let m = parse(&hex(s));
            assert_eq!(m.len(), 1, "{s}");
            m[0].2.clone()
        };
        assert_eq!(
            one("fd1c000005010121000040e201004c52401c44f417053a73070034300000960024ffe2ff87694fe0"),
            Msg::GlobalPos { lat: 473977420, lon: 85455940, alt_mm: 488250, rel_mm: 12340 }
        );
        assert_eq!(
            one("fd1c0000090101210000e80300009efeebea343117a740e9080000000000000000000000ffff9277"),
            Msg::GlobalPos { lat: -353632610, lon: -1491652300, alt_mm: 584000, rel_mm: 0 }
        );
        assert_eq!(one("fd1e000006010118000000f15365000000004a52401c40f41705087307005500820000022823030efe65"), Msg::GpsRaw { fix: 3, sats: 14, lat: 473977418, lon: 85455936 });
        assert_eq!(
            one("fd2b000007010118000000f15365000000009efeebeacccee85840e9080046006e000000ffff0413c02709002c01000090010000325095"),
            Msg::GpsRaw { fix: 4, sats: 19, lat: -353632610, lon: 1491652300 }
        );
        assert_eq!(
            one("fd1c0000080101f200004c52401c44f41705407207000000003f0000a0bfcdcccc3d0000803f303b"),
            Msg::Home { lat: 473977420, lon: 85455940, alt_mm: 488000 }
        );
    }

    fn heartbeat(armed: bool) -> Msg {
        Msg::Heartbeat { custom_mode: 3 << 16, autopilot: 12, base_mode: if armed { 0x81 } else { 1 } }
    }

    #[test]
    fn the_position_on_the_globe_and_home() {
        let mut v = Vehicle::default();
        v.update(1, 1, &heartbeat(false));
        // ArduPilot before its fix: 0, 0 is no position, and no home
        v.update(1, 1, &Msg::GlobalPos { lat: 0, lon: 0, alt_mm: 5000, rel_mm: 0 });
        v.update(1, 1, &Msg::Home { lat: 0, lon: 0, alt_mm: 5000 });
        assert!(v.global().is_none() && v.home_global().is_none() && v.gps_fix().is_none() && v.place().is_none());
        assert_eq!(v.home_amsl, Some(5.0));
        // the receiver has a fix, the autopilot no position of its own yet: drawn where the
        // receiver is; then by the autopilot's own
        v.update(1, 1, &Msg::GpsRaw { fix: 1, sats: 3, lat: 473970000, lon: 85450000 });
        assert!(v.place().is_none());
        v.update(1, 1, &Msg::GpsRaw { fix: 3, sats: 12, lat: 473977000, lon: 85455000 });
        let ((lat, lon), from) = v.place().unwrap();
        assert!(from == Place::Receiver && (lat - 47.3977).abs() < 1e-9 && (lon - 8.5455).abs() < 1e-9 && v.global().is_none());
        v.update(1, 1, &Msg::GlobalPos { lat: 473977420, lon: 85455940, alt_mm: 488000, rel_mm: 0 });
        let (lat, lon) = v.global().unwrap();
        assert!((lat - 47.397742).abs() < 1e-9 && (lon - 8.545594).abs() < 1e-9);
        assert_eq!(v.gps_fix(), Some((3, 12)));
        assert_eq!(v.place(), Some(((lat, lon), Place::Autopilot)));

        // the fix gone (indoors): the last place the receiver told, and from there the way the
        // local frame goes - whose position PX4 sends with or without a fix
        let pos = |x: f32, y: f32| Msg::LocalPos { x, y, z: 0.0, vx: 0.0, vy: 0.0, vz: 0.0 };
        let mut v = Vehicle::default();
        v.update(1, 1, &Msg::GpsRaw { fix: 3, sats: 9, lat: 473977000, lon: 85455000 });
        v.update(1, 1, &pos(5.0, -2.0));
        v.gps = None;
        let ((lat, lon), from) = v.place().unwrap();
        assert!(from == Place::Last && (lat - 47.3977).abs() < 1e-9 && (lon - 8.5455).abs() < 1e-9);
        v.update(1, 1, &pos(105.0, -2.0));
        let ((lat, lon), from) = v.place().unwrap();
        assert!(from == Place::Last && ((lat - 47.3977) * 111_320.0 - 100.0).abs() < 0.01 && (lon - 8.5455).abs() < 1e-9);
        // a place kept from an earlier run: the local frame carries it on from its first word
        let mut v = Vehicle::default();
        v.last_ll = Some((473977000, 85455000));
        assert_eq!(v.place().map(|p| p.1), Some(Place::Last));
        v.update(1, 1, &pos(40.0, 7.0));
        v.update(1, 1, &pos(40.0, 57.0));
        let (_, lon) = v.place().unwrap().0;
        assert!(((lon - 8.5455) * 111_320.0 * 47.3977f64.to_radians().cos() - 50.0).abs() < 0.01);
        // no home told: where it armed
        v.update(1, 1, &heartbeat(true));
        assert_eq!(v.home_global(), v.global());
        v.update(1, 1, &Msg::Home { lat: 473977000, lon: 85455000, alt_mm: 488000 });
        let (lat, lon) = v.home_global().unwrap();
        assert!((lat - 47.3977).abs() < 1e-9 && (lon - 8.5455).abs() < 1e-9);
    }

    #[test]
    fn the_track_is_the_flight() {
        let pos = |x: f32, y: f32| Msg::LocalPos { x, y, z: 0.0, vx: 0.0, vy: 0.0, vz: 0.0 };
        let mut v = Vehicle::default();
        v.update(1, 1, &heartbeat(false));
        // carried about before it arms: no track
        v.update(1, 1, &pos(3.0, 4.0));
        assert!(v.track.is_empty() && v.home_local() == (0.0, 0.0));
        v.update(1, 1, &heartbeat(true));
        assert_eq!(v.home_local(), (3.0, 4.0));
        for i in 0..50 {
            v.update(1, 1, &pos(3.0 + i as f32 * 0.2, 4.0));
            v.update(1, 1, &Msg::GlobalPos { lat: 473977420 + i * 30, lon: 85455940, alt_mm: 0, rel_mm: 0 });
        }
        // (a point every 0.3 m local, every metre on the globe: 30e-7 degrees are a third of one)
        assert!(v.track.len() > 20 && v.track.len() < 40, "{}", v.track.len());
        assert!(v.gtrack.len() > 12 && v.gtrack.len() < 20, "{}", v.gtrack.len());
        // the next flight starts its own
        v.update(1, 1, &heartbeat(false));
        assert!(!v.track.is_empty());
        v.update(1, 1, &heartbeat(true));
        assert!(v.track.is_empty() && v.gtrack.is_empty());
    }

    #[test]
    fn a_long_track_thins_itself() {
        let mut t = VecDeque::new();
        let mut thinned = 0u8;
        for i in 0..20_000 {
            track_push(&mut t, &mut thinned, 1.0, i as f32 * 1.5, |a: f32, b: f32| (a - b).abs());
        }
        assert!(t.len() <= TRACK_MAX && t.len() >= TRACK_MAX / 2 && thinned > 0);
        assert_eq!(t[0], 0.0);
        assert!(*t.back().unwrap() > 29_000.0);
        assert!((ll_dist((473977420, 85455940), (473977420 + 900, 85455940)) - 10.02).abs() < 0.05);
        assert!((ll_dist((473977420, 85455940), (473977420, 85455940 + 1000)) - 7.535).abs() < 0.05);
    }

    #[test]
    fn a_bad_crc_is_skipped() {
        let mut buf = hex("fd09000004010100000000000300020c91040370ed");
        buf[12] ^= 1;
        assert!(parse(&buf).is_empty());
    }
}
