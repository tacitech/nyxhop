//! An RC transmitter plugged in as a USB joystick (EdgeTX and OpenTX radios: RadioMaster, Jumper,
//! FrSky, BetaFPV...), read as an RC pilot holds it: its channels in their order (AETR, as EdgeTX
//! makes a model, or TAER), each stick where it stands - the throttle too, which does not spring
//! back to the middle. Read through Windows' own joystick API: right from the first frame, where a
//! gamepad library hears an axis only once it has moved (the throttle resting at the bottom read
//! as the middle until then).

/// The radios' USB ids (vendor, product): EdgeTX / OpenTX (pid.codes), OpenTX on the older STM32
/// builds (FrSky Taranis and its kin)
const RADIOS: [(u16, u16); 2] = [(0x1209, 0x4F54), (0x0483, 0x5710)];

/// Where Windows puts each channel among its six axes (X Y Z R U V): EdgeTX's joystick gives
/// channels 1-3 as X, Y, Z and channel 4 as V (a RadioMaster read on 12/11, R and U still while
/// the rudder moved V end to end)
#[cfg_attr(not(windows), allow(dead_code))]
const CH_AXES: [usize; 6] = [0, 1, 2, 5, 3, 4];

/// The order of the channels the radio sends (the first four)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Order {
    /// roll, pitch, throttle, yaw: EdgeTX's and OpenTX's own
    Aetr,
    /// throttle, roll, pitch, yaw: Spektrum's and Futaba's
    Taer,
}

impl Order {
    pub const ALL: [Order; 2] = [Order::Aetr, Order::Taer];

    pub fn key(self) -> &'static str {
        match self {
            Order::Aetr => "AETR",
            Order::Taer => "TAER",
        }
    }

    pub fn from_key(k: &str) -> Option<Order> {
        Order::ALL.into_iter().find(|o| o.key().eq_ignore_ascii_case(k.trim()))
    }

    /// (roll, pitch, throttle, yaw) of the channels `ch` in -1..1 (right, forward, up +)
    pub fn sticks(self, ch: &[f32; 6]) -> (f32, f32, f32, f32) {
        match self {
            Order::Aetr => (ch[0], ch[1], ch[2], ch[3]),
            Order::Taer => (ch[1], ch[2], ch[0], ch[3]),
        }
    }
}

/// A radio found: Windows' joystick number, its axes' ranges, its name
pub struct Rc {
    pub name: String,
    #[cfg_attr(not(windows), allow(dead_code))]
    id: u32,
    #[cfg_attr(not(windows), allow(dead_code))]
    range: [(u32, u32); 6],
}

#[cfg(windows)]
mod sys {
    #[repr(C)]
    pub struct JoyCapsW {
        pub mid: u16,
        pub pid: u16,
        pub pname: [u16; 32],
        pub xmin: u32,
        pub xmax: u32,
        pub ymin: u32,
        pub ymax: u32,
        pub zmin: u32,
        pub zmax: u32,
        pub num_buttons: u32,
        pub period_min: u32,
        pub period_max: u32,
        pub rmin: u32,
        pub rmax: u32,
        pub umin: u32,
        pub umax: u32,
        pub vmin: u32,
        pub vmax: u32,
        pub caps: u32,
        pub max_axes: u32,
        pub num_axes: u32,
        pub max_buttons: u32,
        pub reg_key: [u16; 32],
        pub oem_vxd: [u16; 260],
    }

    #[repr(C)]
    #[derive(Default)]
    pub struct JoyInfoEx {
        pub size: u32,
        pub flags: u32,
        pub pos: [u32; 6],
        pub buttons: u32,
        pub button_number: u32,
        pub pov: u32,
        pub reserved1: u32,
        pub reserved2: u32,
    }

    pub const JOY_RETURNALL: u32 = 0xFF;

    #[link(name = "winmm")]
    unsafe extern "system" {
        pub fn joyGetNumDevs() -> u32;
        pub fn joyGetDevCapsW(id: usize, caps: *mut JoyCapsW, size: u32) -> u32;
        pub fn joyGetPosEx(id: u32, info: *mut JoyInfoEx) -> u32;
    }
}

impl Rc {
    /// The first radio plugged in (None: none, or not Windows)
    #[cfg(windows)]
    pub fn find() -> Option<Rc> {
        let n = unsafe { sys::joyGetNumDevs() }.min(16);
        for id in 0..n {
            // SAFETY: a plain C struct Windows fills (zeroed is a valid value for it)
            let mut c: sys::JoyCapsW = unsafe { std::mem::zeroed() };
            if unsafe { sys::joyGetDevCapsW(id as usize, &mut c, std::mem::size_of::<sys::JoyCapsW>() as u32) } != 0 {
                continue;
            }
            if !RADIOS.contains(&(c.mid, c.pid)) {
                continue;
            }
            // (Windows names every such joystick "Microsoft PC-joystick driver": the radio's own
            // firmware instead)
            let rc = Rc {
                name: if c.mid == 0x1209 { "EdgeTX".into() } else { "OpenTX".into() },
                id,
                range: [(c.xmin, c.xmax), (c.ymin, c.ymax), (c.zmin, c.zmax), (c.rmin, c.rmax), (c.umin, c.umax), (c.vmin, c.vmax)],
            };
            // plugged in now (a joystick unplugged keeps its caps): it reads
            if rc.channels().is_some() {
                return Some(rc);
            }
        }
        None
    }

    #[cfg(not(windows))]
    pub fn find() -> Option<Rc> {
        None
    }

    /// Its first six channels in -1..1; None: unplugged
    #[cfg(windows)]
    pub fn channels(&self) -> Option<[f32; 6]> {
        let mut j = sys::JoyInfoEx { size: std::mem::size_of::<sys::JoyInfoEx>() as u32, flags: sys::JOY_RETURNALL, ..Default::default() };
        if unsafe { sys::joyGetPosEx(self.id, &mut j) } != 0 {
            return None;
        }
        let mut ch = [0.0f32; 6];
        for (c, &a) in ch.iter_mut().zip(CH_AXES.iter()) {
            *c = scale(j.pos[a], self.range[a]);
        }
        Some(ch)
    }

    #[cfg(not(windows))]
    pub fn channels(&self) -> Option<[f32; 6]> {
        None
    }
}

/// A raw axis in its range to -1..1 (the middle 0)
#[cfg_attr(not(windows), allow(dead_code))]
fn scale(v: u32, (lo, hi): (u32, u32)) -> f32 {
    if hi <= lo {
        return 0.0;
    }
    let x = (v.clamp(lo, hi) - lo) as f32 / (hi - lo) as f32;
    let x = x * 2.0 - 1.0;
    // the middle a step either side of the exact half (0..65535 has none): 0
    if x.abs() < 0.002 { 0.0 } else { x }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_channels() {
        assert_eq!(scale(0, (0, 65535)), -1.0);
        assert_eq!(scale(65535, (0, 65535)), 1.0);
        assert_eq!(scale(32767, (0, 65535)), 0.0);
        assert_eq!(scale(5, (0, 0)), 0.0);
        let ch = [0.1, 0.2, 0.3, 0.4, 0.0, 0.0];
        assert_eq!(Order::Aetr.sticks(&ch), (0.1, 0.2, 0.3, 0.4));
        assert_eq!(Order::Taer.sticks(&ch), (0.2, 0.3, 0.1, 0.4));
        assert_eq!(Order::from_key(" taer"), Some(Order::Taer));
        assert_eq!(Order::from_key("x"), None);
    }
}
