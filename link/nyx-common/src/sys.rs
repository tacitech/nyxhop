//! What Windows has to be told by an app that must keep up with the link.

/// Asks Windows to run this process as foreground work whatever window is in front, a notch above
/// normal priority. Windows 11 gives a process whose window is not in front EcoQoS, and on a hybrid
/// CPU (P, E and LP-E cores) that means the efficiency cores: on a Core Ultra 7 165H the DVB-S2
/// receiver's demodulators ran at 0.8x what they needed as background work against 1.9x not
/// (dvbs2/README.md). The app minimised, or behind the player, must not fall behind the air.
/// True if Windows took both; elsewhere nothing to do.
pub fn keep_up() -> bool {
    imp::keep_up()
}

#[cfg(windows)]
mod imp {
    #[repr(C)]
    struct PowerThrottling {
        version: u32,
        control: u32,
        state: u32,
    }

    unsafe extern "system" {
        fn GetCurrentProcess() -> isize;
        fn SetProcessInformation(process: isize, class: i32, info: *const PowerThrottling, size: u32) -> i32;
        fn SetPriorityClass(process: isize, class: u32) -> i32;
    }

    /// PROCESS_INFORMATION_CLASS ProcessPowerThrottling
    const PROCESS_POWER_THROTTLING: i32 = 4;
    const EXECUTION_SPEED: u32 = 1;
    const ABOVE_NORMAL_PRIORITY_CLASS: u32 = 0x8000;

    pub fn keep_up() -> bool {
        // execution speed under our control and not throttled: high QoS
        let s = PowerThrottling { version: 1, control: EXECUTION_SPEED, state: 0 };
        // SAFETY: documented calls on the pseudo handle of this process, with the documented structure
        unsafe {
            let p = GetCurrentProcess();
            let qos = SetProcessInformation(p, PROCESS_POWER_THROTTLING, &s, std::mem::size_of::<PowerThrottling>() as u32) != 0;
            let prio = SetPriorityClass(p, ABOVE_NORMAL_PRIORITY_CLASS) != 0;
            qos && prio
        }
    }
}

#[cfg(not(windows))]
mod imp {
    pub fn keep_up() -> bool {
        true
    }
}
