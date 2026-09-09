//! TX network side: maintain one outgoing connection to the channel node,
//! send messages, and forward incoming Feedback/Nack to the worker.

use std::net::{SocketAddr, TcpStream, UdpSocket};
use std::sync::atomic::Ordering;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use nyx_common::logging::log;
use nyx_proto::{ChunkHdr, Msg, UDP_CAPTURE_PORT, pack_chunk, read_msg, write_msg};

use crate::Shared;

pub struct Net {
    writer: Mutex<Option<TcpStream>>,
    /// UDP socket used when UDP mode is on (GUI toggle): connectionless.
    udp: Mutex<Option<UdpSocket>>,
    pub nacks: Mutex<Receiver<u64>>,
}

/// The daemon's UDP destination = the host in channel_addr + the standard UDP port.
fn udp_target(shared: &Shared) -> Option<SocketAddr> {
    let addr = shared.channel_addr.lock().unwrap().clone();
    let host = addr.split(':').next()?;
    format!("{host}:{UDP_CAPTURE_PORT}").parse().ok()
}

impl Net {
    /// Send a message over the current connection; drops it (and marks the
    /// link down) on error.
    pub fn send(&self, shared: &Shared, msg: &Msg) -> bool {
        // UDP mode: an IqFrame goes as a chunk sequence, stateless. No TCP back-pressure any more;
        // the worker's pacing + rate control are the main brake (they have been for a long time); a
        // frame lost on the LAN is patched by seq-gap NACK/ARQ like a loss on air.
        if shared.udp_mode.load(Ordering::Relaxed) {
            let Msg::IqFrame { seq, mcs, rv, samples } = msg else {
                return true; // other kinds do not travel this way
            };
            let Some(target) = udp_target(shared) else { return false };
            let mut guard = self.udp.lock().unwrap();
            if guard.is_none() {
                match UdpSocket::bind("0.0.0.0:0") {
                    Ok(s) => {
                        log(&format!("UDP TX mode -> {target}"));
                        *guard = Some(s);
                    }
                    Err(e) => {
                        log(&format!("udp bind failed: {e}"));
                        return false;
                    }
                }
            }
            let sock = guard.as_ref().unwrap();
            let nchunks = samples.len().div_ceil(nyx_proto::CHUNK_SAMPLES);
            let mut pkt = Vec::with_capacity(1500);
            for (ci, sl) in samples.chunks(nyx_proto::CHUNK_SAMPLES).enumerate() {
                pack_chunk(
                    &mut pkt,
                    &ChunkHdr {
                        cap_seq: *seq,
                        mcs: *mcs,
                        rv: *rv,
                        chunk: ci as u16,
                        nchunks: nchunks as u16,
                    },
                    sl,
                );
                if sock.send_to(&pkt, target).is_err() {
                    return false;
                }
            }
            shared.connected.store(true, Ordering::Relaxed);
            return true;
        }
        let mut guard = self.writer.lock().unwrap();
        if let Some(stream) = guard.as_mut() {
            match write_msg(stream, msg) {
                Ok(()) => return true,
                Err(e) => {
                    log(&format!("send failed, dropping connection: {e}"));
                    *guard = None;
                    shared.connected.store(false, Ordering::Relaxed);
                }
            }
        }
        false
    }
}

pub fn spawn(shared: Arc<Shared>) -> Arc<Net> {
    let (nack_tx, nack_rx) = channel::<u64>();
    let net = Arc::new(Net {
        writer: Mutex::new(None),
        udp: Mutex::new(None),
        nacks: Mutex::new(nack_rx),
    });
    let net2 = net.clone();
    let nack_tx2 = nack_tx.clone();
    let shared2 = shared.clone();
    std::thread::Builder::new()
        .name("tx-net".into())
        .spawn(move || connect_loop(shared2, net2, nack_tx2))
        .expect("spawn tx-net");
    // Two-board mode: the RX runs against the OTHER board, whose daemon
    // relays Feedback/Nack to whoever is connected on ITS tx port — which
    // is nobody. `--feedback <rx-board:7010>` opens a read-only connection
    // there purely to collect that relay; the daemon needs no changes.
    let fb_addr = nyx_proto::cli_arg("--feedback", "");
    if !fb_addr.is_empty() {
        std::thread::Builder::new()
            .name("tx-feedback".into())
            .spawn(move || feedback_loop(shared, fb_addr, nack_tx))
            .expect("spawn tx-feedback");
    }
    net
}

/// Keep a read-only connection to the receive-side board's tx port and
/// absorb the Feedback/Nack it relays (two-board mode).
fn feedback_loop(shared: Arc<Shared>, addr: String, nack_tx: Sender<u64>) {
    let mut logged_fail = false;
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        match TcpStream::connect(&addr) {
            Ok(stream) => {
                let _ = stream.set_nodelay(true);
                // v21.1: watchdog half-open (xem connect_loop)
                let _ = stream.set_read_timeout(Some(Duration::from_secs(15)));
                log(&format!("feedback link connected to {addr}"));
                logged_fail = false;
                read_side(&shared, stream, &nack_tx);
                log("feedback link lost");
            }
            Err(e) => {
                if !logged_fail {
                    log(&format!(
                        "cannot reach feedback board at {addr} ({e}); retrying"
                    ));
                    logged_fail = true;
                }
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

fn connect_loop(shared: Arc<Shared>, net: Arc<Net>, nack_tx: Sender<u64>) {
    let mut logged_fail = false;
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        if shared.udp_mode.load(Ordering::Relaxed) {
            // UDP mode: no TCP to hold or connect; connectionless, so count as up.
            if net.writer.lock().unwrap().take().is_some() {
                log("tcp connection closed (udp mode)");
            }
            shared.connected.store(true, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(300));
            continue;
        }
        if net.writer.lock().unwrap().is_some() {
            std::thread::sleep(Duration::from_millis(200));
            continue;
        }
        let addr = shared.channel_addr.lock().unwrap().clone();
        match TcpStream::connect(&addr) {
            Ok(stream) => {
                let _ = stream.set_nodelay(true);
                // v21.1: write WITH A TIME LIMIT: when the daemon stops reading (stuck on the radio
                // lock, wedged) the buffer fills and an UNBOUNDED blocking write once froze the
                // worker while holding the writer lock (24/7 bench: nyx-tx went mute). Timeout ->
                // send() errors -> drop the connection -> reconnect.
                let _ = stream.set_write_timeout(Some(Duration::from_secs(2)));
                // Half-open watchdog on the read side: the feedback relay runs at ~16 Hz while the
                // system lives; 15 s of silence = a dead connection with no FIN -> reconnect (as
                // nyx-rx does).
                let _ = stream.set_read_timeout(Some(Duration::from_secs(15)));
                log(&format!("connected to channel node {addr}"));
                logged_fail = false;
                shared.connected.store(true, Ordering::Relaxed);
                let reader = stream.try_clone().expect("clone stream");
                // Stash a clone so the GUI can shutdown() to force reconnect.
                *shared.ctl_stream.lock().unwrap() =
                    Some(stream.try_clone().expect("clone stream"));
                *net.writer.lock().unwrap() = Some(stream);
                // Reader runs inline in this thread; when it exits the
                // connection is gone and we loop back to reconnecting.
                read_side(&shared, reader, &nack_tx);
                shared.connected.store(false, Ordering::Relaxed);
                *net.writer.lock().unwrap() = None;
                *shared.ctl_stream.lock().unwrap() = None;
                log("connection to channel node lost");
            }
            Err(e) => {
                if !logged_fail {
                    log(&format!(
                        "cannot reach channel node at {addr} ({e}); retrying every 1s"
                    ));
                    logged_fail = true;
                }
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

fn read_side(shared: &Shared, mut stream: TcpStream, nack_tx: &Sender<u64>) {
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        match read_msg(&mut stream) {
            Ok(Msg::Feedback { snr_db, bler, segs_ok, segs_lost, need_idr, ok_mcs, ok_base }) => {
                let mut fb = shared.feedback.lock().unwrap();
                fb.snr_db = snr_db;
                fb.bler = bler;
                fb.segs_ok = segs_ok;
                fb.segs_lost = segs_lost;
                fb.ok_mcs = ok_mcs;
                fb.ok_base = ok_base;
                // Sticky until the worker consumes it with a keyframe.
                fb.need_idr |= need_idr;
                fb.updated = Some(std::time::Instant::now());
            }
            Ok(Msg::Nack { seq }) => {
                let _ = nack_tx.send(seq);
            }
            Ok(Msg::UserText { text }) => {
                nyx_common::logging::log(&format!("MSG received: {text}"));
                let mut m = shared.msgs.lock().unwrap();
                m.push(format!("← {text}"));
                if m.len() > 200 { m.remove(0); }
            }
            Ok(Msg::Tlm { bytes }) => {
                let n = shared.tlm_rx.fetch_add(1, Ordering::Relaxed) + 1;
                if n == 1 {
                    nyx_common::logging::log(&format!(
                        "telemetry: first packet from the air ({} B) -> UDP out", bytes.len()));
                }
                if let Some((s, addr)) = shared.tlm_out.lock().unwrap().as_ref() {
                    let _ = s.send_to(&bytes, addr);
                }
            }
            Ok(Msg::LicInfo { dna, state, minutes, trial_min, ver }) => {
                let name = match state { 0 => "locked", 1 => "trial", 2 => "licensed", _ => "nogate" };
                *shared.lic_info.lock().unwrap() = format!(
                    "lic_dna={dna:015x}\nlic_state={name}\nlic_min={minutes}\nlic_trial={trial_min}\nlic_ver={ver}\n");
            }
            Ok(Msg::TxChan { hz }) => {
                if shared.chan_hz.swap(hz, Ordering::Relaxed) != hz {
                    shared.chan_changes.fetch_add(1, Ordering::Relaxed);
                }
            }
            Ok(_) => {}
            Err(_) => return,
        }
    }
}
