//! Board console client (the nyx-radio-node daemon on :7202) embedded in the GUI: a separate TCP
//! worker; the GUI only reads key=value snapshots and fires commands. The address is taken through
//! a closure on every use (follows the GUI's address box).

use std::collections::{HashMap, VecDeque};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::logging::log;

#[derive(Default, Clone)]
pub struct BoardState {
    /// latest key=value from the poll commands (get/stats/capstat/rssi/trig).
    pub kv: HashMap<String, String>,
    /// The capstat verdict line (the line without '=').
    pub verdict: String,
    pub connected: bool,
}

pub struct BoardCtl {
    state: Arc<Mutex<BoardState>>,
    cmd_tx: Sender<String>,
}

impl BoardCtl {
    /// `addr` returns the console "ip:port" each time the worker needs to connect (changeable at
    /// runtime). `poll` = the commands refreshed every second while the panel is open.
    pub fn spawn(
        addr: Arc<dyn Fn() -> String + Send + Sync>,
        poll: &'static [&'static str],
    ) -> BoardCtl {
        let state = Arc::new(Mutex::new(BoardState::default()));
        let (cmd_tx, cmd_rx) = channel::<String>();
        let st = state.clone();
        std::thread::Builder::new()
            .name("boardctl".into())
            .spawn(move || worker(st, addr, poll, cmd_rx))
            .expect("spawn boardctl");
        BoardCtl { state, cmd_tx }
    }

    pub fn send(&self, cmd: impl Into<String>) {
        let _ = self.cmd_tx.send(cmd.into());
    }

    pub fn snapshot(&self) -> BoardState {
        self.state.lock().unwrap().clone()
    }
}

/// Send one command and read up to the closing "ok"/"err…" line.
fn transact(stream: &mut TcpStream, cmd: &str) -> std::io::Result<Vec<String>> {
    stream.write_all(format!("{cmd}\n").as_bytes())?;
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut out = Vec::new();
    loop {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "console closed",
            ));
        }
        let t = line.trim().to_string();
        let done = t == "ok" || t.starts_with("err");
        out.push(t);
        if done {
            return Ok(out);
        }
    }
}

fn worker(
    state: Arc<Mutex<BoardState>>,
    addr: Arc<dyn Fn() -> String + Send + Sync>,
    poll: &'static [&'static str],
    cmd_rx: Receiver<String>,
) {
    let mut conn: Option<(TcpStream, String)> = None;
    let mut last_poll = Instant::now() - Duration::from_secs(10);
    loop {
        let user_cmd = cmd_rx.recv_timeout(Duration::from_millis(250)).ok();
        let mut batch: Vec<String> = Vec::new();
        if let Some(c) = user_cmd {
            batch.push(c);
        }
        if last_poll.elapsed() > Duration::from_secs(1) {
            last_poll = Instant::now();
            batch.extend(poll.iter().map(|s| s.to_string()));
        }
        if batch.is_empty() {
            continue;
        }

        let want = addr();
        // Address changed (GUI connected elsewhere) -> drop the old connection.
        if conn.as_ref().is_some_and(|(_, a)| *a != want) {
            conn = None;
        }
        if conn.is_none() {
            match want.parse() {
                Ok(sa) => match TcpStream::connect_timeout(
                    &sa,
                    Duration::from_millis(1200),
                ) {
                    Ok(s) => {
                        let _ = s.set_read_timeout(Some(Duration::from_millis(1500)));
                        let _ = s.set_nodelay(true);
                        log(&format!("boardctl: connected {want}"));
                        conn = Some((s, want.clone()));
                    }
                    Err(_) => {
                        state.lock().unwrap().connected = false;
                        continue;
                    }
                },
                Err(_) => {
                    state.lock().unwrap().connected = false;
                    continue;
                }
            }
        }
        state.lock().unwrap().connected = true;

        for cmd in batch {
            let Some((s, _)) = conn.as_mut() else { break };
            match transact(s, &cmd) {
                Ok(lines) => absorb(&state, &cmd, &lines),
                Err(_) => {
                    conn = None;
                    state.lock().unwrap().connected = false;
                    break;
                }
            }
        }
    }
}

fn absorb(state: &Mutex<BoardState>, cmd: &str, lines: &[String]) {
    let mut st = state.lock().unwrap();
    for l in lines {
        if l == "ok" || l.starts_with("err") {
            continue;
        }
        if l.contains('=') {
            // capstat packs several fields on one line: split on whitespace.
            if l.matches('=').count() > 1 && l.contains(' ') {
                for part in l.split_whitespace() {
                    if let Some((k, v)) = part.split_once('=') {
                        st.kv.insert(k.to_string(), v.to_string());
                    }
                }
            } else if let Some((k, v)) = l.split_once('=') {
                st.kv.insert(k.trim().to_string(), v.trim().to_string());
            }
        } else if cmd == "capstat" {
            st.verdict = l.clone();
        }
    }
}

/// A shared VecDeque log for the console: a small helper for a GUI that wants to show replies to
/// manual commands (unused in the compact embedded version).
pub type ConsoleLog = VecDeque<String>;
