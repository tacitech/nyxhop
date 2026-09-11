//! Line-based TCP control console: every app exposes its runtime
//! parameters over a socket so tests can flip options without restarting
//! (telnet-friendly; one command per line, one response block per command).
//!
//! Conventions: `get` dumps config, `stats` dumps live metrics, `set <key>
//! <value>` mutates, `quit` closes the connection. Every response ends
//! with a line `ok` or `err <reason>`.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;

use crate::logging::log;

pub type Handler = Arc<dyn Fn(&str) -> String + Send + Sync>;

pub fn spawn(addr: String, handler: Handler) {
    // `--ctl 0`: no console at all (the phone hosts the screen in-process and may build it
    // more than once; a listener that stays bound would refuse the second time).
    if addr == "0" || addr.is_empty() {
        return;
    }
    std::thread::Builder::new()
        .name("ctl".into())
        .spawn(move || {
            let listener = match TcpListener::bind(&addr) {
                Ok(l) => l,
                Err(e) => {
                    log(&format!("control: cannot bind {addr}: {e}"));
                    return;
                }
            };
            log(&format!(
                "control console on {addr} (line commands: get / stats / set <key> <value>)"
            ));
            for conn in listener.incoming() {
                if let Ok(stream) = conn {
                    let h = handler.clone();
                    std::thread::spawn(move || serve_conn(stream, h));
                }
            }
        })
        .expect("spawn ctl");
}

fn serve_conn(stream: TcpStream, h: Handler) {
    let Ok(read_half) = stream.try_clone() else { return };
    let mut reader = BufReader::new(read_half);
    let mut w = stream;
    let mut line = String::new();
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) | Err(_) => return,
            Ok(_) => {}
        }
        let cmd = line.trim();
        if cmd.is_empty() {
            continue;
        }
        if cmd == "quit" {
            return;
        }
        let mut resp = h(cmd);
        if !resp.ends_with('\n') {
            resp.push('\n');
        }
        if w.write_all(resp.as_bytes()).is_err() {
            return;
        }
    }
}

/// Parse an on/off style boolean.
pub fn parse_bool(v: &str) -> Option<bool> {
    match v.to_ascii_lowercase().as_str() {
        "1" | "on" | "true" | "yes" => Some(true),
        "0" | "off" | "false" | "no" => Some(false),
        _ => None,
    }
}
