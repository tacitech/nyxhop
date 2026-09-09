//! v40.33 licence file format - shared by the vendor tool (crates/nyx-license), the
//! daemon (loads it and lets the PL gate verify it) and the apps (show, paste, push).
//!
//! Text, one `key=value` per line, first line `nyxhop-license 1`:
//!   dna=<15 hex>   the board's device DNA (57 bits)
//!   aux=<16 hex>   {kind 1, ver_max, tier, flags, meta_hash32} - what the PL mixes in
//!   word=<16 hex>  U = XTEA_K(DNA ^ AUX) - what the PL compares against
//!   name= email= issued= note=  display only (bound to `aux` through meta_hash32)
//! `aux` and `word` are authoritative: tier / ver_max / flags are read out of `aux`,
//! never out of the text, so an edited file changes nothing the gate sees.

pub const KIND_LIC: u8 = 1;
pub const KIND_REC: u8 = 3;
pub const DNA_MASK: u64 = (1u64 << 57) - 1;
pub const FLAG_ENABLE: u8 = 1;
pub const TIER_HOBBY: u8 = 0;
pub const TIER_PRO: u8 = 1;
pub const TIER_OEM: u8 = 2;
/// v40.38: the gate generation baked into the bitstreams that are out now (`VER` in
/// fpga/rtl/nyx_lic.v). Bump it together with VER - only for a PAID feature generation;
/// bug-fix releases keep VER so every licence of the generation loads them.
pub const GATE_VER: u8 = 1;
/// v40.38: a paid licence covers this many generations beyond the current one.
pub const PAID_VER_SPAN: u8 = 4;
/// v40.38: what `ver_max` a licence of this tier gets by default: free = this
/// generation only (its bug fixes for ever, never the next paid one), pro = the next
/// PAID_VER_SPAN generations, oem = everything.
pub fn default_ver_max(tier: u8) -> u8 {
    match tier {
        TIER_PRO => GATE_VER.saturating_add(PAID_VER_SPAN),
        TIER_OEM => 255,
        _ => GATE_VER,
    }
}
const HEADER: &str = "nyxhop-license 1";

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct License {
    pub dna: u64,
    pub aux: u64,
    pub word: u64,
    pub name: String,
    pub email: String,
    pub issued: String,
    pub note: String,
}

pub fn tier_name(t: u8) -> &'static str {
    match t {
        TIER_HOBBY => "hobby",
        TIER_PRO => "pro",
        TIER_OEM => "oem",
        _ => "?",
    }
}

pub fn tier_from_name(s: &str) -> Option<u8> {
    match s.trim().to_ascii_lowercase().as_str() {
        "hobby" | "0" => Some(TIER_HOBBY),
        "pro" | "1" => Some(TIER_PRO),
        "oem" | "2" => Some(TIER_OEM),
        _ => None,
    }
}

pub fn dna_hex(dna: u64) -> String {
    format!("{:015x}", dna & DNA_MASK)
}

pub fn parse_hex_u64(s: &str) -> Option<u64> {
    let t = s.trim().trim_start_matches("0x");
    if t.is_empty() || t.len() > 16 {
        return None;
    }
    u64::from_str_radix(t, 16).ok()
}

pub fn parse_dna(s: &str) -> Option<u64> {
    parse_hex_u64(s).filter(|d| *d != 0 && *d <= DNA_MASK)
}

/// FNV-1a 32 over the display fields - binds them into `aux` (cosmetic, not a MAC).
pub fn meta_hash(name: &str, email: &str, issued: &str, note: &str) -> u32 {
    let mut h: u32 = 0x811C_9DC5;
    for part in [name, email, issued, note] {
        for b in part.trim().bytes().chain(std::iter::once(b'\n')) {
            h ^= u32::from(b);
            h = h.wrapping_mul(0x0100_0193);
        }
    }
    h
}

pub fn make_aux(ver_max: u8, tier: u8, flags: u8, meta: u32) -> u64 {
    (u64::from(KIND_LIC) << 56) | (u64::from(ver_max) << 48) | (u64::from(tier) << 40)
        | (u64::from(flags) << 32) | u64::from(meta)
}

/// AUX of the hours record the PL signs (kind 3, minutes in the low word).
pub fn rec_aux(minutes: u32) -> u64 {
    (u64::from(KIND_REC) << 56) | u64::from(minutes)
}

impl License {
    pub fn kind(&self) -> u8 {
        (self.aux >> 56) as u8
    }
    pub fn ver_max(&self) -> u8 {
        (self.aux >> 48) as u8
    }
    pub fn tier(&self) -> u8 {
        (self.aux >> 40) as u8
    }
    pub fn flags(&self) -> u8 {
        (self.aux >> 32) as u8
    }
    pub fn meta(&self) -> u32 {
        self.aux as u32
    }
    /// Sanity for the parts the gate does not check itself.
    pub fn well_formed(&self) -> Result<(), String> {
        if self.dna == 0 || self.dna > DNA_MASK {
            return Err("bad dna".into());
        }
        if self.kind() != KIND_LIC {
            return Err(format!("aux kind {} is not a licence", self.kind()));
        }
        if self.word == 0 {
            return Err("word missing".into());
        }
        Ok(())
    }
    pub fn summary(&self) -> String {
        format!(
            "{} v<={} for {}{}",
            tier_name(self.tier()),
            self.ver_max(),
            dna_hex(self.dna),
            if self.name.is_empty() { String::new() } else { format!(" ({})", self.name) }
        )
    }
    pub fn to_text(&self) -> String {
        let clean = |s: &str| s.replace(['\r', '\n'], " ").trim().to_string();
        format!(
            "{HEADER}\ndna={}\naux={:016x}\nword={:016x}\nname={}\nemail={}\nissued={}\nnote={}\n",
            dna_hex(self.dna),
            self.aux,
            self.word,
            clean(&self.name),
            clean(&self.email),
            clean(&self.issued),
            clean(&self.note)
        )
    }
    pub fn parse(text: &str) -> Result<License, String> {
        let mut lines = text.lines().map(str::trim).filter(|l| !l.is_empty());
        match lines.next() {
            // Files issued before the rename carry the old header; keep reading them.
            Some(h) if h.starts_with("nyxhop-license") || h.starts_with("nyxlink-license") => {}
            _ => return Err("not a NyxHop licence (header missing)".into()),
        }
        let mut l = License::default();
        for line in lines {
            let Some((k, v)) = line.split_once('=') else { continue };
            let v = v.trim();
            match k.trim() {
                "dna" => l.dna = parse_dna(v).ok_or("bad dna")?,
                "aux" => l.aux = parse_hex_u64(v).ok_or("bad aux")?,
                "word" => l.word = parse_hex_u64(v).ok_or("bad word")?,
                "name" => l.name = v.to_string(),
                "email" => l.email = v.to_string(),
                "issued" => l.issued = v.to_string(),
                "note" => l.note = v.to_string(),
                _ => {}
            }
        }
        l.well_formed()?;
        Ok(l)
    }
    /// One-line transport for the console (`license put <hex>`): hex of the text.
    pub fn to_hex(&self) -> String {
        self.to_text().bytes().map(|b| format!("{b:02x}")).collect()
    }
    pub fn from_hex(h: &str) -> Result<License, String> {
        let h = h.trim();
        if h.len() % 2 != 0 {
            return Err("odd hex length".into());
        }
        let mut bytes = Vec::with_capacity(h.len() / 2);
        for i in (0..h.len()).step_by(2) {
            bytes.push(u8::from_str_radix(&h[i..i + 2], 16).map_err(|_| "bad hex")?);
        }
        let text = String::from_utf8(bytes).map_err(|_| "not utf-8")?;
        License::parse(&text)
    }
    /// The 24 bytes the aircraft needs when the licence travels over the air.
    pub fn core(&self) -> (u64, u64, u64) {
        (self.dna, self.aux, self.word)
    }
    pub fn from_core(dna: u64, aux: u64, word: u64, note: &str) -> License {
        License { dna, aux, word, note: note.to_string(), ..Default::default() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_round_trip() {
        let l = License {
            dna: 0x0123456789abcde,
            aux: make_aux(1, TIER_PRO, FLAG_ENABLE, meta_hash("A B", "a@b", "2026-09-07", "")),
            word: 0xdead_beef_0123_4567,
            name: "A B".into(),
            email: "a@b".into(),
            issued: "2026-09-07".into(),
            note: String::new(),
        };
        let t = l.to_text();
        let p = License::parse(&t).unwrap();
        assert_eq!(p, l);
        assert_eq!(p.tier(), TIER_PRO);
        assert_eq!(p.ver_max(), 1);
        assert_eq!(p.flags(), FLAG_ENABLE);
        assert_eq!(License::from_hex(&l.to_hex()).unwrap(), l);
        assert!(License::parse("hello\ndna=1").is_err());
        assert!(License::parse("nyxhop-license 1\ndna=0\naux=0100000000000000\nword=1").is_err());
        // a licence issued before the rename still parses
        assert_eq!(License::parse(&t.replacen("nyxhop", "nyxlink", 1)).unwrap(), l);
    }

    #[test]
    fn meta_hash_is_stable() {
        assert_eq!(meta_hash("", "", "", ""), 0x6a00_f6a5);
        assert_ne!(meta_hash("a", "", "", ""), meta_hash("", "a", "", ""));
    }
}

#[cfg(test)]
mod model_tests {
    use super::*;

    #[test]
    fn default_ver_max_per_tier() {
        assert_eq!(default_ver_max(TIER_HOBBY), GATE_VER, "free: this generation only");
        assert_eq!(default_ver_max(TIER_PRO), GATE_VER + PAID_VER_SPAN, "paid: four more generations");
        assert_eq!(default_ver_max(TIER_OEM), 255);
        // the gate opens while VER <= ver_max: a free licence of generation 1 does not open
        // generation 2, a paid one does
        assert!(GATE_VER <= default_ver_max(TIER_HOBBY));
        assert!(GATE_VER + 1 > default_ver_max(TIER_HOBBY));
        assert!(GATE_VER + 1 <= default_ver_max(TIER_PRO));
    }
}
