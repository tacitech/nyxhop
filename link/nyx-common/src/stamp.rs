//! v40.46 latency test: a millisecond counter. The aircraft end burns it into the picture
//! before encoding, the ground end shows the same counter under the video and freezes both
//! with one tap: the difference of the two numbers is the delay. The counter is the system
//! clock's milliseconds modulo 1000 s, the same number in every process on one computer
//! (the bench runs both windows on one PC); a camera filming the counter on the screen
//! makes it glass-to-glass.

use crate::RgbFrame;

/// Milliseconds of the system clock modulo 1000 s.
pub fn counter_ms() -> u32 {
    let d = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    (d.as_millis() % 1_000_000) as u32
}

/// `sss.mmm`
pub fn fmt(ms: u32) -> String {
    format!("{:03}.{:03}", ms / 1000, ms % 1000)
}

/// 5x7 glyphs, bit 4 = the left column.
fn glyph(c: char) -> [u8; 7] {
    match c {
        '0' => [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1' => [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        '3' => [0b11111, 0b00010, 0b00100, 0b00010, 0b00001, 0b10001, 0b01110],
        '4' => [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5' => [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6' => [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8' => [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9' => [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        '.' => [0, 0, 0, 0, 0, 0b01100, 0b01100],
        _ => [0; 7],
    }
}

/// Burn `fmt(ms)` into the top-left corner, white on black, big enough (a pixel per
/// 1/100 of the width) to stay readable through the encoder at the lowest rate.
pub fn burn(f: &mut RgbFrame, ms: u32) {
    if f.rgb.len() != f.width * f.height * 3 {
        return;
    }
    let s = (f.width / 100).max(2);
    let text = fmt(ms);
    fill(f, 0, 0, text.len() * 6 * s + s, 9 * s, 0);
    for (i, c) in text.chars().enumerate() {
        for (row, bits) in glyph(c).iter().enumerate() {
            for col in 0..5 {
                if (bits >> (4 - col)) & 1 == 1 {
                    fill(f, s + (i * 6 + col) * s, s + row * s, s, s, 255);
                }
            }
        }
    }
}

fn fill(f: &mut RgbFrame, x: usize, y: usize, w: usize, h: usize, v: u8) {
    let (x1, y1) = ((x + w).min(f.width), (y + h).min(f.height));
    for yy in y.min(y1)..y1 {
        let row = yy * f.width;
        for xx in x.min(x1)..x1 {
            let i = (row + xx) * 3;
            f.rgb[i..i + 3].fill(v);
        }
    }
}

/// Read the counter `burn` wrote, from a decoded picture (the receiving end's own latency
/// meter: on one computer the difference to `counter_ms` is the delay from burn to here).
/// Samples the middle of every glyph cell and takes the nearest glyph; None when the corner
/// does not hold a counter (a picture without one, or too damaged to trust).
pub fn read(f: &RgbFrame) -> Option<u32> {
    if f.rgb.len() != f.width * f.height * 3 || f.width < 100 {
        return None;
    }
    let s = (f.width / 100).max(2);
    if 9 * s > f.height {
        return None;
    }
    let lum = |x: usize, y: usize| {
        let i = (y * f.width + x) * 3;
        (u16::from(f.rgb[i]) + u16::from(f.rgb[i + 1]) + u16::from(f.rgb[i + 2])) / 3
    };
    let mut ms: u32 = 0;
    for i in 0..7 {
        let mut cell = [0u8; 7];
        for (row, bits) in cell.iter_mut().enumerate() {
            for col in 0..5 {
                let x = s + (i * 6 + col) * s + s / 2;
                let y = s + row * s + s / 2;
                if lum(x, y) > 128 {
                    *bits |= 1 << (4 - col);
                }
            }
        }
        let want: &[char] = if i == 3 { &['.'] } else { &['0', '1', '2', '3', '4', '5', '6', '7', '8', '9'] };
        let (best, dist) = want
            .iter()
            .map(|&c| {
                let g = glyph(c);
                let d: u32 = g.iter().zip(cell.iter()).map(|(a, b)| (a ^ b).count_ones()).sum();
                (c, d)
            })
            .min_by_key(|&(_, d)| d)?;
        // a clean glyph matches exactly; allow a few cells lost to the encoder, no more
        if dist > 3 {
            return None;
        }
        if let Some(v) = best.to_digit(10) {
            ms = ms * 10 + v;
        }
    }
    Some(ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_back_what_was_burned() {
        for &(w, h) in &[(640usize, 480usize), (1280, 720), (320, 240)] {
            let mut f = RgbFrame { width: w, height: h, rgb: vec![90; w * h * 3] };
            burn(&mut f, 123_456);
            assert_eq!(read(&f), Some(123_456), "{w}x{h}");
        }
        let plain = RgbFrame { width: 640, height: 480, rgb: vec![90; 640 * 480 * 3] };
        assert_eq!(read(&plain), None);
    }
}
