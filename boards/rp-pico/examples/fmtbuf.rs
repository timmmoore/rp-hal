/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 256 bytes - more than enough for the send to thingspeak.
pub struct FmtBuf {
    pub buf: [u8; 256],
    pub ptr: usize,
}

impl FmtBuf {
    pub fn new() -> Self {
        Self {
            buf: [0; 256],
            ptr: 0,
        }
    }

    pub fn reset(&mut self) {
        self.ptr = 0;
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
