use std::{
    io::{self, Write},
    sync::Mutex,
};

const UART_RX_FIFO_SIZE: usize = 64;

const UART_IER_RDI: u8 = 0x01;
const UART_IER_THRI: u8 = 0x02;

const UART_IIR_NO_INT: u8 = 0x01;
const UART_IIR_RDI: u8 = 0x04;
const UART_IIR_THRI: u8 = 0x02;

const UART_LSR_DR: u8 = 0x01;
const UART_LSR_THRE: u8 = 0x20;
const UART_LSR_TEMT: u8 = 0x40;

#[derive(Debug)]
struct Uart16550Inner {
    rbr: u8,
    thr: u8,
    ier: u8,
    iir: u8,
    lcr: u8,
    mcr: u8,
    lsr: u8,
    msr: u8,
    scr: u8,
    dll: u8,
    dlm: u8,
    dlab: bool,
    rx_buf: [u8; UART_RX_FIFO_SIZE],
    rx_head: usize,
    rx_tail: usize,
    irq_pending: bool,
}

impl Uart16550Inner {
    fn new() -> Self {
        Self {
            rbr: 0,
            thr: 0,
            ier: 0,
            iir: UART_IIR_NO_INT,
            lcr: 0,
            mcr: 0,
            lsr: UART_LSR_THRE | UART_LSR_TEMT,
            msr: 0,
            scr: 0,
            dll: 0,
            dlm: 0,
            dlab: false,
            rx_buf: [0; UART_RX_FIFO_SIZE],
            rx_head: 0,
            rx_tail: 0,
            irq_pending: false,
        }
    }

    fn rx_fifo_empty(&self) -> bool {
        self.rx_head == self.rx_tail
    }

    fn rx_fifo_full(&self) -> bool {
        (self.rx_head + 1) % UART_RX_FIFO_SIZE == self.rx_tail
    }

    fn rx_fifo_push(&mut self, ch: u8) {
        if self.rx_fifo_full() {
            self.rx_tail = (self.rx_tail + 1) % UART_RX_FIFO_SIZE;
        }
        self.rx_buf[self.rx_head] = ch;
        self.rx_head = (self.rx_head + 1) % UART_RX_FIFO_SIZE;
    }

    fn rx_fifo_pop(&mut self) -> u8 {
        if self.rx_fifo_empty() {
            return 0;
        }

        let ch = self.rx_buf[self.rx_tail];
        self.rx_tail = (self.rx_tail + 1) % UART_RX_FIFO_SIZE;
        self.rbr = ch;
        ch
    }

    fn update_irq(&mut self) {
        self.irq_pending = false;
        self.iir = UART_IIR_NO_INT;

        if (self.lsr & UART_LSR_DR) != 0 && (self.ier & UART_IER_RDI) != 0 {
            self.irq_pending = true;
            self.iir = UART_IIR_RDI;
            return;
        }

        if (self.lsr & UART_LSR_THRE) != 0 && (self.ier & UART_IER_THRI) != 0 {
            self.irq_pending = true;
            self.iir = UART_IIR_THRI;
        }
    }

    fn read(&mut self, offset: u8) -> u8 {
        match offset {
            0 => {
                if self.dlab {
                    self.dll
                } else {
                    let value = self.rx_fifo_pop();
                    if self.rx_fifo_empty() {
                        self.lsr &= !UART_LSR_DR;
                    }
                    self.update_irq();
                    value
                }
            }
            1 => {
                if self.dlab {
                    self.dlm
                } else {
                    self.ier
                }
            }
            2 => {
                let value = self.iir;
                self.irq_pending = false;
                value
            }
            3 => self.lcr,
            4 => self.mcr,
            5 => self.lsr,
            6 => self.msr,
            7 => self.scr,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u8, value: u8) -> io::Result<()> {
        match offset {
            0 => {
                if self.dlab {
                    self.dll = value;
                } else {
                    self.thr = value;
                    serial_output(value)?;
                    self.lsr |= UART_LSR_THRE | UART_LSR_TEMT;
                    self.update_irq();
                }
            }
            1 => {
                if self.dlab {
                    self.dlm = value;
                } else {
                    self.ier = value;
                    self.update_irq();
                }
            }
            3 => {
                self.lcr = value;
                self.dlab = ((value >> 7) & 1) != 0;
            }
            4 => self.mcr = value,
            7 => self.scr = value,
            _ => {}
        }

        Ok(())
    }
}

#[derive(Debug)]
pub struct Uart16550 {
    inner: Mutex<Uart16550Inner>,
}

impl Uart16550 {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(Uart16550Inner::new()),
        }
    }

    pub fn receive_byte(&self, ch: u8) {
        let mut inner = self.inner.lock().expect("uart lock poisoned");
        inner.rx_fifo_push(ch);
        inner.lsr |= UART_LSR_DR;
        inner.update_irq();
    }

    pub fn read(&self, offset: u8) -> u8 {
        self.inner.lock().expect("uart lock poisoned").read(offset)
    }

    pub fn write(&self, offset: u8, value: u8) -> io::Result<()> {
        self.inner
            .lock()
            .expect("uart lock poisoned")
            .write(offset, value)
    }
}

impl Default for Uart16550 {
    fn default() -> Self {
        Self::new()
    }
}

fn serial_output(value: u8) -> io::Result<()> {
    let mut stdout = io::stdout().lock();
    stdout.write_all(&[value])?;
    stdout.flush()
}

#[cfg(test)]
mod tests {
    use super::Uart16550;

    #[test]
    fn receive_then_read_round_trips() {
        let uart = Uart16550::new();
        uart.receive_byte(b'A');
        assert_eq!(uart.read(0), b'A');
    }
}
