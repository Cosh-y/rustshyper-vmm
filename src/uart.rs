use std::{io, sync::Mutex};

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
    irq_edge: bool,
    thr_interrupt_pending: bool,
    thr_interrupt_armed: bool,
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
            irq_edge: false,
            thr_interrupt_pending: false,
            thr_interrupt_armed: false,
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
        let was_pending = self.irq_pending;
        self.irq_pending = false;
        self.iir = UART_IIR_NO_INT;

        if (self.lsr & UART_LSR_DR) != 0 && (self.ier & UART_IER_RDI) != 0 {
            self.irq_pending = true;
            self.iir = UART_IIR_RDI;
            if !was_pending {
                self.irq_edge = true;
            }
            return;
        }

        if self.thr_interrupt_pending && (self.ier & UART_IER_THRI) != 0 {
            self.irq_pending = true;
            self.iir = UART_IIR_THRI;
            if !was_pending {
                self.irq_edge = true;
            }
        }
    }

    fn latch_thr_interrupt_if_enabled(&mut self) {
        if self.thr_interrupt_armed
            && (self.lsr & UART_LSR_THRE) != 0
            && (self.ier & UART_IER_THRI) != 0
        {
            self.thr_interrupt_pending = true;
        }
        self.update_irq();
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
                if value == UART_IIR_THRI {
                    self.thr_interrupt_pending = false;
                    self.thr_interrupt_armed = false;
                }
                self.update_irq();
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
                    self.thr_interrupt_pending = false;
                    self.thr_interrupt_armed = true;
                    self.latch_thr_interrupt_if_enabled();
                }
            }
            1 => {
                if self.dlab {
                    self.dlm = value;
                } else {
                    self.ier = value;
                    self.latch_thr_interrupt_if_enabled();
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
        if (inner.ier & UART_IER_RDI) != 0 {
            inner.irq_edge = true;
        }
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

    pub fn poll_tx(&self) {
        self.inner.lock().expect("uart lock poisoned").update_irq();
    }

    pub fn interrupt_pending(&self) -> bool {
        self.inner.lock().expect("uart lock poisoned").irq_pending
    }

    pub fn take_interrupt_edge(&self) -> bool {
        let mut inner = self.inner.lock().expect("uart lock poisoned");
        let edge = inner.irq_edge;
        inner.irq_edge = false;
        edge
    }
}

impl Default for Uart16550 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(not(test))]
fn serial_output(value: u8) -> io::Result<()> {
    use std::io::Write as _;

    let mut stdout = io::stdout().lock();
    stdout.write_all(&[value])?;
    stdout.flush()
}

#[cfg(test)]
fn serial_output(_value: u8) -> io::Result<()> {
    Ok(())
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

    #[test]
    fn transmit_empty_interrupt_latches_on_write_and_ier_enable() {
        let uart = Uart16550::new();
        assert!(!uart.interrupt_pending());
        assert!(!uart.take_interrupt_edge());
        uart.write(1, 0x02).unwrap();
        assert!(!uart.interrupt_pending());
        assert!(!uart.take_interrupt_edge());
        uart.write(0, b'X').unwrap();
        assert!(uart.interrupt_pending());
        assert!(uart.take_interrupt_edge());
        assert!(!uart.take_interrupt_edge());
        assert_eq!(uart.read(2), 0x02);
        assert!(!uart.interrupt_pending());

        uart.write(1, 0x00).unwrap();
        uart.write(0, b'Y').unwrap();
        assert!(!uart.interrupt_pending());
        uart.write(1, 0x02).unwrap();
        assert!(uart.interrupt_pending());
        assert!(uart.take_interrupt_edge());
    }
}
