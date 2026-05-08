use std::{
    env, fs, io,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering},
        mpsc::{self, Receiver},
    },
    thread,
    time::{Duration, Instant},
};

use libc::{ECHO, ICANON, ICRNL, ISIG, TCSANOW, VMIN, VTIME, tcgetattr, tcsetattr, termios};

use crate::{
    api::{GuestMemory, RustShyper, VcpuHandle, VmHandle},
    ioctl::{
        COM1_BASE, COM1_END, RunState, UserMemoryRegion, VMX_EXIT_REASON_HLT,
        VMX_EXIT_REASON_IO_INSTRUCTION, VMX_EXIT_REASON_PAUSE_INSTRUCTION,
        VMX_EXIT_REASON_PREEMPTION_TIMER, VMX_EXIT_REASON_TRIPLE_FAULT, VcpuDtable, VcpuRegs,
        VcpuSegment, VcpuSregs,
    },
    linux::{self, LinuxBootConfig},
    uart::Uart16550,
};

const SYSTEM_CONTROL_PORT_B: u16 = 0x61;
const PORT_B_TIMER2_GATE: u8 = 1 << 0;
const PORT_B_SPEAKER_DATA: u8 = 1 << 1;
const PORT_B_REFRESH_TOGGLE: u8 = 1 << 4;
const PORT_B_TIMER2_OUT: u8 = 1 << 5;
const PORT_B_WRITABLE_MASK: u8 = PORT_B_TIMER2_GATE | PORT_B_SPEAKER_DATA;
const IO_DELAY_PORT: u16 = 0x80;
const CMOS_INDEX_PORT: u16 = 0x70;
const CMOS_DATA_PORT: u16 = 0x71;
const I8042_DATA_PORT: u16 = 0x60;
const I8042_COMMAND_STATUS_PORT: u16 = 0x64;
const DMA_PAGE_REGISTER_BASE: u16 = 0x80;
const DMA_PAGE_REGISTER_PORTS: std::ops::RangeInclusive<u16> = 0x81..=0x8f;
const PIC_MASTER_COMMAND_PORT: u16 = 0x20;
const PIC_MASTER_DATA_PORT: u16 = 0x21;
const PIC_SLAVE_COMMAND_PORT: u16 = 0xa0;
const PIC_SLAVE_DATA_PORT: u16 = 0xa1;
const PIC_MASTER_ELCR_PORT: u16 = 0x4d0;
const PIC_SLAVE_ELCR_PORT: u16 = 0x4d1;
const PIT_CHANNEL0_PORT: u16 = 0x40;
const PIT_CHANNEL2_PORT: u16 = 0x42;
const PIT_COMMAND_PORT: u16 = 0x43;
const PIT_PORTS: std::ops::RangeInclusive<u16> = PIT_CHANNEL0_PORT..=PIT_COMMAND_PORT;
const PIT_INPUT_CLOCK_HZ: u64 = 1_193_182;
const PIT_IRQ_LINE: u32 = 0;
const PIT_DEFAULT_TICK_INTERVAL: Duration = Duration::from_millis(4);
const UART_IRQ_LINE: u32 = 4;
const LEGACY_SERIAL_PORT_WIDTH: u16 = 8;
const ABSENT_LEGACY_SERIAL_BASES: [u16; 3] = [0x2f8, 0x3e8, 0x2e8];
const UNMAPPED_PORT_READ_VALUE: u8 = 0xff;

#[derive(Debug, Clone)]
pub struct VmmConfig {
    pub device_path: PathBuf,
    pub guest_path: PathBuf,
    pub initrd_path: Option<PathBuf>,
    pub cmdline: Option<String>,
    pub guest_mem_size: usize,
    pub load_addr: u64,
    pub entry_point: u64,
    pub stack_pointer: u64,
}

impl VmmConfig {
    pub fn new(guest_path: impl Into<PathBuf>) -> Self {
        Self {
            device_path: PathBuf::from("/dev/rustshyper"),
            guest_path: guest_path.into(),
            initrd_path: None,
            cmdline: None,
            guest_mem_size: 0x20_0000,
            load_addr: 0x10_0000,
            entry_point: 0x10_0000,
            stack_pointer: 0x1f_f000,
        }
    }
}

pub struct Vmm {
    _hypervisor: RustShyper,
    vm: VmHandle,
    vcpu: VcpuHandle,
    guest_memory: GuestMemory,
    uart: Arc<Uart16550>,
    system_control_port_b: AtomicU8,
    cmos: CmosRtc,
    dma_page_registers: [u8; 16],
    pic_master: Pic8259,
    pic_slave: Pic8259,
    pit_counter: AtomicU16,
    pit_next_read_high: AtomicBool,
    pit_channel0_low_byte: Option<u8>,
    pit_channel2_low_byte: Option<u8>,
    pit_channel2_out_high_at: Option<Instant>,
    pit_irq_enabled: bool,
    pit_irq_interval: Duration,
    next_pit_irq: Instant,
    pit_irq_count: u64,
    trace_timers: bool,
    trace_uart: bool,
    uart_trace_count: u64,
    uart_trace_start: u64,
    uart_trace_end: Option<u64>,
    diagnostic_deadline: Option<Instant>,
    stdin_rx: Receiver<u8>,
    _stdin_thread_alive: Arc<AtomicBool>,
}

impl Vmm {
    pub fn new(config: &VmmConfig) -> io::Result<Self> {
        let hypervisor = RustShyper::open(&config.device_path)?;
        let _ = hypervisor.api_version()?;

        let vm = hypervisor.create_vm()?;
        let mut guest_memory = GuestMemory::new(config.guest_mem_size)?;
        let guest_image = fs::read(&config.guest_path)?;
        let initrd = config.initrd_path.as_ref().map(fs::read).transpose()?;

        let boot_state = if linux::looks_like_bzimage(&guest_image) {
            Some(linux::load_bzimage(
                guest_memory.as_mut_slice(),
                &guest_image,
                &LinuxBootConfig {
                    kernel_load_addr: config.load_addr,
                    guest_mem_size: config.guest_mem_size as u64,
                    initrd: initrd.as_deref(),
                    cmdline: config.cmdline.as_deref(),
                    stack_pointer: config.stack_pointer,
                },
            )?)
        } else {
            guest_memory.load(config.load_addr as usize, &guest_image)?;
            None
        };

        let region = UserMemoryRegion {
            slot: 0,
            flags: 0,
            guest_phys_addr: 0,
            memory_size: guest_memory.len() as u64,
            userspace_addr: guest_memory.userspace_addr(),
        };
        vm.set_user_memory_region(&region)?;

        let vcpu = vm.create_vcpu(0)?;
        if let Some(boot_state) = boot_state {
            vcpu.set_sregs(&boot_state.sregs)?;
            vcpu.set_regs(&boot_state.regs)?;
        } else {
            let regs = VcpuRegs {
                rip: config.entry_point,
                rsp: config.stack_pointer,
                rflags: 0x2,
                ..VcpuRegs::default()
            };
            vcpu.set_regs(&regs)?;
        }

        let uart = Arc::new(Uart16550::new());
        let (stdin_rx, alive) = spawn_stdin_thread()?;

        Ok(Self {
            _hypervisor: hypervisor,
            vm,
            vcpu,
            guest_memory,
            uart,
            system_control_port_b: AtomicU8::new(0),
            cmos: CmosRtc::new(),
            dma_page_registers: [0; 16],
            pic_master: Pic8259::new(),
            pic_slave: Pic8259::new(),
            pit_counter: AtomicU16::new(0xffff),
            pit_next_read_high: AtomicBool::new(false),
            pit_channel0_low_byte: None,
            pit_channel2_low_byte: None,
            pit_channel2_out_high_at: None,
            pit_irq_enabled: false,
            pit_irq_interval: PIT_DEFAULT_TICK_INTERVAL,
            next_pit_irq: Instant::now() + PIT_DEFAULT_TICK_INTERVAL,
            pit_irq_count: 0,
            trace_timers: env::var_os("RUSTSHYPER_TRACE_TIMERS").is_some(),
            trace_uart: env::var_os("RUSTSHYPER_TRACE_UART").is_some(),
            uart_trace_count: 0,
            uart_trace_start: env_u64("RUSTSHYPER_TRACE_UART_START")?.unwrap_or(1),
            uart_trace_end: env_u64("RUSTSHYPER_TRACE_UART_END")?,
            diagnostic_deadline: diagnostic_deadline_from_env()?,
            stdin_rx,
            _stdin_thread_alive: alive,
        })
    }

    pub fn run(&mut self) -> io::Result<()> {
        let _raw_mode = TerminalRawMode::enable()?;
        let mut run_count = 0_u64;
        let mut hlt_count = 0_u64;
        let mut pause_count = 0_u64;
        let mut preemption_count = 0_u64;

        loop {
            if self
                .diagnostic_deadline
                .is_some_and(|deadline| Instant::now() >= deadline)
            {
                eprintln!("rustshyper-vmm: diagnostic deadline reached");
                self.dump_current_diagnostics();
                return Err(io::Error::new(
                    io::ErrorKind::Interrupted,
                    "rustshyper-vmm diagnostic deadline reached",
                ));
            }

            self.pump_stdin();
            self.inject_pending_uart_irq()?;
            self.inject_due_pit_irq()?;

            run_count = run_count.wrapping_add(1);
            if self.trace_timers && run_count == 1 {
                eprintln!("rustshyper-vmm: entering vcpu.run() count={run_count}");
            }
            let run_state = self.vcpu.run()?;

            match run_state.exit_reason {
                VMX_EXIT_REASON_IO_INSTRUCTION => self.handle_io_exit(&run_state)?,
                VMX_EXIT_REASON_PREEMPTION_TIMER => {
                    preemption_count = preemption_count.wrapping_add(1);
                    if self.trace_timers
                        && (preemption_count <= 64 || preemption_count.is_multiple_of(1024))
                    {
                        self.trace_poll_exit("PREEMPTION_TIMER", preemption_count, &run_state);
                    }
                    continue;
                }
                VMX_EXIT_REASON_PAUSE_INSTRUCTION => {
                    pause_count = pause_count.wrapping_add(1);
                    if self.trace_timers && (pause_count <= 64 || pause_count.is_multiple_of(1024))
                    {
                        self.trace_poll_exit("PAUSE", pause_count, &run_state);
                    }
                    continue;
                }
                VMX_EXIT_REASON_HLT => {
                    hlt_count = hlt_count.wrapping_add(1);
                    if self.trace_timers && (hlt_count <= 64 || hlt_count.is_multiple_of(1024)) {
                        self.trace_poll_exit("HLT", hlt_count, &run_state);
                    }
                    thread::sleep(Duration::from_millis(1));
                }
                other => {
                    eprintln!(
                        "rustshyper-vmm: vcpu.run() returned exit={} rip={:#x} len={} qual={:#x}",
                        exit_reason_name(run_state.exit_reason),
                        run_state.guest_rip,
                        run_state.instruction_len,
                        run_state.exit_qualification
                    );
                    self.dump_exit_diagnostics(&run_state);
                    return Err(io::Error::other(format!(
                        "{} at rip {:#x}",
                        exit_reason_name(other),
                        run_state.guest_rip
                    )));
                }
            }
        }
    }

    fn inject_due_pit_irq(&mut self) -> io::Result<()> {
        if !self.pit_irq_enabled {
            return Ok(());
        }

        let now = Instant::now();
        if now < self.next_pit_irq {
            return Ok(());
        }

        let due_at = self.next_pit_irq;
        self.vm.inject_irq_line(PIT_IRQ_LINE)?;
        self.pit_irq_count = self.pit_irq_count.wrapping_add(1);
        if self.trace_timers
            && (self.pit_irq_count <= 16 || self.pit_irq_count.is_multiple_of(1024))
        {
            eprintln!(
                "rustshyper-vmm: PIT inject count={} late_us={} interval_us={}",
                self.pit_irq_count,
                now.saturating_duration_since(due_at).as_micros(),
                self.pit_irq_interval.as_micros()
            );
        }
        while self.next_pit_irq <= now {
            self.next_pit_irq += self.pit_irq_interval;
        }
        Ok(())
    }

    fn trace_poll_exit(&self, name: &str, count: u64, run_state: &RunState) {
        eprintln!(
            "rustshyper-vmm: {name} exit count={count} rip={:#x} len={} qual={:#x} pit_count={}",
            run_state.guest_rip,
            run_state.instruction_len,
            run_state.exit_qualification,
            self.pit_irq_count,
        );
    }

    fn pump_stdin(&self) {
        for byte in self.stdin_rx.try_iter() {
            self.uart.receive_byte(byte);
        }
    }

    fn inject_pending_uart_irq(&mut self) -> io::Result<()> {
        self.uart.poll_tx();
        if self.uart.take_interrupt_edge() {
            self.vm.inject_irq_line(UART_IRQ_LINE)?;
        }
        Ok(())
    }

    fn handle_io_exit(&mut self, run_state: &RunState) -> io::Result<()> {
        if run_state.io_is_string() || run_state.io_is_repeat() {
            return Err(io::Error::other(
                "string or REP-prefixed port I/O exits are not supported yet",
            ));
        }

        let port = run_state.io_port();
        // Some motherboards use port 0x61 as a simple way to detect whether the PIT channel 2 output is connected to the PC speaker, 
        // so we need to handle reads and writes to it in order for some guest software to work properly. 
        // The actual PC speaker functionality is not implemented, 
        // but we do toggle the refresh bit on each read to allow software that uses that bit as a simple timer to work.
        if port == SYSTEM_CONTROL_PORT_B {
            println!("rustshyper-vmm: Guest access to system control port B at {:#x}", port);
            return self.handle_system_control_port_b(run_state);
        }
        // Many older x86 systems use an access to port 0x80 as a simple I/O delay mechanism, so we trap it and just ignore the access.
        if port == IO_DELAY_PORT {
            println!("rustshyper-vmm: Guest access to I/O delay port at {:#x}", port);
            return self.handle_io_delay(run_state);
        }
        // The PC/AT CMOS(RAM) RTC uses port 0x70 as an index register and 0x71 as
        // the selected data register. Linux reads a small RTC/status subset
        // during early boot, so the VMM provides stable synthetic values here.
        if is_cmos_port(port) {
            println!("rustshyper-vmm: Guest access to CMOS I/O at port {:#x}", port);
            return self.handle_pic_io_(run_state);
        }
        // PS/2 keyboard mouse controller
        if is_i8042_port(port) {
            // read 0xff and ignore write
            println!("rustshyper-vmm: Guest access to i8042 I/O at port {:#x}", port);
            return self.handle_absent_i8042_io(run_state);
        }
        if DMA_PAGE_REGISTER_PORTS.contains(&port) {
            return self.handle_dma_page_io(run_state);
        }
        if is_pic_port(port) {
            println!("rustshyper-vmm: Guest access to PIC I/O at port {:#x}", port);
            return self.handle_pic_io_(run_state);
        }
        // The legacy 8253/8254 PIT lives at ports 0x40..0x43. Linux programs
        // channel 0 for early timer ticks, and may use channel 2 together with
        // port 0x61 for PC/AT timer and speaker-related probing.
        if PIT_PORTS.contains(&port) {
            println!("rustshyper-vmm: Guest access to PIT I/O at port {:#x}", port);
            return self.handle_pic_io_(run_state);
        }
        if is_absent_legacy_serial_port(port) {
            // read 0xff and ignore write
            return self.handle_absent_legacy_serial_io(run_state);
        }

        if !(COM1_BASE..COM1_END).contains(&port) {
            return Err(io::Error::other(format!(
                "unsupported I/O port exit at {port:#x}"
            )));
        }

        let offset = (port - COM1_BASE) as u8;
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported UART access width {size}"
            )));
        }

        if run_state.io_is_in() {
            let mut regs = self.vcpu.get_regs()?;
            let value = self.uart.read(offset);
            self.trace_uart_io("in", offset, value);
            regs.rax = (regs.rax & !0xff) | u64::from(value);
            if run_state.instruction_len != 0 {
                regs.rip = run_state
                    .guest_rip
                    .wrapping_add(run_state.instruction_len as u64);
            }
            self.vcpu.set_regs(&regs)?;
        } else {
            let mut regs = self.vcpu.get_regs()?;
            let value = regs.rax as u8;
            self.uart.write(offset, value)?;
            self.trace_uart_io("out", offset, value);
            if run_state.instruction_len != 0 {
                regs.rip = run_state
                    .guest_rip
                    .wrapping_add(run_state.instruction_len as u64);
            }
            self.vcpu.set_regs(&regs)?;
        }

        Ok(())
    }

    fn trace_uart_io(&mut self, direction: &str, offset: u8, value: u8) {
        if !self.trace_uart {
            return;
        }

        self.uart_trace_count = self.uart_trace_count.wrapping_add(1);
        let in_window = self.uart_trace_count >= self.uart_trace_start
            && self
                .uart_trace_end
                .is_none_or(|end| self.uart_trace_count <= end);
        if in_window || self.uart_trace_count <= 512 || self.uart_trace_count.is_multiple_of(1024) {
            eprintln!(
                "rustshyper-vmm: uart {}#{} off={:#x} value={:#x} irq_pending={}",
                direction,
                self.uart_trace_count,
                offset,
                value,
                self.uart.interrupt_pending()
            );
        }
    }

    fn handle_absent_i8042_io(&self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported i8042 access width {size}"
            )));
        }

        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            regs.rax = (regs.rax & !0xff) | u64::from(UNMAPPED_PORT_READ_VALUE);
        }
        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_absent_legacy_serial_io(&self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported absent UART access width {size}"
            )));
        }

        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            regs.rax = (regs.rax & !0xff) | u64::from(UNMAPPED_PORT_READ_VALUE);
        }
        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_io_delay(&self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported I/O delay access width {size}"
            )));
        }

        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            regs.rax &= !0xff;
        }
        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_cmos_io(&self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported CMOS access width {size}"
            )));
        }

        let port = run_state.io_port();
        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            let value = if port == CMOS_DATA_PORT {
                self.cmos.read_data()
            } else {
                self.cmos.read_index()
            };
            regs.rax = (regs.rax & !0xff) | u64::from(value);
        } else if port == CMOS_INDEX_PORT {
            self.cmos.write_index(regs.rax as u8);
        } else {
            self.cmos.write_data(regs.rax as u8);
        }

        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_dma_page_io(&mut self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported DMA page-register access width {size}"
            )));
        }

        let index = usize::from(run_state.io_port() - DMA_PAGE_REGISTER_BASE);
        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            regs.rax = (regs.rax & !0xff) | u64::from(self.dma_page_registers[index]);
        } else {
            self.dma_page_registers[index] = regs.rax as u8;
        }

        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_pic_io_(&self, run_state: &RunState) -> io::Result<()> {
        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            regs.rax = regs.rax & !0xff;
        }
        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_pic_io(&self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported PIC access width {size}"
            )));
        }

        let port = run_state.io_port();
        let pic = match port {
            PIC_MASTER_COMMAND_PORT | PIC_MASTER_DATA_PORT | PIC_MASTER_ELCR_PORT => {
                &self.pic_master
            }
            PIC_SLAVE_COMMAND_PORT | PIC_SLAVE_DATA_PORT | PIC_SLAVE_ELCR_PORT => &self.pic_slave,
            _ => unreachable!(),
        };
        let is_command = matches!(port, PIC_MASTER_COMMAND_PORT | PIC_SLAVE_COMMAND_PORT);
        let is_elcr = matches!(port, PIC_MASTER_ELCR_PORT | PIC_SLAVE_ELCR_PORT);

        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            let value = if is_elcr {
                pic.read_elcr()
            } else if is_command {
                pic.read_command()
            } else {
                pic.read_data()
            };
            regs.rax = (regs.rax & !0xff) | u64::from(value);
        } else if is_elcr {
            pic.write_elcr(regs.rax as u8);
        } else if is_command {
            pic.write_command(regs.rax as u8);
        } else {
            pic.write_data(regs.rax as u8);
        }

        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_system_control_port_b(&mut self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported port 0x61 access width {size}"
            )));
        }

        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            let mut value = self
                .system_control_port_b
                .fetch_xor(PORT_B_REFRESH_TOGGLE, Ordering::Relaxed)
                ^ PORT_B_REFRESH_TOGGLE;
            if self.pit_channel2_output_high() {
                value |= PORT_B_TIMER2_OUT;
            } else {
                value &= !PORT_B_TIMER2_OUT;
            }
            regs.rax = (regs.rax & !0xff) | u64::from(value);
        } else {
            let value = (regs.rax as u8) & PORT_B_WRITABLE_MASK;
            self.system_control_port_b.store(value, Ordering::Relaxed);
            if value & PORT_B_TIMER2_GATE == 0 {
                self.pit_channel2_out_high_at = None;
            }
        }

        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn handle_pit_io(&mut self, run_state: &RunState) -> io::Result<()> {
        let size = run_state.io_size();
        if size != 1 {
            return Err(io::Error::other(format!(
                "unsupported PIT access width {size}"
            )));
        }

        let port = run_state.io_port();
        let mut regs = self.vcpu.get_regs()?;
        if run_state.io_is_in() {
            let value = if port != PIT_COMMAND_PORT {
                let read_high = self.pit_next_read_high.fetch_xor(true, Ordering::Relaxed);
                let counter = self.pit_counter.fetch_sub(0x100, Ordering::Relaxed);
                if read_high {
                    (counter >> 8) as u8
                } else {
                    counter as u8
                }
            } else {
                0
            };
            regs.rax = (regs.rax & !0xff) | u64::from(value);
        } else {
            let value = regs.rax as u8;
            let pit_command_channel = (port == PIT_COMMAND_PORT).then_some(value >> 6);
            if port == PIT_COMMAND_PORT {
                if self.trace_timers {
                    eprintln!("rustshyper-vmm: PIT command value={value:#04x}");
                }
                self.pit_next_read_high.store(false, Ordering::Relaxed);
                if pit_command_channel == Some(0) {
                    self.pit_channel0_low_byte = None;
                }
                if pit_command_channel == Some(2) {
                    self.pit_channel2_low_byte = None;
                    self.pit_channel2_out_high_at = None;
                }
            }
            if port == PIT_CHANNEL0_PORT {
                self.write_pit_channel0(value);
            }
            if port == PIT_CHANNEL2_PORT {
                self.write_pit_channel2(value);
            }
        }

        if run_state.instruction_len != 0 {
            regs.rip = run_state
                .guest_rip
                .wrapping_add(run_state.instruction_len as u64);
        }
        self.vcpu.set_regs(&regs)
    }

    fn write_pit_channel0(&mut self, value: u8) {
        if let Some(low_byte) = self.pit_channel0_low_byte.take() {
            let reload = u16::from(low_byte) | (u16::from(value) << 8);
            self.arm_pit_channel0(if reload == 0 {
                0x1_0000
            } else {
                u32::from(reload)
            });
        } else {
            self.pit_channel0_low_byte = Some(value);
        }
    }

    fn arm_pit_channel0(&mut self, reload: u32) {
        let nanos = (u128::from(reload) * 1_000_000_000_u128) / u128::from(PIT_INPUT_CLOCK_HZ);
        self.pit_counter.store(reload as u16, Ordering::Relaxed);
        self.pit_irq_interval = Duration::from_nanos((nanos as u64).max(1));
        self.pit_irq_enabled = true;
        self.next_pit_irq = Instant::now() + self.pit_irq_interval;
        if self.trace_timers {
            eprintln!(
                "rustshyper-vmm: PIT channel0 armed reload={reload} interval_us={}",
                self.pit_irq_interval.as_micros()
            );
        }
    }

    fn write_pit_channel2(&mut self, value: u8) {
        if let Some(low_byte) = self.pit_channel2_low_byte.take() {
            let reload = u16::from(low_byte) | (u16::from(value) << 8);
            self.arm_pit_channel2(if reload == 0 {
                0x1_0000
            } else {
                u32::from(reload)
            });
        } else {
            self.pit_channel2_low_byte = Some(value);
        }
    }

    fn arm_pit_channel2(&mut self, reload: u32) {
        if self.system_control_port_b.load(Ordering::Relaxed) & PORT_B_TIMER2_GATE == 0 {
            self.pit_channel2_out_high_at = None;
            return;
        }

        let nanos = (u128::from(reload) * 1_000_000_000_u128) / u128::from(PIT_INPUT_CLOCK_HZ);
        self.pit_channel2_out_high_at = Some(Instant::now() + Duration::from_nanos(nanos as u64));
    }

    fn pit_channel2_output_high(&self) -> bool {
        self.pit_channel2_out_high_at
            .is_some_and(|deadline| Instant::now() >= deadline)
    }

    pub fn guest_memory(&mut self) -> &mut GuestMemory {
        &mut self.guest_memory
    }

    fn dump_exit_diagnostics(&self, run_state: &RunState) {
        eprintln!(
            "rustshyper-vmm: VM exit {} rip={:#x} instruction_len={} qualification={:#x} guest_phys_addr={:#x}",
            exit_reason_name(run_state.exit_reason),
            run_state.guest_rip,
            run_state.instruction_len,
            run_state.exit_qualification,
            run_state.guest_phys_addr
        );

        match self.vcpu.get_regs() {
            Ok(regs) => {
                eprintln!("rustshyper-vmm: regs {}", format_regs(&regs));
                if run_state.exit_reason == VMX_EXIT_REASON_HLT {
                    self.dump_early_exception_frame(&regs);
                }
            }
            Err(err) => eprintln!("rustshyper-vmm: failed to read regs after VM exit: {err}"),
        }

        match self.vcpu.get_sregs() {
            Ok(sregs) => dump_sregs(&sregs),
            Err(err) => eprintln!("rustshyper-vmm: failed to read sregs after VM exit: {err}"),
        }
    }

    fn dump_current_diagnostics(&self) {
        match self.vcpu.get_regs() {
            Ok(regs) => {
                eprintln!("rustshyper-vmm: current regs {}", format_regs(&regs));
                if let Ok(sregs) = self.vcpu.get_sregs() {
                    self.dump_guest_code_bytes(&sregs, regs.rip);
                    self.dump_guest_qwords(&sregs, "current stack", regs.rsp, 16);
                    self.dump_guest_qwords(&sregs, "current r14", regs.r14, 32);
                    dump_sregs(&sregs);
                }
            }
            Err(err) => eprintln!("rustshyper-vmm: failed to read current regs: {err}"),
        }
    }

    fn dump_guest_code_bytes(&self, sregs: &VcpuSregs, rip: u64) {
        let Ok(gpa) = self.translate_guest_addr(sregs, rip) else {
            return;
        };
        let Ok(start) = usize::try_from(gpa) else {
            return;
        };
        let end = start
            .saturating_add(16)
            .min(self.guest_memory.as_slice().len());
        let Some(bytes) = self.guest_memory.as_slice().get(start..end) else {
            return;
        };
        eprintln!("rustshyper-vmm: current code gpa={gpa:#x} bytes={bytes:02x?}");
    }

    fn dump_guest_qwords(&self, sregs: &VcpuSregs, label: &str, gva: u64, count: usize) {
        let Ok(gpa) = self.translate_guest_addr(sregs, gva) else {
            return;
        };
        let Ok(start) = usize::try_from(gpa) else {
            return;
        };
        let end = start
            .saturating_add(count * core::mem::size_of::<u64>())
            .min(self.guest_memory.as_slice().len());
        let Some(bytes) = self.guest_memory.as_slice().get(start..end) else {
            return;
        };

        let mut words = Vec::new();
        for chunk in bytes.chunks_exact(core::mem::size_of::<u64>()) {
            words.push(u64::from_le_bytes(chunk.try_into().expect("chunk size")));
        }
        eprintln!("rustshyper-vmm: {label} gva={gva:#x} gpa={gpa:#x} qwords={words:#x?}");
    }

    fn dump_early_exception_frame(&self, regs: &VcpuRegs) {
        let Ok(sregs) = self.vcpu.get_sregs() else {
            return;
        };

        let frame_gva = regs.rbx;
        let mut words = [0_u64; 6];
        for (index, offset) in [0x70_u64, 0x78, 0x80, 0x88, 0x90, 0x98]
            .into_iter()
            .enumerate()
        {
            let Ok(word) = self.read_guest_u64(&sregs, frame_gva.wrapping_add(offset)) else {
                return;
            };
            words[index] = word;
        }

        eprintln!(
            "rustshyper-vmm: early-exception frame gva={:#x} vector={:#x} qword70={:#x} qword78={:#x} qword80={:#x} qword88={:#x} qword90={:#x} qword98={:#x}",
            frame_gva, regs.rbp, words[0], words[1], words[2], words[3], words[4], words[5],
        );
    }

    fn read_guest_u64(&self, sregs: &VcpuSregs, gva: u64) -> io::Result<u64> {
        let gpa = self.translate_guest_addr(sregs, gva)?;
        let start = usize::try_from(gpa)
            .map_err(|_| io::Error::other(format!("guest address {gpa:#x} overflows usize")))?;
        let end = start
            .checked_add(8)
            .ok_or_else(|| io::Error::other("guest memory read overflows usize"))?;
        let bytes = self
            .guest_memory
            .as_slice()
            .get(start..end)
            .ok_or_else(|| io::Error::other(format!("guest read out of range gpa={gpa:#x}")))?;
        Ok(u64::from_le_bytes(
            bytes.try_into().expect("slice length checked"),
        ))
    }

    fn translate_guest_addr(&self, sregs: &VcpuSregs, gva: u64) -> io::Result<u64> {
        const CR0_PG: u64 = 1 << 31;
        const PTE_PRESENT: u64 = 1 << 0;
        const PTE_HUGE: u64 = 1 << 7;
        const PTE_ADDR_MASK: u64 = 0x000f_ffff_ffff_f000;
        const PAGE_2M_MASK: u64 = (1 << 21) - 1;
        const PAGE_1G_MASK: u64 = (1 << 30) - 1;

        if (sregs.cr0 & CR0_PG) == 0 {
            return Ok(gva);
        }

        let cr3 = sregs.cr3 & !0xfff;
        let pml4e = self.read_guest_phys_u64(cr3 + (((gva >> 39) & 0x1ff) * 8))?;
        if (pml4e & PTE_PRESENT) == 0 {
            return Err(io::Error::other("guest PML4 entry is not present"));
        }

        let pdpte =
            self.read_guest_phys_u64((pml4e & PTE_ADDR_MASK) + (((gva >> 30) & 0x1ff) * 8))?;
        if (pdpte & PTE_PRESENT) == 0 {
            return Err(io::Error::other("guest PDPT entry is not present"));
        }
        if (pdpte & PTE_HUGE) != 0 {
            return Ok((pdpte & PTE_ADDR_MASK) | (gva & PAGE_1G_MASK));
        }

        let pde =
            self.read_guest_phys_u64((pdpte & PTE_ADDR_MASK) + (((gva >> 21) & 0x1ff) * 8))?;
        if (pde & PTE_PRESENT) == 0 {
            return Err(io::Error::other("guest PD entry is not present"));
        }
        if (pde & PTE_HUGE) != 0 {
            return Ok((pde & PTE_ADDR_MASK) | (gva & PAGE_2M_MASK));
        }

        let pte = self.read_guest_phys_u64((pde & PTE_ADDR_MASK) + (((gva >> 12) & 0x1ff) * 8))?;
        if (pte & PTE_PRESENT) == 0 {
            return Err(io::Error::other("guest PT entry is not present"));
        }

        Ok((pte & PTE_ADDR_MASK) | (gva & 0xfff))
    }

    fn read_guest_phys_u64(&self, gpa: u64) -> io::Result<u64> {
        let start = usize::try_from(gpa)
            .map_err(|_| io::Error::other(format!("guest address {gpa:#x} overflows usize")))?;
        let end = start
            .checked_add(8)
            .ok_or_else(|| io::Error::other("guest memory read overflows usize"))?;
        let bytes = self
            .guest_memory
            .as_slice()
            .get(start..end)
            .ok_or_else(|| io::Error::other(format!("guest read out of range gpa={gpa:#x}")))?;
        Ok(u64::from_le_bytes(
            bytes.try_into().expect("slice length checked"),
        ))
    }
}

fn exit_reason_name(reason: u32) -> &'static str {
    match reason {
        VMX_EXIT_REASON_TRIPLE_FAULT => "TRIPLE_FAULT",
        VMX_EXIT_REASON_HLT => "HLT",
        VMX_EXIT_REASON_IO_INSTRUCTION => "IO_INSTRUCTION",
        VMX_EXIT_REASON_PAUSE_INSTRUCTION => "PAUSE_INSTRUCTION",
        _ => "unhandled VM exit reason",
    }
}

fn format_regs(regs: &VcpuRegs) -> String {
    format!(
        concat!(
            "rip={:#x} rsp={:#x} rbp={:#x} rflags={:#x} ",
            "rax={:#x} rbx={:#x} rcx={:#x} rdx={:#x} ",
            "rsi={:#x} rdi={:#x} r8={:#x} r9={:#x} ",
            "r10={:#x} r11={:#x} r12={:#x} r13={:#x} r14={:#x} r15={:#x}"
        ),
        regs.rip,
        regs.rsp,
        regs.rbp,
        regs.rflags,
        regs.rax,
        regs.rbx,
        regs.rcx,
        regs.rdx,
        regs.rsi,
        regs.rdi,
        regs.r8,
        regs.r9,
        regs.r10,
        regs.r11,
        regs.r12,
        regs.r13,
        regs.r14,
        regs.r15
    )
}

fn dump_sregs(sregs: &VcpuSregs) {
    eprintln!(
        "rustshyper-vmm: control cr0={:#x} cr2={:#x} cr3={:#x} cr4={:#x} efer={:#x} apic_base={:#x}",
        sregs.cr0, sregs.cr2, sregs.cr3, sregs.cr4, sregs.efer, sregs.apic_base
    );
    eprintln!(
        "rustshyper-vmm: tables gdt={} idt={}",
        format_dtable(&sregs.gdt),
        format_dtable(&sregs.idt)
    );
    eprintln!(
        "rustshyper-vmm: segments cs={} ss={} ds={} es={} fs={} gs={} tr={} ldt={}",
        format_segment(&sregs.cs),
        format_segment(&sregs.ss),
        format_segment(&sregs.ds),
        format_segment(&sregs.es),
        format_segment(&sregs.fs),
        format_segment(&sregs.gs),
        format_segment(&sregs.tr),
        format_segment(&sregs.ldt)
    );
}

fn format_segment(segment: &VcpuSegment) -> String {
    format!(
        concat!(
            "sel={:#x} base={:#x} limit={:#x} type={:#x} ",
            "p={} dpl={} db={} s={} l={} g={} avl={} unusable={}"
        ),
        segment.selector,
        segment.base,
        segment.limit,
        segment.type_,
        segment.present,
        segment.dpl,
        segment.db,
        segment.s,
        segment.l,
        segment.g,
        segment.avl,
        segment.unusable
    )
}

fn format_dtable(table: &VcpuDtable) -> String {
    format!("base={:#x} limit={:#x}", table.base, table.limit)
}

struct CmosRtc {
    index: AtomicU8,
    status_b: AtomicU8,
}

impl CmosRtc {
    const fn new() -> Self {
        Self {
            index: AtomicU8::new(0),
            status_b: AtomicU8::new(0x02),
        }
    }

    // 0x70
    fn read_index(&self) -> u8 {
        self.index.load(Ordering::Relaxed)
    }

    fn write_index(&self, value: u8) {
        self.index.store(value & 0x7f, Ordering::Relaxed);
    }

    // 0x71
    fn read_data(&self) -> u8 {
        let status_b = self.status_b.load(Ordering::Relaxed);
        let binary = status_b & 0x04 != 0;
        match self.index.load(Ordering::Relaxed) {
            0x00 => encode_cmos_value(0, binary),
            0x02 => encode_cmos_value(22, binary),
            0x04 => encode_cmos_value(21, binary),
            0x06 => encode_cmos_value(5, binary),
            0x07 => encode_cmos_value(24, binary),
            0x08 => encode_cmos_value(4, binary),
            0x09 => encode_cmos_value(26, binary),
            0x0a => 0x26,
            0x0b => status_b,
            0x0c => 0,
            0x0d => 0x80,
            0x32 => encode_cmos_value(20, binary),
            _ => 0,
        }
    }

    fn write_data(&self, value: u8) {
        if self.index.load(Ordering::Relaxed) == 0x0b {
            self.status_b.store(value, Ordering::Relaxed);
        }
    }
}

fn encode_cmos_value(value: u8, binary: bool) -> u8 {
    if binary {
        value
    } else {
        ((value / 10) << 4) | (value % 10)
    }
}

fn is_cmos_port(port: u16) -> bool {
    matches!(port, CMOS_INDEX_PORT | CMOS_DATA_PORT)
}

fn is_i8042_port(port: u16) -> bool {
    matches!(port, I8042_DATA_PORT | I8042_COMMAND_STATUS_PORT)
}

fn is_absent_legacy_serial_port(port: u16) -> bool {
    ABSENT_LEGACY_SERIAL_BASES
        .into_iter()
        .any(|base| (base..base + LEGACY_SERIAL_PORT_WIDTH).contains(&port))
}

struct Pic8259 {
    imr: AtomicU8,
    elcr: AtomicU8,
    init_step: AtomicU8,
    expect_icw4: AtomicBool,
}

impl Pic8259 {
    const fn new() -> Self {
        Self {
            imr: AtomicU8::new(0xff),
            elcr: AtomicU8::new(0),
            init_step: AtomicU8::new(0),
            expect_icw4: AtomicBool::new(false),
        }
    }

    fn read_command(&self) -> u8 {
        0
    }

    fn read_data(&self) -> u8 {
        self.imr.load(Ordering::Relaxed)
    }

    fn write_command(&self, value: u8) {
        if value & 0x10 != 0 {
            self.expect_icw4.store(value & 0x01 != 0, Ordering::Relaxed);
            self.init_step.store(1, Ordering::Relaxed);
        }
    }

    fn write_data(&self, value: u8) {
        match self.init_step.load(Ordering::Relaxed) {
            0 => self.imr.store(value, Ordering::Relaxed),
            1 => self.init_step.store(2, Ordering::Relaxed),
            2 if self.expect_icw4.load(Ordering::Relaxed) => {
                self.init_step.store(3, Ordering::Relaxed);
            }
            2 | 3 => self.init_step.store(0, Ordering::Relaxed),
            _ => self.init_step.store(0, Ordering::Relaxed),
        }
    }

    fn read_elcr(&self) -> u8 {
        self.elcr.load(Ordering::Relaxed)
    }

    fn write_elcr(&self, value: u8) {
        self.elcr.store(value, Ordering::Relaxed);
    }
}

fn is_pic_port(port: u16) -> bool {
    matches!(
        port,
        PIC_MASTER_COMMAND_PORT
            | PIC_MASTER_DATA_PORT
            | PIC_SLAVE_COMMAND_PORT
            | PIC_SLAVE_DATA_PORT
            | PIC_MASTER_ELCR_PORT
            | PIC_SLAVE_ELCR_PORT
    )
}

fn diagnostic_deadline_from_env() -> io::Result<Option<Instant>> {
    let Some(seconds) = env_u64("RUSTSHYPER_DIAG_AFTER_SECS")? else {
        return Ok(None);
    };
    Ok(Some(Instant::now() + Duration::from_secs(seconds)))
}

fn env_u64(name: &str) -> io::Result<Option<u64>> {
    let Some(value) = env::var_os(name) else {
        return Ok(None);
    };

    value
        .to_string_lossy()
        .parse::<u64>()
        .map(Some)
        .map_err(|err| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("invalid {name} value: {err}"),
            )
        })
}

fn spawn_stdin_thread() -> io::Result<(Receiver<u8>, Arc<AtomicBool>)> {
    let (tx, rx) = mpsc::channel();
    let alive = Arc::new(AtomicBool::new(true));
    let alive_for_thread = Arc::clone(&alive);

    thread::Builder::new()
        .name("rustshyper-vmm-stdin".into())
        .spawn(move || {
            let mut stdin = io::stdin().lock();
            let mut buf = [0u8; 1];

            while alive_for_thread.load(Ordering::Relaxed) {
                match io::Read::read(&mut stdin, &mut buf) {
                    Ok(0) => break,
                    Ok(_) => {
                        if tx.send(buf[0]).is_err() {
                            break;
                        }
                    }
                    Err(err) if err.kind() == io::ErrorKind::Interrupted => continue,
                    Err(_) => break,
                }
            }
        })
        .map_err(|err| io::Error::other(format!("failed to spawn stdin thread: {err}")))?;

    Ok((rx, alive))
}

struct TerminalRawMode {
    original: termios,
}

impl TerminalRawMode {
    fn enable() -> io::Result<Self> {
        let fd = libc::STDIN_FILENO;
        let mut original = unsafe { std::mem::zeroed::<termios>() };

        if unsafe { tcgetattr(fd, &mut original) } != 0 {
            return Err(io::Error::last_os_error());
        }

        let mut raw = original;
        raw.c_lflag &= !(ICANON | ECHO | ISIG);
        raw.c_iflag &= !ICRNL;
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;

        if unsafe { tcsetattr(fd, TCSANOW, &raw) } != 0 {
            return Err(io::Error::last_os_error());
        }

        Ok(Self { original })
    }
}

impl Drop for TerminalRawMode {
    fn drop(&mut self) {
        let _ = unsafe { tcsetattr(libc::STDIN_FILENO, TCSANOW, &self.original) };
    }
}

pub fn parse_u64(value: &str) -> io::Result<u64> {
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(invalid_number)
    } else {
        value.parse::<u64>().map_err(invalid_number)
    }
}

fn invalid_number(err: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, err.to_string())
}

pub fn load_config_from_args(args: &[String]) -> io::Result<VmmConfig> {
    let mut guest_path: Option<PathBuf> = None;
    let mut device_path = PathBuf::from("/dev/rustshyper");
    let mut initrd_path: Option<PathBuf> = None;
    let mut cmdline: Option<String> = None;
    let mut guest_mem_size = 0x20_0000_u64;
    let mut load_addr = 0x10_0000_u64;
    let mut entry_point = 0x10_0000_u64;
    let mut stack_pointer = 0x1f_f000_u64;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--guest" => {
                i += 1;
                guest_path = args.get(i).map(PathBuf::from);
            }
            "--device" => {
                i += 1;
                device_path = PathBuf::from(required_arg(args, i, "--device")?);
            }
            "--initrd" => {
                i += 1;
                initrd_path = Some(PathBuf::from(required_arg(args, i, "--initrd")?));
            }
            "--cmdline" => {
                i += 1;
                cmdline = Some(required_arg(args, i, "--cmdline")?.to_owned());
            }
            "--mem-size" => {
                i += 1;
                guest_mem_size = parse_u64(required_arg(args, i, "--mem-size")?)?;
            }
            "--load-addr" => {
                i += 1;
                load_addr = parse_u64(required_arg(args, i, "--load-addr")?)?;
            }
            "--entry" => {
                i += 1;
                entry_point = parse_u64(required_arg(args, i, "--entry")?)?;
            }
            "--stack" => {
                i += 1;
                stack_pointer = parse_u64(required_arg(args, i, "--stack")?)?;
            }
            "--help" | "-h" => {
                return Err(io::Error::new(io::ErrorKind::Interrupted, usage()));
            }
            other => {
                if guest_path.is_none() && Path::new(other).exists() {
                    guest_path = Some(PathBuf::from(other));
                } else {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        format!("unknown argument: {other}\n\n{}", usage()),
                    ));
                }
            }
        }
        i += 1;
    }

    let guest_path = guest_path.ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("missing guest image path\n\n{}", usage()),
        )
    })?;

    Ok(VmmConfig {
        device_path,
        guest_path,
        initrd_path,
        cmdline,
        guest_mem_size: guest_mem_size as usize,
        load_addr,
        entry_point,
        stack_pointer,
    })
}

fn required_arg<'a>(args: &'a [String], index: usize, flag: &str) -> io::Result<&'a str> {
    args.get(index).map(String::as_str).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("missing value for {flag}"),
        )
    })
}

pub fn usage() -> &'static str {
    "Usage: rustshyper-vmm --guest <path> [--device /dev/rustshyper] [--initrd <path>] [--cmdline <args>] [--mem-size bytes] [--load-addr addr] [--entry addr] [--stack addr]"
}
