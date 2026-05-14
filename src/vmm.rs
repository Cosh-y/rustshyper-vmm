use std::{
    env, fs, io,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
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

const UART_IRQ_LINE: u32 = 4;

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
        if !(COM1_BASE..COM1_END).contains(&port) {
            // read 0x0 and ignore write
            println!("rustshyper-vmm: Guest access to unmapped I/O port at {:#x}", port);
            return self.handle_unmapped_io(run_state)
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

    fn handle_unmapped_io(&self, run_state: &RunState) -> io::Result<()> {
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

    fn trace_poll_exit(&self, name: &str, count: u64, run_state: &RunState) {
        eprintln!(
            "rustshyper-vmm: {name} exit count={count} rip={:#x} len={} qual={:#x}",
            run_state.guest_rip,
            run_state.instruction_len,
            run_state.exit_qualification,
        );
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
