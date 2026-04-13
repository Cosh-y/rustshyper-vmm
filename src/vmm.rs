use std::{
    fs, io,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver},
        Arc,
    },
    thread,
};

use libc::{tcgetattr, tcsetattr, termios, ECHO, ICANON, ICRNL, ISIG, TCSANOW, VMIN, VTIME};

use crate::{
    api::{GuestMemory, RustShyper, VcpuHandle, VmHandle},
    ioctl::{
        RunState, UserMemoryRegion, VcpuRegs, COM1_BASE, COM1_END, VMX_EXIT_REASON_HLT,
        VMX_EXIT_REASON_IO_INSTRUCTION,
    },
    linux::{self, LinuxBootConfig},
    uart::Uart16550,
};

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
    _vm: VmHandle,
    vcpu: VcpuHandle,
    guest_memory: GuestMemory,
    uart: Arc<Uart16550>,
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
            _vm: vm,
            vcpu,
            guest_memory,
            uart,
            stdin_rx,
            _stdin_thread_alive: alive,
        })
    }

    pub fn run(&mut self) -> io::Result<()> {
        let _raw_mode = TerminalRawMode::enable()?;

        loop {
            self.pump_stdin();

            let run_state = self.vcpu.run()?;

            match run_state.exit_reason {
                VMX_EXIT_REASON_IO_INSTRUCTION => self.handle_io_exit(&run_state)?,
                VMX_EXIT_REASON_HLT => return Ok(()),
                other => {
                    return Err(io::Error::other(format!(
                        "unhandled VM exit reason {other} at rip {:#x}",
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

    fn handle_io_exit(&self, run_state: &RunState) -> io::Result<()> {
        if run_state.io_is_string() || run_state.io_is_repeat() {
            return Err(io::Error::other(
                "string or REP-prefixed port I/O exits are not supported yet",
            ));
        }

        let port = run_state.io_port();
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
            regs.rax = (regs.rax & !0xff) | u64::from(value);
            if run_state.instruction_len != 0 {
                regs.rip = run_state
                    .guest_rip
                    .wrapping_add(run_state.instruction_len as u64);
            }
            self.vcpu.set_regs(&regs)?;
        } else {
            let mut regs = self.vcpu.get_regs()?;
            self.uart.write(offset, regs.rax as u8)?;
            if run_state.instruction_len != 0 {
                regs.rip = run_state
                    .guest_rip
                    .wrapping_add(run_state.instruction_len as u64);
            }
            self.vcpu.set_regs(&regs)?;
        }

        Ok(())
    }

    pub fn guest_memory(&mut self) -> &mut GuestMemory {
        &mut self.guest_memory
    }
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
