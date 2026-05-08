use std::{
    fs::{File, OpenOptions},
    io,
    os::fd::{AsRawFd, FromRawFd, RawFd},
    path::Path,
    ptr::NonNull,
};

use libc::{MAP_ANONYMOUS, MAP_FAILED, MAP_PRIVATE, PROT_READ, PROT_WRITE, c_void, mmap, munmap};

use crate::ioctl::{
    RSH_CREATE_VCPU, RSH_CREATE_VM, RSH_GET_API_VERSION, RSH_GET_REGS, RSH_GET_SREGS,
    RSH_INJECT_INTERRUPT, RSH_INJECT_IRQ, RSH_RUN, RSH_SET_REGS, RSH_SET_SREGS,
    RSH_SET_USER_MEMORY_REGION, RunState, UserMemoryRegion, VcpuRegs, VcpuSregs,
};

fn ioctl_request(req: libc::c_ulong) -> libc::Ioctl {
    req as libc::Ioctl
}

fn with_context(err: io::Error, context: &str) -> io::Error {
    io::Error::new(err.kind(), format!("{context}: {err}"))
}

fn ioctl_with_ref<T>(fd: RawFd, req: libc::c_ulong, value: &T) -> io::Result<i32> {
    let ret = unsafe { libc::ioctl(fd, ioctl_request(req), value as *const T) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret)
    }
}

fn ioctl_with_mut_ref<T>(fd: RawFd, req: libc::c_ulong, value: &mut T) -> io::Result<i32> {
    let ret = unsafe { libc::ioctl(fd, ioctl_request(req), value as *mut T) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret)
    }
}

fn ioctl_no_arg(fd: RawFd, req: libc::c_ulong) -> io::Result<i32> {
    let ret = unsafe { libc::ioctl(fd, ioctl_request(req)) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret)
    }
}

#[derive(Debug)]
pub struct RustShyper {
    file: File,
}

impl RustShyper {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let path = path.as_ref();
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|err| with_context(err, &format!("failed to open {}", path.display())))?;
        Ok(Self { file })
    }

    pub fn api_version(&self) -> io::Result<i32> {
        ioctl_no_arg(self.file.as_raw_fd(), RSH_GET_API_VERSION)
            .map_err(|err| with_context(err, "RSH_GET_API_VERSION failed"))
    }

    pub fn create_vm(&self) -> io::Result<VmHandle> {
        let fd = ioctl_no_arg(self.file.as_raw_fd(), RSH_CREATE_VM)
            .map_err(|err| with_context(err, "RSH_CREATE_VM failed"))?;
        let vm_file = unsafe { File::from_raw_fd(fd) };
        Ok(VmHandle { file: vm_file })
    }
}

#[derive(Debug)]
pub struct VmHandle {
    file: File,
}

impl VmHandle {
    pub fn create_vcpu(&self, vcpu_id: u32) -> io::Result<VcpuHandle> {
        let fd =
            ioctl_with_ref(self.file.as_raw_fd(), RSH_CREATE_VCPU, &vcpu_id).map_err(|err| {
                with_context(
                    err,
                    &format!("RSH_CREATE_VCPU failed for vcpu_id={vcpu_id}"),
                )
            })?;
        let vcpu_file = unsafe { File::from_raw_fd(fd) };
        Ok(VcpuHandle { file: vcpu_file })
    }

    pub fn set_user_memory_region(&self, region: &UserMemoryRegion) -> io::Result<()> {
        ioctl_with_ref(self.file.as_raw_fd(), RSH_SET_USER_MEMORY_REGION, region).map_err(
            |err| {
                with_context(
                    err,
                    &format!(
                        "RSH_SET_USER_MEMORY_REGION failed for slot={} gpa={:#x} size={:#x} hva={:#x}",
                        region.slot,
                        region.guest_phys_addr,
                        region.memory_size,
                        region.userspace_addr
                    ),
                )
            },
        )?;
        Ok(())
    }

    pub fn inject_irq_line(&self, irq: u32) -> io::Result<()> {
        ioctl_with_ref(self.file.as_raw_fd(), RSH_INJECT_IRQ, &irq)
            .map_err(|err| with_context(err, &format!("RSH_INJECT_IRQ failed for irq={irq}")))?;
        Ok(())
    }
}

#[derive(Debug)]
pub struct VcpuHandle {
    file: File,
}

impl VcpuHandle {
    pub fn run(&self) -> io::Result<RunState> {
        let mut run_state = RunState::default();
        ioctl_with_mut_ref(self.file.as_raw_fd(), RSH_RUN, &mut run_state)
            .map_err(|err| with_context(err, "RSH_RUN failed"))?;
        Ok(run_state)
    }

    pub fn get_regs(&self) -> io::Result<VcpuRegs> {
        let mut regs = VcpuRegs::default();
        ioctl_with_mut_ref(self.file.as_raw_fd(), RSH_GET_REGS, &mut regs)
            .map_err(|err| with_context(err, "RSH_GET_REGS failed"))?;
        Ok(regs)
    }

    pub fn set_regs(&self, regs: &VcpuRegs) -> io::Result<()> {
        ioctl_with_ref(self.file.as_raw_fd(), RSH_SET_REGS, regs)
            .map_err(|err| with_context(err, "RSH_SET_REGS failed"))?;
        Ok(())
    }

    pub fn get_sregs(&self) -> io::Result<VcpuSregs> {
        let mut sregs = VcpuSregs::default();
        ioctl_with_mut_ref(self.file.as_raw_fd(), RSH_GET_SREGS, &mut sregs)
            .map_err(|err| with_context(err, "RSH_GET_SREGS failed"))?;
        Ok(sregs)
    }

    pub fn set_sregs(&self, sregs: &VcpuSregs) -> io::Result<()> {
        ioctl_with_ref(self.file.as_raw_fd(), RSH_SET_SREGS, sregs)
            .map_err(|err| with_context(err, "RSH_SET_SREGS failed"))?;
        Ok(())
    }

    pub fn inject_interrupt(&self, vector: u32) -> io::Result<()> {
        ioctl_with_ref(self.file.as_raw_fd(), RSH_INJECT_INTERRUPT, &vector).map_err(|err| {
            with_context(
                err,
                &format!("RSH_INJECT_INTERRUPT failed for vector={vector}"),
            )
        })?;
        Ok(())
    }
}

#[derive(Debug)]
pub struct GuestMemory {
    ptr: NonNull<u8>,
    len: usize,
}

impl GuestMemory {
    pub fn new(len: usize) -> io::Result<Self> {
        let ptr = unsafe {
            mmap(
                std::ptr::null_mut(),
                len,
                PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS,
                -1,
                0,
            )
        };

        if ptr == MAP_FAILED {
            return Err(io::Error::last_os_error());
        }

        Ok(Self {
            ptr: NonNull::new(ptr.cast::<u8>()).expect("mmap returned null"),
            len,
        })
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn userspace_addr(&self) -> u64 {
        self.ptr.as_ptr() as usize as u64
    }

    pub fn load(&mut self, guest_addr: usize, image: &[u8]) -> io::Result<()> {
        let end = guest_addr.saturating_add(image.len());
        if end > self.len {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "guest image does not fit in guest memory",
            ));
        }

        self.as_mut_slice()[guest_addr..end].copy_from_slice(image);
        Ok(())
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.len) }
    }

    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
    }
}

impl Drop for GuestMemory {
    fn drop(&mut self) {
        let _ = unsafe { munmap(self.ptr.as_ptr().cast::<c_void>(), self.len) };
    }
}
