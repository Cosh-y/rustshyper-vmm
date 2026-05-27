use core::mem::size_of;
use libc::c_ulong;

const IOC_NRBITS: u32 = 8;
const IOC_TYPEBITS: u32 = 8;
const IOC_SIZEBITS: u32 = 14;

const IOC_NRSHIFT: u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + IOC_NRBITS;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + IOC_TYPEBITS;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + IOC_SIZEBITS;

const IOC_NONE: u32 = 0;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

const fn ioc(dir: u32, ty: u8, nr: u8, size: usize) -> c_ulong {
    ((dir << IOC_DIRSHIFT)
        | ((ty as u32) << IOC_TYPESHIFT)
        | ((nr as u32) << IOC_NRSHIFT)
        | ((size as u32) << IOC_SIZESHIFT)) as c_ulong
}

const fn io(ty: u8, nr: u8) -> c_ulong {
    ioc(IOC_NONE, ty, nr, 0)
}

const fn ior<T>(ty: u8, nr: u8) -> c_ulong {
    ioc(IOC_READ, ty, nr, size_of::<T>())
}

const fn iow<T>(ty: u8, nr: u8) -> c_ulong {
    ioc(IOC_WRITE, ty, nr, size_of::<T>())
}

const fn iowr<T>(ty: u8, nr: u8) -> c_ulong {
    ioc(IOC_READ | IOC_WRITE, ty, nr, size_of::<T>())
}

pub const RSH_IOC_TYPE: u8 = b'H';

pub const RSH_GET_API_VERSION: c_ulong = io(RSH_IOC_TYPE, 0x00);
pub const RSH_CREATE_VM: c_ulong = io(RSH_IOC_TYPE, 0x01);
pub const RSH_CHECK_EXTENSION: c_ulong = iow::<i32>(RSH_IOC_TYPE, 0x03);

pub const RSH_CREATE_VCPU: c_ulong = iow::<u32>(RSH_IOC_TYPE, 0x41);
pub const RSH_GET_DIRTY_LOG: c_ulong = iowr::<DirtyLog>(RSH_IOC_TYPE, 0x42);
pub const RSH_INJECT_IRQ: c_ulong = iow::<u32>(RSH_IOC_TYPE, 0x43);
pub const RSH_SET_USER_MEMORY_REGION: c_ulong = iow::<UserMemoryRegion>(RSH_IOC_TYPE, 0x46);

pub const RSH_RUN: c_ulong = ior::<RunState>(RSH_IOC_TYPE, 0x80);
pub const RSH_GET_REGS: c_ulong = ior::<VcpuRegs>(RSH_IOC_TYPE, 0x81);
pub const RSH_SET_REGS: c_ulong = iow::<VcpuRegs>(RSH_IOC_TYPE, 0x82);
pub const RSH_GET_SREGS: c_ulong = ior::<VcpuSregs>(RSH_IOC_TYPE, 0x83);
pub const RSH_SET_SREGS: c_ulong = iow::<VcpuSregs>(RSH_IOC_TYPE, 0x84);
pub const RSH_INJECT_INTERRUPT: c_ulong = iow::<u32>(RSH_IOC_TYPE, 0x85);

pub const VMX_EXIT_REASON_HLT: u32 = 12;
pub const VMX_EXIT_REASON_TRIPLE_FAULT: u32 = 2;
pub const VMX_EXIT_REASON_IO_INSTRUCTION: u32 = 30;
pub const VMX_EXIT_REASON_PAUSE_INSTRUCTION: u32 = 40;
pub const VMX_EXIT_REASON_EPT_VIOLATION: u32 = 48;
pub const VMX_EXIT_REASON_PREEMPTION_TIMER: u32 = 52;

pub const COM1_BASE: u16 = 0x3f8;
pub const COM1_END: u16 = COM1_BASE + 8;

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct UserMemoryRegion {
    pub slot: u32,
    pub flags: u32,
    pub guest_phys_addr: u64,
    pub memory_size: u64,
    pub userspace_addr: u64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct DirtyLog {
    pub slot: u32,
    pub padding: u32,
    pub dirty_bitmap: u64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VcpuRegs {
    pub rax: u64,
    pub rbx: u64,
    pub rcx: u64,
    pub rdx: u64,
    pub rsi: u64,
    pub rdi: u64,
    pub rbp: u64,
    pub rsp: u64,
    pub r8: u64,
    pub r9: u64,
    pub r10: u64,
    pub r11: u64,
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
    pub rip: u64,
    pub rflags: u64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VcpuSegment {
    pub base: u64,
    pub limit: u32,
    pub selector: u16,
    pub type_: u8,
    pub present: u8,
    pub dpl: u8,
    pub db: u8,
    pub s: u8,
    pub l: u8,
    pub g: u8,
    pub avl: u8,
    pub unusable: u8,
    pub padding: u8,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VcpuDtable {
    pub base: u64,
    pub limit: u16,
    pub padding: [u16; 3],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VcpuSregs {
    pub cs: VcpuSegment,
    pub ds: VcpuSegment,
    pub es: VcpuSegment,
    pub fs: VcpuSegment,
    pub gs: VcpuSegment,
    pub ss: VcpuSegment,
    pub tr: VcpuSegment,
    pub ldt: VcpuSegment,
    pub gdt: VcpuDtable,
    pub idt: VcpuDtable,
    pub cr0: u64,
    pub cr2: u64,
    pub cr3: u64,
    pub cr4: u64,
    pub efer: u64,
    pub apic_base: u64,
    pub interrupt_bitmap: [u64; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct IoExit {
    pub port: u16,
    pub size: u8,
    pub is_in: u8,
    pub is_string: u8,
    pub is_repeat: u8,
    pub reserved: [u8; 2],
    pub count: u32,
    pub data: u64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct MmioInfo {
    pub phys_addr: u64,
    pub data: u64,
    pub len: u32,
    pub is_write: u8,
    pub reserved: [u8; 3],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RunState {
    pub exit_reason: u32,
    pub instruction_len: u32,
    pub guest_rip: u64,
    pub guest_phys_addr: u64,
    pub exit_qualification: u64,
    pub io: IoExit,
    pub mmio: MmioInfo,
}

impl RunState {
    pub fn io_port(&self) -> u16 {
        if self.io.port != 0 {
            self.io.port
        } else {
            ((self.exit_qualification >> 16) & 0xffff) as u16
        }
    }

    pub fn io_size(&self) -> u8 {
        if self.io.size != 0 {
            self.io.size
        } else {
            ((self.exit_qualification & 0b111) as u8) + 1
        }
    }

    pub fn io_is_in(&self) -> bool {
        if self.io.is_in != 0 {
            true
        } else {
            (self.exit_qualification & (1 << 3)) != 0
        }
    }

    pub fn io_is_string(&self) -> bool {
        if self.io.is_string != 0 {
            true
        } else {
            (self.exit_qualification & (1 << 4)) != 0
        }
    }

    pub fn io_is_repeat(&self) -> bool {
        if self.io.is_repeat != 0 {
            true
        } else {
            (self.exit_qualification & (1 << 5)) != 0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_numbers_are_stable() {
        assert_eq!(RSH_GET_API_VERSION, 0x4800);
        assert_eq!(RSH_CREATE_VM, 0x4801);
        assert_eq!(RSH_CREATE_VCPU, 0x40044841);
        assert_eq!(RSH_INJECT_IRQ, iow::<u32>(RSH_IOC_TYPE, 0x43));
        assert_eq!(RSH_SET_SREGS, iow::<VcpuSregs>(RSH_IOC_TYPE, 0x84));
        assert_eq!(RSH_INJECT_INTERRUPT, iow::<u32>(RSH_IOC_TYPE, 0x85));
    }
}
