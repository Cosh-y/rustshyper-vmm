use core::mem::size_of;
use std::io;

use crate::ioctl::{VcpuDtable, VcpuRegs, VcpuSegment, VcpuSregs};

const SETUP_HEADER_OFFSET: usize = 0x1f1;
const LINUX_BOOT_HEADER_MAGIC: u32 = 0x5372_6448;
const LINUX_BOOT_FLAG_MAGIC: u16 = 0xaa55;
const LINUX_MIN_BOOT_PROTOCOL: u16 = 0x020f;
const CAN_USE_HEAP: u8 = 0x80;
const LOADED_HIGH: u8 = 0x01;

const ZERO_PAGE_GPA: u64 = 0x0001_0000;
const CMDLINE_GPA: u64 = 0x0002_0000;
const GDT_GPA: u64 = 0x0000_5000;
const EBDA_END: u64 = 0x0009_f000;
const LOW_MEM_HOLE_END: u64 = 0x0010_0000;
const DEFAULT_KERNEL_LOAD_ADDR: u64 = 0x0010_0000;
const DEFAULT_BOOT_STACK_PTR: u64 = 0x001f_f000;
const DEFAULT_CMDLINE: &str = "console=ttyS0 earlycon=uart8250,io,0x3f8 nokaslr";

const ACPI_RSDP_GPA: u64 = 0x000f_0000;
const ACPI_RSDT_GPA: u32 = 0x000f_1000;
const ACPI_FADT_GPA: u32 = 0x000f_2000;
const ACPI_MADT_GPA: u32 = 0x000f_3000;
const LAPIC_BASE: u32 = 0xfee0_0000;
const IOAPIC_BASE: u32 = 0xfec0_0000;
const IOAPIC_ID: u8 = 1;

const X86_CR0_PE: u64 = 1 << 0;
const X86_CR0_ET: u64 = 1 << 4;
const X86_CR0_NE: u64 = 1 << 5;

const E820_RAM: u32 = 1;
const E820_RESERVED: u32 = 2;
const E820_MAX_ENTRIES_ZEROPAGE: usize = 128;

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
struct SetupHeader {
    setup_sects: u8,
    root_flags: u16,
    syssize: u32,
    ram_size: u16,
    vid_mode: u16,
    root_dev: u16,
    boot_flag: u16,
    jump: u16,
    header: u32,
    version: u16,
    realmode_swtch: u32,
    start_sys_seg: u16,
    kernel_version: u16,
    type_of_loader: u8,
    loadflags: u8,
    setup_move_size: u16,
    code32_start: u32,
    ramdisk_image: u32,
    ramdisk_size: u32,
    bootsect_kludge: u32,
    heap_end_ptr: u16,
    ext_loader_ver: u8,
    ext_loader_type: u8,
    cmd_line_ptr: u32,
    initrd_addr_max: u32,
    kernel_alignment: u32,
    relocatable_kernel: u8,
    min_alignment: u8,
    xloadflags: u16,
    cmdline_size: u32,
    hardware_subarch: u32,
    hardware_subarch_data: u64,
    payload_offset: u32,
    payload_length: u32,
    setup_data: u64,
    pref_address: u64,
    init_size: u32,
    handover_offset: u32,
    kernel_info_offset: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
struct BootE820Entry {
    addr: u64,
    size: u64,
    typ: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
struct BootParams {
    screen_info: [u8; 0x040],
    apm_bios_info: [u8; 0x014],
    pad2: [u8; 0x004],
    tboot_addr: u64,
    ist_info: [u8; 0x010],
    acpi_rsdp_addr: u64,
    pad3: [u8; 0x008],
    hd0_info: [u8; 16],
    hd1_info: [u8; 16],
    sys_desc_table: [u8; 16],
    olpc_ofw_header: [u8; 0x10],
    ext_ramdisk_image: u32,
    ext_ramdisk_size: u32,
    ext_cmd_line_ptr: u32,
    pad4: [u8; 112],
    cc_blob_address: u32,
    edid_info: [u8; 128],
    efi_info: [u8; 32],
    alt_mem_k: u32,
    scratch: u32,
    e820_entries: u8,
    eddbuf_entries: u8,
    edd_mbr_sig_buf_entries: u8,
    kbd_status: u8,
    secure_boot: u8,
    pad5: [u8; 2],
    sentinel: u8,
    pad6: [u8; 1],
    hdr: SetupHeader,
    pad7: [u8; 0x290 - 0x1f1 - size_of::<SetupHeader>()],
    edd_mbr_sig_buffer: [u32; 16],
    e820_table: [BootE820Entry; E820_MAX_ENTRIES_ZEROPAGE],
    pad8: [u8; 48],
    eddbuf: [u8; 0x1ec],
    pad9: [u8; 276],
}

impl Default for BootParams {
    fn default() -> Self {
        Self {
            screen_info: [0; 0x040],
            apm_bios_info: [0; 0x014],
            pad2: [0; 0x004],
            tboot_addr: 0,
            ist_info: [0; 0x010],
            acpi_rsdp_addr: 0,
            pad3: [0; 0x008],
            hd0_info: [0; 16],
            hd1_info: [0; 16],
            sys_desc_table: [0; 16],
            olpc_ofw_header: [0; 0x10],
            ext_ramdisk_image: 0,
            ext_ramdisk_size: 0,
            ext_cmd_line_ptr: 0,
            pad4: [0; 112],
            cc_blob_address: 0,
            edid_info: [0; 128],
            efi_info: [0; 32],
            alt_mem_k: 0,
            scratch: 0,
            e820_entries: 0,
            eddbuf_entries: 0,
            edd_mbr_sig_buf_entries: 0,
            kbd_status: 0,
            secure_boot: 0,
            pad5: [0; 2],
            sentinel: 0,
            pad6: [0; 1],
            hdr: SetupHeader::default(),
            pad7: [0; 0x290 - 0x1f1 - size_of::<SetupHeader>()],
            edd_mbr_sig_buffer: [0; 16],
            e820_table: [BootE820Entry::default(); E820_MAX_ENTRIES_ZEROPAGE],
            pad8: [0; 48],
            eddbuf: [0; 0x1ec],
            pad9: [0; 276],
        }
    }
}

#[derive(Debug, Clone)]
pub struct LinuxBootConfig<'a> {
    pub kernel_load_addr: u64,
    pub guest_mem_size: u64,
    pub vcpu_count: u32,
    pub initrd: Option<&'a [u8]>,
    pub cmdline: Option<&'a str>,
    pub stack_pointer: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct LinuxBootState {
    pub regs: VcpuRegs,
    pub sregs: VcpuSregs,
    pub entry_point: u64,
    pub zero_page_addr: u64,
}

pub fn looks_like_bzimage(image: &[u8]) -> bool {
    parse_setup_header(image).is_ok()
}

pub fn load_bzimage(
    guest_memory: &mut [u8],
    bzimage: &[u8],
    config: &LinuxBootConfig<'_>,
) -> io::Result<LinuxBootState> {
    let mut header = parse_setup_header(bzimage)?;
    let setup_sects = if header.setup_sects == 0 {
        4
    } else {
        header.setup_sects
    };
    let setup_bytes = (usize::from(setup_sects) + 1) * 512;
    if setup_bytes >= bzimage.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "bzImage setup sectors exceed image size",
        ));
    }

    let kernel_load_addr = if config.kernel_load_addr == 0 {
        DEFAULT_KERNEL_LOAD_ADDR
    } else {
        config.kernel_load_addr
    };
    let protected_payload = &bzimage[setup_bytes..];
    write_guest(guest_memory, kernel_load_addr, protected_payload)?;

    let gdt_entries = build_linux_boot_gdt();
    write_guest(guest_memory, GDT_GPA, as_bytes(&gdt_entries))?;

    let cmdline = config.cmdline.unwrap_or(DEFAULT_CMDLINE);
    let mut cmdline_buf = Vec::with_capacity(cmdline.len() + 1);
    cmdline_buf.extend_from_slice(cmdline.as_bytes());
    cmdline_buf.push(0);
    write_guest(guest_memory, CMDLINE_GPA, &cmdline_buf)?;

    let initrd_addr = if let Some(initrd) = config.initrd {
        let addr = place_initrd(config.guest_mem_size, &header, initrd.len() as u64)?;
        write_guest(guest_memory, addr, initrd)?;
        Some(addr)
    } else {
        None
    };

    header.type_of_loader = 0xff;
    header.loadflags |= CAN_USE_HEAP | LOADED_HIGH;
    header.heap_end_ptr = 0xfe00;
    header.code32_start = kernel_load_addr as u32;
    header.cmd_line_ptr = CMDLINE_GPA as u32;
    header.cmdline_size = cmdline_buf.len() as u32;
    if let Some(initrd_addr) = initrd_addr {
        header.ramdisk_image = initrd_addr as u32;
        header.ramdisk_size = config.initrd.unwrap().len() as u32;
    } else {
        header.ramdisk_image = 0;
        header.ramdisk_size = 0;
    }

    let mut boot_params = BootParams {
        hdr: header,
        ..BootParams::default()
    };
    boot_params.acpi_rsdp_addr = ACPI_RSDP_GPA;
    let e820_entries = make_e820_entries(config.guest_mem_size);
    boot_params.e820_entries = e820_entries.len() as u8;
    for (index, entry) in e820_entries.iter().enumerate() {
        boot_params.e820_table[index] = *entry;
    }
    write_acpi_tables(guest_memory, config.vcpu_count)?;
    write_guest(guest_memory, ZERO_PAGE_GPA, as_bytes(&boot_params))?;

    let entry_point = u64::from(header.code32_start);
    println!("rustshyper-vmm: Linux entry point at {entry_point:#x}");
    Ok(LinuxBootState {
        regs: VcpuRegs {
            rip: entry_point,
            rsp: if config.stack_pointer == 0 {
                DEFAULT_BOOT_STACK_PTR
            } else {
                config.stack_pointer
            },
            rsi: ZERO_PAGE_GPA,
            rflags: 0x2,
            ..VcpuRegs::default()
        },
        sregs: linux32_boot_sregs(),
        entry_point,
        zero_page_addr: ZERO_PAGE_GPA,
    })
}

fn parse_setup_header(image: &[u8]) -> io::Result<SetupHeader> {
    if image.len() < SETUP_HEADER_OFFSET + size_of::<SetupHeader>() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "image is too small to contain a Linux setup header",
        ));
    }

    let header = unsafe {
        image
            .as_ptr()
            .add(SETUP_HEADER_OFFSET)
            .cast::<SetupHeader>()
            .read_unaligned()
    };
    if header.boot_flag != LINUX_BOOT_FLAG_MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid Linux boot flag",
        ));
    }
    if header.header != LINUX_BOOT_HEADER_MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid Linux setup header magic",
        ));
    }
    if header.version < LINUX_MIN_BOOT_PROTOCOL {
        let version = header.version;
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "Linux boot protocol {:#x} is too old, need at least {:#x}",
                version, LINUX_MIN_BOOT_PROTOCOL
            ),
        ));
    }
    Ok(header)
}

fn place_initrd(guest_mem_size: u64, header: &SetupHeader, initrd_len: u64) -> io::Result<u64> {
    let max_addr = if header.initrd_addr_max == 0 {
        guest_mem_size.saturating_sub(1)
    } else {
        guest_mem_size
            .saturating_sub(1)
            .min(u64::from(header.initrd_addr_max))
    };
    let end = align_down(max_addr.saturating_add(1), 0x1000);
    let start = align_down(end.saturating_sub(initrd_len), 0x1000);
    if start < LOW_MEM_HOLE_END {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "guest memory is too small to place the initrd below initrd_addr_max",
        ));
    }
    Ok(start)
}

fn make_e820_entries(guest_mem_size: u64) -> Vec<BootE820Entry> {
    let mut entries = Vec::new();
    entries.push(BootE820Entry {
        addr: 0,
        size: EBDA_END,
        typ: E820_RAM,
    });
    entries.push(BootE820Entry {
        addr: EBDA_END,
        size: LOW_MEM_HOLE_END - EBDA_END,
        typ: E820_RESERVED,
    });
    if guest_mem_size > LOW_MEM_HOLE_END {
        entries.push(BootE820Entry {
            addr: LOW_MEM_HOLE_END,
            size: guest_mem_size - LOW_MEM_HOLE_END,
            typ: E820_RAM,
        });
    }
    entries
}

fn write_acpi_tables(guest_memory: &mut [u8], vcpu_count: u32) -> io::Result<()> {
    let madt = build_madt(vcpu_count);
    let rsdt = build_rsdt(&[ACPI_MADT_GPA, ACPI_FADT_GPA]);
    let rsdp = build_rsdp(ACPI_RSDT_GPA);
    let fadt = build_fadt();

    write_guest(guest_memory, u64::from(ACPI_MADT_GPA), &madt)?;
    write_guest(guest_memory, u64::from(ACPI_RSDT_GPA), &rsdt)?;
    write_guest(guest_memory, ACPI_RSDP_GPA, &rsdp)?;
    write_guest(guest_memory, u64::from(ACPI_FADT_GPA), &fadt)?;
    Ok(())
}

fn build_madt(vcpu_count: u32) -> Vec<u8> {
    let mut table = acpi_header(*b"APIC", 56, 1);
    push_u32(&mut table, LAPIC_BASE);
    // PCAT_COMPAT flag. If set means dual 8259 PIC is present.
    push_u32(&mut table, 0);

    /* https://uefi.org/htmlspecs/ACPI_Spec_6_4_html/05_ACPI_Software_Programming_Model/ACPI_Software_Programming_Model.html#multiple-apic-description-table-madt */
    for vcpu_id in 0..vcpu_count {
        table.extend_from_slice(&[
            0,
            8, // Processor local APIC
            vcpu_id as u8,
            vcpu_id as u8, // ACPI processor ID, APIC ID
        ]);
        push_u32(&mut table, 1);
    }

    table.extend_from_slice(&[
        1, 12, // I/O APIC
        IOAPIC_ID, 0, // I/O APIC ID, reserved
    ]);
    push_u32(&mut table, IOAPIC_BASE);
    push_u32(&mut table, 0);

    finish_acpi_table(table)
}

fn build_fadt() -> Vec<u8> {
    let mut table = acpi_header(*b"FACP", 244, 6);
    push_u32(&mut table, 0); // Firmware Control
    push_u32(&mut table, 0); // DSDT
    table.push(0); // Reserved
    table.push(0); // Preferred PM Profile

    /* https://uefi.org/htmlspecs/ACPI_Spec_6_4_html/05_ACPI_Software_Programming_Model/ACPI_Software_Programming_Model.html#fixed-acpi-description-table-fadt
    / System vector the SCI interrupt is wired to in 8259 mode.
    / On systems that do not contain the 8259,
    / this field contains the Global System interrupt number of the SCI interrupt.
    */
    push_u16(&mut table, 9); // SCI_INT

    table.resize(244, 0);

    table[109..111].copy_from_slice(&0u16.to_le_bytes()); // IAPC_BOOT_ARCH

    finish_acpi_table(table)
}

fn build_rsdt(entries: &[u32]) -> Vec<u8> {
    let mut table = acpi_header(*b"RSDT", 36 + entries.len() as u32 * 4, 1);
    for entry in entries {
        push_u32(&mut table, *entry);
    }
    finish_acpi_table(table)
}

fn build_rsdp(rsdt_gpa: u32) -> Vec<u8> {
    let mut rsdp = Vec::with_capacity(36);
    rsdp.extend_from_slice(b"RSD PTR ");
    rsdp.push(0);
    rsdp.extend_from_slice(b"RSHVMM");
    rsdp.push(2);
    push_u32(&mut rsdp, rsdt_gpa);
    push_u32(&mut rsdp, 36);
    push_u64(&mut rsdp, 0);
    rsdp.push(0);
    rsdp.extend_from_slice(&[0; 3]);

    rsdp[8] = acpi_checksum(&rsdp[..20]);
    rsdp[32] = acpi_checksum(&rsdp);
    rsdp
}

fn acpi_header(signature: [u8; 4], length: u32, revision: u8) -> Vec<u8> {
    let mut header = Vec::with_capacity(length as usize);
    header.extend_from_slice(&signature);
    push_u32(&mut header, length);
    header.push(revision);
    header.push(0);
    header.extend_from_slice(b"RSHVMM");
    header.extend_from_slice(b"RUSTSHYP");
    push_u32(&mut header, 1);
    push_u32(&mut header, u32::from_le_bytes(*b"RSHV"));
    push_u32(&mut header, 1);
    header
}

fn finish_acpi_table(mut table: Vec<u8>) -> Vec<u8> {
    let length = table.len() as u32;
    table[4..8].copy_from_slice(&length.to_le_bytes());
    table[9] = acpi_checksum(&table);
    table
}

fn acpi_checksum(bytes: &[u8]) -> u8 {
    0_u8.wrapping_sub(bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte)))
}

fn push_u16(buf: &mut Vec<u8>, value: u16) {
    buf.extend_from_slice(&value.to_le_bytes());
}

fn push_u32(buf: &mut Vec<u8>, value: u32) {
    buf.extend_from_slice(&value.to_le_bytes());
}

fn push_u64(buf: &mut Vec<u8>, value: u64) {
    buf.extend_from_slice(&value.to_le_bytes());
}

fn build_linux_boot_gdt() -> [u64; 5] {
    [
        0,
        0,
        gdt_descriptor(0x0b),
        gdt_descriptor(0x03),
        gdt_system_descriptor(0x0b),
    ]
}

fn gdt_descriptor(type_bits: u8) -> u64 {
    let limit = 0xffff_u64;
    let base = 0_u64;
    limit
        | ((base & 0x00ff_ffff) << 16)
        | (u64::from(type_bits) << 40)
        | (1_u64 << 44)
        | (1_u64 << 47)
        | (((limit >> 16) & 0xf) << 48)
        | (1_u64 << 54)
        | (1_u64 << 55)
        | (((base >> 24) & 0xff) << 56)
}

fn gdt_system_descriptor(type_bits: u8) -> u64 {
    let limit = 0xffff_u64;
    let base = 0_u64;
    limit
        | ((base & 0x00ff_ffff) << 16)
        | (u64::from(type_bits) << 40)
        | (1_u64 << 47)
        | (((limit >> 16) & 0xf) << 48)
        | (((base >> 24) & 0xff) << 56)
}

pub fn linux32_boot_sregs() -> VcpuSregs {
    let code = flat_segment(0x10, 0x0b);
    let data = flat_segment(0x18, 0x03);

    VcpuSregs {
        cs: code,
        ds: data,
        es: data,
        fs: data,
        gs: data,
        ss: data,
        tr: system_segment(0x20, 0x0b),
        ldt: VcpuSegment {
            unusable: 1,
            ..VcpuSegment::default()
        },
        gdt: VcpuDtable {
            base: GDT_GPA,
            limit: (size_of::<u64>() * 5 - 1) as u16,
            ..VcpuDtable::default()
        },
        idt: VcpuDtable::default(),
        cr0: X86_CR0_PE | X86_CR0_ET | X86_CR0_NE,
        cr2: 0,
        cr3: 0,
        cr4: 0,
        efer: 0,
        apic_base: 0,
        interrupt_bitmap: [0; 4],
    }
}

fn flat_segment(selector: u16, type_: u8) -> VcpuSegment {
    VcpuSegment {
        base: 0,
        limit: 0xffff_ffff,
        selector,
        type_,
        present: 1,
        dpl: 0,
        db: 1,
        s: 1,
        l: 0,
        g: 1,
        avl: 0,
        unusable: 0,
        padding: 0,
    }
}

fn system_segment(selector: u16, type_: u8) -> VcpuSegment {
    VcpuSegment {
        base: 0,
        limit: 0xffff,
        selector,
        type_,
        present: 1,
        dpl: 0,
        db: 0,
        s: 0,
        l: 0,
        g: 0,
        avl: 0,
        unusable: 0,
        padding: 0,
    }
}

fn write_guest(guest_memory: &mut [u8], guest_addr: u64, data: &[u8]) -> io::Result<()> {
    let start = guest_addr as usize;
    let end = start.saturating_add(data.len());
    if end > guest_memory.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "guest data at {guest_addr:#x} with size {:#x} does not fit in guest memory",
                data.len()
            ),
        ));
    }
    guest_memory[start..end].copy_from_slice(data);
    Ok(())
}

fn align_down(value: u64, align: u64) -> u64 {
    value & !(align - 1)
}

fn as_bytes<T>(value: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts((value as *const T).cast::<u8>(), size_of::<T>()) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_linux_header() {
        let mut image = vec![0_u8; SETUP_HEADER_OFFSET + size_of::<SetupHeader>()];
        let header = SetupHeader {
            boot_flag: LINUX_BOOT_FLAG_MAGIC,
            header: LINUX_BOOT_HEADER_MAGIC,
            version: LINUX_MIN_BOOT_PROTOCOL,
            ..SetupHeader::default()
        };
        image[SETUP_HEADER_OFFSET..SETUP_HEADER_OFFSET + size_of::<SetupHeader>()]
            .copy_from_slice(as_bytes(&header));

        assert!(looks_like_bzimage(&image));
    }

    #[test]
    fn linux_sregs_use_protected_mode_segments() {
        let sregs = linux32_boot_sregs();
        assert_eq!(sregs.cs.selector, 0x10);
        assert_eq!(sregs.ds.selector, 0x18);
        assert_eq!(sregs.tr.selector, 0x20);
        assert_eq!(sregs.ldt.unusable, 1);
        assert_eq!(sregs.cr0, X86_CR0_PE | X86_CR0_ET | X86_CR0_NE);
    }

    #[test]
    fn acpi_tables_have_valid_checksums() {
        let madt = build_madt(4);
        let rsdt = build_rsdt(&[ACPI_MADT_GPA]);
        let rsdp = build_rsdp(ACPI_RSDT_GPA);

        assert_eq!(
            madt.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte)),
            0
        );
        assert_eq!(
            rsdt.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte)),
            0
        );
        assert_eq!(
            rsdp[..20]
                .iter()
                .fold(0_u8, |sum, byte| sum.wrapping_add(*byte)),
            0
        );
        assert_eq!(
            rsdp.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte)),
            0
        );
    }

    #[test]
    fn madt_advertises_each_vcpu() {
        let madt = build_madt(3);
        let local_apic_entries = madt.windows(2).filter(|entry| *entry == [0, 8]).count();

        assert_eq!(local_apic_entries, 3);
        assert!(
            madt.windows(8)
                .any(|entry| entry == [0, 8, 2, 2, 1, 0, 0, 0])
        );
    }
}
