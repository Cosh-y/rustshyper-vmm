# rustshyper-vmm

`rustshyper-vmm` is a userspace VMM crate that talks to `/dev/rustshyper` on Asterinas.

It provides:

- a KVM-like control flow: open device, create VM, map guest memory, create VCPU, set registers, run loop
- a Linux `bzImage` loader that builds a local `boot_params` zero page, E820 map, command line, and optional initrd placement
- a userspace-selected 32-bit protected-mode guest entry state, passed to the kernel through `SET_REGS` and `SET_SREGS`
- a userspace UART16550 emulator that forwards guest TX to `stdout`
- a raw-stdin keyboard loop that pushes host input into the guest UART RX FIFO
- multi-vCPU Linux boot plumbing: `--vcpus N` creates vCPU 0 as the BSP and vCPU 1..N-1 as APs that Linux discovers through ACPI MADT and starts with INIT/SIPI

## Current ABI expectation

The VMM expects `RSH_RUN` to return a run-state structure that contains at least:

- `exit_reason`
- `exit_qualification`
- `guest_rip`
- `instruction_len`

With those fields, the VMM can decode I/O exits for COM1 and emulate UART in userspace.

The VMM also expects `RSH_SET_SREGS` to exist. The userspace side now programs Linux-style
protected-mode segment/control-register state through that ioctl. The current kernel side can keep
this as a placeholder until VMCS guest-state synchronization is wired up.

## Example

```bash
cargo run -- \
  --guest ./bzImage \
  --initrd ./initramfs.cpio.gz \
  --cmdline "console=ttyS0 earlycon=uart8250,io,0x3f8 rdinit=/init nokaslr" \
  --mem-size 0x4000000 \
  --vcpus 4
```

`--idle-cpus N` is still accepted as a compatibility alias for `--vcpus N+1`.
