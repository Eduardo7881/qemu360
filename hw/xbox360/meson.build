xbox360_ss = ss.source_set()

# Core
xbox360_ss.add(files(
  'xbox360.c',
))

# Hardware
xbox360_ss.add(files(
  'xbox360_mmu.c',
  'xbox360_gic.c',
  'xbox360_pcie.c',
  'xbox360_dma.c',
  'xbox360_xma.c',
  'xbox360_timer.c',
  'xbox360_sata.c',
  'xbox360_usb.c',
  'xbox360_network.c',
  'xbox360_storage.c',
))

# System
xbox360_ss.add(files(
  'xbox360_boot.c',
  'xbox360_gpu.c',
  'xbox360_kernel.c',
  'xbox360_syscall.c',
  'xbox360_smc.c',
  'xbox360_nand.c',
  'xbox360_integration.c',
))

# Deps
xbox360_ss.add(audio)
xbox360_ss.add(block)
xbox360_ss.add(usb)
xbox360_ss.add(net)
xbox360_ss.add(display)

# Crypto
xbox360_ss.add(when: 'CONFIG_XBOX360', if_true: files('crypto/xe_crypto.c'))

# Register the Machine
machines += {
  'xbox360': {
    'targets': ['ppc'],
    'dependencies': ['pci', 'audio', 'usb', 'net'],
    'sources': xbox360_ss,
  }
}

# Add to QEMU build
qemu_system_ppc_ss.add_all(xbox360_ss)
