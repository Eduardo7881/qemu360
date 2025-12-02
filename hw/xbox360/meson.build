xbox360_ss = ss.source_set()
xbox360_ss.add(files(
  'xbox360.c',
  'xbox360_mmu.c',
  'xbox360_gic.c',
  'xbox360_pcie.c',      # New
  'xbox360_dma.c',       # New
  'xbox360_integration.c',
  'xbox360_boot.c',
  'xbox360_gpu.c',
  'xbox360_kernel.c',
  'xbox360_syscall.c',
  'xbox360_smc.c',
  'xbox360_nand.c',
))

hw_arch += {'xbox360': xbox360_ss}
