xbox360_ss = ss.source_set()
xbox360_ss.add(files(
  'xbox360.c',
  'xbox360_mmu.c',      # New
  'xbox360_gic.c',      # New
  'xbox360_integration.c', # New
  'xbox360_boot.c',
  'xbox360_gpu.c',
  'xbox360_kernel.c',
  'xbox360_syscall.c',
  'xbox360_smc.c',
  'xbox360_nand.c',
  'crypto/xe_keys.c',
  'crypto/xe_crypt.c'
))

hw_arch += {'xbox360': xbox360_ss}
