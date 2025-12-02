xbox360_ss = ss.source_set()
xbox360_ss.add(files(
  'xbox360.c',
  'xbox360_nand.c',
  'xbox360_boot.c',
  'xbox360_smc.c',
  'xbox360_gpu.c',
  'xbox360_kernel.c',
  'xbox360_syscall.c'
))

# Add Crypto
xbox360_ss.add(when: 'CONFIG_XBOX360', if_true: files(
  'crypto/xe_crypto.c',
  'crypto/xe_keys.c',
))

hw_arch += {'xbox360': xbox360_ss}
