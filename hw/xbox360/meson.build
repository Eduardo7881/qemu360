xbox360_ss = ss.source_set()
xbox360_ss.add(files('xbox360.c'))

xbox360_ss.add(when: 'CONFIG_XBOX360', if_true: files(
  'xbox360_nand.c',
  'xbox360_smc.c',
  'xbox360_gpu.c',
  'crypto/xe_keys.c',
  'crypto/xe_crypto.c'
))

hw_arch += {'xbox360': xbox360_ss}
