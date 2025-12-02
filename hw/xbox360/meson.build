xbox360_ss = ss.source_set()
xbox360_ss.add(files(
  'xbox360.c',
  'xbox360_nand.c',
  'xbox360_boot.c',
))

# Add Crypto
xbox360_ss.add(when: 'CONFIG_XBOX360', if_true: files(
  'crypto/xe_crypto.c',
  'crypto/xe_keys.c',
))

# Add future devices
xbox360_ss.add(when: ['CONFIG_XBOX360', 'CONFIG_XBOX360_SMC'], if_true: files(
  'xbox360_smc.c',
))

xbox360_ss.add(when: ['CONFIG_XBOX360', 'CONFIG_XBOX360_GPU'], if_true: files(
  'xbox360_gpu.c',
))

hw_arch += {'xbox360': xbox360_ss}
