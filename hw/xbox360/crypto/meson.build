xbox360_crypto_ss = ss.source_set()
xbox360_crypto_ss.add(files(
  'xe_crypto.c',
  'xe_keys.c',
))

if config_all_devices.has_key('CONFIG_XBOX360')
  system_ss.add(when: 'CONFIG_XBOX360', if_true: xbox360_crypto_ss)
endif
